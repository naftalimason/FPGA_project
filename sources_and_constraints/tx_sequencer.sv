`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tx_sequencer
// Description: Envelope of the TX burst-read group (v2), mirror of
//   rx_sequencer: tx_ahb_master_mac (clk) + tx_afifo[R,G,B] + tx_consumer_fsm
//   (gclk). CDC glue: start_read pulse-toggle into clk; fifo_srst (from
//   seq_top_fsm, whole-trio session flush, BEFORE start_read) applied to the
//   gclk read ports directly and toggle-synced into clk for the write ports.
//   The consumer's live progress monitor (img_tx_monitor) is gclk native -
//   exported directly; its bit16 is the system read-completion. v2 removals:
//   pixel_target, cons_read_done pulse, tx MAC read_done (local-only now).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module tx_sequencer #(
    parameter int unsigned PROG_W = IMG_PROG_W,   // TB-only override
    parameter int unsigned ADDR_W = AHB_ADDR_W
)(
    input  logic mmcm_clk,
    input  logic gclk_en,
    input  logic clk,
    input  logic mmcm_reset_n,            // reset, deassert synced to mmcm_clk
    input  logic reset_n,                 // reset, deassert synced to clk

    // ---- From seq_top_fsm (gclk) ----
    input  logic start_read,              // 1-tick: begin burst-read session
    input  logic fifo_srst,               // 1-tick: flush the tx_afifo trio

    // ---- To seq_top_fsm / img_rgf (gclk) ----
    output logic cons_busy,               // consumer: message in flight
    output logic [IMG_PROG_W-1:0] tx_monitor, // consumer progress (gclk native;
                                          //   bit16 = read completion)

    // ---- Message handshake (gclk, via tx_msg_arbiter) ----
    input  logic tx_msg_done,
    output logic tx_msg_valid,
    output logic [CHAN_WIDTH-1:0] tx_line_r, tx_line_g, tx_line_b,

    // ---- To/from ahb_master_phy[R,G,B] (clk) ----
    output logic              xfer_req,
    output logic [ADDR_W-1:0] haddr,
    output logic              hwrite,
    input  logic [CHAN_WIDTH-1:0] hrdata_r, hrdata_g, hrdata_b,
    input  logic phy_done_r, phy_done_g, phy_done_b,
    input  logic hresp_err_r, hresp_err_g, hresp_err_b,

    // ---- Read-only FIFO mirrors for fifo_rgf ----
    output logic [2:0] fifo_full,         // {B,G,R}                (clk dom)
    output logic [2:0] fifo_almost_full,  //                        (clk dom)
    output logic [2:0] fifo_empty,        //                        (gclk dom)
    output logic [2:0] fifo_almost_empty, //                        (gclk dom)
    output logic [2:0] fifo_overflow,     // sticky                 (clk dom)
    output logic [2:0] fifo_underflow     // sticky                 (gclk dom)
);

    logic [2:0] push, pop;
    logic [CHAN_WIDTH-1:0] wdata [3];
    logic [CHAN_WIDTH-1:0] rdata [3];

    logic push_r, push_g, push_b;
    logic [CHAN_WIDTH-1:0] wdata_r, wdata_g, wdata_b;
    assign push     = {push_b, push_g, push_r};
    assign wdata[0] = wdata_r;
    assign wdata[1] = wdata_g;
    assign wdata[2] = wdata_b;

    logic pop_r, pop_g, pop_b;
    assign pop = {pop_b, pop_g, pop_r};

    // ================= CDC glue =================
    logic start_tgl_g, start_tgl_s, start_tgl_d, start_read_s;
    always_ff @(posedge mmcm_clk or negedge mmcm_reset_n) begin
        if (!mmcm_reset_n)       start_tgl_g <= 1'b0;
        else if (gclk_en) begin  // effective clock: outermost gate
            if (start_read)      start_tgl_g <= ~start_tgl_g;
        end
    end
    bus_sync_2ff #(.W(1)) u_start_sync (
        .clk(clk), .rst_n(reset_n), .en(1'b1),        // clk domain: full rate
        .d(start_tgl_g), .q(start_tgl_s)
    );
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) start_tgl_d <= 1'b0;
        else          start_tgl_d <= start_tgl_s;
    end
    assign start_read_s = start_tgl_s ^ start_tgl_d;

    logic srst_tgl_g, srst_tgl_s, srst_tgl_d, fifo_srst_s;
    always_ff @(posedge mmcm_clk or negedge mmcm_reset_n) begin
        if (!mmcm_reset_n)       srst_tgl_g <= 1'b0;
        else if (gclk_en) begin  // effective clock: outermost gate
            if (fifo_srst)       srst_tgl_g <= ~srst_tgl_g;
        end
    end
    bus_sync_2ff #(.W(1)) u_srst_sync (
        .clk(clk), .rst_n(reset_n), .en(1'b1),        // clk domain: full rate
        .d(srst_tgl_g), .q(srst_tgl_s)
    );
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) srst_tgl_d <= 1'b0;
        else          srst_tgl_d <= srst_tgl_s;
    end
    assign fifo_srst_s = srst_tgl_s ^ srst_tgl_d;

    // ================= AHB MAC (clk) - fills the FIFOs =================
    logic [2:0] full, almost_full, ovf;

    tx_ahb_master_mac #(.PROG_W(PROG_W), .ADDR_W(ADDR_W)) u_mac (
        .clk(clk), .reset_n(reset_n),
        .start_read_s(start_read_s),
        .xfer_req(xfer_req), .haddr(haddr), .hwrite(hwrite),
        .hrdata_r(hrdata_r), .hrdata_g(hrdata_g), .hrdata_b(hrdata_b),
        .phy_done_r(phy_done_r), .phy_done_g(phy_done_g),
        .phy_done_b(phy_done_b),
        .hresp_err_r(hresp_err_r), .hresp_err_g(hresp_err_g),
        .hresp_err_b(hresp_err_b),
        .full_r(full[0]), .full_g(full[1]), .full_b(full[2]),
        .almost_full_r(almost_full[0]), .almost_full_g(almost_full[1]),
        .almost_full_b(almost_full[2]),
        .push_r(push_r), .push_g(push_g), .push_b(push_b),
        .wdata_r(wdata_r), .wdata_g(wdata_g), .wdata_b(wdata_b),
        .ahb_busy()
    );

    // ================= tx_afifo[R,G,B] =================
    logic [2:0] empty, almost_empty, unf;

    generate
        for (genvar ch = 0; ch < 3; ch++) begin : g_tx_afifo
            async_fifo #(
                .WIDTH(AFIFO_WIDTH), .DEPTH(AFIFO_DEPTH),
                .AF_THRESH(AFIFO_AF_THRESH), .AE_THRESH(AFIFO_AE_THRESH)
            ) u_afifo (
                .wr_clk(clk), .wr_rst_n(reset_n),
                .wr_clk_en(1'b1),                     // 100 MHz side: full rate
                .wr_srst(fifo_srst_s),
                .wr_en(push[ch]), .wdata(wdata[ch]),
                .full(full[ch]), .almost_full(almost_full[ch]),
                .err_overflow(ovf[ch]),
                .rd_clk(mmcm_clk), .rd_rst_n(mmcm_reset_n),
                .rd_clk_en(gclk_en),                  // effective clock, gclk side
                .rd_srst(fifo_srst),                  // RAW strobes: the FIFO
                .rd_en(pop[ch]), .rdata(rdata[ch]),   //   samples under rd_clk_en
                .empty(empty[ch]), .almost_empty(almost_empty[ch]),
                .err_underflow(unf[ch])
            );
        end
    endgenerate

    // ================= Consumer (gclk) - drains into the TX handshake ======
    tx_consumer_fsm #(.PROG_W(PROG_W)) u_consumer (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk_en),
        .start_read(start_read),
        .rdata_r(rdata[0]), .rdata_g(rdata[1]), .rdata_b(rdata[2]),
        .empty_r(empty[0]), .empty_g(empty[1]), .empty_b(empty[2]),
        .almost_empty_r(almost_empty[0]), .almost_empty_g(almost_empty[1]),
        .almost_empty_b(almost_empty[2]),
        .pop_r(pop_r), .pop_g(pop_g), .pop_b(pop_b),
        .tx_msg_done(tx_msg_done), .tx_msg_valid(tx_msg_valid),
        .tx_line_r(tx_line_r), .tx_line_g(tx_line_g), .tx_line_b(tx_line_b),
        .monitor(tx_monitor), .busy(cons_busy)
    );

    assign fifo_full         = full;
    assign fifo_almost_full  = almost_full;
    assign fifo_empty        = empty;
    assign fifo_almost_empty = almost_empty;
    assign fifo_overflow     = ovf;
    assign fifo_underflow    = unf;

endmodule
