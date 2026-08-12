`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_sequencer
// Description: Envelope of the RX burst-write group (v2):
//     rx_producer_fsm (gclk) + rx_afifo[R,G,B] (async) + rx_ahb_master_mac (clk)
//   Structural wrapper; its only logic is CDC glue:
//     * start_write (gclk 1-tick) -> toggle -> 2-FF -> 1-clk pulse into clk.
//     * fifo_srst   (gclk 1-tick, from seq_top_fsm - whole-trio session flush):
//       applied to the gclk write ports directly (gated w/ gclk_en) and
//       toggle-synced into clk for the read ports. seq_top_fsm issues it
//       BEFORE start_write, so both domains flush before any traffic.
//     * rx MAC's live progress monitor (clk domain) -> 2-FF bus into gclk as
//       rx_monitor_s: bit16 is the functional write-complete level (glitch-free
//       single-bit through the sync); [15:0] row/col are informational mirrors
//       (transient incoherence during bursts is acceptable for RO status).
//   v2 removals: pixel_target (bit16 rollover replaces it), push_done pulse
//   (prod_complete level instead), store_done (folded into the monitor).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_sequencer #(
    parameter int unsigned PROG_W = IMG_PROG_W,   // TB-only override
    parameter int unsigned ADDR_W = AHB_ADDR_W
)(
    input  logic mmcm_clk,
    input  logic gclk_en,
    input  logic clk,
    input  logic mmcm_reset_n,            // reset, deassert synced to mmcm_clk
    input  logic reset_n,                 // reset, deassert synced to clk

    // ---- From classifier (gclk) ----
    input  logic       new_msg,
    input  msg_type_t  msg_type,
    input  logic [CHAN_WIDTH-1:0] pixel_r, pixel_g, pixel_b,

    // ---- From seq_top_fsm (gclk) ----
    input  logic start_write,             // 1-tick: begin image session
    input  logic fifo_srst,               // 1-tick: flush the rx_afifo trio

    // ---- To seq_top_fsm / img_rgf (gclk) ----
    output logic prod_busy,               // producer: push in flight
    output logic prod_complete,           // producer bit16: image fully pushed
    output logic [IMG_PROG_W-1:0] rx_monitor_s, // MAC progress, synced to gclk
                                          //   (bit16 = write completion)

    // ---- To/from ahb_master_phy[R,G,B] (clk) ----
    output logic              xfer_req,
    output logic [ADDR_W-1:0] haddr,
    output logic              hwrite,
    output logic [CHAN_WIDTH-1:0] hwdata_r, hwdata_g, hwdata_b,
    input  logic phy_done_r, phy_done_g, phy_done_b,
    input  logic hresp_err_r, hresp_err_g, hresp_err_b,

    // ---- Read-only FIFO mirrors for fifo_rgf ----
    output logic [2:0] fifo_full,         // {B,G,R}                (gclk dom)
    output logic [2:0] fifo_almost_full,  //                        (gclk dom)
    output logic [2:0] fifo_empty,        //                        (clk dom)
    output logic [2:0] fifo_almost_empty, //                        (clk dom)
    output logic [2:0] fifo_overflow,     // sticky                 (gclk dom)
    output logic [2:0] fifo_underflow     // sticky                 (clk dom)
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
    // start_write: pulse -> toggle -> 2-FF -> pulse (gclk -> clk)
    logic start_tgl_g, start_tgl_s, start_tgl_d, start_write_s;
    always_ff @(posedge mmcm_clk or negedge mmcm_reset_n) begin
        if (!mmcm_reset_n)       start_tgl_g <= 1'b0;
        else if (gclk_en) begin  // effective clock: outermost gate
            if (start_write)     start_tgl_g <= ~start_tgl_g;
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
    assign start_write_s = start_tgl_s ^ start_tgl_d;

    // fifo_srst: same pulse-toggle CDC for the clk-side (read) ports
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

    // MAC progress monitor: clk -> gclk (bit16 functional, rest informational)
    logic [IMG_PROG_W-1:0] monitor_clk;
    bus_sync_2ff #(.W(IMG_PROG_W)) u_mon_sync (
        .clk(mmcm_clk), .rst_n(mmcm_reset_n), .en(gclk_en), // gclk-paced side
        .d(monitor_clk), .q(rx_monitor_s)
    );

    // ================= Producer (gclk) =================
    logic [2:0] full, almost_full;

    rx_producer_fsm #(.PROG_W(PROG_W)) u_producer (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk_en),
        .new_msg(new_msg), .msg_type(msg_type),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .full_r(full[0]), .full_g(full[1]), .full_b(full[2]),
        .almost_full_r(almost_full[0]), .almost_full_g(almost_full[1]),
        .almost_full_b(almost_full[2]),
        .start_write(start_write),
        .busy(prod_busy), .complete(prod_complete),
        .push_r(push_r), .push_g(push_g), .push_b(push_b),
        .wdata_r(wdata_r), .wdata_g(wdata_g), .wdata_b(wdata_b)
    );

    // ================= rx_afifo[R,G,B] =================
    logic [2:0] empty, almost_empty, ovf, unf;

    generate
        for (genvar ch = 0; ch < 3; ch++) begin : g_rx_afifo
            async_fifo #(
                .WIDTH(AFIFO_WIDTH), .DEPTH(AFIFO_DEPTH),
                .AF_THRESH(AFIFO_AF_THRESH), .AE_THRESH(AFIFO_AE_THRESH)
            ) u_afifo (
                .wr_clk(mmcm_clk), .wr_rst_n(mmcm_reset_n),
                .wr_clk_en(gclk_en),                  // effective clock, gclk side
                .wr_srst(fifo_srst),                  // RAW strobes: the FIFO
                .wr_en(push[ch]), .wdata(wdata[ch]),  //   samples under wr_clk_en
                .full(full[ch]), .almost_full(almost_full[ch]),
                .err_overflow(ovf[ch]),
                .rd_clk(clk), .rd_rst_n(reset_n),
                .rd_clk_en(1'b1),                     // 100 MHz side: full rate
                .rd_srst(fifo_srst_s),
                .rd_en(pop[ch]), .rdata(rdata[ch]),
                .empty(empty[ch]), .almost_empty(almost_empty[ch]),
                .err_underflow(unf[ch])
            );
        end
    endgenerate

    // ================= AHB MAC (clk) =================
    rx_ahb_master_mac #(.PROG_W(PROG_W), .ADDR_W(ADDR_W)) u_mac (
        .clk(clk), .reset_n(reset_n),
        .start_write_s(start_write_s),
        .rdata_r(rdata[0]), .rdata_g(rdata[1]), .rdata_b(rdata[2]),
        .empty_r(empty[0]), .empty_g(empty[1]), .empty_b(empty[2]),
        .almost_empty_r(almost_empty[0]), .almost_empty_g(almost_empty[1]),
        .almost_empty_b(almost_empty[2]),
        .pop_r(pop_r), .pop_g(pop_g), .pop_b(pop_b),
        .xfer_req(xfer_req), .haddr(haddr), .hwrite(hwrite),
        .hwdata_r(hwdata_r), .hwdata_g(hwdata_g), .hwdata_b(hwdata_b),
        .phy_done_r(phy_done_r), .phy_done_g(phy_done_g),
        .phy_done_b(phy_done_b),
        .hresp_err_r(hresp_err_r), .hresp_err_g(hresp_err_g),
        .hresp_err_b(hresp_err_b),
        .monitor(monitor_clk),
        .ahb_busy()
    );

    assign fifo_full         = full;
    assign fifo_almost_full  = almost_full;
    assign fifo_empty        = empty;
    assign fifo_almost_empty = almost_empty;
    assign fifo_overflow     = ovf;
    assign fifo_underflow    = unf;

endmodule
