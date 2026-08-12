`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: sequencer_top
// Description: Structural container of the RX/TX image-pipeline stage
//   (sequencer_top_spec_v1). Instantiates and wires: seq_top_fsm (orchestrator),
//   rx_sequencer (producer + rx_afifo x3 + rx AHB MAC), tx_sequencer (tx AHB
//   MAC + tx_afifo x3 + consumer), apb_master (mac+phy), ahb_master_phy x3
//   (shared, one per colour), composer and tx_msg_arbiter. Owns the shared-
//   resource muxes (spec sec 5): the ahb_dir control-bundle mux driving the
//   three phys, the phy_done fan-in steering to the selected MAC, and the TX
//   grant (via the arbiter). Per-channel HWDATA/HRDATA stay point-to-point.
//
//   ahb_dir is generated on the gclk side and used on the clk side; it is
//   QUASI-STATIC (changes only in S_IDLE<->S_BURST_RD transitions while both
//   MACs are idle), so a 2-FF sync makes the crossing safe.
//
//   External face (spec sec 3): classifier fields in / busy out; burst_mode to
//   the parser; the CS-independent img_rgf sideband; BAR chip-selects in;
//   per-channel AHB SINGLE master buses to the chiptop ahb_slaves; the complete
//   tx_msg[127:0] + valid/done to/from the chiptop tx_mac; one APB bus to the
//   chiptop apb_interconnect; read-only FIFO mirrors for fifo_rgf.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module sequencer_top #(
    parameter int unsigned PROG_W = IMG_PROG_W,   // TB-only override
    parameter int unsigned ADDR_W = AHB_ADDR_W
)(
    // ---- clocks / reset ----
    input  logic mmcm_clk,                // PC-side physical clock
    input  logic gclk_en,                 // gclk = baud*16 clock-enable
    input  logic clk,                     // 100 MHz SRAM/AHB clock
    input  logic mmcm_reset_n,            // reset, deassert synced to mmcm_clk
    input  logic reset_n,                 // reset, deassert synced to clk

    // ---- <-> rx_classifier (gclk) ----
    input  logic       new_msg,
    input  msg_type_t  msg_type,
    input  logic [LOCAL_ADDR_WIDTH-1:0] local_addr,
    input  logic [BAR_ADDR_WIDTH-1:0]   bar_addr,
    input  logic [WDATA_WIDTH-1:0]      wdata,
    input  logic [DIM_WIDTH-1:0]        img_height,
    input  logic [DIM_WIDTH-1:0]        img_width,
    input  logic [CHAN_WIDTH-1:0]       pixel_r, pixel_g, pixel_b,
    output logic       busy,

    // ---- -> rx_parser ----
    output logic burst_mode,

    // ---- <-> img_rgf (sideband + v2 control handshake + live monitors) ----
    output logic [2*DIM_WIDTH-1:0] img_size,
    output logic                   img_size_wr,
    output logic                   req_write,
    output logic                   req_read,
    input  logic                   start_write_i,   // grants from img_rgf
    input  logic                   start_read_i,
    input  logic                   ready_to_read,
    input  logic                   write_complete,  // monitor bit16 levels
    input  logic                   read_complete,
    output logic [IMG_PROG_W-1:0]  rx_monitor,      // -> img_rx_monitor (synced)
    output logic [IMG_PROG_W-1:0]  tx_monitor,      // -> img_tx_monitor

    // ---- <- chiptop BAR ----
    input  logic                 sram_sel,
    input  logic [RGF_SEL_W-1:0] rgf_sel,

    // ---- <-> SRAM AHB slaves (clk; per channel, SINGLE) ----
    output logic [ADDR_W-1:0] haddr_r, haddr_g, haddr_b,
    output logic [1:0]        htrans_r, htrans_g, htrans_b,
    output logic              hwrite_r, hwrite_g, hwrite_b,
    output logic [2:0]        hsize_r,  hsize_g,  hsize_b,
    output logic [2:0]        hburst_r, hburst_g, hburst_b,
    output logic [31:0]       hwdata_r, hwdata_g, hwdata_b,
    input  logic [31:0]       hrdata_r, hrdata_g, hrdata_b,
    input  logic              hready_r, hready_g, hready_b,
    input  logic              hresp_r,  hresp_g,  hresp_b,

    // ---- <-> chiptop tx_mac (gclk) ----
    output logic [MSG_WIDTH-1:0] tx_msg,
    output logic                 tx_msg_valid,
    input  logic                 tx_msg_done,

    // ---- <-> chiptop apb_interconnect (gclk) ----
    output logic                  psel,
    output logic                  penable,
    output logic [RGF_ADDR_W-1:0] paddr,
    output logic                  pwrite,
    output logic [31:0]           pwdata,
    output logic [RGF_SEL_W-1:0]  slave_sel,
    input  logic [31:0]           prdata,
    input  logic                  pready,
    output logic                  rgf_r0c_read, // R0C qualifier (apb_master_phy)

    // ---- read-only FIFO mirrors for fifo_rgf ----
    output logic [2:0] rxf_full, rxf_almost_full, rxf_empty, rxf_almost_empty,
    output logic [2:0] rxf_overflow, rxf_underflow,
    output logic [2:0] txf_full, txf_almost_full, txf_empty, txf_almost_empty,
    output logic [2:0] txf_overflow, txf_underflow
);

    // =========================================================================
    // Orchestrator
    // =========================================================================
    logic prod_busy, prod_complete;
    logic cons_busy;
    logic apb_done, ahb_dir, tx_src;
    logic start_write, start_read, rx_fifo_srst, tx_fifo_srst;
    logic req, is_write;

    seq_top_fsm u_fsm (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk_en),
        .new_msg(new_msg), .msg_type(msg_type),
        .img_height(img_height), .img_width(img_width),
        .sram_sel(sram_sel), .rgf_sel(rgf_sel),
        .prod_busy(prod_busy), .prod_complete(prod_complete),
        .cons_busy(cons_busy),
        .apb_done(apb_done), .tx_msg_done(tx_msg_done),
        .req_write(req_write), .req_read(req_read),
        .start_write_i(start_write_i), .start_read_i(start_read_i),
        .ready_to_read(ready_to_read),
        .write_complete(write_complete), .read_complete(read_complete),
        .img_size(img_size), .img_size_wr(img_size_wr),
        .busy(busy), .burst_mode(burst_mode),
        .ahb_dir(ahb_dir), .tx_src(tx_src),
        .start_write(start_write), .start_read(start_read),
        .rx_fifo_srst(rx_fifo_srst), .tx_fifo_srst(tx_fifo_srst),
        .req(req), .is_write(is_write)
    );

    // ---- ahb_dir into the clk domain (quasi-static level) ----
    logic ahb_dir_s;
    bus_sync_2ff #(.W(1)) u_dir_sync (
        .clk(clk), .rst_n(reset_n), .en(1'b1),        // clk domain: full rate
        .d(ahb_dir), .q(ahb_dir_s)
    );

    // =========================================================================
    // RX burst-write group
    // =========================================================================
    logic rx_xfer_req, rx_hwrite;
    logic [ADDR_W-1:0] rx_haddr;
    logic [CHAN_WIDTH-1:0] rx_hwdata [3];
    logic phy_done [3];
    logic hresp_err [3];

    rx_sequencer #(.PROG_W(PROG_W), .ADDR_W(ADDR_W)) u_rx_seq (
        .mmcm_clk(mmcm_clk), .gclk_en(gclk_en), .clk(clk),
        .mmcm_reset_n(mmcm_reset_n), .reset_n(reset_n),
        .new_msg(new_msg), .msg_type(msg_type),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .start_write(start_write), .fifo_srst(rx_fifo_srst),
        .prod_busy(prod_busy), .prod_complete(prod_complete),
        .rx_monitor_s(rx_monitor),
        .xfer_req(rx_xfer_req), .haddr(rx_haddr), .hwrite(rx_hwrite),
        .hwdata_r(rx_hwdata[0]), .hwdata_g(rx_hwdata[1]),
        .hwdata_b(rx_hwdata[2]),
        .phy_done_r(phy_done[0] & ~ahb_dir_s),
        .phy_done_g(phy_done[1] & ~ahb_dir_s),
        .phy_done_b(phy_done[2] & ~ahb_dir_s),
        .hresp_err_r(hresp_err[0]), .hresp_err_g(hresp_err[1]),
        .hresp_err_b(hresp_err[2]),
        .fifo_full(rxf_full), .fifo_almost_full(rxf_almost_full),
        .fifo_empty(rxf_empty), .fifo_almost_empty(rxf_almost_empty),
        .fifo_overflow(rxf_overflow), .fifo_underflow(rxf_underflow)
    );

    // =========================================================================
    // TX burst-read group
    // =========================================================================
    logic tx_xfer_req, tx_hwrite;
    logic [ADDR_W-1:0] tx_haddr;
    logic [CHAN_WIDTH-1:0] phy_hrdata [3];
    logic cons_tx_valid, cons_tx_done;
    logic [CHAN_WIDTH-1:0] tx_line_r, tx_line_g, tx_line_b;

    tx_sequencer #(.PROG_W(PROG_W), .ADDR_W(ADDR_W)) u_tx_seq (
        .mmcm_clk(mmcm_clk), .gclk_en(gclk_en), .clk(clk),
        .mmcm_reset_n(mmcm_reset_n), .reset_n(reset_n),
        .start_read(start_read), .fifo_srst(tx_fifo_srst),
        .cons_busy(cons_busy), .tx_monitor(tx_monitor),
        .tx_msg_done(cons_tx_done), .tx_msg_valid(cons_tx_valid),
        .tx_line_r(tx_line_r), .tx_line_g(tx_line_g), .tx_line_b(tx_line_b),
        .xfer_req(tx_xfer_req), .haddr(tx_haddr), .hwrite(tx_hwrite),
        .hrdata_r(phy_hrdata[0]), .hrdata_g(phy_hrdata[1]),
        .hrdata_b(phy_hrdata[2]),
        .phy_done_r(phy_done[0] & ahb_dir_s),
        .phy_done_g(phy_done[1] & ahb_dir_s),
        .phy_done_b(phy_done[2] & ahb_dir_s),
        .hresp_err_r(hresp_err[0]), .hresp_err_g(hresp_err[1]),
        .hresp_err_b(hresp_err[2]),
        .fifo_full(txf_full), .fifo_almost_full(txf_almost_full),
        .fifo_empty(txf_empty), .fifo_almost_empty(txf_almost_empty),
        .fifo_overflow(txf_overflow), .fifo_underflow(txf_underflow)
    );

    // =========================================================================
    // Shared AHB phys x3 - control bundle muxed by ahb_dir (spec 5.1/6.3);
    // per-channel data point-to-point (write data from the RX group, read data
    // to the TX group)
    // =========================================================================
    logic mux_xfer_req, mux_hwrite;
    logic [ADDR_W-1:0] mux_haddr;
    assign mux_xfer_req = ahb_dir_s ? tx_xfer_req : rx_xfer_req;
    assign mux_haddr    = ahb_dir_s ? tx_haddr    : rx_haddr;
    assign mux_hwrite   = ahb_dir_s ? tx_hwrite   : rx_hwrite;

    logic [ADDR_W-1:0] b_haddr  [3];
    logic [1:0]        b_htrans [3];
    logic              b_hwrite [3];
    logic [2:0]        b_hsize  [3];
    logic [2:0]        b_hburst [3];
    logic [31:0]       b_hwdata [3];
    logic [31:0]       b_hrdata [3];
    logic              b_hready [3];
    logic              b_hresp  [3];

    assign b_hrdata[0] = hrdata_r;  assign b_hrdata[1] = hrdata_g;
    assign b_hrdata[2] = hrdata_b;
    assign b_hready[0] = hready_r;  assign b_hready[1] = hready_g;
    assign b_hready[2] = hready_b;
    assign b_hresp[0]  = hresp_r;   assign b_hresp[1]  = hresp_g;
    assign b_hresp[2]  = hresp_b;

    generate
        for (genvar ch = 0; ch < 3; ch++) begin : g_phy
            ahb_master_phy #(.ADDR_W(ADDR_W)) u_phy (
                .clk(clk), .reset_n(reset_n),
                .xfer_req(mux_xfer_req), .haddr_i(mux_haddr),
                .hwrite_i(mux_hwrite), .hwdata_i(rx_hwdata[ch]),
                .HADDR(b_haddr[ch]), .HTRANS(b_htrans[ch]),
                .HWRITE(b_hwrite[ch]), .HSIZE(b_hsize[ch]),
                .HBURST(b_hburst[ch]), .HWDATA(b_hwdata[ch]),
                .HRDATA(b_hrdata[ch]), .HREADY(b_hready[ch]),
                .HRESP(b_hresp[ch]),
                .hrdata_o(phy_hrdata[ch]), .phy_done(phy_done[ch]),
                .hresp_err(hresp_err[ch])
            );
        end
    endgenerate

    assign {haddr_r, haddr_g, haddr_b}    = {b_haddr[0], b_haddr[1], b_haddr[2]};
    assign {htrans_r, htrans_g, htrans_b} = {b_htrans[0], b_htrans[1], b_htrans[2]};
    assign {hwrite_r, hwrite_g, hwrite_b} = {b_hwrite[0], b_hwrite[1], b_hwrite[2]};
    assign {hsize_r, hsize_g, hsize_b}    = {b_hsize[0], b_hsize[1], b_hsize[2]};
    assign {hburst_r, hburst_g, hburst_b} = {b_hburst[0], b_hburst[1], b_hburst[2]};
    assign {hwdata_r, hwdata_g, hwdata_b} = {b_hwdata[0], b_hwdata[1], b_hwdata[2]};

    // =========================================================================
    // APB master (register path)
    // =========================================================================
    logic rd_resp_valid, resp_captured;
    logic [31:0] phy_prdata;

    apb_master u_apb (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk_en),
        .req(req), .is_write(is_write), .local_addr(local_addr),
        .bar_addr(bar_addr), .rgf_sel(rgf_sel), .wdata(wdata),
        .done(apb_done),
        .rd_resp_valid(rd_resp_valid), .resp_captured(resp_captured),
        .phy_prdata(phy_prdata),
        .psel(psel), .penable(penable), .paddr(paddr), .pwrite(pwrite),
        .pwdata(pwdata), .slave_sel(slave_sel), .prdata(prdata),
        .pready(pready), .rgf_r0c_read(rgf_r0c_read)
    );

    // =========================================================================
    // Shared TX resources: arbiter (grant/steer) + composer (datapath)
    // =========================================================================
    logic fmt_sel;

    tx_msg_arbiter u_arb (
        .tx_src(tx_src),
        .rd_resp_valid(rd_resp_valid), .resp_captured(resp_captured),
        .cons_tx_valid(cons_tx_valid), .cons_tx_done(cons_tx_done),
        .fmt_sel(fmt_sel), .tx_msg_valid(tx_msg_valid),
        .tx_msg_done(tx_msg_done)
    );

    composer u_comp (
        .fmt_sel(fmt_sel),
        .bar_addr(bar_addr), .local_addr(local_addr), .rdata(phy_prdata),
        .tx_line_r(tx_line_r), .tx_line_g(tx_line_g), .tx_line_b(tx_line_b),
        .tx_msg(tx_msg)
    );

endmodule
