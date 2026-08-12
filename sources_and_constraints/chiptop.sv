`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: chiptop
// Description: Project top (chiptop_spec_v1) - a PURE STRUCTURAL netlist. Holds
//   no functional FSM of its own: it instantiates and wires the clock/reset
//   infrastructure, the RX chain, the BAR, sequencer_top, the TX chain, the APB
//   fabric with the three register files, and the RGB SRAM bank behind the
//   per-channel AHB slaves. Board pins per Nexys-A7-100T-Master.xdc (CLK100MHZ
//   E3 @10ns; UART_TXD_IN = PC->FPGA serial; UART_RXD_OUT = FPGA->PC serial;
//   UART_RTS / UART_CTS active-low at the pads - polarity inverted here).
//
//   Reset: one async source (CPU_RESETN & mmcm_locked), TWO protected
//   master_reset_sync instances - mmcm_reset_n (deassert on mmcm_clk) for the
//   PC-side world, reset_n (deassert on clk) for the SRAM/AHB world (on
//   mmcm_clk, qualified by CPU_RESETN & mmcm_locked) fans out to every
//   instance - project rule: no soft_reset anywhere.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module chiptop #(
    parameter int unsigned PROG_W = IMG_PROG_W    // TB-only override
)(
    input  logic CLK100MHZ,               // board oscillator (XDC pin E3)
    input  logic CPU_RESETN,              // board reset button (active low)
    input  logic UART_TXD_IN,             // PC -> FPGA serial (our rx_line)
    // Board-swap fix (2026-07-23, verified with ILA + modem-line probing):
    // the Nexys A7 nets are named from the FT2232's perspective -
    //   UART_RTS (E5) = FT2232 RTS# OUTPUT -> our INPUT  (tx flow ctl)
    //   UART_CTS (D3) = FT2232 CTS# INPUT  -> our OUTPUT (rx flow ctl)
    // Directions were swapped relative to the original assumption; both
    // remain active-low TTL at the pads.
    input  logic UART_RTS,                // PC RTS#, active low (our tx flow ctl)
    output logic UART_RXD_OUT,            // FPGA -> PC serial (our tx_line)
    output logic UART_CTS                 // to PC CTS#, active low (rx flow ctl)
);

    // =========================================================================
    // Clocks / reset
    // =========================================================================
    logic mmcm_clk, clk, gclk, mmcm_locked;
    logic clk_sel;
    logic mmcm_reset_n;                   // deassert synced to mmcm_clk
    logic reset_n;                        // deassert synced to clk (100 MHz)

    logic clk_sel_stat;
    assign clk_sel = 1'b1;                // fixed fast clock (v2: CLK_CTRL reg
                                          //   removed; status RO in uart_rgf)
    clock_gen u_clock_gen (
        .ext_clk(CLK100MHZ), .ext_rst_n(CPU_RESETN), .clk_sel(clk_sel),
        .mmcm_clk(mmcm_clk), .clk(clk), .gclk(gclk),
        .mmcm_locked(mmcm_locked), .clk_sel_stat(clk_sel_stat)
    );

    // Per-domain reset synchronizers: SAME async source (assert together,
    // whole-chip), but each domain gets a deassert synchronized to ITS clock -
    // no recovery/removal hazard in either domain.
    master_reset_sync #(.STAGES(2)) u_reset_sync_mmcm (
        .clk            (mmcm_clk),
        .async_reset_n  (CPU_RESETN & mmcm_locked),
        .master_reset_n (mmcm_reset_n)
    );

    master_reset_sync #(.STAGES(2)) u_reset_sync_clk (
        .clk            (clk),
        .async_reset_n  (CPU_RESETN & mmcm_locked),
        .master_reset_n (reset_n)
    );

    // =========================================================================
    // RX chain (mmcm_clk + gclk enable)
    // =========================================================================
    logic rts_int, cts_int;
    logic flush_rx, msg_taken;
    logic phy_error, parity_odd;
    logic [D_SIZE-1:0] msg_buf [MSG_LEN-1:0];
    logic [1:0] num_submsg;
    logic is_msg, mac_err;
    logic msg_legal;
    msg_type_t msg_type;
    logic new_msg, busy, burst_mode;
    msg_type_t msg_type_o;
    logic [LOCAL_ADDR_WIDTH-1:0] local_addr;
    logic [WDATA_WIDTH-1:0] wdata;
    logic [DIM_WIDTH-1:0] img_height, img_width;
    logic [CHAN_WIDTH-1:0] pixel_r, pixel_g, pixel_b;
    logic [BAR_ADDR_WIDTH-1:0] bar_addr;
    logic classifier_err;

    assign UART_CTS = ~rts_int;           // drive FT2232 CTS# (active low)
    assign cts_int  = ~UART_RTS;          // read FT2232 RTS# (active low)

    UART_Rx u_rx (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .rx_line(UART_TXD_IN), .msg_taken(msg_taken), .flush_rx(flush_rx),
        .parity_odd(parity_odd),
        .rts(rts_int), .msg_buf(msg_buf), .num_submsg(num_submsg),
        .is_msg(is_msg), .phy_error(phy_error), .mac_err(mac_err)
    );

    parser u_parser (
        .msg_buf(msg_buf), .num_submsg(num_submsg), .is_msg(is_msg),
        .burst_mode(burst_mode), .msg_legal(msg_legal), .msg_type(msg_type)
    );

    classifier u_classifier (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .msg_buf(msg_buf), .num_submsg(num_submsg), .is_msg(is_msg),
        .msg_legal(msg_legal), .msg_type(msg_type), .busy(busy),
        .msg_taken(msg_taken), .new_msg(new_msg), .msg_type_o(msg_type_o),
        .local_addr(local_addr), .wdata(wdata),
        .img_height(img_height), .img_width(img_width),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .bar_addr(bar_addr), .classifier_err(classifier_err)
    );

    // =========================================================================
    // BAR (comb chip-select decode)
    // =========================================================================
    logic sram_sel_w, rgf_hit, sram_hit;
    logic [RGF_SEL_W-1:0] rgf_sel;

    bar u_bar (
        .bar_addr(bar_addr), .sram_sel(sram_sel_w), .rgf_sel(rgf_sel),
        .rgf_hit(rgf_hit), .sram_hit(sram_hit)
    );

    // =========================================================================
    // sequencer_top (both domains)
    // =========================================================================
    logic [2*DIM_WIDTH-1:0] img_size;
    logic img_size_wr;
    logic req_write, req_read, start_write_g, start_read_g;
    logic ready_to_read, write_complete, read_complete;
    logic [IMG_PROG_W-1:0] rx_monitor, tx_monitor;
    logic [AHB_ADDR_W-1:0] s_haddr [3];
    logic [1:0]  s_htrans [3];
    logic        s_hwrite [3];
    logic [2:0]  s_hsize  [3];
    logic [2:0]  s_hburst [3];
    logic [31:0] s_hwdata [3];
    logic [31:0] s_hrdata [3];
    logic        s_hready [3];
    logic        s_hresp  [3];
    logic [MSG_WIDTH-1:0] tx_msg;
    logic tx_msg_valid, tx_msg_done;
    logic psel, penable, pwrite, pready;
    logic [RGF_ADDR_W-1:0] paddr;
    logic [31:0] pwdata, prdata;
    logic [RGF_SEL_W-1:0] slave_sel;
    logic rgf_r0c_read;
    logic [2:0] rxf_full, rxf_af, rxf_e, rxf_ae, rxf_ovf, rxf_unf;
    logic [2:0] txf_full, txf_af, txf_e, txf_ae, txf_ovf, txf_unf;

    sequencer_top #(.PROG_W(PROG_W)) u_sequencer_top (
        .mmcm_clk(mmcm_clk), .gclk_en(gclk), .clk(clk),
        .mmcm_reset_n(mmcm_reset_n), .reset_n(reset_n),
        .new_msg(new_msg), .msg_type(msg_type_o),
        .local_addr(local_addr), .bar_addr(bar_addr), .wdata(wdata),
        .img_height(img_height), .img_width(img_width),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .busy(busy),
        .burst_mode(burst_mode),
        .img_size(img_size), .img_size_wr(img_size_wr),
        .req_write(req_write), .req_read(req_read),
        .start_write_i(start_write_g), .start_read_i(start_read_g),
        .ready_to_read(ready_to_read),
        .write_complete(write_complete), .read_complete(read_complete),
        .rx_monitor(rx_monitor), .tx_monitor(tx_monitor),
        .sram_sel(sram_sel_w), .rgf_sel(rgf_sel),
        .haddr_r(s_haddr[0]), .haddr_g(s_haddr[1]), .haddr_b(s_haddr[2]),
        .htrans_r(s_htrans[0]), .htrans_g(s_htrans[1]), .htrans_b(s_htrans[2]),
        .hwrite_r(s_hwrite[0]), .hwrite_g(s_hwrite[1]), .hwrite_b(s_hwrite[2]),
        .hsize_r(s_hsize[0]), .hsize_g(s_hsize[1]), .hsize_b(s_hsize[2]),
        .hburst_r(s_hburst[0]), .hburst_g(s_hburst[1]), .hburst_b(s_hburst[2]),
        .hwdata_r(s_hwdata[0]), .hwdata_g(s_hwdata[1]), .hwdata_b(s_hwdata[2]),
        .hrdata_r(s_hrdata[0]), .hrdata_g(s_hrdata[1]), .hrdata_b(s_hrdata[2]),
        .hready_r(s_hready[0]), .hready_g(s_hready[1]), .hready_b(s_hready[2]),
        .hresp_r(s_hresp[0]), .hresp_g(s_hresp[1]), .hresp_b(s_hresp[2]),
        .tx_msg(tx_msg), .tx_msg_valid(tx_msg_valid), .tx_msg_done(tx_msg_done),
        .psel(psel), .penable(penable), .paddr(paddr), .pwrite(pwrite),
        .pwdata(pwdata), .slave_sel(slave_sel), .prdata(prdata),
        .pready(pready), .rgf_r0c_read(rgf_r0c_read),
        .rxf_full(rxf_full), .rxf_almost_full(rxf_af),
        .rxf_empty(rxf_e), .rxf_almost_empty(rxf_ae),
        .rxf_overflow(rxf_ovf), .rxf_underflow(rxf_unf),
        .txf_full(txf_full), .txf_almost_full(txf_af),
        .txf_empty(txf_e), .txf_almost_empty(txf_ae),
        .txf_overflow(txf_ovf), .txf_underflow(txf_unf)
    );

    // =========================================================================
    // RGB SRAM bank: 3x (ahb_slave + sram), clk domain
    // =========================================================================
    logic m_wen [3];
    logic [13:0] m_waddr [3];
    logic [13:0] m_raddr [3];
    logic [31:0] m_wdata [3];
    logic [31:0] m_rdata [3];

    generate
        for (genvar ch = 0; ch < 3; ch++) begin : g_bank
            ahb_slave u_slave (
                .clk(clk), .reset_n(reset_n),
                .HADDR(s_haddr[ch]), .HTRANS(s_htrans[ch]),
                .HWRITE(s_hwrite[ch]), .HSIZE(s_hsize[ch]),
                .HBURST(s_hburst[ch]), .HWDATA(s_hwdata[ch]),
                .HRDATA(s_hrdata[ch]), .HREADY(s_hready[ch]),
                .HRESP(s_hresp[ch]),
                .sram_sel(1'b1),          // bank CS: future-use, tied (d3)
                .wen(m_wen[ch]), .waddr(m_waddr[ch]), .wdata(m_wdata[ch]),
                .raddr(m_raddr[ch]), .rdata(m_rdata[ch])
            );
            sram u_sram (
                .clk(clk),
                .wen(m_wen[ch]), .waddr(m_waddr[ch]), .wdata(m_wdata[ch]),
                .raddr(m_raddr[ch]), .rdata(m_rdata[ch])
            );
        end
    endgenerate

    // =========================================================================
    // APB fabric + register files (mmcm_clk + gclk)
    // =========================================================================
    logic [N_RGF-1:0] psel_s;
    logic penable_s, pwrite_s;
    logic [RGF_ADDR_W-1:0] paddr_s;
    logic [31:0] pwdata_s;
    logic [N_RGF-1:0][31:0] prdata_s;
    logic [N_RGF-1:0] pready_s;
    logic env_wen [3];
    logic [RGF_ADDR_W-1:0] env_addr [3];
    logic [31:0] env_wdata [3];
    logic [31:0] env_rdata [3];
    logic resend_pending, resend_sent, tx_busy_mac;

    // ---- RX error/resend controller (v2: functional owner, outside the RGF) ----
    rx_resend_ctrl u_resend_ctrl (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .phy_err_evt(phy_error), .mac_err_evt(mac_err),
        .clsf_err_evt(classifier_err),
        .resend_pending(resend_pending), .resend_sent(resend_sent),
        .flush_rx(flush_rx)
    );

    apb_interconnect u_apb_interconnect (
        .psel(psel), .penable(penable), .pwrite(pwrite), .paddr(paddr),
        .pwdata(pwdata), .slave_sel(slave_sel), .prdata(prdata),
        .pready(pready),
        .psel_o(psel_s), .penable_o(penable_s), .pwrite_o(pwrite_s),
        .paddr_o(paddr_s), .pwdata_o(pwdata_s),
        .prdata_i(prdata_s), .pready_i(pready_s)
    );

    generate
        for (genvar i = 0; i < 3; i++) begin : g_env
            apb_slave_envelope u_env (
                .psel(psel_s[i]), .penable(penable_s), .pwrite(pwrite_s),
                .paddr(paddr_s), .pwdata(pwdata_s),
                .prdata(prdata_s[i]), .pready(pready_s[i]),
                .wen(env_wen[i]), .addr(env_addr[i]), .wdata(env_wdata[i]),
                .rdata(env_rdata[i])
            );
        end
    endgenerate

    img_rgf u_img_rgf (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .wen(env_wen[0]), .addr(env_addr[0]), .wdata(env_wdata[0]),
        .rdata(env_rdata[0]),
        .img_size(img_size), .img_size_wr(img_size_wr),
        .req_write(req_write), .req_read(req_read),
        .rx_monitor(rx_monitor), .tx_monitor(tx_monitor),
        .start_write(start_write_g), .start_read(start_read_g),
        .ready_to_read(ready_to_read),
        .write_complete(write_complete), .read_complete(read_complete)
    );

    uart_rgf u_uart_rgf (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .wen(env_wen[1]), .rgf_r0c_read(rgf_r0c_read),
        .addr(env_addr[1]), .wdata(env_wdata[1]),
        .rdata(env_rdata[1]),
        .phy_err_evt(phy_error), .mac_err_evt(mac_err),
        .clsf_err_evt(classifier_err),
        .clk_sel_stat(clk_sel_stat),
        .parity_odd(parity_odd)
    );

    fifo_rgf u_fifo_rgf (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .wen(env_wen[2]), .addr(env_addr[2]), .wdata(env_wdata[2]),
        .rdata(env_rdata[2]),
        .rx_full(rxf_full), .rx_almost_full(rxf_af),
        .rx_empty(rxf_e), .rx_almost_empty(rxf_ae),
        .rx_overflow(rxf_ovf), .rx_underflow(rxf_unf),
        .tx_full(txf_full), .tx_almost_full(txf_af),
        .tx_empty(txf_e), .tx_almost_empty(txf_ae),
        .tx_overflow(txf_ovf), .tx_underflow(txf_unf)
    );

    // =========================================================================
    // TX chain (mmcm_clk + gclk enable)
    // =========================================================================
    logic [D_SIZE-1:0] tx_byte;
    logic byte_send, byte_done, tx_busy_phy;

    tx_mac u_tx_mac (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .tx_msg(tx_msg), .tx_msg_valid(tx_msg_valid), .tx_msg_done(tx_msg_done),
        .resend_pending(resend_pending), .resend_sent(resend_sent),
        .tx_byte(tx_byte), .byte_send(byte_send), .byte_done(byte_done),
        .tx_busy(tx_busy_mac)
    );

    tx_phy u_tx_phy (
        .clk(mmcm_clk), .reset_n(mmcm_reset_n), .enable(gclk),
        .tx_byte(tx_byte), .byte_send(byte_send), .byte_done(byte_done),
        .tx_busy(tx_busy_phy),
        .parity_odd(parity_odd),
        .cts(cts_int), .tx_line(UART_RXD_OUT)
    );

endmodule
