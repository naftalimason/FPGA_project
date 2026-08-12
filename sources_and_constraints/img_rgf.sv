`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: img_rgf
// Description: Image control/status register file (v2, per IMG RGF.xlsx, written
//   in the rgf.sv template style: packed structs, ADDR/RST/field localparams,
//   unique-case address decode). Beyond the PC-visible registers it is the
//   image-command CONTROL POINT (v2 change 1):
//     seq_top_fsm -> req_write / req_read;  img_rgf -> start_write / start_read
//     pulses back. start_write is always granted and clears ready_to_read until
//     the write completes; start_read is granted only while ready_to_read==1.
//   Completion levels come back as the monitor bit16 mirrors (write_complete =
//   img_rx_monitor[16], read_complete = img_tx_monitor[16]) and are forwarded
//   to seq_top_fsm. Monitors are LIVE mirrors of the operational counters.
//   Clocking: mmcm_clk + gclk enable. Mem intf via apb_slave_envelope
//   (addr valid only during ACCESS, so R0C-style reads clear after capture).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module img_rgf #(
    parameter int unsigned ADDR_WIDTH = RGF_ADDR_W,   // full 16-bit byte address
    parameter int unsigned DATA_WIDTH = 32
)(
    input  logic clk,
    input  logic reset_n,
    input  logic enable,                  // gclk clock-enable

    // Mem intf
    input  logic                  wen,
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic [DATA_WIDTH-1:0] wdata,
    output logic [DATA_WIDTH-1:0] rdata,

    // ---- seq_top_fsm sideband (CS-independent) ----
    input  logic [2*DIM_WIDTH-1:0] img_size,     // {height[23:0], width[23:0]}
    input  logic                   img_size_wr,  // 1-tick: latch H/W
    input  logic                   req_write,    // 1-tick: image write cmd seen
    input  logic                   req_read,     // 1-tick: image read cmd seen

    // ---- live progress monitors (Input RO registers) ----
    input  logic [IMG_PROG_W-1:0]  rx_monitor,   // {complete, row, col} (synced)
    input  logic [IMG_PROG_W-1:0]  tx_monitor,   // {complete, row, col}

    // ---- control outputs to seq_top_fsm ----
    output logic start_write,             // 1-tick grant pulse
    output logic start_read,              // 1-tick grant pulse (gated by ready)
    output logic ready_to_read,           // level: valid image stored in SRAM
    output logic write_complete,          // = rx_monitor[16] (level)
    output logic read_complete            // = tx_monitor[16] (level)
);

localparam IMG_STATUS_ADDR  = 16'h0000;
localparam IMG_RX_MON_ADDR  = 16'h0004;
localparam IMG_TX_MON_ADDR  = 16'h0008;
localparam IMG_CTRL_ADDR    = 16'h000C;
localparam IMG_CHECKER_ADDR = 16'h0010;

localparam IMG_CHECKER_RST = 32'h0000_0000;

typedef struct packed {
  logic       ready_to_read;             // [18]
  logic [8:0] height;                    // [17:9]  (9 bits: holds 256 directly)
  logic [8:0] width;                     // [8:0]
} img_status_t;

typedef struct packed {
  logic       complete;                  // [16]
  logic [7:0] row;                       // [15:8]
  logic [7:0] col;                       // [7:0]
} img_monitor_t;

typedef struct packed {
  logic req_read;                        // [3]
  logic req_write;                       // [2]
  logic start_write;                     // [1]
  logic start_read;                      // [0]
} img_ctrl_t;

typedef struct packed {
  logic [7:0] field_2;                   // [23:16]
  logic [7:0] field_1;                   // [15:8]
  logic [7:0] field_0;                   // [7:0]
} img_checker_t;

img_status_t  img_status;
img_monitor_t img_rx_monitor;
img_monitor_t img_tx_monitor;
img_ctrl_t    img_ctrl;
img_checker_t img_checker;

logic img_status_sel, img_rx_mon_sel, img_tx_mon_sel, img_ctrl_sel;
logic img_checker_sel, addr_err_comb;
logic [8:0] height_q, width_q;
logic ready_q, wc_d;

// Address Decoder
always_comb begin : reg_addr_dec
  img_status_sel  = 1'b0;
  img_rx_mon_sel  = 1'b0;
  img_tx_mon_sel  = 1'b0;
  img_ctrl_sel    = 1'b0;
  img_checker_sel = 1'b0;
  addr_err_comb   = 1'b0;
  unique case (addr)
    IMG_STATUS_ADDR  : img_status_sel  = 1'b1;
    IMG_RX_MON_ADDR  : img_rx_mon_sel  = 1'b1;
    IMG_TX_MON_ADDR  : img_tx_mon_sel  = 1'b1;
    IMG_CTRL_ADDR    : img_ctrl_sel    = 1'b1;
    IMG_CHECKER_ADDR : img_checker_sel = 1'b1;
    default          : addr_err_comb   = 1'b1;
  endcase
end

// ---- H/W latch (sideband, CS-independent) ----
always_ff @(posedge clk, negedge reset_n) begin
  if (!reset_n) begin
    height_q <= '0;
    width_q  <= '0;
  end
  else if (enable) begin                    // effective clock: outermost gate
    if (img_size_wr) begin
      height_q <= img_size[DIM_WIDTH +: 9]; // low 9 bits of {H2,H1,H0}
      width_q  <= img_size[0 +: 9];         // low 9 bits of {W2,W1,W0}
    end
  end
end

// ---- ready_to_read + control handshake (v2 change 1) ----
//   start_write: always granted; drops ready until the write completes.
//   start_read : granted only while ready_to_read==1 (else no pulse - the
//                refusal is silent; seq_top_fsm handles the busy release).
always_ff @(posedge clk, negedge reset_n) begin
  if (!reset_n) begin
    ready_q     <= 1'b0;                  // no valid image after power-up
    wc_d        <= 1'b0;
    start_write <= 1'b0;
    start_read  <= 1'b0;
  end
  else if (enable) begin
    start_write <= req_write;             // grant pulses (1 tick)
    start_read  <= req_read && ready_q;
    wc_d        <= rx_monitor[IMG_PROG_W-1];
    if (req_write)                                       ready_q <= 1'b0;
    else if (rx_monitor[IMG_PROG_W-1] && !wc_d)          ready_q <= 1'b1;
  end                                     // write_complete rising -> image valid
end

assign ready_to_read  = ready_q;
assign write_complete = rx_monitor[IMG_PROG_W-1];
assign read_complete  = tx_monitor[IMG_PROG_W-1];

// ---- RW register (checker scratch) ----
always_ff @(posedge clk, negedge reset_n) begin : img_checker_wr
  if (!reset_n)
    img_checker <= IMG_CHECKER_RST[23:0];
  else if (enable) begin                    // effective clock: outermost gate
    if (wen && img_checker_sel) begin
      img_checker.field_0 <= wdata[7:0];
      img_checker.field_1 <= wdata[15:8];
      img_checker.field_2 <= wdata[23:16];
    end
  end
end

// RO fields (live mirrors)
assign img_status.ready_to_read = ready_q;
assign img_status.height        = height_q;
assign img_status.width         = width_q;
assign img_rx_monitor           = img_monitor_t'(rx_monitor);
assign img_tx_monitor           = img_monitor_t'(tx_monitor);
assign img_ctrl.start_read      = start_read;
assign img_ctrl.start_write     = start_write;
assign img_ctrl.req_write       = req_write;
assign img_ctrl.req_read        = req_read;

/* verilator lint_off WIDTHEXPAND */
// Reg Read
always_comb begin : reg_read_dec
  rdata = '0;
  if (!wen) begin
    unique case (1'b1)
      img_status_sel  : rdata = img_status;
      img_rx_mon_sel  : rdata = img_rx_monitor;
      img_tx_mon_sel  : rdata = img_tx_monitor;
      img_ctrl_sel    : rdata = img_ctrl;
      img_checker_sel : rdata = img_checker;
      default         : rdata = '0;
    endcase
  end
end
/* verilator lint_on WIDTHEXPAND */

endmodule
