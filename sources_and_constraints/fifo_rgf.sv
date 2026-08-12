`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: fifo_rgf
// Description: Async-FIFO status register file (v2, per FIFO_RGF.xlsx, rgf.sv
//   template style). One RO register per FIFO (rx r/g/b, tx r/g/b), each the
//   packed struct {ae_th, af_th, overflow, underflow, almost_empty, empty,
//   almost_full, full}. overflow/underflow are the FIFOs' STICKY flags
//   (cleared only by the session soft-reset). The threshold fields are
//   read-only mirrors of the fixed structural parameters (never PC-writable).
//   All flag inputs pass through 2-FF synchronizers (half of each group lives
//   in the 100 MHz domain; transient RO mirrors only need metastability
//   protection). Clocking: mmcm_clk + gclk enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module fifo_rgf #(
    parameter  int unsigned ADDR_WIDTH = RGF_ADDR_W,
    parameter  int unsigned DATA_WIDTH = 32,
    localparam int unsigned TH_W       = $clog2(AFIFO_DEPTH)   // 4 for DEPTH 16
)(
    input  logic clk,
    input  logic reset_n,
    input  logic enable,                  // gclk clock-enable

    // Mem intf
    input  logic                  wen,
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic [DATA_WIDTH-1:0] wdata,
    output logic [DATA_WIDTH-1:0] rdata,

    // ---- rx_afifo mirrors, index 0=R 1=G 2=B ----
    input  logic [2:0] rx_full,           // gclk domain
    input  logic [2:0] rx_almost_full,    // gclk domain
    input  logic [2:0] rx_empty,          // clk domain (synced here)
    input  logic [2:0] rx_almost_empty,   // clk domain (synced here)
    input  logic [2:0] rx_overflow,       // sticky, gclk domain
    input  logic [2:0] rx_underflow,      // sticky, clk domain (synced here)
    // ---- tx_afifo mirrors ----
    input  logic [2:0] tx_full,           // clk domain (synced here)
    input  logic [2:0] tx_almost_full,    // clk domain (synced here)
    input  logic [2:0] tx_empty,          // gclk domain
    input  logic [2:0] tx_almost_empty,   // gclk domain
    input  logic [2:0] tx_overflow,       // sticky, clk domain (synced here)
    input  logic [2:0] tx_underflow       // sticky, gclk domain
);

localparam FIFO_RX_R_ADDR       = 16'h0000;
localparam FIFO_RX_G_ADDR       = 16'h0004;
localparam FIFO_RX_B_ADDR       = 16'h0008;
localparam FIFO_TX_R_ADDR       = 16'h000C;
localparam FIFO_TX_G_ADDR       = 16'h0010;
localparam FIFO_TX_B_ADDR       = 16'h0014;
localparam FIFO_CHECKER_ADDR    = 16'h0018;

localparam FIFO_CHECKER_RST = 32'h0000_0000;

typedef struct packed {
  logic [TH_W-1:0] almost_empty_th;      // [13:10] param mirror
  logic [TH_W-1:0] almost_full_th;       // [9:6]   param mirror
  logic            overflow;             // [5] sticky
  logic            underflow;            // [4] sticky
  logic            almost_empty;         // [3]
  logic            empty;                // [2]
  logic            almost_full;          // [1]
  logic            full;                 // [0]
} fifo_stat_t;

typedef struct packed {
  logic [7:0] field_2;                   // [23:16]
  logic [7:0] field_1;                   // [15:8]
  logic [7:0] field_0;                   // [7:0]
} fifo_checker_t;

fifo_checker_t fifo_checker;
logic [6:0] reg_sel;                     // one-hot: 6 FIFO regs + checker
logic addr_err_comb;

// Address Decoder
always_comb begin : reg_addr_dec
  reg_sel       = '0;
  addr_err_comb = 1'b0;
  unique case (addr)
    FIFO_RX_R_ADDR    : reg_sel[0] = 1'b1;
    FIFO_RX_G_ADDR    : reg_sel[1] = 1'b1;
    FIFO_RX_B_ADDR    : reg_sel[2] = 1'b1;
    FIFO_TX_R_ADDR    : reg_sel[3] = 1'b1;
    FIFO_TX_G_ADDR    : reg_sel[4] = 1'b1;
    FIFO_TX_B_ADDR    : reg_sel[5] = 1'b1;
    FIFO_CHECKER_ADDR : reg_sel[6] = 1'b1;
    default           : addr_err_comb = 1'b1;
  endcase
end

// ---- 2-FF sync of every mirror bit (36 bits total) ----
logic [17:0] rx_stat_s, tx_stat_s;
bus_sync_2ff #(.W(18)) u_rx_stat_sync (
    .clk(clk), .rst_n(reset_n), .en(enable),   // sample at the gclk rate
    .d({rx_underflow, rx_overflow, rx_almost_empty, rx_empty,
        rx_almost_full, rx_full}),
    .q(rx_stat_s)
);
bus_sync_2ff #(.W(18)) u_tx_stat_sync (
    .clk(clk), .rst_n(reset_n), .en(enable),   // sample at the gclk rate
    .d({tx_underflow, tx_overflow, tx_almost_empty, tx_empty,
        tx_almost_full, tx_full}),
    .q(tx_stat_s)
);

// per-FIFO packed status assembly (i: 0..2 = rx R/G/B, 3..5 = tx R/G/B)
function automatic fifo_stat_t fifo_stat(input logic [17:0] grp, input int ch);
  fifo_stat_t s;
  s.full            = grp[0  + ch];
  s.almost_full     = grp[3  + ch];
  s.empty           = grp[6  + ch];
  s.almost_empty    = grp[9  + ch];
  s.overflow        = grp[12 + ch];
  s.underflow       = grp[15 + ch];
  s.almost_full_th  = TH_W'(AFIFO_AF_THRESH);   // fixed parameter mirrors
  s.almost_empty_th = TH_W'(AFIFO_AE_THRESH);
  return s;
endfunction

// ---- RW checker scratch ----
always_ff @(posedge clk, negedge reset_n) begin : fifo_checker_wr
  if (!reset_n)
    fifo_checker <= FIFO_CHECKER_RST[23:0];
  else if (enable) begin                     // effective clock: outermost gate
    if (wen && reg_sel[6]) begin
      fifo_checker.field_0 <= wdata[7:0];
      fifo_checker.field_1 <= wdata[15:8];
      fifo_checker.field_2 <= wdata[23:16];
    end
  end
end

/* verilator lint_off WIDTHEXPAND */
// Reg Read
always_comb begin : reg_read_dec
  rdata = '0;
  if (!wen) begin
    unique case (1'b1)
      reg_sel[0] : rdata = fifo_stat(rx_stat_s, 0);
      reg_sel[1] : rdata = fifo_stat(rx_stat_s, 1);
      reg_sel[2] : rdata = fifo_stat(rx_stat_s, 2);
      reg_sel[3] : rdata = fifo_stat(tx_stat_s, 0);
      reg_sel[4] : rdata = fifo_stat(tx_stat_s, 1);
      reg_sel[5] : rdata = fifo_stat(tx_stat_s, 2);
      reg_sel[6] : rdata = fifo_checker;
      default    : rdata = '0;
    endcase
  end
end
/* verilator lint_on WIDTHEXPAND */

endmodule
