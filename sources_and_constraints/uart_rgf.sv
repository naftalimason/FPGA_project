`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: uart_rgf
// Description: UART status/config register file (v2, per UART RGF.xlsx, rgf.sv
//   template style). v2 role change: uart_rgf ONLY counts - it receives the
//   THREE DISTINCT error events (phy / mac / classifier) directly and keeps
//   three 8-bit saturating counters in uart_reg (type R0C: one read returns
//   the counts and clears all three). The resend flow lives OUTSIDE in
//   rx_resend_ctrl. uart_config (WO) owns the runtime parity mode driven into
//   both PHYs (reset default = package PARITY_ODD; the PC never reads WO
//   registers - reads return 0). clk_status (RO, kept from the old map by
//   request) reports the actual glitchless-mux selection.
//   Clocking: mmcm_clk + gclk enable. R0C timing (v2.1): the clear is
//   qualified by rgf_r0c_read from apb_master_phy (PSEL&&PENABLE&&PREADY&&
//   !PWRITE), i.e. the completed-read ACCESS cycle - the same edge prdata_o
//   captures PRDATA, with PADDR protocol-valid from SETUP onward.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module uart_rgf #(
    parameter int unsigned ADDR_WIDTH = RGF_ADDR_W,
    parameter int unsigned DATA_WIDTH = 32
)(
    input  logic clk,
    input  logic reset_n,
    input  logic enable,                  // gclk clock-enable

    // Mem intf
    input  logic                  wen,
    input  logic                  rgf_r0c_read, // from apb_master_phy (v2.1):
                                                //   PSEL&&PENABLE&&PREADY&&
                                                //   !PWRITE - completed read
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic [DATA_WIDTH-1:0] wdata,
    output logic [DATA_WIDTH-1:0] rdata,

    // ---- distinct error events (1-tick, gclk domain) ----
    input  logic phy_err_evt,             // rx_phy: parity / framing / stop
    input  logic mac_err_evt,             // rx_mac: frame structure/delimiters
    input  logic clsf_err_evt,            // classifier: illegal content

    // ---- status inputs ----
    input  logic clk_sel_stat,            // actual glitchless-mux selection

    // ---- config outputs ----
    output logic parity_odd               // runtime parity mode -> both PHYs
);

localparam UART_REG_ADDR     = 16'h0000;
localparam UART_CONFIG_ADDR  = 16'h0004;
localparam UART_CHECKER_ADDR = 16'h0008;
localparam CLK_STATUS_ADDR   = 16'h000C;

localparam UART_REG_RST     = 32'h0000_0000;
localparam UART_CHECKER_RST = 32'h0000_0000;

typedef struct packed {
  logic [7:0] classifier_err;            // [23:16]
  logic [7:0] mac_err;                   // [15:8]
  logic [7:0] phy_err;                   // [7:0]
} uart_reg_t;

typedef struct packed {
  logic parity_odd;                      // [0]
} uart_config_t;

typedef struct packed {
  logic [7:0] field_2;                   // [23:16]
  logic [7:0] field_1;                   // [15:8]
  logic [7:0] field_0;                   // [7:0]
} uart_checker_t;

typedef struct packed {
  logic clk_sel;                         // [0]
} clk_status_t;

uart_reg_t     uart_reg;
uart_config_t  uart_config;
uart_checker_t uart_checker;
clk_status_t   clk_status;

logic uart_reg_sel, uart_config_sel, uart_checker_sel, clk_status_sel;
logic addr_err_comb;

// saturating 8-bit increment
function automatic logic [7:0] sat_inc8(input logic [7:0] c);
  return (c == 8'hFF) ? c : c + 8'd1;
endfunction

// Address Decoder
always_comb begin : reg_addr_dec
  uart_reg_sel     = 1'b0;
  uart_config_sel  = 1'b0;
  uart_checker_sel = 1'b0;
  clk_status_sel   = 1'b0;
  addr_err_comb    = 1'b0;
  unique case (addr)
    UART_REG_ADDR     : uart_reg_sel     = 1'b1;
    UART_CONFIG_ADDR  : uart_config_sel  = 1'b1;
    UART_CHECKER_ADDR : uart_checker_sel = 1'b1;
    CLK_STATUS_ADDR   : clk_status_sel   = 1'b1;
    default           : addr_err_comb    = 1'b1;
  endcase
end

// ---- R0C error counters: cleared by a read, else count events ----
always_ff @(posedge clk, negedge reset_n) begin : uart_reg_r0c
  if (!reset_n)
    uart_reg <= UART_REG_RST[23:0];
  else if (enable) begin
    // R0C (v2.1): clear only on the COMPLETED READ cycle, qualified by the
    // master-phy signal - PADDR stays protocol-valid from SETUP, and the clear
    // commits on the very edge the APB PHY captures PRDATA (pre-clear value)
    if (rgf_r0c_read && uart_reg_sel)
      uart_reg <= '0;
    else begin
      if (phy_err_evt)  uart_reg.phy_err        <= sat_inc8(uart_reg.phy_err);
      if (mac_err_evt)  uart_reg.mac_err        <= sat_inc8(uart_reg.mac_err);
      if (clsf_err_evt) uart_reg.classifier_err <= sat_inc8(uart_reg.classifier_err);
    end
  end
end

// ---- WO config: writable, never read back ----
always_ff @(posedge clk, negedge reset_n) begin : uart_config_wr
  if (!reset_n)
    uart_config.parity_odd <= PARITY_ODD;      // package value = reset default
  else if (enable) begin                     // effective clock: outermost gate
    if (wen && uart_config_sel)
      uart_config.parity_odd <= wdata[0];
  end
end

assign parity_odd = uart_config.parity_odd;

// ---- RW checker scratch ----
always_ff @(posedge clk, negedge reset_n) begin : uart_checker_wr
  if (!reset_n)
    uart_checker <= UART_CHECKER_RST[23:0];
  else if (enable) begin                     // effective clock: outermost gate
    if (wen && uart_checker_sel) begin
      uart_checker.field_0 <= wdata[7:0];
      uart_checker.field_1 <= wdata[15:8];
      uart_checker.field_2 <= wdata[23:16];
    end
  end
end

// RO fields
assign clk_status.clk_sel = clk_sel_stat;

/* verilator lint_off WIDTHEXPAND */
// Reg Read (WO uart_config excluded - reads as 0)
always_comb begin : reg_read_dec
  rdata = '0;
  if (!wen) begin
    unique case (1'b1)
      uart_reg_sel     : rdata = uart_reg;
      uart_checker_sel : rdata = uart_checker;
      clk_status_sel   : rdata = clk_status;
      default          : rdata = '0;
    endcase
  end
end
/* verilator lint_on WIDTHEXPAND */

endmodule
