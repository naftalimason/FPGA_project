`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: apb_slave_envelope
// Description: Generic APB-slave wrapper for one RGF (apb_slave_envelope_spec).
//   Identical envelope around each of img_rgf / uart_rgf / fifo_rgf so the
//   interconnect treats all three uniformly: translates APB into the RGF's
//   simple mem interface (wen/addr/wdata/rdata) and generates pready. Default
//   contract (all three project RGFs): combinational read, 1-cycle write,
//   always ready (0 wait states). Purely combinational - no state.
//   Packed-struct registers imply PC-side read-modify-write for single-field
//   writes (spec sec 4); the envelope only ever sees full-word writes.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module apb_slave_envelope #(
    parameter int unsigned ADDR_W = RGF_ADDR_W
)(
    // ---- APB slave side (from apb_interconnect) ----
    input  logic              psel,
    input  logic              penable,
    input  logic              pwrite,
    input  logic [ADDR_W-1:0] paddr,
    input  logic [31:0]       pwdata,
    output logic [31:0]       prdata,
    output logic              pready,

    // ---- RGF mem side ----
    output logic              wen,       // register write strobe (ACCESS phase)
    output logic [ADDR_W-1:0] addr,      // register offset (parked when idle)
    output logic [31:0]       wdata,
    input  logic [31:0]       rdata      // combinational register read data
);

    assign wen    = psel & penable & pwrite;        // write strobe in ACCESS
    // APB-correct (v2.1): PADDR is presented from SETUP through ACCESS exactly
    // as the protocol drives it. Read-to-clear timing is NOT handled here: the
    // R0C registers are qualified by rgf_r0c_read from apb_master_phy
    // (PSEL && PENABLE && PREADY && !PWRITE - the completed-read cycle).
    assign addr   = psel ? paddr : {ADDR_W{1'b1}};
    assign wdata  = pwdata;
    assign prdata = rdata;                          // combinational read
    assign pready = psel & penable;                 // 0 wait states

endmodule
