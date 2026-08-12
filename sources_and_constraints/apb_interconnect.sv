`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: apb_interconnect
// Description: Shared APB bus fabric (apb_interconnect_spec_v1). Fans the ONE
//   APB master out to N_SLAVES register-file envelopes: broadcasts penable/
//   pwrite/paddr/pwdata, demuxes PSEL by the BAR-decoded slave_sel, muxes
//   PRDATA/PREADY back, and idle-guards (pready=0, prdata=0 when psel==0) so
//   the master PHY never hangs. Purely combinational routing; N_SLAVES is the
//   single growth knob. slave_sel is NEVER combined with any protocol select
//   (design note 5). Slave index encoding (address_map_v1): 0=img_rgf,
//   1=uart_rgf, 2=fifo_rgf.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module apb_interconnect #(
    parameter int unsigned N_SLAVES = N_RGF,
    parameter int unsigned SEL_W    = RGF_SEL_W,
    parameter int unsigned ADDR_W   = RGF_ADDR_W
)(
    // ---- master side (from apb_master) ----
    input  logic              psel,
    input  logic              penable,
    input  logic              pwrite,
    input  logic [ADDR_W-1:0] paddr,
    input  logic [31:0]       pwdata,
    input  logic [SEL_W-1:0]  slave_sel,
    output logic [31:0]       prdata,
    output logic              pready,

    // ---- slave side (N_SLAVES packed channels) ----
    output logic [N_SLAVES-1:0]       psel_o,    // demuxed per-slave select
    output logic                      penable_o, // broadcast
    output logic                      pwrite_o,  // broadcast
    output logic [ADDR_W-1:0]         paddr_o,   // broadcast
    output logic [31:0]               pwdata_o,  // broadcast
    input  logic [N_SLAVES-1:0][31:0] prdata_i,  // per-slave read data
    input  logic [N_SLAVES-1:0]       pready_i   // per-slave ready
);

    // broadcast
    assign penable_o = penable;
    assign pwrite_o  = pwrite;
    assign paddr_o   = paddr;
    assign pwdata_o  = pwdata;

    // demux psel
    always_comb begin
        psel_o = '0;
        if (psel && (32'(slave_sel) < N_SLAVES))
            psel_o[slave_sel] = 1'b1;
    end

    // mux back + idle guard (never float / never hang the PHY)
    always_comb begin
        if (psel && (32'(slave_sel) < N_SLAVES)) begin
            prdata = prdata_i[slave_sel];
            pready = pready_i[slave_sel];
        end
        else begin
            prdata = '0;
            pready = psel;      // unmapped select: complete immediately with 0
        end
    end

endmodule
