`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: sram
// Description: Per-channel image memory (sram_spec_v1) - one parametric 1R1W
//   synchronous RAM, instantiated three times at chiptop (R,G,B), each behind
//   its own ahb_slave. Each 32-bit word stores 4 samples of ONE colour (one
//   image row-quad). Follows the shape of the provided ram_1r1w.sv template
//   (separate write/read ports, registered read); port names per the binding
//   spec: wen/waddr/wdata + raddr/rdata (template mapping: wen = !cs_n & !wr_n,
//   waddr=wr_addr, raddr=rd_addr, wdata=data_in, rdata=data_out).
//
//   - Synchronous write; synchronous read with 1-cycle latency: raddr presented
//     in the AHB ADDRESS phase yields rdata during the DATA phase (zero-wait
//     read - ahb_slave_spec sec 4).
//   - No reset on the memory array (block-RAM style; contents undefined until
//     written). Write/read same-address collision cannot occur: burst write and
//     burst read are separate commands and never overlap.
//   Clocking: clk = 100 MHz SRAM/AHB domain, full rate, no enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module sram #(
    parameter  int unsigned DATA_W = 32,
    parameter  int unsigned DEPTH  = 16384,          // 256*256/4 words
    localparam int unsigned ADDR_W = $clog2(DEPTH)   // 14
)(
    input  logic              clk,
    // ---- write port (RX burst-write drain, via ahb_slave) ----
    input  logic              wen,
    input  logic [ADDR_W-1:0] waddr,
    input  logic [DATA_W-1:0] wdata,
    // ---- read port (TX burst-read fill, via ahb_slave) ----
    input  logic [ADDR_W-1:0] raddr,
    output logic [DATA_W-1:0] rdata     // registered: valid 1 cycle after raddr
);

    logic [DATA_W-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (wen) mem[waddr] <= wdata;   // synchronous write
        rdata <= mem[raddr];            // synchronous read, 1-cycle latency
    end

endmodule
