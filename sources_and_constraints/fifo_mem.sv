`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: fifo_mem
// Description: Async-FIFO storage array (async_fifo_spec_v1 sec 3). Dual-port:
//   synchronous write in the write clock domain, COMBINATIONAL read - this is
//   what makes the FIFO first-word-fall-through (FWFT): the head word appears on
//   rdata as soon as raddr points at it, so the consumer (fifo consumer FSM /
//   AHB MAC) sees valid head data whenever !empty, with no read latency.
//   Sized small (DEPTH x 32) -> infers distributed/LUT RAM.
//////////////////////////////////////////////////////////////////////////////////

module fifo_mem #(
    parameter  int unsigned WIDTH  = 32,
    parameter  int unsigned DEPTH  = 16,
    localparam int unsigned ADDR_W = $clog2(DEPTH)
)(
    // write port (write clock domain)
    input  logic              wr_clk,
    input  logic              en,       // write-domain OPERATIONAL-rate enable -
                                        //   the EFFECTIVE CLOCK (gclk enable on
                                        //   the mmcm side, 1'b1 on the 100 MHz
                                        //   side); outermost gate of the write
    input  logic              wen,      // accepted push (wr_en & !full), RAW -
                                        //   sampled only under en
    input  logic [ADDR_W-1:0] waddr,
    input  logic [WIDTH-1:0]  wdata,
    // read port (address from the read domain; data is combinational)
    input  logic [ADDR_W-1:0] raddr,
    output logic [WIDTH-1:0]  rdata     // FWFT head - valid when !empty
);

    logic [WIDTH-1:0] mem [0:DEPTH-1];

    // en is the write domain's EFFECTIVE CLOCK (project rule): outermost gate,
    // wen is a plain condition evaluated only under it.
    always_ff @(posedge wr_clk)
        if (en) begin
            if (wen) mem[waddr] <= wdata;   // write only on an accepted push
        end

    assign rdata = mem[raddr];          // combinational read -> FWFT

endmodule
