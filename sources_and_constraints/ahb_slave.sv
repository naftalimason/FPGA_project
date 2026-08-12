`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ahb_slave
// Description: AHB slave protocol envelope of one colour-channel SRAM
//   (ahb_slave_spec_v1). Three instances at chiptop, each facing its own
//   ahb_master_phy and translating AHB SINGLE transfers into the 1R1W memory
//   port of sram. Zero wait states (on-chip RAM): HREADY = constant 1, HRESP =
//   constant OKAY. SINGLE only: HTRANS IDLE(00)/NONSEQ(10).
//
//   Address mapping (address_map_v1 sec 4): the masters emit the pixel-stride
//   address (0,4,...,65,532); the physical word is word_addr = HADDR[15:2]
//   (drop the 2 LSBs; HADDR[16] unused in the fixed 256x256 configuration).
//
//   Zero-wait READ trick (spec sec 4): raddr is driven COMBINATIONALLY from
//   HADDR during the ADDRESS phase, so the synchronous-read RAM presents rdata
//   during the DATA phase - no wait state needed with a 1-cycle RAM.
//
//   sram_sel (BAR bank chip-select, design note 5): an INDEPENDENT qualifier,
//   never merged with the protocol signals; the current message set makes the
//   AHB path bank-implicit, so chiptop ties it 1 (address_map_v1 d3) - kept as
//   a port for future maps. Clocking: clk = 100 MHz, full rate, no enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module ahb_slave #(
    parameter  int unsigned ADDR_W_AHB = AHB_ADDR_W,  // 17: pixel-stride space
    parameter  int unsigned MEM_ADDR_W = 14           // word address (16384 deep)
)(
    input  logic clk,                     // 100 MHz SRAM/AHB clock
    input  logic reset_n,                 // control regs only (RAM has no reset)

    // ---- AHB slave interface (from ahb_master_phy[ch]) ----
    input  logic [ADDR_W_AHB-1:0] HADDR, // pixel-stride address
    input  logic [1:0]            HTRANS,// IDLE(00) / NONSEQ(10) only
    input  logic                  HWRITE,
    input  logic [2:0]            HSIZE, // 3'b010 (32-bit) - not decoded
    input  logic [2:0]            HBURST,// 3'b000 SINGLE   - not decoded
    input  logic [31:0]           HWDATA,// write data (data phase)
    output logic [31:0]           HRDATA,// read data (data phase) = sram rdata
    output logic                  HREADY,// constant 1 (zero wait states)
    output logic                  HRESP, // constant 0 = OKAY

    // ---- bank chip-select (from chiptop BAR; tie 1 today) ----
    input  logic                  sram_sel,

    // ---- memory side (to sram[ch], 1R1W) ----
    output logic                  wen,
    output logic [MEM_ADDR_W-1:0] waddr,
    output logic [31:0]           wdata,
    output logic [MEM_ADDR_W-1:0] raddr, // driven in the ADDRESS phase (comb)
    input  logic [31:0]           rdata  // registered; valid in the data phase
);

    // ---- address-phase accept (SINGLE: NONSEQ only) ----
    logic accept;
    assign accept = (HTRANS == 2'b10) && sram_sel;   // HREADY==1 always

    // 1-deep address-phase pipeline register (the AHB 2-stage pipeline)
    logic                  dphase_v;      // a data phase is in flight
    logic                  wr_d;          // latched direction
    logic [MEM_ADDR_W-1:0] addr_d;        // latched word address

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            dphase_v <= 1'b0;
            wr_d     <= 1'b0;
            addr_d   <= '0;
        end
        else begin
            dphase_v <= accept;
            if (accept) begin
                wr_d   <= HWRITE;
                addr_d <= HADDR[15:2];   // pixel-stride -> word (drop 2 LSBs)
            end
        end
    end

    // ---- write path: commits in the data phase ----
    assign wen   = dphase_v && wr_d;
    assign waddr = addr_d;
    assign wdata = HWDATA;               // AHB delivers HWDATA in the data phase

    // ---- read path: raddr in the ADDRESS phase -> rdata in the data phase ----
    assign raddr  = HADDR[15:2];
    assign HRDATA = rdata;

    // ---- zero-wait, always-OKAY ----
    assign HREADY = 1'b1;
    assign HRESP  = 1'b0;

endmodule
