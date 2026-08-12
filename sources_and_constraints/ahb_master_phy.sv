`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ahb_master_phy
// Description: AHB master protocol layer, per channel (ahb_master_phy_spec_v1).
//   Three instances (R,G,B) at sequencer_top, SHARED between the two AHB MACs -
//   the rx-vs-tx selection mux (ahb_dir) lives at sequencer_top, NOT here. This
//   block is slave-agnostic and direction-agnostic: it runs whatever SINGLE
//   transfer its owning MAC requests and reports completion.
//
//   "ahbSingle": every transfer is a SINGLE beat - HTRANS IDLE/NONSEQ only,
//   HBURST=SINGLE, HSIZE=32-bit word. One outstanding transfer at a time.
//
//   Protocol FSM (spec sec 3, classic AHB 2-phase):
//     S_IDLE : HTRANS=IDLE. xfer_req -> latch addr/dir/wdata -> S_ADDR.
//     S_ADDR : address phase - HTRANS=NONSEQ, drive HADDR/HWRITE/HSIZE/HBURST.
//              Advance when HREADY samples the address phase.
//     S_DATA : data phase - drive HWDATA (writes); hold while !HREADY. On
//              HREADY: reads register HRDATA -> hrdata_o (the ONLY read-data
//              register - handed to tx_afifo by the owning MAC), pulse
//              phy_done, capture hresp_err. -> S_IDLE.
//   With the zero-wait ahb_slave a transfer takes exactly 2 cycles after the
//   request. hrdata_o stays stable from phy_done until the next transfer.
//   Clocking: clk = 100 MHz, full rate, no enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module ahb_master_phy #(
    parameter int unsigned ADDR_W = AHB_ADDR_W       // pixel-stride address width
)(
    input  logic clk,                     // 100 MHz SRAM/AHB clock
    input  logic reset_n,                 // external sync-reset module

    // ---- request from the owning MAC (broadcast bundle, muxed by ahb_dir) ----
    input  logic              xfer_req,   // 1-clk: start ONE single transfer
    input  logic [ADDR_W-1:0] haddr_i,    // transfer address (+4/row stride)
    input  logic              hwrite_i,   // 1=write (RX path), 0=read (TX path)
    input  logic [31:0]       hwdata_i,   // write data (RX path only)

    // ---- AHB master interface to this channel's ahb_slave ----
    output logic [ADDR_W-1:0] HADDR,
    output logic [1:0]        HTRANS,     // IDLE(00) / NONSEQ(10)
    output logic              HWRITE,
    output logic [2:0]        HSIZE,      // 3'b010 = 32-bit word
    output logic [2:0]        HBURST,     // 3'b000 = SINGLE
    output logic [31:0]       HWDATA,     // driven in the data phase
    input  logic [31:0]       HRDATA,
    input  logic              HREADY,     // extends the phase while low
    input  logic              HRESP,      // 0=OKAY / 1=ERROR

    // ---- status back to the owning MAC ----
    output logic [31:0]       hrdata_o,   // captured read data (registered)
    output logic              phy_done,   // 1-clk: transfer completed
    output logic              hresp_err   // 1-clk: HRESP=ERROR seen
);

    typedef enum logic [1:0] { S_IDLE, S_ADDR, S_DATA } state_t;
    state_t state, next;

    // latched request (held stable through both phases)
    logic [ADDR_W-1:0] addr_q;
    logic              wr_q;
    logic [31:0]       wdata_q;

    // ---- State register + request latch + read-data capture ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state    <= S_IDLE;
            addr_q   <= '0;
            wr_q     <= 1'b0;
            wdata_q  <= '0;
            hrdata_o <= '0;
        end
        else begin
            state <= next;
            if (state == S_IDLE && xfer_req) begin
                addr_q  <= haddr_i;
                wr_q    <= hwrite_i;
                wdata_q <= hwdata_i;
            end
            if (state == S_DATA && HREADY && !wr_q)
                hrdata_o <= HRDATA;              // the only read-data register
        end
    end

    // ---- Next-state logic ----
    always_comb begin
        next = state;
        case (state)
            S_IDLE: if (xfer_req) next = S_ADDR;
            S_ADDR: if (HREADY)   next = S_DATA;   // address phase sampled
            S_DATA: if (HREADY)   next = S_IDLE;   // data phase complete
            default: next = S_IDLE;
        endcase
    end

    // ---- AHB bus outputs (Moore per phase) ----
    assign HADDR  = addr_q;
    assign HTRANS = (state == S_ADDR) ? 2'b10 : 2'b00;   // NONSEQ / IDLE
    assign HWRITE = wr_q;
    assign HSIZE  = 3'b010;                              // 32-bit word
    assign HBURST = 3'b000;                              // SINGLE
    assign HWDATA = wdata_q;                             // valid in data phase

    // ---- completion strobes (1-clk: S_DATA lasts one cycle when HREADY=1) ----
    assign phy_done  = (state == S_DATA) && HREADY;
    assign hresp_err = (state == S_DATA) && HREADY && HRESP;

endmodule
