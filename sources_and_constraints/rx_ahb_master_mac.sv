`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_ahb_master_mac
// Description: RX-write AHB master, application layer (v2). Pops the three
//   rx_afifos in parallel and drains them to SRAM via the shared phys, one AHB
//   SINGLE write per 32-bit row. Progress is the packed counter
//   {complete[PROG_W-1], row, col}, cleared by the (synchronised) start pulse
//   and incremented +4 per completed AHB transaction. counter[PROG_W-2:0] IS
//   the SRAM write address (pixel-stride from 0 - full-image only); the
//   complete bit stops further pops and, through the img_rx_monitor mirror in
//   img_rgf, is the write_complete status the system observes. The counter is
//   exported as the live monitor (synced to gclk by the envelope).
//   PROG_W structural 17; smaller = TB only. Drain gates on empty ONLY.
//   Clocking: clk = 100 MHz, full rate, no enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_ahb_master_mac #(
    parameter int unsigned PROG_W = IMG_PROG_W,
    parameter int unsigned ADDR_W = AHB_ADDR_W
)(
    input  logic clk,                     // 100 MHz SRAM/AHB clock
    input  logic reset_n,                 // external sync-reset module

    // ---- From seq_top_fsm (synchronised into clk by rx_sequencer) ----
    input  logic start_write_s,           // 1-clk pulse: session armed
                                          //   (clears the progress counter)

    // ---- rx_afifo read side (x3, FWFT heads) ----
    input  logic [CHAN_WIDTH-1:0] rdata_r, rdata_g, rdata_b,
    input  logic empty_r, empty_g, empty_b,
    input  logic almost_empty_r, almost_empty_g, almost_empty_b, // advisory only
    output logic pop_r, pop_g, pop_b,     // 1-clk parallel pops (after done x3)

    // ---- AHB phy side (broadcast control; per-channel data) ----
    output logic              xfer_req,
    output logic [ADDR_W-1:0] haddr,      // = progress counter (pixel-stride)
    output logic              hwrite,     // = 1
    output logic [CHAN_WIDTH-1:0] hwdata_r, hwdata_g, hwdata_b,
    input  logic phy_done_r, phy_done_g, phy_done_b,
    input  logic hresp_err_r, hresp_err_g, hresp_err_b,  // unused (SRAM: OKAY)

    // ---- live progress monitor -> img_rx_monitor (via envelope gclk sync) ----
    output logic [IMG_PROG_W-1:0] monitor, // {complete, row, col} zero-extended
    output logic ahb_busy                  // a write is in flight (status)
);

    assign hwdata_r = rdata_r;
    assign hwdata_g = rdata_g;
    assign hwdata_b = rdata_b;
    assign hwrite   = 1'b1;

    typedef enum logic [1:0] { M_IDLE, M_WAIT, M_ISSUE, M_DONE } state_t;
    state_t state, next;

    logic [PROG_W-1:0] prog;              // {complete, row, col} = write address
    logic done_r_q, done_g_q, done_b_q;
    logic data_ok, all_done, row_done, complete;

    assign complete = prog[PROG_W-1];
    assign data_ok  = !(empty_r | empty_g | empty_b);
    assign all_done = (done_r_q | phy_done_r) &
                      (done_g_q | phy_done_g) &
                      (done_b_q | phy_done_b);
    assign row_done = (state == M_DONE) && all_done;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) state <= M_IDLE;
        else          state <= next;
    end

    always_comb begin
        next = state;
        case (state)
            M_IDLE:  if (start_write_s)        next = M_WAIT;
            M_WAIT:  if (data_ok && !complete) next = M_ISSUE;
            M_ISSUE: next = M_DONE;
            M_DONE:  if (all_done)
                         next = (prog + PROG_W'(4) >= {1'b1, {(PROG_W-1){1'b0}}})
                                ? M_IDLE : M_WAIT;    // last row -> session over
            default: next = M_IDLE;
        endcase
    end

    // ---- packed progress counter: cleared by start, +4 per AHB completion ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)                              prog <= '0;
        else if (state == M_IDLE && start_write_s) prog <= '0;
        else if (row_done)                         prog <= prog + PROG_W'(4);
    end

    // ---- per-channel done latches (survive skew between phys) ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)                              {done_r_q, done_g_q, done_b_q} <= '0;
        else if (state == M_ISSUE || (state == M_DONE && !all_done)) begin
            done_r_q <= done_r_q | phy_done_r;
            done_g_q <= done_g_q | phy_done_g;
            done_b_q <= done_b_q | phy_done_b;
        end
        else                                       {done_r_q, done_g_q, done_b_q} <= '0;
    end

    assign xfer_req = (state == M_ISSUE);
    assign haddr    = ADDR_W'(prog[PROG_W-2:0]);   // counter IS the address
    assign ahb_busy = (state == M_ISSUE) || (state == M_DONE);
    assign pop_r    = row_done;
    assign pop_g    = row_done;
    assign pop_b    = row_done;
    // live monitor, zero-extended to the structural 17-bit layout
    assign monitor  = {complete, 16'(prog[PROG_W-2:0])};

endmodule
