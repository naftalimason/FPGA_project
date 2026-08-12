`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_producer_fsm
// Description: Burst pixel producer, write side (v2). One parallel push of the
//   three pre-aligned channel words per accepted PIXEL_PAYLOAD. Progress is the
//   packed counter {complete[PROG_W-1], row, col}, cleared by start_write and
//   incremented +4 per pushed burst frame (one RGB word group = 4 pixels);
//   automatic row rollover; the complete bit (bit16 structurally) is the
//   end-of-image condition: it stops further pushes and goes DIRECTLY to
//   seq_top_fsm, which holds busy high until command completion. Not mirrored.
//   PROG_W is structural (17); smaller values are for TESTBENCH USE ONLY.
//   busy (P_PUSH) is the per-frame accept toward the classifier handshake;
//   pushes gate on full+almost_full (early throttle). push is a RAW strobe;
//   the FIFO samples it under its wr_clk_en (effective-clock discipline: the
//   gclk enable is never combined combinationally into a strobe).
//   Clocking: mmcm_clk + gclk enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_producer_fsm #(
    parameter int unsigned PROG_W = IMG_PROG_W
)(
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- From classifier ----
    input  logic       new_msg,           // held request (refined handshake)
    input  msg_type_t  msg_type,          // filter: only PIXEL_PAYLOAD
    input  logic [CHAN_WIDTH-1:0] pixel_r, pixel_g, pixel_b,

    // ---- rx_afifo write-side flags (x3) ----
    input  logic full_r, full_g, full_b,
    input  logic almost_full_r, almost_full_g, almost_full_b,

    // ---- From seq_top_fsm ----
    input  logic start_write,             // 1-tick: begin image session
                                          //   (clears the progress counter)

    // ---- To seq_top_fsm ----
    output logic busy,                    // a push is in flight (accept level)
    output logic complete,                // progress bit16 LEVEL: image fully
                                          //   pushed - lock busy, stop pushing

    // ---- rx_afifo write side (x3) ----
    output logic push_r, push_g, push_b,  // 1-tick parallel write strobes
    output logic [CHAN_WIDTH-1:0] wdata_r, wdata_g, wdata_b
);

    assign wdata_r = pixel_r;             // direct wire - pixels arrive aligned
    assign wdata_g = pixel_g;
    assign wdata_b = pixel_b;

    typedef enum logic [1:0] { P_IDLE, P_ARMED, P_PUSH, P_ACK } state_t;
    state_t state, next;

    logic [PROG_W-1:0] prog;              // {complete, row, col} packed counter
    logic space_ok, payload_req;

    assign complete    = prog[PROG_W-1];
    assign space_ok    = !(full_r | full_g | full_b) &&
                         !(almost_full_r | almost_full_g | almost_full_b);
    assign payload_req = new_msg && (msg_type == MSG_PIXEL_PAYLOAD);

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)    state <= P_IDLE;
        else if (enable) state <= next;
    end

    always_comb begin
        next = state;
        case (state)
            P_IDLE:  if (start_write)               next = P_ARMED;
            P_ARMED: if (payload_req && !complete)  next = P_PUSH;
            P_PUSH:  if (space_ok)                  next = P_ACK;
            P_ACK:   next = complete ? P_IDLE : P_ARMED;  // bit16 -> session over
            default: next = P_IDLE;
        endcase
    end

    // ---- packed progress counter: cleared by start, +4 per pushed frame ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)                              prog <= '0;
        else if (enable) begin
            if (state == P_IDLE && start_write)    prog <= '0;
            else if (state == P_PUSH && space_ok)  prog <= prog + PROG_W'(4);
        end
    end

    assign busy   = (state == P_PUSH);
    assign push_r = (state == P_PUSH) && space_ok;   // RAW strobe: FIFO samples
                                                     //   it under its wr_clk_en
    assign push_g = push_r;
    assign push_b = push_r;

endmodule
