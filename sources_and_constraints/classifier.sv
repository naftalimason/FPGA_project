`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: classifier
// Description: Message sampler + field router (classifier_spec_v1, refined
//   handshake). Samples a frame only when it is structurally valid (is_msg,
//   rx_mac) AND content-legal (msg_legal, parser) AND the sequencer is free
//   (!busy). The 4 delimiter bytes (buf 15/10/5/0) are dropped; only the 12
//   payload bytes are latched into the packed holding register pay[11:0]
//   (spec sec 2A). Every output field is then PRE-ALIGNED, pure wiring off
//   pay[] - no downstream module re-muxes. For pixel payloads the outputs
//   de-interleave into three per-channel 32-bit words (exact stride-3 map,
//   spec sec 5).
//
//   Handshake (spec sec 7): new_msg is a HELD-VALID REQUEST carried against the
//   single sequencer-owned busy signal:
//     busy 0->1 = command accepted ; busy=1 = processing ; busy 1->0 = complete.
//   C_WAIT_ACCEPT holds new_msg=1 + fields stable until busy=1; C_WAIT_DONE
//   lowers new_msg but keeps the fields held until busy=0; C_RELEASE then
//   pulses msg_taken so rx_mac frees the frame - only after COMPLETE command
//   completion. The C_WAIT_ACCEPT/C_WAIT_DONE split guarantees busy=1 is
//   observed before a later busy=0 is treated as completion.
//
//   Content rejects are NOT sampled: classifier_err pulses exactly once and the
//   FSM parks in C_REJECT until uart_rgf's resend flow flushes the held frame
//   out of rx_mac (is_msg falls) - no deadlock, no double count.
//
//   Owns NO burst state: burst_mode is driven to the parser by seq_top_fsm and
//   the pixel counter lives in fifo_rx_producer_fsm (architecture note).
//   Clocking: clk = mmcm_clk; enable = gclk = baud*16 clock-enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module classifier (
    // ---- Clocking (from chiptop) ----
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- From rx_mac ----
    input  logic [D_SIZE-1:0] msg_buf [MSG_LEN-1:0], // 16-byte frame, buf[15]='{'
    input  logic [1:0]        num_submsg, // valid-byte span 1/2/3 (informational:
                                          //   stale-tail gating is by msg_type -
                                          //   consumers read only live fields)
    input  logic              is_msg,     // structurally valid frame present

    // ---- From parser (combinational, same cycle) ----
    input  logic              msg_legal,  // contents legal for current mode
    input  msg_type_t         msg_type,   // decoded format

    // ---- From sequencer_top (seq_top_fsm) ----
    input  logic              busy,       // 0->1 accept, 1 processing, 1->0 done

    // ---- To rx_mac ----
    output logic              msg_taken,  // 1-tick ack -> release the held frame

    // ---- To sequencer_top ----
    output logic              new_msg,    // HELD request: high until busy accepts
    output msg_type_t         msg_type_o, // type presented to the sequencer
    output logic [LOCAL_ADDR_WIDTH-1:0] local_addr, // {A1,A0} RGF local offset
    output logic [WDATA_WIDTH-1:0]      wdata,      // {DH1,DH0,DL1,DL0}
    output logic [DIM_WIDTH-1:0]        img_height, // {H2,H1,H0}
    output logic [DIM_WIDTH-1:0]        img_width,  // {W2,W1,W0}
    output logic [CHAN_WIDTH-1:0]       pixel_r,    // {R0,R1,R2,R3}
    output logic [CHAN_WIDTH-1:0]       pixel_g,    // {G0,G1,G2,G3}
    output logic [CHAN_WIDTH-1:0]       pixel_b,    // {B0,B1,B2,B3}

    // ---- To chiptop BAR (top-level port, not via sequencer) ----
    output logic [BAR_ADDR_WIDTH-1:0]   bar_addr,   // A2 -> per-module chip-select

    // ---- To uart_rgf ----
    output logic              classifier_err // 1-tick: content reject -> resend flow
);

    // =========================================================================
    // Sampling + handshake FSM (spec sec 7, held-request refinement).
    // =========================================================================
    typedef enum logic [2:0] {
        C_IDLE,        // wait for a frame; sample on accept, count once on reject
        C_WAIT_ACCEPT, // hold new_msg=1 + fields stable; wait busy=1 (accepted)
        C_WAIT_DONE,   // new_msg low, fields still held; wait busy=0 (complete)
        C_RELEASE,     // 1 tick: msg_taken -> rx_mac releases the frame
        C_REJECT       // parked: wait for uart_rgf's flush to clear the frame
    } state_t;

    state_t state, next;
    logic   accept, reject;

    assign accept = is_msg &&  msg_legal && !busy;
    assign reject = is_msg && !msg_legal;

    // ---- State register ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)    state <= C_IDLE;
        else if (enable) state <= next;
    end

    // ---- Next-state logic ----
    always_comb begin
        next = state;
        case (state)
            C_IDLE:        begin
                if      (reject) next = C_REJECT;
                else if (accept) next = C_WAIT_ACCEPT;
            end
            C_WAIT_ACCEPT: if ( busy)   next = C_WAIT_DONE;  // sequencer accepted
            C_WAIT_DONE:   if (!busy)   next = C_RELEASE;    // complete command done
            C_RELEASE:     next = C_IDLE;
            C_REJECT:      if (!is_msg) next = C_IDLE;       // frame was flushed
            default:       next = C_IDLE;
        endcase
    end

    // ---- Moore outputs ----
    //   new_msg  : held level for the whole C_WAIT_ACCEPT window (request).
    //   msg_taken: one enable-tick (C_RELEASE lasts one tick).
    //   The accept-before-done ordering needs no busy_seen flag - it is
    //   structural: C_WAIT_DONE is only reachable through busy=1.
    assign new_msg   = (state == C_WAIT_ACCEPT);
    assign msg_taken = (state == C_RELEASE);

    // ---- classifier_err: registered 1-tick pulse, exactly once per rejected
    //      frame (the FSM leaves C_IDLE on the same edge, so the condition is
    //      true for a single tick; C_REJECT then waits for the flush) ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)    classifier_err <= 1'b0;
        else if (enable) classifier_err <= (state == C_IDLE) && reject;
    end

    // =========================================================================
    // Holding register pay[11:0] - 12 payload bytes, delimiters dropped
    // (spec sec 2A):  pay idx : 11 10  9  8 |  7  6  5  4 |  3  2  1  0
    //                 buf idx : 14 13 12 11 |  9  8  7  6 |  4  3  2  1
    // Sampled once on accept, held through C_WAIT_ACCEPT/C_WAIT_DONE/C_RELEASE.
    // =========================================================================
    logic [D_SIZE-1:0] pay [11:0];

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (int i = 0; i < 12; i++) pay[i] <= '0;
            msg_type_o <= MSG_NONE;
        end
        else if (enable) begin
            if (state == C_IDLE && accept) begin
                pay[11] <= msg_buf[14];  pay[10] <= msg_buf[13];
                pay[9]  <= msg_buf[12];  pay[8]  <= msg_buf[11];
                pay[7]  <= msg_buf[9];   pay[6]  <= msg_buf[8];
                pay[5]  <= msg_buf[7];   pay[4]  <= msg_buf[6];
                pay[3]  <= msg_buf[4];   pay[2]  <= msg_buf[3];
                pay[1]  <= msg_buf[2];   pay[0]  <= msg_buf[1];
                msg_type_o <= msg_type;
            end
        end
    end

    // =========================================================================
    // Field extraction - pure wiring off the held pay[] register (spec sec 3/5).
    // Only the fields meaningful for msg_type_o carry live data; the rest are
    // don't-care that command (the sequencer reads only what the type selects).
    // =========================================================================
    assign bar_addr   = pay[10];                             // A2 (buf13)
    assign local_addr = {pay[9], pay[8]};                    // {A1,A0}
    assign wdata      = {pay[5], pay[4], pay[1], pay[0]};    // {DH1,DH0,DL1,DL0}
    assign img_height = {pay[6], pay[5], pay[4]};            // {H2,H1,H0}
    assign img_width  = {pay[2], pay[1], pay[0]};            // {W2,W1,W0}

    // Pixel de-interleave: clean stride-3 in pay[] space (spec sec 5, exact map)
    assign pixel_r    = {pay[11], pay[8], pay[5], pay[2]};   // {R0,R1,R2,R3}
    assign pixel_g    = {pay[10], pay[7], pay[4], pay[1]};   // {G0,G1,G2,G3}
    assign pixel_b    = {pay[9],  pay[6], pay[3], pay[0]};   // {B0,B1,B2,B3}

endmodule
