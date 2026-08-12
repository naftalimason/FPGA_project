`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tx_consumer_fsm
// Description: Burst pixel consumer, read/TX side (v2). Pops the three
//   tx_afifos in parallel and runs the message handshake with tx_mac through
//   the arbiter (lines held stable from valid until done; pop only after
//   tx_msg_done). Progress is the packed counter {complete[PROG_W-1],row,col},
//   cleared by start_read, +4 per tx_msg_done (one message = one RGB word
//   group). This counter IS the live img_tx_monitor mirror (gclk native); its
//   complete bit stops further pops and, through img_rgf, is the read_complete
//   status the system observes. PROG_W structural 17; smaller = TB only.
//   Drain gates on empty ONLY. pop is a RAW strobe; the FIFO samples it under
//   its rd_clk_en (effective-clock discipline: the gclk enable is never
//   combined combinationally into a strobe). Clocking: mmcm_clk + gclk enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module tx_consumer_fsm #(
    parameter int unsigned PROG_W = IMG_PROG_W
)(
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- From seq_top_fsm ----
    input  logic start_read,              // 1-tick: begin burst-read drain
                                          //   (clears the progress counter)

    // ---- tx_afifo read side (x3, FWFT heads) ----
    input  logic [CHAN_WIDTH-1:0] rdata_r, rdata_g, rdata_b,
    input  logic empty_r, empty_g, empty_b,
    input  logic almost_empty_r, almost_empty_g, almost_empty_b, // advisory only
    output logic pop_r, pop_g, pop_b,     // 1-tick parallel pops (on the ack)

    // ---- Message handshake (via tx_msg_arbiter) ----
    input  logic tx_msg_done,
    output logic tx_msg_valid,
    output logic [CHAN_WIDTH-1:0] tx_line_r, tx_line_g, tx_line_b,

    // ---- live progress monitor -> img_tx_monitor (gclk native) ----
    output logic [IMG_PROG_W-1:0] monitor, // {complete, row, col} zero-extended
    output logic busy                      // a message is in flight
);

    assign tx_line_r = rdata_r;           // FWFT heads, direct wire
    assign tx_line_g = rdata_g;
    assign tx_line_b = rdata_b;

    typedef enum logic [1:0] { C_IDLE, C_ARMED, C_SEND } state_t;
    state_t state, next;

    logic [PROG_W-1:0] prog;
    logic data_ok, complete;

    assign complete = prog[PROG_W-1];
    assign data_ok  = !(empty_r | empty_g | empty_b);

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)    state <= C_IDLE;
        else if (enable) state <= next;
    end

    always_comb begin
        next = state;
        case (state)
            C_IDLE:  if (start_read)             next = C_ARMED;
            C_ARMED: if (data_ok && !complete)   next = C_SEND;
            C_SEND:  if (tx_msg_done)
                         next = (prog + PROG_W'(4) >= {1'b1, {(PROG_W-1){1'b0}}})
                                ? C_IDLE : C_ARMED;   // last message -> done
            default: next = C_IDLE;
        endcase
    end

    // ---- packed progress counter: cleared by start, +4 per sent message ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)                                 prog <= '0;
        else if (enable) begin
            if (state == C_IDLE && start_read)        prog <= '0;
            else if (state == C_SEND && tx_msg_done)  prog <= prog + PROG_W'(4);
        end
    end

    assign busy         = (state == C_SEND);
    assign tx_msg_valid = (state == C_SEND);
    assign pop_r        = (state == C_SEND) && tx_msg_done; // RAW strobe: FIFO
                                                            //   samples under its
                                                            //   rd_clk_en
    assign pop_g        = pop_r;
    assign pop_b        = pop_r;
    assign monitor      = {complete, 16'(prog[PROG_W-2:0])};

endmodule
