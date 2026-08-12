`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tx_mac
// Description: UART TX framing/message layer (tx_mac_spec_v1) - mirror of rx_mac
//   on the transmit side. Serialises a COMPLETE 16-byte reply from the composer
//   (tx_msg[127:0], '{' = [127:120] first on the wire) byte-by-byte MSByte-first
//   to tx_phy, running the message handshake (tx_msg_valid in / tx_msg_done
//   out). ALSO owns the RX-error RESEND notification: when uart_rgf raises
//   resend_pending it emits the bare 6-byte {E<0,0,0>} directly (NOT via the
//   composer/arbiter), then pulses resend_sent so uart_rgf can clear the flag
//   and flush the RX chain (error_resend_flow_v1).
//
//   PRIORITY (spec sec 3): {E} wins when idle; an in-flight reply always
//   completes first. Only one message at a time.
//
//   S_GAP release state: after the completion pulse the granted source drops
//   its valid (or uart_rgf clears resend_pending) one tick later; S_GAP waits
//   for that release before re-sampling, so the same message can never be
//   latched twice. Clocking: clk = mmcm_clk; enable = gclk clock-enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module tx_mac (
    // ---- Clocking ----
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- Reply message (from composer, handshake via tx_msg_arbiter) ----
    input  logic [MSG_WIDTH-1:0] tx_msg,  // complete framed 16-byte reply
    input  logic tx_msg_valid,            // message ready (held until done)
    output logic tx_msg_done,             // 1-tick: reply fully sent to PC

    // ---- Resend notification (from uart_rgf) ----
    input  logic resend_pending,          // an RX error needs a {E} resend request
    output logic resend_sent,             // 1-tick: {E} fully sent

    // ---- Byte interface to tx_phy ----
    output logic [D_SIZE-1:0] tx_byte,    // next byte (MSByte-first)
    output logic byte_send,               // 1-tick: send this byte
    input  logic byte_done,               // tx_phy: byte fully shifted

    output logic tx_busy                  // a message is being transmitted
);

    // {E<0,0,0>} resend request, top-aligned MSByte-first (message_format sec 5)
    localparam logic [47:0] E_MSG = {START_BYTE, 8'h45, 8'h00, 8'h00, 8'h00,
                                     END_BYTE};

    typedef enum logic [2:0] { S_IDLE, S_REQ, S_WAITB, S_DONE, S_GAP } state_t;
    typedef enum logic       { SRC_REPLY, SRC_RESEND } src_t;

    state_t state, next;
    src_t   src_q;
    logic [MSG_WIDTH-1:0] msg_q;          // message buffer (E uses the top 6 bytes)
    logic [3:0] idx;                      // byte index, walks 15 -> down
    logic [4:0] sent;                     // bytes completed
    logic [4:0] len_q;                    // 6 or 16
    logic       last_byte;

    assign last_byte = (sent == len_q - 5'd1);

    // ---- State register + message latch + byte walk ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_IDLE;
            src_q <= SRC_REPLY;
            msg_q <= '0;
            idx   <= 4'd15;
            sent  <= '0;
            len_q <= '0;
        end
        else if (enable) begin
            state <= next;
            if (state == S_IDLE) begin
                idx  <= 4'd15;
                sent <= '0;
                if (resend_pending) begin              // {E} has priority
                    msg_q[MSG_WIDTH-1 -: 48] <= E_MSG;
                    len_q <= 5'd6;
                    src_q <= SRC_RESEND;
                end
                else if (tx_msg_valid) begin
                    msg_q <= tx_msg;                   // composer holds it stable
                    len_q <= 5'd16;
                    src_q <= SRC_REPLY;
                end
            end
            else if (state == S_WAITB && byte_done && !last_byte) begin
                idx  <= idx - 4'd1;                    // next lower byte
                sent <= sent + 5'd1;
            end
        end
    end

    // ---- Next-state logic ----
    always_comb begin
        next = state;
        case (state)
            S_IDLE:  if (resend_pending || tx_msg_valid) next = S_REQ;
            S_REQ:   next = S_WAITB;                   // byte_send strobed
            S_WAITB: if (byte_done) next = last_byte ? S_DONE : S_REQ;
            S_DONE:  next = S_GAP;                     // completion pulse strobed
            S_GAP:   begin
                // RESEND: single gap tick - uart_rgf's registered clear of
                // resend_pending lands during it, so S_IDLE samples the post-
                // clear value. Waiting for pending LOW would deadlock if a NEW
                // error re-arms it inside the gap; after this tick a still-high
                // pending is a new episode and correctly earns another {E}.
                if (src_q == SRC_RESEND) next = S_IDLE;
                // REPLY: wait for the granted source to drop its valid (it
                // does so one tick after tx_msg_done, guaranteed by the
                // consumer/APB FSMs) so the same message is never re-latched.
                else if (!tx_msg_valid)  next = S_IDLE;
            end
            default: next = S_IDLE;
        endcase
    end

    // ---- Outputs ----
    assign tx_byte     = msg_q[8*idx +: 8];            // MSByte-first walk
    assign byte_send   = (state == S_REQ);             // 1 tick per byte
    assign tx_msg_done = (state == S_DONE) && (src_q == SRC_REPLY);
    assign resend_sent = (state == S_DONE) && (src_q == SRC_RESEND);
    assign tx_busy     = (state != S_IDLE);

endmodule
