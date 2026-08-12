`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_mac
// Description: UART RX framing layer (rx_mac_spec_v1). Accepts parity-good bytes
//   from rx_phy and assembles them, top-aligned, into a 16-byte frame buffer:
//   '{' lands at msg_buf[15], each following byte at the next lower index via a
//   down-counter k. Boundary slots k in {10,5,0} must hold ',' or '}'. A complete
//   frame is held (rts low) until the classifier acks with msg_taken, or until
//   uart_rgf pulses flush_rx on a content reject (resend flow - sec 8.4).
//   Structural framing only - no opcode/value decode (that is rx_parser's role).
//
//   Error reporting (rx_mac_spec sec 8.3):
//     mac_err    = 1-tick, LOCAL frame-structure/overrun error only.
//     rx_err_any = 1-tick, mac_err OR passed-through phy_error - the OR that
//                  arms the uart_rgf resend/flush flow.
//   Control lives in rx_mac_fsm (pure Moore); this file is the datapath.
//   Clocking: clk = mmcm_clk; enable = gclk = baud*16 clock-enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_mac (
    // ---- Clocking (from chiptop) ----
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- From rx_phy ----
    input  logic [D_SIZE-1:0] rx_byte,    // data byte
    input  logic              byte_valid, // 1-tick accept strobe (parity good)
    input  logic              phy_error,  // 1-tick flush directive (parity/frame
                                          //   error; functional flush, NOT a reset)

    // ---- From uart_rgf (resend flow) ----
    input  logic              flush_rx,   // drop held/partial frame -> HUNT

    // ---- From rx_classifier (via chiptop) ----
    input  logic              msg_taken,  // release the held frame (normal path)

    // ---- To rx_parser / rx_classifier ----
    output logic [D_SIZE-1:0] msg_buf [MSG_LEN-1:0], // 16-byte frame, msg_buf[15]='{'
    output logic [1:0]        num_submsg, // 1 / 2 / 3 (valid-byte span)
    output logic              is_msg,     // structurally valid frame present

    // ---- Flow control / errors ----
    output logic              rts,        // -> rx_phy.mac_ready : low = pause
    output logic              mac_err     // -> uart_rgf + rx_resend_ctrl :
                                          //    1-tick frame-structure error ONLY
                                          //    (v2: phy errors are reported by
                                          //    rx_phy directly - no shared
                                          //    rx_err_any, no reconstruction)
);

    // ---- Write pointer (down counter 15..0) ----
    localparam int unsigned K_W = $clog2(MSG_LEN);
    logic [K_W-1:0] k;

    // ---- Byte comparators + pointer flags ----
    logic is_start, is_end, is_comma, at_boundary, at_last;
    assign is_start    = (rx_byte == START_BYTE); // '{'
    assign is_end      = (rx_byte == END_BYTE);   // '}'
    assign is_comma    = (rx_byte == CHAR_COMMA); // ','
    assign at_boundary = (k == 4'd10) || (k == 4'd5) || (k == 4'd0);
    assign at_last     = (k == 4'd0);

    // ---- Control FSM (pure Moore) ----
    logic collect_en;
    rx_mac_fsm u_fsm (
        .clk        (clk),
        .reset_n    (reset_n),
        .enable     (enable),
        .byte_valid (byte_valid),
        .is_start   (is_start),
        .is_end     (is_end),
        .is_comma   (is_comma),
        .at_boundary(at_boundary),
        .at_last    (at_last),
        .phy_error  (phy_error),
        .flush_rx   (flush_rx),
        .msg_taken  (msg_taken),
        .collect_en (collect_en),
        .is_msg     (is_msg),
        .rts        (rts)
    );

    // ---- Accept / store / flush decode (datapath gates) ----
    logic hunt, boundary_ok, accept_start, store_collect, terminating, framing_err, flush;
    assign hunt          = !collect_en && !is_msg;                    // HUNT state
    assign boundary_ok   = is_end || (is_comma && !at_last);          // legal boundary byte
    assign accept_start  = hunt && byte_valid && is_start;            // '{' -> buf[15]
    assign terminating   = collect_en && byte_valid && at_boundary && is_end; // '}' ends frame
    assign store_collect = collect_en && byte_valid && (!at_boundary || boundary_ok);
    assign framing_err   = collect_en && byte_valid && at_boundary && !boundary_ok;
    assign flush         = phy_error || flush_rx || framing_err || (is_msg && msg_taken);

    // ---- Error reporting (v2): local frame-structure errors ONLY ----
    assign mac_err = framing_err;

    // ---- Write pointer update ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)                          k <= K_W'(MSG_LEN - 1); // 15
        else if (enable) begin
            if (flush)                         k <= K_W'(MSG_LEN - 1);
            else if (accept_start)             k <= K_W'(MSG_LEN - 2); // 14
            else if (store_collect && !terminating) k <= k - 1'b1;
        end
    end

    // ---- Frame buffer (indexed write - low toggle power vs. a 128b shifter) ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            for (int i = 0; i < MSG_LEN; i++) msg_buf[i] <= '0;
        end
        else if (enable) begin
            if (accept_start)       msg_buf[MSG_LEN-1] <= rx_byte;    // '{' -> buf[15]
            else if (store_collect) msg_buf[k]         <= rx_byte;
        end
    end

    // ---- Sub-message count, latched at the closing '}' ----
    //   Terminating k -> span: k=10 -> 1, k=5 -> 2, k=0 -> 3 (rx_mac_spec sec 3).
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)          num_submsg <= 2'd0;
        else if (enable)
            if (terminating)   num_submsg <= (k == 4'd10) ? 2'd1 :
                                             (k == 4'd5)  ? 2'd2 : 2'd3;
    end

endmodule
