`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_mac_fsm
// Description: Framing control FSM for rx_mac (rx_mac_spec_v1 sec 4). Pure Moore:
//   its outputs (collect_en / is_msg / rts) depend on state only. The datapath in
//   rx_mac uses these plus the byte comparators to gate buffer writes and the
//   k-pointer. Three states: HUNT (seek '{'), COLLECT (gather payload + check
//   boundaries), HOLD (freeze a complete frame until released).
//
//   Two release paths out of HOLD (rx_mac_spec sec 4/8):
//     msg_taken - normal release by the classifier (frame was sampled).
//     flush_rx  - content-reject path from uart_rgf: the classifier did NOT
//                 sample; the {E} resend request was sent to the PC and the held
//                 frame is dropped so the PC can resend (no HOLD deadlock).
//   phy_error / flush_rx at ANY state -> HUNT (functional flush, never a reset).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_mac_fsm (
    // ---- Clocking (from rx_mac) ----
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- Frame events (from rx_mac datapath) ----
    input  logic byte_valid,              // 1-tick: a byte is available to accept
    input  logic is_start,                // rx_byte == '{'
    input  logic is_end,                  // rx_byte == '}'
    input  logic is_comma,                // rx_byte == ','
    input  logic at_boundary,             // k in {10,5,0}
    input  logic at_last,                 // k == 0 (must be '}')

    // ---- Flush directives ----
    input  logic phy_error,               // rx_phy: parity/framing -> flush + re-hunt
    input  logic flush_rx,                // uart_rgf: drop held/partial frame -> HUNT

    // ---- Handshake (from rx_classifier, via chiptop) ----
    input  logic msg_taken,               // release the held frame (normal path)

    // ---- Moore outputs (state only) ----
    output logic collect_en,              // -> datapath: accept payload bytes
    output logic is_msg,                  // -> rx_parser/rx_classifier: frame present
    output logic rts                      // -> rx_phy.mac_ready: low = pause (HOLD)
);

    typedef enum logic [1:0] { HUNT, COLLECT, HOLD } state_t;
    state_t state, next;

    // Boundary byte is legal only as ',' (not at k=0) or '}'.
    logic boundary_ok;
    assign boundary_ok = is_end || (is_comma && !at_last);

    // ---- State register (phy_error / flush_rx override -> immediate re-sync) ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)          state <= HUNT;
        else if (enable)       state <= (phy_error || flush_rx) ? HUNT : next;
    end

    // ---- Next-state logic ----
    always_comb begin
        next = state;
        case (state)
            HUNT: begin
                if (byte_valid && is_start) next = COLLECT;
            end

            COLLECT: begin
                if (byte_valid && at_boundary) begin
                    if (is_end)            next = HOLD;      // frame complete
                    else if (boundary_ok)  next = COLLECT;   // ',' -> keep collecting
                    else                   next = HUNT;      // framing/overrun error
                end
            end

            HOLD: begin
                if (msg_taken)             next = HUNT;      // classifier consumed it
            end

            default: next = HUNT;
        endcase
    end

    // ---- Moore outputs ----
    assign collect_en = (state == COLLECT);
    assign is_msg     = (state == HOLD);
    assign rts        = (state != HOLD);   // hold backpressure while a frame waits

endmodule
