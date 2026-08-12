`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: UART_Rx
// Description: RX front end = rx_phy + rx_mac (organisational wrapper, no logic).
//   Integrates the physical layer and the framing layer per rx_phy_spec_v1 /
//   rx_mac_spec_v1 and exposes the framed message to the chiptop rx_parser /
//   rx_classifier. The parser/classifier/uart_rgf live at chiptop, so msg_taken
//   and flush_rx enter here as inputs and the error reports leave as outputs.
//   (chiptop_spec_v1 lists rx_phy and rx_mac as separate instances; this wrapper
//   is optional packaging and may be bypassed by instantiating them directly.)
//
//   Internal PHY<->MAC connection (the "spec-to-spec" integration):
//     phy.rx_byte    -> mac.rx_byte       phy.byte_valid -> mac.byte_valid
//     phy.phy_error  -> mac.phy_error     mac.rts        -> phy.mac_ready
//   flush_rx (uart_rgf) fans out to BOTH layers (error_resend_flow_v1).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module UART_Rx (
    // ---- Clocking / control (from chiptop) ----
    input  logic clk,                      // mmcm_clk physical clock
    input  logic reset_n,                  // external sync-reset module
    input  logic enable,                   // gclk = baud*16 clock-enable

    // ---- External line + handshakes ----
    input  logic rx_line,                  // raw serial input from PC
    input  logic msg_taken,                // <- rx_classifier: release the held frame
    input  logic flush_rx,                 // <- rx_resend_ctrl: re-sync after {E}
    input  logic parity_odd,               // <- uart_rgf: runtime parity mode

    // ---- To PC (chiptop pin) ----
    output logic rts,                      // -> RTS pad -> PC CTS

    // ---- To rx_parser / rx_classifier (chiptop) ----
    output logic [D_SIZE-1:0] msg_buf [MSG_LEN-1:0], // framed 16-byte message
    output logic [1:0]        num_submsg,  // valid-byte span (1/2/3)
    output logic              is_msg,      // structurally valid frame present

    // ---- Distinct error events (v2) -> uart_rgf + rx_resend_ctrl ----
    output logic phy_error,                // 1-tick: parity/framing/stop error
    output logic mac_err                   // 1-tick: frame-structure error
);

    // ---- PHY <-> MAC nets ----
    logic [D_SIZE-1:0] phy_byte;           // phy.rx_byte    -> mac.rx_byte
    logic              phy_byte_valid;     // phy.byte_valid -> mac.byte_valid
    logic              mac_ready;          // mac.rts        -> phy.mac_ready

    // ---- Physical layer ----
    rx_phy u_phy (
        .clk        (clk),
        .reset_n    (reset_n),
        .enable     (enable),
        .rx_line    (rx_line),
        .mac_ready  (mac_ready),           // backpressure from MAC
        .flush_rx   (flush_rx),
        .parity_odd (parity_odd),
        .rx_byte    (phy_byte),
        .byte_valid (phy_byte_valid),
        .phy_error  (phy_error),           // flush directive + error report
        .rts        (rts)                  // physical RTS pin
    );

    // ---- Framing layer ----
    rx_mac u_mac (
        .clk        (clk),
        .reset_n    (reset_n),
        .enable     (enable),
        .rx_byte    (phy_byte),
        .byte_valid (phy_byte_valid),
        .phy_error  (phy_error),           // phy error flushes the framer
        .flush_rx   (flush_rx),
        .msg_taken  (msg_taken),
        .msg_buf    (msg_buf),
        .num_submsg (num_submsg),
        .is_msg     (is_msg),
        .rts        (mac_ready),           // ready/hold request to PHY
        .mac_err    (mac_err)
    );

endmodule
