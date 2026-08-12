`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_phy
// Description: UART RX physical layer (rx_phy_spec_v1). Recovers the UART frame
//   on rx_line (start / 8 data / ODD parity / stop) at 16x oversampling,
//   deserializes LSB-first into a byte, checks odd parity, and presents clean
//   bytes to rx_mac with a 1-tick byte_valid. Owns the physical RTS pin, driven
//   from rx_mac's ready request and updated only at byte boundaries (sec 6).
//   flush_rx (from uart_rgf) aborts any frame in progress after a resend {E}
//   was sent to the PC (sec 4/8, error_resend_flow_v1).
//
//   Fixed sub-blocks (unchanged): differentiator (3-FF line sync / de-glitch),
//   baud_ctr (bit timer), rx_shift_reg (deserializer). Sequencing: rx_phy_fsm.
//   Clocking: clk = mmcm_clk (PC side); enable = gclk = baud*16 clock-enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_phy (
    // ---- Clocking (from chiptop) ----
    input  logic clk,                     // mmcm_clk physical clock (~280 MHz)
    input  logic reset_n,                 // reset (external sync-reset module)
    input  logic enable,                  // gclk = baud*16 clock-enable (sample tick)

    // ---- Line + flow control ----
    input  logic rx_line,                 // raw serial input from PC (idles high)
    input  logic mac_ready,               // <- rx_mac.rts : downstream ready/hold request

    // ---- Re-sync (from rx_resend_ctrl) ----
    input  logic flush_rx,                // abort frame hunt -> RESYNC (after {E})

    // ---- Runtime parity mode (from uart_rgf; v2 change 3) ----
    input  logic parity_odd,              // 1 = odd (latched at frame start)

    // ---- To rx_mac ----
    output logic [D_SIZE-1:0] rx_byte,    // assembled data byte (LSB-first)
    output logic              byte_valid, // 1-tick: byte valid AND parity good
    output logic              phy_error,  // 1-tick: parity/framing error (flush
                                          //   directive to rx_mac + error report;
                                          //   NOT a reset - no soft_reset exists)

    // ---- To PC (chiptop pin) ----
    output logic              rts         // -> RTS pad -> PC CTS (assert = OK to send)
);

    // ---- Internal nets ----
    logic zero_detect, rx_stable, rx_val;             // from differentiator
    logic midpoint;                                   // from baud_ctr
    logic clr_baud, en_16th_clk;                      // to/from fsm
    logic [RX_MIDPOINT_WIDTH-1:0] threshold;
    logic shift_en, parity_en;
    logic byte_done, stop_ok, active;                 // from fsm
    logic rx_parity_bit;                              // captured parity bit
    logic parity_bad;
    logic parity_mode_q;                              // mode latched per frame

    // ---- Datapath sub-blocks (baud_16th tick == enable) ----
    differentiator diff (
        .rx_line       (rx_line),
        .reset_n       (reset_n),
        .sys_clk       (clk),
        .enable        (enable),
        .baud_16th_clk (enable),
        .zero_detect   (zero_detect),
        .rx_stable     (rx_stable),
        .rx_val        (rx_val)
    );

    baud_ctr timer (
        .reset_n         (reset_n),
        .sys_clk         (clk),
        .enable          (enable),
        .baud_16th_clk   (enable),
        .clr             (clr_baud),
        .count_threshold (threshold),
        .midpoint        (midpoint)
    );

    rx_phy_fsm ctrl (
        .clk             (clk),
        .reset_n         (reset_n),
        .enable          (enable),
        .flush_rx        (flush_rx),
        .zero_detect     (zero_detect),
        .rx_stable       (rx_stable),
        .rx_val          (rx_val),
        .midpoint        (midpoint),
        .clr_baud_ctr    (clr_baud),
        .en_16th_clk     (en_16th_clk),
        .count_threshold (threshold),
        .shift_en        (shift_en),
        .parity_en       (parity_en),
        .byte_done       (byte_done),
        .stop_ok         (stop_ok),
        .active          (active)
    );

    rx_shift_reg rx_storage (
        .sys_clk  (clk),
        .reset_n  (reset_n),
        .enable   (enable),
        .rx_val   (rx_val),
        .shift_en (shift_en),
        .rx_byte  (rx_byte)
    );

    // ---- Parity capture + check (runtime mode, v2: uart_rgf drives it;
    //      latched when the frame STARTS so it cannot change mid-frame) ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_parity_bit <= 1'b0;
            parity_mode_q <= PARITY_ODD;              // package reset default
        end
        else if (enable) begin
            if (!active)       parity_mode_q <= parity_odd;  // frame boundary
            if (parity_en)     rx_parity_bit <= rx_val;
        end
    end

    // XOR of all 9 bits: 1 => odd count of ones. Odd parity is good when it is 1.
    assign parity_bad = parity_mode_q ? ~(^{rx_byte, rx_parity_bit})
                                      :  (^{rx_byte, rx_parity_bit});

    // ---- Output gating (rx_phy_spec sec 5) ----
    assign phy_error  = byte_done && ( parity_bad || !stop_ok);
    assign byte_valid = byte_done && !parity_bad &&  stop_ok;

    // ---- RTS flow control (rx_phy_spec sec 6) ----
    //   rts_ready = mac_ready (no local RX FIFO - the frame held in rx_mac is the
    //   slack). Registered, glitch-free, updated only at a byte boundary (while
    //   no frame is in progress) so the character on the wire finishes cleanly.
    //   Reset/idle = not-ready (0). Pin polarity is set in the wrapper/XDC.
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)          rts <= 1'b0;
        else if (enable)
            if (!active)       rts <= mac_ready;      // sample only between frames
    end

endmodule
