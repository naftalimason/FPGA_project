`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_phy_fsm
// Description: UART RX frame recovery FSM (rx_phy_spec_v1 sec 4). Walks the frame
//   IDLE / START / DATA / PARITY / STOP / CLEANUP using the bit-timer midpoint
//   strobe, at 16x oversampling. Pure sequencing: the datapath (rx_phy) does the
//   parity check and output gating.
//
//   flush_rx (from uart_rgf, via rx_phy) aborts the frame in progress at ANY
//   point and returns to IDLE, ready for the PC's resent frame after a resend
//   {E} notification (error_resend_flow_v1). Functional flush, NOT a reset.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_phy_fsm (
    // ---- Clocking (from rx_phy) ----
    input  logic clk,                                 // mmcm_clk physical clock
    input  logic reset_n,                             // external sync-reset module
    input  logic enable,                              // gclk = baud*16 clock-enable

    // ---- Re-sync directive (from uart_rgf, via rx_phy) ----
    input  logic flush_rx,                            // abandon frame -> IDLE

    // ---- From differentiator (3-FF line sync / de-glitch) ----
    input  logic zero_detect,                         // falling edge = start-bit onset
    input  logic rx_stable,                           // synced line is glitch-free
    input  logic rx_val,                              // synced line level

    // ---- From baud_ctr (bit timer) ----
    input  logic midpoint,                            // sample strobe at count_threshold

    // ---- To baud_ctr (bit timer) ----
    output logic clr_baud_ctr,                        // restart the timer at start onset
    output logic en_16th_clk,                         // enable 16x generator (tied high)
    output logic [RX_MIDPOINT_WIDTH-1:0] count_threshold, // sample point for the phase

    // ---- To rx_shift_reg (deserializer) ----
    output logic shift_en,                            // shift in one data bit

    // ---- To rx_phy datapath ----
    output logic parity_en,                           // capture the received parity bit
    output logic byte_done,                           // 1-tick: frame finished (good or bad)
    output logic stop_ok,                             // stop bit was valid (qualifies byte_done)
    output logic active                               // frame in progress (RTS boundary timing)
);

    typedef enum logic [2:0] {
        IDLE, START, DATA, PARITY, STOP, CLEANUP, RESYNC
    } state_t;

    state_t     state, next;
    logic [2:0] bit_cnt;                              // 0..7 data bit index
    logic       stop_ok_r;                            // latched stop-bit result
    logic       zero_seen_r;                          // start edge seen during CLEANUP
                                                      //   (back-to-back frames: the next
                                                      //   byte's start can fall inside the
                                                      //   cleanup tail - latch, don't lose it)
    logic [RX_RESYNC_CNT_W-1:0] quiet_cnt;            // post-flush idle-high counter

    // 16x generator always running; enable is the real sample gate.
    assign en_16th_clk = 1'b1;
    assign active      = (state != IDLE);
    assign stop_ok     = stop_ok_r;

    // ---- Sequential: state, bit counter, stop-bit capture ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= IDLE;
            bit_cnt     <= '0;
            stop_ok_r   <= 1'b0;
            zero_seen_r <= 1'b0;
        end
        else if (enable) begin
            // flush anytime -> RESYNC: skip whatever is still arriving (the
            // tail of the errored frame) and re-arm only from a clean idle
            state <= flush_rx ? RESYNC : next;

            if (state == IDLE)
                bit_cnt <= '0;
            else if (state == DATA && midpoint && rx_stable)
                bit_cnt <= bit_cnt + 1'b1;

            if (state == STOP && midpoint)
                stop_ok_r <= (rx_stable && rx_val == 1'b1);

            if (state == CLEANUP && !flush_rx)
                zero_seen_r <= zero_seen_r | zero_detect;
            else
                zero_seen_r <= 1'b0;

            // RESYNC: count continuous idle-high ticks; any low restarts
            if (state == RESYNC && rx_stable && rx_val)
                quiet_cnt <= quiet_cnt + 1'b1;
            else
                quiet_cnt <= '0;
        end
    end

    // ---- Combinational: next state + phase controls ----
    always_comb begin
        next            = state;
        clr_baud_ctr    = 1'b0;
        count_threshold = RX_FULL_THRESHOLD;
        shift_en        = 1'b0;
        parity_en       = 1'b0;
        byte_done       = 1'b0;

        case (state)
            IDLE: begin
                if (zero_detect) begin
                    clr_baud_ctr = 1'b1;             // arm timer on start onset
                    next         = START;
                end
            end

            START: begin
                count_threshold = RX_START_THRESHOLD; // sample near mid of start cell
                if (midpoint)
                    next = (rx_stable && rx_val == 1'b0) ? DATA : IDLE; // confirm real 0
            end

            DATA: begin
                if (midpoint) begin
                    if (rx_stable) begin
                        shift_en = 1'b1;             // capture data bit (LSB first)
                        if (bit_cnt == 3'(D_SIZE-1))
                            next = PARITY;
                    end
                    else next = IDLE;               // glitch -> abort
                end
            end

            PARITY: begin
                if (midpoint) begin
                    if (rx_stable) begin
                        parity_en = 1'b1;           // latch received parity bit
                        next      = STOP;
                    end
                    else next = IDLE;
                end
            end

            STOP: begin
                if (midpoint)
                    next = CLEANUP;                 // result captured in stop_ok_r
            end

            CLEANUP: begin
                count_threshold = RX_CLEANUP_THRESHOLD; // short tail for back-to-back frames
                if (midpoint) begin
                    byte_done = !flush_rx;          // one strobe per completed frame
                    // Back-to-back: if the next byte's start edge already fell
                    // inside the cleanup tail, dispatch straight to START (re-arm
                    // the bit timer) instead of returning to IDLE and losing it.
                    if (zero_seen_r || zero_detect) begin
                        clr_baud_ctr = 1'b1;
                        next         = START;
                    end
                    else next = IDLE;               // (flush this tick -> IDLE override)
                end
            end

            RESYNC: begin
                // no sampling, no byte_done, no error reports: deliberately
                // blind until one full character time of idle-high line
                if (quiet_cnt >= RX_RESYNC_CNT_W'(RX_RESYNC_TICKS - 1))
                    next = IDLE;
            end

            default: next = IDLE;
        endcase
    end

endmodule
