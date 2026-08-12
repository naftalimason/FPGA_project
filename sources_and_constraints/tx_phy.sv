`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tx_phy
// Description: UART TX physical layer (tx_phy_spec_v1) - bit-level mirror of
//   rx_phy. Takes one byte at a time from tx_mac and shifts it onto tx_line:
//   START / 8 data LSB-first / ODD parity / STOP, one bit = 16 enable ticks
//   (gclk = baud*16, same time base as the RX oversampler). Honours the PC's
//   flow control on cts, sampled ONLY at byte boundaries (idle) so a character
//   in progress always finishes cleanly.
//
//   byte_send is latched into a pending flag (with the byte), so a request
//   issued while cts is low is not lost - transmission starts as soon as cts
//   returns high. byte_done pulses one tick at the end of the stop bit; tx_mac
//   advances one byte per byte_done. Parity MUST match rx_phy (PARITY_ODD from
//   UART_package). Clocking: clk = mmcm_clk; enable = gclk clock-enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module tx_phy (
    // ---- Clocking ----
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- From/to tx_mac ----
    input  logic [D_SIZE-1:0] tx_byte,    // byte to transmit
    input  logic              byte_send,  // 1-tick: send tx_byte (latched here)
    output logic              byte_done,  // 1-tick: byte fully shifted (post-stop)
    output logic              tx_busy,    // a character is in flight / pending

    // ---- Runtime parity mode (from uart_rgf; v2 change 3) ----
    input  logic parity_odd,              // 1 = odd (captured with each byte)

    // ---- PC flow control / line ----
    input  logic cts,                     // PC RTS: high = PC can accept
    output logic tx_line                  // serial output (idles high)
);

    typedef enum logic [2:0] { P_IDLE, P_START, P_DATA, P_PARITY, P_STOP } state_t;
    state_t state, next;

    logic [D_SIZE-1:0] byte_q;            // latched byte
    logic              pending;           // a byte is waiting to start
    logic [3:0]        tick_cnt;          // 16 enable ticks per bit
    logic [2:0]        bit_cnt;           // data bit index (LSB first)
    logic              parity_q;          // computed parity of byte_q

    logic bit_end;
    assign bit_end = (tick_cnt == 4'd15);

    // ---- Request latch (survives cts-low waits) ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            byte_q   <= '0;
            parity_q <= 1'b0;
            pending  <= 1'b0;
        end
        else if (enable) begin
            if (byte_send) begin
                byte_q   <= tx_byte;
                // parity captured per byte from the runtime mode (v2):
                // odd => total ones (data+parity) odd; even => even
                parity_q <= parity_odd ? ~(^tx_byte) : (^tx_byte);
                pending  <= 1'b1;
            end
            else if (state == P_IDLE && pending && cts)
                pending <= 1'b0;          // request consumed - frame starts
        end
    end

    // ---- State / counters ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state    <= P_IDLE;
            tick_cnt <= '0;
            bit_cnt  <= '0;
        end
        else if (enable) begin
            state <= next;
            if (state == P_IDLE) begin
                tick_cnt <= '0;
                bit_cnt  <= '0;
            end
            else begin
                tick_cnt <= bit_end ? 4'd0 : tick_cnt + 4'd1;
                if (state == P_DATA && bit_end)
                    bit_cnt <= bit_cnt + 3'd1;
            end
        end
    end

    // ---- Next-state (bit boundaries every 16 ticks) ----
    always_comb begin
        next = state;
        case (state)
            P_IDLE:   if (pending && cts) next = P_START; // cts: boundary only
            P_START:  if (bit_end)        next = P_DATA;
            P_DATA:   if (bit_end && bit_cnt == 3'd7) next = P_PARITY;
            P_PARITY: if (bit_end)        next = P_STOP;
            P_STOP:   if (bit_end)        next = P_IDLE;
            default:  next = P_IDLE;
        endcase
    end

    // ---- Line output (Moore per phase; idles high) ----
    always_comb begin
        case (state)
            P_START:  tx_line = 1'b0;
            P_DATA:   tx_line = byte_q[bit_cnt];   // LSB first
            P_PARITY: tx_line = parity_q;
            default:  tx_line = 1'b1;              // idle / stop
        endcase
    end

    // ---- Completion / status ----
    assign byte_done = (state == P_STOP) && bit_end;   // 1-tick, end of stop bit
    assign tx_busy   = (state != P_IDLE) || pending;

endmodule
