`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 04/24/2026 10:42:57 PM
// Design Name:
// Module Name: baud_rate_16th
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   Parameterized 16x UART RX baud tick generator.
//
//   The tick rate is derived from parameters in UART_package.sv.
//   This module does not contain fixed baud-rate numbers.
//
//   Algorithm:
//     This module uses a Bresenham / phase-accumulator style divider.
//     The package reduces the desired tick ratio at compile time:
//
//         UART_RX_TICK_RATE / UART_CLK_FREQ = BAUD_16_NUM / BAUD_16_DEN
//
//     On every sys_clk cycle, BAUD_16_NUM is added into acc.
//     When acc reaches BAUD_16_DEN, one baud_16th_clk enable pulse is created
//     and BAUD_16_DEN is subtracted back out of acc.
//
//     Therefore the generator creates exactly BAUD_16_NUM pulses every
//     BAUD_16_DEN sys_clk cycles, evenly distributed over time.
//
//     The exact numeric ratio is not written here. It comes only from
//     UART_package.sv through BAUD_16_NUM and BAUD_16_DEN.
//
//   Important:
//     baud_16th_clk is not a real clock. It is a one-cycle clock-enable pulse.
//     No generated clock is routed through normal FPGA data fabric.
//
// Dependencies:
//   UART_package.sv
//
// Revision:
// Revision 0.03 - Reduced-ratio Bresenham tick generator
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module baud_rate_16th(
    input  logic reset_n,
    input  logic sys_clk,
    input  logic en_16th_clk,
    // max_fanout: SYNTHESIS-ONLY physical directive (no logic change) - the
    // enable pulse fans out to ~300 loads chip-wide and its distribution is
    // inherently single-cycle (cannot be multicycle-relaxed), so synthesis
    // must replicate this register to close 3.571 ns on default strategies.
    // XDC MAX_FANOUT (cell or net form) is silently dropped by 2025.2 synth;
    // the RTL attribute is the documented reliable form (UG901).
    (* max_fanout = 40 *) output logic baud_16th_clk
);

    // Reduced-ratio accumulator.
    // Width comes from UART_package.sv and is based on BAUD_16_DEN/BAUD_16_NUM.
    logic [BAUD_16_ACC_WIDTH-1:0] acc;
    logic [BAUD_16_ACC_WIDTH-1:0] acc_next;

    // Combinational next sum:
    // acc_next represents the fractional phase after adding one tick step.
    // If acc_next crosses BAUD_16_DEN, the output enable pulses for one cycle.
    always_comb begin
        acc_next = acc + BAUD_16_NUM;
    end

    always_ff @(posedge sys_clk or negedge reset_n) begin
        if (!reset_n) begin
            acc           <= '0;
            baud_16th_clk <= 1'b0;
        end
        else if (!en_16th_clk) begin
            // Keep the generator aligned to a clean starting phase when disabled.
            acc           <= '0;
            baud_16th_clk <= 1'b0;
        end
        else if (acc_next >= BAUD_16_DEN) begin
            // Threshold crossed:
            // generate one 16x baud enable pulse and keep the remainder.
            acc           <= acc_next - BAUD_16_DEN;
            baud_16th_clk <= 1'b1;
        end
        else begin
            // No pulse this cycle:
            // keep accumulating fractional phase.
            acc           <= acc_next;
            baud_16th_clk <= 1'b0;
        end
    end

endmodule
