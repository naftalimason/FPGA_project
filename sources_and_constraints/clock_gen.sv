`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: clock_gen
// Description: Project clock generation (clock_gen_spec_v1). Produces:
//     mmcm_clk    - PC-side physical clock (~280 MHz via MMCM; the glitchless
//                   BUFGMUX can fall back to the 100 MHz board clock, clk_sel)
//     clk         - 100 MHz SRAM/AHB physical clock (board clock pass-through;
//                   XDC: CLK100MHZ pin E3, 10.00 ns)
//     gclk        - baud*16 clock-ENABLE riding on mmcm_clk (from the protected
//                   baud_rate_16th Bresenham generator; ratio comes from
//                   UART_package BAUD_16_NUM/DEN) - never a gated clock
//     mmcm_locked - MMCM lock status (mirrored in img_rgf CLK_CTRL)
//
//   SYNTHESIS PATH: instantiates the Vivado clk_wiz_0 IP (280 MHz, per the XDC
//   generated-clock constraints) and the protected glitchless_mux (BUFGMUX).
//   SIMULATION PATH (`VERILATOR` define, set automatically by Verilator): a
//   behavioral ~280 MHz oscillator + simple mux + delayed lock model, so the
//   whole chip simulates with real asynchronous clock ratios.
//
//   NOTE: clk_sel=0 (100 MHz PC side) is a legacy fallback; the 8 Mbaud UART
//   requires the fast clock (baud*16 = 128 MHz > 100 MHz), so the system
//   default is clk_sel=1 (img_rgf CLK_CTRL resets to 1).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module clock_gen (
    input  logic ext_clk,                 // 100 MHz board oscillator
    input  logic ext_rst_n,               // board reset (async, active low)
    input  logic clk_sel,                 // 1 = fast MMCM clock (default)
    output logic mmcm_clk,                // PC-side physical clock
    output logic clk,                     // 100 MHz SRAM/AHB clock
    output logic gclk,                    // baud*16 clock-enable on mmcm_clk
    output logic mmcm_locked,
    output logic clk_sel_stat             // ACTUAL mux selection (1 = fast) -
                                          //   RO mirror in uart_rgf (v2)
);

    assign clk = ext_clk;                 // SRAM/AHB side runs on the board clock

    logic clk_fast;
    logic clk_sel_safe;
    assign clk_sel_safe = clk_sel & mmcm_locked;
    assign clk_sel_stat = clk_sel_safe;

`ifdef VERILATOR
    // ---- behavioral simulation model ----
    logic clk280 = 1'b0;
    always #1.786 clk280 = ~clk280;       // ~280 MHz (3.572 ns period)
    assign clk_fast = clk280;

    // lock rises a few board-clock cycles after reset release
    logic [3:0] lock_cnt = '0;
    always @(posedge ext_clk or negedge ext_rst_n) begin
        if (!ext_rst_n)            lock_cnt <= '0;
        else if (!(&lock_cnt))     lock_cnt <= lock_cnt + 4'd1;
    end
    assign mmcm_locked = &lock_cnt;

    // simple behavioral clock mux (BUFGMUX equivalent for sim)
    assign mmcm_clk = clk_sel_safe ? clk_fast : ext_clk;
`else
    // ---- synthesis path: Vivado MMCM IP + protected glitchless BUFGMUX ----
    clk_wiz_0 u_clk_wiz_0 (
        .clk_in1  (ext_clk),
        .reset    (~ext_rst_n),
        .clk_out1 (clk_fast),             // 280 MHz (XDC: clk_out1_clk_wiz_0)
        .locked   (mmcm_locked)
    );

    glitchless_mux u_glitchless_mux (
        .clk0    (ext_clk),
        .clk1    (clk_fast),
        .rst_n   (ext_rst_n),
        .sel     (clk_sel_safe),
        .clk_out (mmcm_clk)
    );
`endif

    // ---- gclk = baud*16 enable (protected Bresenham generator) ----
    // reset qualified by lock so the phase starts clean once mmcm_clk is stable
    baud_rate_16th u_baud16_gen (
        .reset_n       (ext_rst_n & mmcm_locked),
        .sys_clk       (mmcm_clk),
        .en_16th_clk   (1'b1),
        .baud_16th_clk (gclk)
    );

endmodule
