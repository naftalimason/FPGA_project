`timescale 1ns / 1ps


module glitchless_mux (
    input  logic clk0,
    input  logic clk1,
    input  logic rst_n,
    input  logic sel,      // 0 = clk0, 1 = clk1
    output logic clk_out
);

    // rst_n is kept for interface compatibility.
    // BUFGMUX performs the clock switch inside the FPGA clocking fabric.

    BUFGMUX #(
        .CLK_SEL_TYPE("SYNC")
    ) u_bufgmux (
        .O  (clk_out),
        .I0 (clk0),
        .I1 (clk1),
        .S  (sel)
    );

endmodule
