`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: clk_wiz_0 (simulation stub)
// Description: Behavioural black box for the Vivado clk_wiz_0 MMCM IP (280 MHz
//   from the 100 MHz board clock). Under Verilator the DUT clock_gen compiles
//   its own `VERILATOR` behavioural model instead, so this stub exists only so
//   the file list stays complete/portable to other simulators.
//////////////////////////////////////////////////////////////////////////////////

module clk_wiz_0 (
    input  wire clk_in1,
    input  wire reset,
    output wire clk_out1,
    output reg  locked
);

    reg clk280 = 1'b0;
    always #1.786 clk280 = ~clk280;      // ~280 MHz
    assign clk_out1 = clk280;

    reg [3:0] lock_cnt = '0;
    always @(posedge clk_in1 or posedge reset) begin
        if (reset)             lock_cnt <= '0;
        else if (!(&lock_cnt)) lock_cnt <= lock_cnt + 4'd1;
    end
    always @* locked = &lock_cnt;

endmodule
