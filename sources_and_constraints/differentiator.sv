`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/24/2026 10:07:22 PM
// Design Name: 
// Module Name: differentiator
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module differentiator(
    input logic rx_line,
    input logic reset_n,
    input logic sys_clk,
    input logic enable,
    input logic baud_16th_clk,
    output logic zero_detect,
    output logic rx_stable,
    output logic rx_val
    );

    logic diff1, diff2, diff3;
    
    always_ff @(posedge sys_clk, negedge reset_n)
            if (!reset_n) begin
                diff1 <= 1'b1;
                diff2 <= 1'b1;
                diff3 <= 1'b1;
            end
            else if (enable) begin
                if (baud_16th_clk) begin
                    diff1 <= rx_line;
                    diff2 <= diff1;
                    diff3 <= diff2;
                end
            end            
                    
    assign zero_detect = ~diff1 & diff2;
    assign rx_stable = (diff1 == diff2) && (diff2 == diff3);
    assign rx_val = diff2;
    
endmodule