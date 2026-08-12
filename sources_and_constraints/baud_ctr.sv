`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/24/2026 11:01:34 PM
// Design Name: 
// Module Name: baud_ctr
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

module baud_ctr(
    input logic reset_n,       
    input logic sys_clk,       
    input logic enable,
    input logic baud_16th_clk, 
    input logic clr,           // from fsm to reset counter after 0 detection
    input logic [RX_MIDPOINT_WIDTH-1:0] count_threshold, //6 for start bit midpoint and 15 for full baud cont
    output logic midpoint
    );
    
    logic [RX_MIDPOINT_WIDTH-1:0] iteration_cout;
    
    always_ff @(posedge sys_clk or negedge reset_n) begin
        if (!reset_n) begin
            iteration_cout <= '0;
            midpoint <= 1'b0;
        end 
        
        else if (enable) begin
            if (clr) begin
                iteration_cout <= '0;
                midpoint <= 1'b0;
            end
            
            else if (baud_16th_clk) begin
                if (iteration_cout == count_threshold) begin   
                    iteration_cout <= '0;
                    midpoint <= 1'b1; 
                end 
                
                else begin
                    iteration_cout <= iteration_cout + 1'b1;
                    midpoint <= 1'b0;
                end
            end 
            
            else begin
                midpoint <= 1'b0;
            end
        end
    end             
endmodule