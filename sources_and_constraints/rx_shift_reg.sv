`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/25/2026 02:25:32 PM
// Design Name: 
// Module Name: rx_shift_reg
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

module rx_shift_reg (
    input  logic sys_clk,
    input  logic reset_n,
    input  logic enable,
    input  logic rx_val,
    input  logic shift_en,
    output logic [D_SIZE-1:0] rx_byte
);
    // --- Internal Logic ---
    logic [D_SIZE-1:0] internal_reg;

    always_ff @(posedge sys_clk or negedge reset_n) begin
        if (!reset_n) begin
            internal_reg <= '0;
        end else if (enable) begin
            if (shift_en) begin
                internal_reg <= {rx_val, internal_reg[D_SIZE-1:1]};
            end
        end
    end
    
    assign rx_byte = internal_reg;

endmodule