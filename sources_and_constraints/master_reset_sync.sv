`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 06/25/2026 01:36:42 PM
// Design Name:
// Module Name: master_reset_sync
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   Shared UART/system parameters.
//
//   IMPORTANT:
//
// Dependencies:
//
// Revision:
// Revision 0.01
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module master_reset_sync #(
    parameter int unsigned STAGES = 2
)(
    input  logic clk,
    input  logic async_reset_n,
    output logic master_reset_n
);


    logic [STAGES-1:0] reset_sync_ff;

    always_ff @(posedge clk or negedge async_reset_n) begin
        if (!async_reset_n) begin
            reset_sync_ff <= '0;
        end
        else begin
            reset_sync_ff[0] <= 1'b1;

            for (int i = 1; i < STAGES; i++) begin
                reset_sync_ff[i] <= reset_sync_ff[i-1];
            end
        end
    end

    assign master_reset_n = reset_sync_ff[STAGES-1];

endmodule
