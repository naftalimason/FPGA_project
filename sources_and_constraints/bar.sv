`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: bar
// Description: BAR address -> per-module CHIP-SELECT decoder (bar_spec_v1 +
//   address_map_v1). PURELY COMBINATIONAL. Decodes the command's high address
//   byte (bar_addr = A2, from the classifier) into chip-selects. Generates ONLY
//   chip-selects (design note 5): never merged with PSEL/PENABLE/HTRANS or any
//   protocol select; never a replacement for the AHB-vs-APB path choice, which
//   seq_top_fsm makes from msg_type alone.
//
//   Decode map (address_map_v1 sec 2, binding):
//     A2 = 0x00 -> RGB SRAM bank (sram_sel = 1)      [effectively future-use:
//     A2 = 0x04 -> img_rgf   (rgf_sel = 0)            the AHB path is bank-
//     A2 = 0x08 -> uart_rgf  (rgf_sel = 1)            implicit today, d3]
//     A2 = 0x0C -> fifo_rgf  (rgf_sel = 2)
//     other     -> no hit (all selects 0)
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module bar (
    input  logic [BAR_ADDR_WIDTH-1:0] bar_addr,  // A2 from the classifier
    output logic                      sram_sel,  // whole-RGB-bank chip-select
    output logic [RGF_SEL_W-1:0]      rgf_sel,   // selected RGF index
    output logic                      rgf_hit,   // bar_addr is an RGF address
    output logic                      sram_hit   // bar_addr is the SRAM region
);

    always_comb begin
        sram_sel = 1'b0;
        rgf_sel  = '0;
        rgf_hit  = 1'b0;
        sram_hit = 1'b0;
        unique case (bar_addr)
            SRAM_BASE_A2: begin sram_sel = 1'b1; sram_hit = 1'b1; end
            IMG_RGF_A2:   begin rgf_sel = 2'd0; rgf_hit = 1'b1; end
            UART_RGF_A2:  begin rgf_sel = 2'd1; rgf_hit = 1'b1; end
            FIFO_RGF_A2:  begin rgf_sel = 2'd2; rgf_hit = 1'b1; end
            default: ;                                   // unmapped: no hit
        endcase
    end

endmodule
