`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: BUFGMUX (simulation stub)
// Description: Behavioural black box for the Xilinx BUFGMUX clock-mux primitive
//   instantiated by the RTL glitchless_mux (synthesis path). Permitted by
//   simulation_instruction.txt ("IPs ... may get a behavioural black box").
//   Under Verilator the DUT clock_gen selects its own `VERILATOR` behavioural
//   path, so this stub only satisfies elaboration of glitchless_mux.
//////////////////////////////////////////////////////////////////////////////////

module BUFGMUX #(
    parameter CLK_SEL_TYPE = "SYNC"
)(
    output wire O,
    input  wire I0,
    input  wire I1,
    input  wire S
);

    assign O = S ? I1 : I0;   // simple functional model (no glitch filtering)

endmodule
