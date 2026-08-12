`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: bus_sync_2ff
// Description: Generic 2-FF synchronizer into the destination clock domain.
//   Used for: the async FIFO gray-pointer crossings (async_fifo_spec_v1 sec 3),
//   the 1-bit status/level crossings between the gclk side and the clk side
//   (seq_top_fsm_spec sec 6), and quasi-static buses that are guaranteed stable
//   while sampled (e.g. pixel_target - set before the start toggle, constant
//   for the whole session).
//   NOTE: only gray-coded, single-bit, or quasi-static values may cross here -
//   an arbitrary changing binary bus through a 2-FF sync is NOT safe.
//////////////////////////////////////////////////////////////////////////////////

module bus_sync_2ff #(
    parameter int unsigned W = 1
)(
    input  logic         clk,       // destination clock
    input  logic         rst_n,     // destination-domain reset
    input  logic         en,        // destination-domain pacing enable: the
                                    //   sync samples at the destination's
                                    //   OPERATIONAL rate (gclk enable on the
                                    //   mmcm side; tie 1'b1 in full-rate
                                    //   domains). Project rule: no module
                                    //   except the baud16 generator, the
                                    //   glitchless mux and master_reset_sync
                                    //   may run at the native 280 MHz rate.
    input  logic [W-1:0] d,         // source-domain value
    output logic [W-1:0] q          // synchronized value (2-FF deep)
);

    logic [W-1:0] meta;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)      {q, meta} <= '0;          // clear both stages
        else if (en)     {q, meta} <= {meta, d};   // shift d through 2 FFs
    end

endmodule
