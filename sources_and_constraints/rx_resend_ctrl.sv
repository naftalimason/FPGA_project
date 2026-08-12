`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: rx_resend_ctrl
// Description: RX error / resend controller (v2 change 4) - the functional
//   owner of the {E} resend flow, extracted from uart_rgf so the RGF only
//   counts. Sits at chiptop between the RX error sources and tx_mac:
//     any of {phy, mac, classifier} error events -> arm resend_pending
//     (sticky) -> tx_mac sends the {E<0,0,0>} request (its existing priority
//     rules) -> resend_sent ack -> clear pending + 1-tick flush_rx to
//     rx_phy (RESYNC) and rx_mac (drop frame -> HUNT).
//   The shared OR of the error sources is created HERE, outside the RGF.
//   Guarantees preserved from the validated flow: one {E} per episode; a new
//   error arriving on the very tick of resend_sent re-arms (arm wins over
//   clear), so no episode ever loses its {E}.
//   Clocking: mmcm_clk + gclk enable (same domain as the whole flow).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module rx_resend_ctrl (
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- distinct error events (1-tick) ----
    input  logic phy_err_evt,             // rx_phy parity/framing/stop
    input  logic mac_err_evt,             // rx_mac frame structure
    input  logic clsf_err_evt,            // classifier content reject

    // ---- tx_mac handshake ----
    output logic resend_pending,          // sticky: an {E} is owed to the PC
    input  logic resend_sent,             // 1-tick: {E} fully transmitted

    // ---- RX chain re-sync ----
    output logic flush_rx                 // 1-tick -> rx_phy + rx_mac
);

    logic any_evt;
    assign any_evt = phy_err_evt | mac_err_evt | clsf_err_evt;   // created here,
                                                                 // not in the RGF
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            resend_pending <= 1'b0;
            flush_rx       <= 1'b0;
        end
        else if (enable) begin
            flush_rx <= 1'b0;

            if (resend_sent) begin        // episode complete
                resend_pending <= 1'b0;
                flush_rx       <= 1'b1;
            end
            if (any_evt)                  // AFTER the clear: same-tick error
                resend_pending <= 1'b1;   //   re-arms - its {E} is not lost
        end
    end

endmodule
