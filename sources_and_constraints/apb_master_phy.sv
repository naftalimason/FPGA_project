`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: apb_master_phy
// Description: APB3 protocol FSM, slave-agnostic (apb_spec_v1 sec 5). Drives ONE
//   logical APB and knows nothing about the slave count - the interconnect owns
//   all slave routing. IDLE -> SETUP (psel, drive addr/ctrl/data) -> ACCESS
//   (psel+penable; hold while !pready_i; on pready_i register prdata_i into
//   prdata_o and pulse phy_done). prdata_o is the ONLY read-data register in
//   the whole master and is handed COMBINATIONALLY to the composer for the
//   read reply (apb_spec sec 8) - it stays stable until the next transfer.
//   Clocking: clk = mmcm_clk; enable = gclk clock-enable (same domain as the
//   classifier / sequencer / RGFs - the whole APB world is gclk-paced).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module apb_master_phy #(
    parameter int unsigned ADDR_W = RGF_ADDR_W
)(
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- request from apb_master_mac ----
    input  logic              transfer,   // 1-tick: start one transfer
    input  logic [ADDR_W-1:0] paddr_i,    // local register offset
    input  logic              pwrite_i,   // direction
    input  logic [31:0]       pwdata_i,   // write data

    // ---- APB bus (to apb_interconnect) ----
    output logic              psel_o,     // SETUP + ACCESS
    output logic              penable_o,  // ACCESS only
    output logic [ADDR_W-1:0] paddr_o,    // held SETUP -> ACCESS
    output logic              pwrite_o,
    output logic [31:0]       pwdata_o,
    input  logic              pready_i,   // muxed ready from interconnect
    input  logic [31:0]       prdata_i,   // muxed read data from interconnect

    // ---- status / data back to the MAC (and composer) ----
    output logic [31:0]       prdata_o,   // read data, REGISTERED at PREADY
    output logic              phy_done,   // 1-tick: PREADY seen in ACCESS
    output logic              rgf_r0c_read // APB read-completion qualifier for
                                          //   read-to-clear registers (v2.1):
                                          //   high exactly in the completing
                                          //   ACCESS cycle of a READ - the same
                                          //   edge prdata_o captures PRDATA
);

    typedef enum logic [1:0] { S_IDLE, S_SETUP, S_ACCESS } state_t;
    state_t state, next;

    logic [ADDR_W-1:0] addr_q;
    logic              wr_q;
    logic [31:0]       wdata_q;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state    <= S_IDLE;
            addr_q   <= '0;
            wr_q     <= 1'b0;
            wdata_q  <= '0;
            prdata_o <= '0;
        end
        else if (enable) begin
            state <= next;
            if (state == S_IDLE && transfer) begin
                addr_q  <= paddr_i;
                wr_q    <= pwrite_i;
                wdata_q <= pwdata_i;
            end
            if (state == S_ACCESS && pready_i && !wr_q)
                prdata_o <= prdata_i;         // the only read-data register
        end
    end

    always_comb begin
        next = state;
        case (state)
            S_IDLE:   if (transfer) next = S_SETUP;
            S_SETUP:  next = S_ACCESS;
            S_ACCESS: if (pready_i) next = S_IDLE;
            default:  next = S_IDLE;
        endcase
    end

    assign psel_o    = (state == S_SETUP) || (state == S_ACCESS);
    assign penable_o = (state == S_ACCESS);
    assign paddr_o   = addr_q;
    assign pwrite_o  = wr_q;
    assign pwdata_o  = wdata_q;
    assign phy_done  = (state == S_ACCESS) && pready_i;   // 1 enable tick

    // R0C qualifier (v2.1): a COMPLETED read data transfer per the APB protocol
    assign rgf_r0c_read = psel_o && penable_o && pready_i && !pwrite_o;

endmodule
