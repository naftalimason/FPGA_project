`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: apb_master_mac
// Description: APB application layer (apb_spec_v1 sec 4) - NO message muxing.
//   Receives ALREADY-EXTRACTED, contiguous fields from the classifier (via the
//   sequencer), decides write vs read from is_write, ROUTES to the target slave
//   using the BAR-decoded rgf_sel (slave_sel = rgf_sel pass-through, no local
//   decode - single decider = BAR), starts the PHY, and runs the sequencer
//   back-pressure. On a read it raises rd_resp_valid to tx_msg_arbiter and
//   HOLDS it until resp_captured (= tx_msg_done via the arbiter), then asserts
//   done. This chains the back-pressure: UART sent -> resp_captured -> done ->
//   seq_top busy low -> classifier releases the frame.
//   Clocking: clk = mmcm_clk; enable = gclk clock-enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module apb_master_mac #(
    parameter int unsigned ADDR_W = RGF_ADDR_W,
    parameter int unsigned SEL_W  = RGF_SEL_W
)(
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- from seq_top_fsm / classifier ----
    input  logic        req,              // 1-tick: start a register access
    input  logic        is_write,         // 1 = REG_WRITE, 0 = REG_READ_REQ
    input  logic [LOCAL_ADDR_WIDTH-1:0] local_addr, // {A1,A0}; low bits -> PADDR
    input  logic [BAR_ADDR_WIDTH-1:0]   bar_addr,   // A2 (echoed by the composer
                                          //   from the classifier directly; kept
                                          //   here per spec for reference)
    input  logic [SEL_W-1:0]            rgf_sel,    // BAR-decoded target slave
    input  logic [WDATA_WIDTH-1:0]      wdata,      // {DH1,DH0,DL1,DL0}

    // ---- PHY handshake ----
    input  logic        phy_done,
    input  logic [31:0] phy_prdata,       // PHY prdata_o (held; composer reads it)
    output logic        transfer,         // 1-tick start pulse to the PHY
    output logic [ADDR_W-1:0] paddr,
    output logic        pwrite,
    output logic [31:0] pwdata,

    // ---- interconnect routing ----
    output logic [SEL_W-1:0] slave_sel,   // = rgf_sel (pass-through)

    // ---- read-reply handshake (tx_msg_arbiter) ----
    output logic        rd_resp_valid,    // a read reply is ready (held)
    input  logic        resp_captured,    // reply fully sent (= tx_msg_done)

    // ---- to seq_top_fsm ----
    output logic        done              // 1-tick: access finished (drops busy)
);

    // Field mapping is pure wiring - the classifier already aligned everything.
    assign paddr     = local_addr[ADDR_W-1:0];
    assign pwrite    = is_write;
    assign pwdata    = wdata;
    assign slave_sel = rgf_sel;

    typedef enum logic [1:0] { S_IDLE, S_XFER, S_WAIT, S_RESP } state_t;
    state_t state, next;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)    state <= S_IDLE;
        else if (enable) state <= next;
    end

    always_comb begin
        next = state;
        case (state)
            S_IDLE: if (req)              next = S_XFER;
            S_XFER: next = S_WAIT;                        // transfer strobed
            S_WAIT: if (phy_done)         next = is_write ? S_IDLE : S_RESP;
            S_RESP: if (resp_captured)    next = S_IDLE;  // reply fully sent
            default: next = S_IDLE;
        endcase
    end

    assign transfer      = (state == S_XFER);             // 1 tick
    assign rd_resp_valid = (state == S_RESP);             // held until captured
    // done: writes complete at phy_done; reads only after the reply was sent
    assign done = (state == S_WAIT && phy_done && is_write) ||
                  (state == S_RESP && resp_captured);

endmodule
