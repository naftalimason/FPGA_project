`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tx_ahb_master_mac
// Description: TX-read AHB master, application layer (v2). Reads the three
//   SRAMs via the shared phys (one SINGLE read per row) and pushes the three
//   tx_afifos in parallel. Progress is the packed counter
//   {complete[PROG_W-1], row, col}, cleared by the (synchronised) start pulse,
//   +4 per completed AHB read; counter[PROG_W-2:0] IS the SRAM read address
//   (pixel-stride from 0). The complete bit is a LOCAL-ONLY finish indicator
//   (stop reading) - per the v2 spec it is NOT mirrored and NOT reported to
//   the RGF or seq_top_fsm (system read-completion comes from the consumer's
//   monitor). T_PUSH is one cycle after the last phy_done so the phys'
//   registered hrdata_o is stable when pushed. Fill gates on full+almost_full
//   (early throttle; consumer drains independently - no deadlock).
//   PROG_W structural 17; smaller = TB only. Clocking: clk, full rate.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module tx_ahb_master_mac #(
    parameter int unsigned PROG_W = IMG_PROG_W,
    parameter int unsigned ADDR_W = AHB_ADDR_W
)(
    input  logic clk,                     // 100 MHz SRAM/AHB clock
    input  logic reset_n,                 // external sync-reset module

    // ---- From seq_top_fsm (synchronised into clk by tx_sequencer) ----
    input  logic start_read_s,            // 1-clk pulse: session armed
                                          //   (clears the progress counter)

    // ---- AHB phy side (broadcast control; per-channel data) ----
    output logic              xfer_req,
    output logic [ADDR_W-1:0] haddr,      // = progress counter (pixel-stride)
    output logic              hwrite,     // = 0
    input  logic [CHAN_WIDTH-1:0] hrdata_r, hrdata_g, hrdata_b,
    input  logic phy_done_r, phy_done_g, phy_done_b,
    input  logic hresp_err_r, hresp_err_g, hresp_err_b,  // unused (SRAM: OKAY)

    // ---- tx_afifo write side (x3) ----
    input  logic full_r, full_g, full_b,
    input  logic almost_full_r, almost_full_g, almost_full_b,
    output logic push_r, push_g, push_b,  // 1-clk pushes (T_PUSH)
    output logic [CHAN_WIDTH-1:0] wdata_r, wdata_g, wdata_b,

    output logic ahb_busy                 // a read is in flight (status)
);

    assign wdata_r = hrdata_r;
    assign wdata_g = hrdata_g;
    assign wdata_b = hrdata_b;
    assign hwrite  = 1'b0;

    typedef enum logic [2:0] { T_IDLE, T_SPACE, T_ISSUE, T_DONE, T_PUSH } state_t;
    state_t state, next;

    logic [PROG_W-1:0] prog;              // {complete, row, col} = read address
    logic done_r_q, done_g_q, done_b_q;
    logic space_ok, all_done, complete;

    assign complete = prog[PROG_W-1];
    assign space_ok = !(full_r | full_g | full_b) &&
                      !(almost_full_r | almost_full_g | almost_full_b);
    assign all_done = (done_r_q | phy_done_r) &
                      (done_g_q | phy_done_g) &
                      (done_b_q | phy_done_b);

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) state <= T_IDLE;
        else          state <= next;
    end

    always_comb begin
        next = state;
        case (state)
            T_IDLE:  if (start_read_s)          next = T_SPACE;
            T_SPACE: if (space_ok && !complete) next = T_ISSUE;
            T_ISSUE: next = T_DONE;
            T_DONE:  if (all_done)              next = T_PUSH;
            T_PUSH:  next = (prog + PROG_W'(4) >= {1'b1, {(PROG_W-1){1'b0}}})
                            ? T_IDLE : T_SPACE;   // local finish - stop reading
            default: next = T_IDLE;
        endcase
    end

    // ---- packed progress counter: cleared by start, +4 per AHB completion ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)                             prog <= '0;
        else if (state == T_IDLE && start_read_s) prog <= '0;
        else if (state == T_PUSH)                 prog <= prog + PROG_W'(4);
    end

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n)                             {done_r_q, done_g_q, done_b_q} <= '0;
        else if (state == T_ISSUE || (state == T_DONE && !all_done)) begin
            done_r_q <= done_r_q | phy_done_r;
            done_g_q <= done_g_q | phy_done_g;
            done_b_q <= done_b_q | phy_done_b;
        end
        else                                      {done_r_q, done_g_q, done_b_q} <= '0;
    end

    assign xfer_req = (state == T_ISSUE);
    assign haddr    = ADDR_W'(prog[PROG_W-2:0]);   // counter IS the address
    assign ahb_busy = (state != T_IDLE);
    assign push_r   = (state == T_PUSH);           // hrdata_o stable (1 cycle late)
    assign push_g   = (state == T_PUSH);
    assign push_b   = (state == T_PUSH);

endmodule
