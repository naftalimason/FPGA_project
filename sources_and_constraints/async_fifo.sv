`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: async_fifo
// Description: Generic dual-clock asynchronous FIFO (async_fifo_spec_v1) - the
//   ONLY legal clock-domain crossing for burst pixel data in sequencer_top.
//   One parametric module instantiated SIX times: rx_afifo[R,G,B] (write = gclk
//   side, read = clk side) and tx_afifo[R,G,B] (write = clk, read = gclk side).
//   Standard gray-code-pointer design with independent full/empty and early-
//   warning almost_full/almost_empty flags; FIRST-WORD-FALL-THROUGH read (the
//   head word is combinationally valid on rdata whenever !empty).
//
//   Split into submodules (one per file) for maintenance/modularity:
//     fifo_mem     - storage array (sync write, comb read -> FWFT)
//     fifo_ptr     - pointer + full/almost (write) / empty/almost (read) engine
//     bus_sync_2ff - 2-FF gray-pointer synchronizers
//
//   err_overflow / err_underflow (v2): STICKY flags in their respective
//   domains - a push while full / pop while empty latches the flag. Cleared
//   ONLY by that domain's session soft-reset (the PC may miss them; accepted).
//   Mirrored per-FIFO in fifo_rgf.
//
//   SESSION SOFT-RESET (v2, supersedes the "no soft_reset" project rule for
//   the FIFOs only): wr_srst / rd_srst synchronously clear that domain's
//   pointer, flags and sticky error. seq_top_fsm flushes the whole FIFO trio
//   of the operation it is about to run (both domains, via the envelopes'
//   CDC) BEFORE issuing the start; the gray-pointer synchronisers converge
//   within 2 destination cycles, before any traffic flows.
//
//   GCLK-SIDE USAGE NOTE (2026-07-23 rework, effective-clock discipline):
//   wr_clk_en / rd_clk_en are the domain OPERATIONAL-rate enables and are
//   treated as the domain's EFFECTIVE CLOCK: inside every submodule they are
//   the OUTERMOST gate of the sequential logic (if (en) begin if (cond) ...),
//   exactly like the rest of the gclk-paced design. Parents therefore pass
//   RAW strobes (wr_en = push, rd_en = pop, srst raw) - the enable must NEVER
//   be combined combinationally into a strobe. One 1-tick FSM strobe still
//   produces exactly one push/pop because both the strobe generator and this
//   FIFO sample under the same enable tick.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module async_fifo #(
    parameter  int unsigned WIDTH     = AFIFO_WIDTH,     // 32 = one SRAM row
    parameter  int unsigned DEPTH     = AFIFO_DEPTH,     // power of two
    parameter  int unsigned AF_THRESH = AFIFO_AF_THRESH, // almost_full fill level
    parameter  int unsigned AE_THRESH = AFIFO_AE_THRESH, // almost_empty fill level
    localparam int unsigned ADDR_W    = $clog2(DEPTH),
    localparam int unsigned PW        = ADDR_W + 1
)(
    // ---- write domain ----
    input  logic             wr_clk,        // gclk-side mmcm_clk (rx) / clk (tx)
    input  logic             wr_rst_n,      // write-domain reset (sync-reset module)
    input  logic             wr_clk_en,     // write-domain OPERATIONAL-rate enable
                                            //   = write-domain EFFECTIVE CLOCK
                                            //   (gclk enable on the mmcm side,
                                            //   1'b1 on the 100 MHz side); paces
                                            //   ALL write-side state (pointer,
                                            //   memory, sticky flag, r->w sync)
    input  logic             wr_srst,       // session soft-reset, write domain
                                            //   (RAW - sampled under wr_clk_en)
    input  logic             wr_en,         // push strobe (RAW 1-tick strobe -
                                            //   sampled under wr_clk_en)
    input  logic [WIDTH-1:0] wdata,         // write data
    output logic             full,          // write-side full (block push)
    output logic             almost_full,   // early warning: fill >= AF_THRESH
    output logic             err_overflow,  // STICKY: push attempted while full
    // ---- read domain ----
    input  logic             rd_clk,        // clk (rx) / gclk-side mmcm_clk (tx)
    input  logic             rd_rst_n,      // read-domain reset (sync-reset module)
    input  logic             rd_clk_en,     // read-domain OPERATIONAL-rate enable
                                            //   = read-domain EFFECTIVE CLOCK
                                            //   (1'b1 on the 100 MHz side, gclk
                                            //   enable on the mmcm side); paces
                                            //   ALL read-side state (pointer,
                                            //   sticky flag, w->r sync)
    input  logic             rd_srst,       // session soft-reset, read domain
                                            //   (RAW - sampled under rd_clk_en)
    input  logic             rd_en,         // pop strobe (RAW - sampled under
                                            //   rd_clk_en; advances the FWFT head)
    output logic [WIDTH-1:0] rdata,         // FWFT: head valid when !empty
    output logic             empty,         // read-side empty (block pop)
    output logic             almost_empty,  // early warning: fill <= AE_THRESH
    output logic             err_underflow  // STICKY: pop attempted while empty
);

    logic [ADDR_W-1:0] waddr, raddr;
    logic [PW-1:0]     wptr, rptr;          // gray pointers (own domain)
    logic [PW-1:0]     wq2_rptr;            // read gray ptr resynced into wr_clk
    logic [PW-1:0]     rq2_wptr;            // write gray ptr resynced into rd_clk
    logic              wen;

    assign wen = wr_en & !full;             // protected push

    // ---- gray-pointer synchronizers ----
    bus_sync_2ff #(.W(PW)) sync_r2w (       // read gray ptr -> write domain
        .clk   (wr_clk),
        .rst_n (wr_rst_n),
        .en    (wr_clk_en),                 // sample at the write side's rate
        .d     (rptr),
        .q     (wq2_rptr)
    );

    bus_sync_2ff #(.W(PW)) sync_w2r (       // write gray ptr -> read domain
        .clk   (rd_clk),
        .rst_n (rd_rst_n),
        .en    (rd_clk_en),                 // sample at the read side's rate
        .d     (wptr),
        .q     (rq2_wptr)
    );

    // ---- write side: pointer + full / almost_full ----
    fifo_ptr #(.DEPTH(DEPTH), .THRESH(AF_THRESH), .IS_WRITE(1'b1)) u_wptr (
        .clk        (wr_clk),
        .rst_n      (wr_rst_n),
        .en         (wr_clk_en),        // effective clock of the write domain
        .srst       (wr_srst),
        .inc        (wr_en),
        .other_gray (wq2_rptr),
        .addr       (waddr),
        .gray       (wptr),
        .stop       (full),
        .almost     (almost_full)
    );

    // ---- read side: pointer + empty / almost_empty ----
    fifo_ptr #(.DEPTH(DEPTH), .THRESH(AE_THRESH), .IS_WRITE(1'b0)) u_rptr (
        .clk        (rd_clk),
        .rst_n      (rd_rst_n),
        .en         (rd_clk_en),        // effective clock of the read domain
        .srst       (rd_srst),
        .inc        (rd_en),
        .other_gray (rq2_wptr),
        .addr       (raddr),
        .gray       (rptr),
        .stop       (empty),
        .almost     (almost_empty)
    );

    // ---- storage (sync write, comb read -> FWFT) ----
    fifo_mem #(.WIDTH(WIDTH), .DEPTH(DEPTH)) u_mem (
        .wr_clk (wr_clk),
        .en     (wr_clk_en),            // effective clock of the write domain
        .wen    (wen),
        .waddr  (waddr),
        .wdata  (wdata),
        .raddr  (raddr),
        .rdata  (rdata)
    );

    // ---- sticky error flags (fifo_rgf mirrors; cleared by session soft-reset) ----
    // The domain enable is the EFFECTIVE CLOCK: outermost gate, then conditions.
    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n)              err_overflow <= 1'b0;
        else if (wr_clk_en) begin
            if (wr_srst)            err_overflow <= 1'b0;
            else if (wr_en & full)  err_overflow <= 1'b1;  // push while full (dropped)
        end
    end
    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n)              err_underflow <= 1'b0;
        else if (rd_clk_en) begin
            if (rd_srst)            err_underflow <= 1'b0;
            else if (rd_en & empty) err_underflow <= 1'b1; // pop while empty
        end
    end

endmodule
