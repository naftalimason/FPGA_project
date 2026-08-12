`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: seq_top_fsm
// Description: Orchestrator (v2). Selects AHB vs APB from msg_type, owns busy
//   (0->1 accept, 1->0 complete) and all shared selects. v2 change 1: image
//   commands are AUTHORISED BY img_rgf - seq_top_fsm asserts req_write /
//   req_read and only arms the operational FSMs when img_rgf answers with the
//   start_write / start_read grant pulse. A read while ready_to_read==0 gets
//   no grant and is completed SILENTLY (normal busy cycle, no reply,
//   burst_mode left low). Completion is observed THROUGH img_rgf
//   (write_complete / read_complete = the monitor bit16 levels), except the
//   producer's complete bit, which comes directly to lock busy at end of
//   ingest. v2 change 2/5: before each image operation this FSM soft-resets
//   the ENTIRE FIFO trio of that operation (rx_fifo_srst / tx_fifo_srst),
//   issued with the request so both FIFO domains are flushed before the grant
//   arrives and any data moves. A completion level is only trusted after it
//   has been observed LOW in the current session (comp_armed guard - the
//   previous session's level persists until the counters clear).
//   v2.4 (2026-07-23): flush-settle guard - the setup states hold the img_rgf
//   grant and arm the operational FSMs only SRST_SETTLE enable ticks after the
//   trio flush, so the far-domain srst and the re-crossing gray pointers have
//   settled and empty/full cannot phantom-deassert (fixes the one-stale-first-
//   payload bug on every image-read session after the first).
//   Clocking: mmcm_clk + gclk enable.
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module seq_top_fsm (
    input  logic clk,                     // mmcm_clk physical clock
    input  logic reset_n,                 // external sync-reset module
    input  logic enable,                  // gclk = baud*16 clock-enable

    // ---- from rx_classifier ----
    input  logic       new_msg,           // held request (until busy accepts)
    input  msg_type_t  msg_type,
    input  logic [DIM_WIDTH-1:0] img_height,
    input  logic [DIM_WIDTH-1:0] img_width,

    // ---- from chiptop BAR (informational; APB MAC consumes rgf_sel) ----
    input  logic                 sram_sel,
    input  logic [RGF_SEL_W-1:0] rgf_sel,

    // ---- neighbour reports ----
    input  logic prod_busy,               // producer: push in flight
    input  logic prod_complete,           // producer bit16: ingest done (direct)
    input  logic cons_busy,               // consumer status (informational)
    input  logic apb_done,                // apb_master: access finished (1-tick)
    input  logic tx_msg_done,             // tx_mac status (informational)

    // ---- img_rgf control handshake (v2 change 1) ----
    output logic req_write,               // 1-tick: image write cmd received
    output logic req_read,                // 1-tick: image read cmd received
    input  logic start_write_i,           // 1-tick grant from img_rgf
    input  logic start_read_i,            // 1-tick grant from img_rgf
    input  logic ready_to_read,           // level: valid image stored
    input  logic write_complete,          // level: img_rx_monitor[16] via img_rgf
    input  logic read_complete,           // level: img_tx_monitor[16] via img_rgf

    // ---- to img_rgf sideband ----
    output logic [2*DIM_WIDTH-1:0] img_size,
    output logic                   img_size_wr,

    // ---- to rx_classifier / rx_parser ----
    output logic busy,
    output logic burst_mode,

    // ---- shared selects ----
    output logic ahb_dir,                 // 0 = RX-write MAC, 1 = TX-read MAC
    output logic tx_src,                  // 0 = APB reply, 1 = burst (= fmt_sel)

    // ---- to the sequencer envelopes ----
    output logic start_write,             // 1-tick: arm producer + rx AHB MAC
    output logic start_read,              // 1-tick: arm consumer + tx AHB MAC
    output logic rx_fifo_srst,            // 1-tick: flush the rx_afifo trio
    output logic tx_fifo_srst,            // 1-tick: flush the tx_afifo trio

    // ---- to apb_master ----
    output logic req,
    output logic is_write
);

    typedef enum logic [2:0] {
        S_IDLE, S_REG_WRITE, S_REG_READ,
        S_WR_SETUP, S_BURST_WR, S_RD_SETUP, S_BURST_RD, S_RD_REFUSE
    } state_t;
    state_t state, next;

    logic comp_armed;                     // completion level seen LOW this session
    logic refuse_cnt;                     // 2-tick silent-refuse busy window

    // v2.4 flush-settle guard: the whole-trio FIFO soft reset reaches the far
    // domain through the envelopes' toggle CDC (~5-6 enable ticks) and the
    // flushed pointer then re-crosses the gray-pointer synchroniser (2 more
    // ticks). Until it lands, the near side compares its zeroed pointer with
    // the STALE synced foreign pointer, so empty/full can phantom-deassert for
    // a few ticks. Arming an operational FSM inside that window let it pop a
    // stale FWFT head (one old first payload on every read session after the
    // first). Fix: hold the img_rgf grant and issue start_write / start_read
    // only after SRST_SETTLE enable ticks (~125 ns; 6+ ticks of margin) have
    // passed since the flush was issued with the request.
    localparam logic [4:0] SRST_SETTLE = 5'd16;
    logic [4:0] settle_cnt;               // enable ticks since the session flush
    logic       grant_q;                  // img_rgf grant latched during settle

    // ---- State register + strobes + session flags ----
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state        <= S_IDLE;
            burst_mode   <= 1'b0;
            img_size     <= '0;
            img_size_wr  <= 1'b0;
            req_write    <= 1'b0;
            req_read     <= 1'b0;
            start_write  <= 1'b0;
            start_read   <= 1'b0;
            rx_fifo_srst <= 1'b0;
            tx_fifo_srst <= 1'b0;
            req          <= 1'b0;
            is_write     <= 1'b0;
            comp_armed   <= 1'b0;
            refuse_cnt   <= 1'b0;
            settle_cnt   <= '0;
            grant_q      <= 1'b0;
        end
        else if (enable) begin
            state        <= next;
            img_size_wr  <= 1'b0;         // all strobes: 1-tick registered
            req_write    <= 1'b0;
            req_read     <= 1'b0;
            start_write  <= 1'b0;
            start_read   <= 1'b0;
            rx_fifo_srst <= 1'b0;
            tx_fifo_srst <= 1'b0;
            req          <= 1'b0;

            case (state)
                S_IDLE: if (new_msg) begin
                    comp_armed <= 1'b0;
                    settle_cnt <= '0;             // new session: settle restart
                    grant_q    <= 1'b0;
                    unique case (msg_type)
                        MSG_REG_WRITE: begin
                            req      <= 1'b1;
                            is_write <= 1'b1;
                        end
                        MSG_REG_READ_REQ: begin
                            req      <= 1'b1;
                            is_write <= 1'b0;
                        end
                        MSG_IMG_WRITE_HDR: begin
                            img_size     <= {img_height, img_width};
                            img_size_wr  <= 1'b1;     // latch H/W into img_rgf
                            req_write    <= 1'b1;     // ask img_rgf for the grant
                            rx_fifo_srst <= 1'b1;     // flush the WHOLE rx trio
                            burst_mode   <= 1'b1;     // arm the parser bypass
                        end
                        MSG_IMG_READ_REQ: begin
                            if (ready_to_read) begin
                                req_read     <= 1'b1; // ask img_rgf for the grant
                                tx_fifo_srst <= 1'b1; // flush the WHOLE tx trio
                            end
                            else refuse_cnt <= 1'b0;  // silent refuse window
                        end
                        default: ;                    // payloads: producer's job
                    endcase
                end

                S_WR_SETUP: begin                     // busy held through setup
                    if (settle_cnt != 5'd31) settle_cnt <= settle_cnt + 5'd1;
                    if (start_write_i)       grant_q    <= 1'b1;   // hold grant
                    // arm only once the trio flush has settled in BOTH domains
                    if ((grant_q || start_write_i) && settle_cnt >= SRST_SETTLE)
                        start_write <= 1'b1;
                end

                S_BURST_WR:
                    if (!write_complete) comp_armed <= 1'b1;  // low seen -> arm
                    else if (comp_armed) begin                // done: image valid
                        burst_mode <= 1'b0;
                        comp_armed <= 1'b0;
                    end

                S_RD_SETUP: begin
                    if (settle_cnt != 5'd31) settle_cnt <= settle_cnt + 5'd1;
                    if (start_read_i)        grant_q    <= 1'b1;   // hold grant
                    if ((grant_q || start_read_i) && settle_cnt >= SRST_SETTLE)
                        start_read <= 1'b1;
                end

                S_BURST_RD:
                    if (!read_complete) comp_armed <= 1'b1;
                    else if (comp_armed) comp_armed <= 1'b0;

                S_RD_REFUSE: refuse_cnt <= 1'b1;              // 2-tick window

                default: ;
            endcase
        end
    end

    // ---- Next-state logic ----
    always_comb begin
        next = state;
        case (state)
            S_IDLE: if (new_msg) begin
                unique case (msg_type)
                    MSG_REG_WRITE:     next = S_REG_WRITE;
                    MSG_REG_READ_REQ:  next = S_REG_READ;
                    MSG_IMG_WRITE_HDR: next = S_WR_SETUP;
                    MSG_IMG_READ_REQ:  next = ready_to_read ? S_RD_SETUP
                                                            : S_RD_REFUSE;
                    default:           next = S_IDLE;
                endcase
            end
            S_REG_WRITE: if (apb_done)                     next = S_IDLE;
            S_REG_READ:  if (apb_done)                     next = S_IDLE;
            S_WR_SETUP:  if ((grant_q || start_write_i) &&
                             settle_cnt >= SRST_SETTLE)    next = S_BURST_WR;
            S_BURST_WR:  if (comp_armed && write_complete) next = S_IDLE;
            S_RD_SETUP:  if ((grant_q || start_read_i) &&
                             settle_cnt >= SRST_SETTLE)    next = S_BURST_RD;
            S_BURST_RD:  if (comp_armed && read_complete)  next = S_IDLE;
            S_RD_REFUSE: if (refuse_cnt)                   next = S_IDLE;
            default:                                       next = S_IDLE;
        endcase
    end

    // ---- Busy shape: ingest follows the producer, then locks on its
    //      complete bit until the store finishes (state exit drops busy) ----
    always_comb begin
        case (state)
            S_IDLE:     busy = 1'b0;
            S_BURST_WR: busy = prod_busy | prod_complete;
            default:    busy = 1'b1;
        endcase
    end

    // ---- Shared selects (stable for the whole command) ----
    assign ahb_dir = (state == S_RD_SETUP) || (state == S_BURST_RD);
    assign tx_src  = ahb_dir;

endmodule
