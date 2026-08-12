`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: fifo_ptr
// Description: One async-FIFO pointer engine (async_fifo_spec_v1 sec 3), used
//   twice per FIFO: IS_WRITE=1 -> write pointer + full/almost_full (write clock
//   domain); IS_WRITE=0 -> read pointer + empty/almost_empty (read clock
//   domain). Binary pointer with one wrap bit (PW = ADDR_W+1); the gray-coded
//   copy is what crosses to the opposite domain (via bus_sync_2ff), and the
//   opposite domain's synced gray pointer comes back in as other_gray.
//     full  : next gray == synced read gray with the two MSBs inverted
//     empty : next gray == synced write gray
//   The fill estimate `used` is computed from this side's LOCAL view (own binary
//   pointer vs gray->binary of the synced foreign pointer), so almost_full /
//   almost_empty are conservative early warnings, as the spec requires:
//     almost_full  (write side) : used >= THRESH   (THRESH = AF fill threshold)
//     almost_empty (read  side) : used <= THRESH   (THRESH = AE fill threshold)
//////////////////////////////////////////////////////////////////////////////////

module fifo_ptr #(
    parameter  int unsigned DEPTH    = 16,
    parameter  int unsigned THRESH   = 4,    // AF_THRESH (write) / AE_THRESH (read)
    parameter  bit          IS_WRITE = 1'b1,
    localparam int unsigned ADDR_W   = $clog2(DEPTH),
    localparam int unsigned PW       = ADDR_W + 1
)(
    input  logic          clk,
    input  logic          rst_n,
    input  logic          en,          // domain OPERATIONAL-rate enable - the
                                       //   EFFECTIVE CLOCK of this domain (gclk
                                       //   enable on the mmcm side, 1'b1 on the
                                       //   100 MHz side). Outermost gate of the
                                       //   sequential block: nothing here may
                                       //   change state on a non-enabled edge.
    input  logic          srst,        // synchronous session soft-reset: pointer
                                       //   to 0, stop to its reset value (v2:
                                       //   seq_top_fsm flushes the FIFO before
                                       //   each image operation). RAW strobe -
                                       //   sampled only under en.
    input  logic          inc,         // push (write) / pop (read) request
                                       //   RAW strobe - sampled only under en
    input  logic [PW-1:0] other_gray,  // synced foreign gray pointer
    output logic [ADDR_W-1:0] addr,    // RAM address (binary, wrap bit dropped)
    output logic [PW-1:0] gray,        // own gray pointer -> other domain
    output logic          stop,        // full (write) / empty (read)
    output logic          almost       // almost_full / almost_empty
);

    logic [PW-1:0] bin, bin_next, gray_next, other_bin, used;
    logic          stop_next;

    assign bin_next  = bin + PW'(inc & !stop);      // never advance past stop
    assign gray_next = bin_next ^ (bin_next >> 1);  // binary -> gray
    assign addr      = bin[ADDR_W-1:0];             // drop wrap bit -> RAM addr

    // gray -> binary of the synced foreign pointer (for the fill estimate)
    always_comb begin
        other_bin = '0;
        for (int i = PW-1; i >= 0; i--) other_bin[i] = ^(other_gray >> i);
    end

    assign stop_next = IS_WRITE
        ? (gray_next == {~other_gray[PW-1:PW-2], other_gray[PW-3:0]})  // full
        : (gray_next == other_gray);                                   // empty

    assign used   = IS_WRITE ? (bin - other_bin) : (other_bin - bin);
    assign almost = IS_WRITE ? (used >= PW'(THRESH)) : (used <= PW'(THRESH));

    // en is the domain's EFFECTIVE CLOCK (project rule): it is the OUTERMOST
    // gate; srst / inc are plain conditions evaluated only under it.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bin  <= '0;
            gray <= '0;
            stop <= !IS_WRITE;      // writer resets not-full, reader resets empty
        end
        else if (en) begin
            if (srst) begin
                bin  <= '0;
                gray <= '0;
                stop <= !IS_WRITE;
            end
            else begin
                bin  <= bin_next;
                gray <= gray_next;
                stop <= stop_next;
            end
        end
    end

endmodule
