`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: composer
// Description: TX message assembler (composer_spec_v1) - PURELY COMBINATIONAL.
//   Builds the outgoing FPGA->PC 16-byte message from ALREADY-ALIGNED fields and
//   presents tx_msg[127:0] to tx_mac ('{' = [127:120], first on the wire). A
//   SHARED resource: tx_msg_arbiter (driven by seq_top_fsm) grants it and picks
//   the format:
//     fmt_sel = 0 : REGISTER READ REPLY  {R<A2,A1,A0>,V<0,DH1,DH0>,V<0,DL1,DL0>}
//                   sources: apb PHY prdata_o (rdata) + classifier-held address
//     fmt_sel = 1 : IMAGE BURST PAYLOAD  {<R0,G0,B0,R1>,<G1,B1,R2,G2>,
//                   <B2,R3,G3,B3>} - re-interleaves the three consumer lines
//   Delimiters are hard constants at bytes 15/10/5/0; every other byte is a 2:1
//   mux on fmt_sel. No clk, no reset, no handshake, no data register - the
//   granted source holds its inputs stable from valid until done (spec sec 8).
//   The {E} resend request is NOT built here (tx_mac-internal, design note 1).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module composer (
    // ---- Format select (= tx_src, from tx_msg_arbiter / seq_top_fsm) ----
    input  logic fmt_sel,                 // 0 = register read reply, 1 = burst

    // ---- Format A datapath (register read reply) ----
    input  logic [BAR_ADDR_WIDTH-1:0]   bar_addr,   // A2 (echoed, classifier-held)
    input  logic [LOCAL_ADDR_WIDTH-1:0] local_addr, // {A1,A0} (echoed)
    input  logic [WDATA_WIDTH-1:0]      rdata,      // apb PHY prdata_o (direct)

    // ---- Format B datapath (image burst payload) ----
    input  logic [CHAN_WIDTH-1:0] tx_line_r,  // {R0,R1,R2,R3} (R0 = MSB)
    input  logic [CHAN_WIDTH-1:0] tx_line_g,  // {G0,G1,G2,G3}
    input  logic [CHAN_WIDTH-1:0] tx_line_b,  // {B0,B1,B2,B3}

    // ---- Output ----
    output logic [MSG_WIDTH-1:0] tx_msg  // complete framed 16-byte message
);

    // Per-byte 2:1 mux + constants (composer_spec sec 4/5 byte maps)
    //                    byte 15..0     Format A                Format B
    assign tx_msg[127:120] = START_BYTE;                        // 15: '{' both
    assign tx_msg[119:112] = fmt_sel ? tx_line_r[31:24]         // 14: R0
                                     : CHAR_R;                  //     'R'
    assign tx_msg[111:104] = fmt_sel ? tx_line_g[31:24]         // 13: G0
                                     : bar_addr;                //     A2
    assign tx_msg[103:96]  = fmt_sel ? tx_line_b[31:24]         // 12: B0
                                     : local_addr[15:8];        //     A1
    assign tx_msg[95:88]   = fmt_sel ? tx_line_r[23:16]         // 11: R1
                                     : local_addr[7:0];         //     A0
    assign tx_msg[87:80]   = CHAR_COMMA;                        // 10: ',' both
    assign tx_msg[79:72]   = fmt_sel ? tx_line_g[23:16]         //  9: G1
                                     : CHAR_V;                  //     'V'
    assign tx_msg[71:64]   = fmt_sel ? tx_line_b[23:16]         //  8: B1
                                     : 8'h00;                   //     0
    assign tx_msg[63:56]   = fmt_sel ? tx_line_r[15:8]          //  7: R2
                                     : rdata[31:24];            //     DH1
    assign tx_msg[55:48]   = fmt_sel ? tx_line_g[15:8]          //  6: G2
                                     : rdata[23:16];            //     DH0
    assign tx_msg[47:40]   = CHAR_COMMA;                        //  5: ',' both
    assign tx_msg[39:32]   = fmt_sel ? tx_line_b[15:8]          //  4: B2
                                     : CHAR_V;                  //     'V'
    assign tx_msg[31:24]   = fmt_sel ? tx_line_r[7:0]           //  3: R3
                                     : 8'h00;                   //     0
    assign tx_msg[23:16]   = fmt_sel ? tx_line_g[7:0]           //  2: G3
                                     : rdata[15:8];             //     DL1
    assign tx_msg[15:8]    = fmt_sel ? tx_line_b[7:0]           //  1: B3
                                     : rdata[7:0];              //     DL0
    assign tx_msg[7:0]     = END_BYTE;                          //  0: '}' both

endmodule
