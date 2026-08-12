`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: parser
// Description: Message validity + type decode (parser_spec_v1). PURE COMBINATIONAL:
//   inspects the frame held by rx_mac and decides, same cycle, whether the
//   sub-message opcodes are legal for the Lab-12 set and which format it is.
//   rx_mac has already proven the SKELETON ('{', commas, '}') and exposes it via
//   is_msg + num_submsg; this block checks CONTENTS (opcode bytes) only.
//
//   Two-level decision, burst_mode FIRST (design note 7 / spec sec 5):
//     burst_mode=1 : ONLY a 3-submessage 16-byte frame is legal (PIXEL_PAYLOAD,
//                    opcodes NOT checked - bypass). A 6-byte frame is NOT decoded
//                    as REG_READ_REQ; an 11-byte frame is illegal.
//     burst_mode=0 : decode by num_submsg / opcode comparators (1-hot).
//   burst_mode is a LEVEL driven by seq_top_fsm (not the classifier) - the parser
//   itself is stateless: no clk, no reset, no registers (spec sec 2/7).
//
//   Opcode bytes sit at FIXED buffer indices (frames are top-aligned by rx_mac):
//     opc0 = msg_buf[14], opc1 = msg_buf[9], opc2 = msg_buf[4].
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module parser (
    // ---- From rx_mac (combinational, same cycle) ----
    input  logic [D_SIZE-1:0] msg_buf [MSG_LEN-1:0], // 16-byte frame, buf[15]='{'
    input  logic [1:0]        num_submsg,            // 1 / 2 / 3 (valid-byte span)
    input  logic              is_msg,                // skeleton-valid frame present

    // ---- From seq_top_fsm (sequencer_top) ----
    input  logic              burst_mode,            // opcode-bypass level

    // ---- To rx_classifier ----
    output logic              msg_legal,             // contents legal for current mode
    output msg_type_t         msg_type               // decoded format; valid only
                                                     //   while msg_legal & is_msg
);

    // ---- Opcode byte taps (fixed positions, spec sec 4) ----
    logic [D_SIZE-1:0] opc0, opc1, opc2;
    assign opc0 = msg_buf[14];    // sub-message 0 opcode (right after '{')
    assign opc1 = msg_buf[9];     // sub-message 1 opcode (16-byte frames only)
    assign opc2 = msg_buf[4];     // sub-message 2 opcode (16-byte frames only)

    // ---- Parallel format matchers (1-hot by construction: distinct opc0) ----
    logic reg_write_m, img_write_hdr_m, img_read_req_m, reg_read_m;
    assign reg_write_m     = (opc0 == CHAR_W) && (opc1 == CHAR_V) && (opc2 == CHAR_V);
    assign img_write_hdr_m = (opc0 == CHAR_I) && (opc1 == CHAR_H) && (opc2 == CHAR_W);
    assign img_read_req_m  = (opc0 == CHAR_R) && (opc1 == CHAR_H) && (opc2 == CHAR_W);
    assign reg_read_m      = (opc0 == CHAR_R);                    // 6-byte path

    // ---- Legality / decode (burst_mode has TOP priority - spec sec 5) ----
    always_comb begin
        msg_legal = 1'b0;
        msg_type  = MSG_NONE;

        if (is_msg) begin
            if (burst_mode) begin
                // ---- BURST MODE: only pixel payloads are expected ----
                if (num_submsg == 2'd3) begin
                    msg_legal = 1'b1;
                    msg_type  = MSG_PIXEL_PAYLOAD;   // opcodes NOT checked (bypass)
                end
                // else: 6/11-byte in burst -> illegal -> classifier_err path
            end
            else begin
                // ---- NORMAL MODE: decode by num_submsg / opcodes ----
                unique case (num_submsg)
                    2'd1: begin                      // 6-byte frame
                        msg_legal = reg_read_m;
                        msg_type  = reg_read_m ? MSG_REG_READ_REQ : MSG_NONE;
                    end
                    2'd3: begin                      // 16-byte frame (1-hot decode)
                        msg_legal = reg_write_m | img_write_hdr_m | img_read_req_m;
                        msg_type  = reg_write_m     ? MSG_REG_WRITE     :
                                    img_write_hdr_m ? MSG_IMG_WRITE_HDR :
                                    img_read_req_m  ? MSG_IMG_READ_REQ  : MSG_NONE;
                    end
                    default: ;                       // 11-byte reserved span (2) and
                                                     //   idle (0): always illegal
                endcase
            end
        end
    end

endmodule
