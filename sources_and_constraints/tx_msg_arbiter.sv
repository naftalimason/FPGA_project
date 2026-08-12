`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tx_msg_arbiter
// Description: TX-path arbiter (tx_msg_arbiter_spec_v1). Grants the SHARED
//   composer + tx_mac to exactly ONE reply source per command and steers the
//   sample/done handshake:
//     tx_src = 0 : APB register-read reply  (composer format A)
//                  rd_resp_valid -> tx_msg_valid ; tx_msg_done -> resp_captured
//     tx_src = 1 : image-burst pixel payload (composer format B)
//                  cons_tx_valid -> tx_msg_valid ; tx_msg_done -> cons_tx_done
//   The non-granted source sees its done line held 0 and stalls. tx_src is set
//   by seq_top_fsm for the whole command, so mutual exclusion is by
//   construction (spec sec 4). Pure combinational steering - the composer is
//   the datapath, each source holds its data stable from valid until done. The
//   {E} resend path is tx_mac-internal and never passes through here (sec 5).
//////////////////////////////////////////////////////////////////////////////////

import UART_P::*;

module tx_msg_arbiter (
    // ---- Grant select (from seq_top_fsm; stable per command) ----
    input  logic tx_src,                  // 0 = APB reply, 1 = burst consumer

    // ---- APB read-reply source (tx_src = 0) ----
    input  logic rd_resp_valid,           // apb_master: a read reply is ready
    output logic resp_captured,           // to apb_master: reply fully sent

    // ---- Burst consumer source (tx_src = 1) ----
    input  logic cons_tx_valid,           // tx_consumer_fsm: message ready
    output logic cons_tx_done,            // to tx_consumer_fsm: fully sent

    // ---- Composer / tx_mac ----
    output logic fmt_sel,                 // composer format select (= tx_src)
    output logic tx_msg_valid,            // sample request to tx_mac
    input  logic tx_msg_done              // tx_mac: message fully sent to PC
);

    assign fmt_sel       = tx_src;
    assign tx_msg_valid  = tx_src ? cons_tx_valid : rd_resp_valid;
    assign cons_tx_done  = tx_src ? tx_msg_done   : 1'b0;
    assign resp_captured = tx_src ? 1'b0          : tx_msg_done;

endmodule
