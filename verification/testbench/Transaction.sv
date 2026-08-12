// ---------------------------------------------------------------------------
// Transaction: one PC->DUT UART message (or raw byte sequence), plus the
// PHY-level error-injection knobs consumed by the Driver. Byte layouts follow
// message_format_v1 exactly (wire order: MSByte '{' first, '}' last).
// Included by tb_pkg.sv - do not compile standalone.
// ---------------------------------------------------------------------------

typedef enum int {
    TXN_REG_WRITE,   // {W<A2,A1,A0>,V<0,DH1,DH0>,V<0,DL1,DL0>}      16 B
    TXN_REG_READ,    // {R<A2,A1,A0>}                                 6 B
    TXN_IMG_HDR,     // {I<0,0,0>,H<H2,H1,H0>,W<W2,W1,W0>}           16 B
    TXN_IMG_RD_REQ,  // {R<0,0,0>,H<H2,H1,H0>,W<W2,W1,W0>}           16 B
    TXN_PIXEL,       // {<R0,G0,B0,R1>,<G1,B1,R2,G2>,<B2,R3,G3,B3>}  16 B
    TXN_RAW          // raw[] bytes sent verbatim (illegal frames etc.)
} txn_kind_e;

class Transaction;

    rand txn_kind_e    kind;
    rand byte unsigned a2;         // A2 (BAR byte)
    rand bit [15:0]    addr;       // {A1,A0} local register offset
    rand bit [31:0]    data;       // register write data
    bit [23:0]         height;     // image header/read-request dimensions
    bit [23:0]         width;
    byte unsigned      pr [4];     // pixel payload bytes, index 0 = oldest pixel
    byte unsigned      pg [4];
    byte unsigned      pb [4];
    byte unsigned      raw [$];    // TXN_RAW byte list

    // ---- Driver error-injection knobs (wire-byte index; -1 = off) ----
    int bad_parity_pos = -1;       // flip the parity bit of this byte
    int bad_stop_pos   = -1;       // force the stop bit of this byte low
    int truncate_after = -1;       // send only wire bytes [0 .. truncate_after]

    // Random phase: legal register accesses to the three RW checker registers
    constraint c_random_phase {
        kind inside {TXN_REG_WRITE, TXN_REG_READ};
        a2   inside {8'h04, 8'h08, 8'h0C};
        (a2 == 8'h04) -> addr == 16'h0010;   // img_rgf  img_checker
        (a2 == 8'h08) -> addr == 16'h0008;   // uart_rgf uart_checker
        (a2 == 8'h0C) -> addr == 16'h0018;   // fifo_rgf fifo_checker
    }

    function new(txn_kind_e k = TXN_RAW);
        kind = k;
    endfunction

    // Build the exact wire byte sequence, MSByte ('{') first.
    function automatic void get_bytes(output byte unsigned q [$]);
        q.delete();
        case (kind)
            TXN_REG_WRITE: begin
                q.push_back(UART_P::START_BYTE); q.push_back(UART_P::CHAR_W);
                q.push_back(a2); q.push_back(addr[15:8]); q.push_back(addr[7:0]);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(UART_P::CHAR_V); q.push_back(8'h00);
                q.push_back(data[31:24]); q.push_back(data[23:16]);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(UART_P::CHAR_V); q.push_back(8'h00);
                q.push_back(data[15:8]); q.push_back(data[7:0]);
                q.push_back(UART_P::END_BYTE);
            end
            TXN_REG_READ: begin
                q.push_back(UART_P::START_BYTE); q.push_back(UART_P::CHAR_R);
                q.push_back(a2); q.push_back(addr[15:8]); q.push_back(addr[7:0]);
                q.push_back(UART_P::END_BYTE);
            end
            TXN_IMG_HDR: begin
                q.push_back(UART_P::START_BYTE); q.push_back(UART_P::CHAR_I);
                q.push_back(8'h00); q.push_back(8'h00); q.push_back(8'h00);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(UART_P::CHAR_H);
                q.push_back(height[23:16]); q.push_back(height[15:8]); q.push_back(height[7:0]);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(UART_P::CHAR_W);
                q.push_back(width[23:16]); q.push_back(width[15:8]); q.push_back(width[7:0]);
                q.push_back(UART_P::END_BYTE);
            end
            TXN_IMG_RD_REQ: begin
                q.push_back(UART_P::START_BYTE); q.push_back(UART_P::CHAR_R);
                q.push_back(8'h00); q.push_back(8'h00); q.push_back(8'h00);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(UART_P::CHAR_H);
                q.push_back(height[23:16]); q.push_back(height[15:8]); q.push_back(height[7:0]);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(UART_P::CHAR_W);
                q.push_back(width[23:16]); q.push_back(width[15:8]); q.push_back(width[7:0]);
                q.push_back(UART_P::END_BYTE);
            end
            TXN_PIXEL: begin
                q.push_back(UART_P::START_BYTE);
                q.push_back(pr[0]); q.push_back(pg[0]); q.push_back(pb[0]); q.push_back(pr[1]);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(pg[1]); q.push_back(pb[1]); q.push_back(pr[2]); q.push_back(pg[2]);
                q.push_back(UART_P::CHAR_COMMA);
                q.push_back(pb[2]); q.push_back(pr[3]); q.push_back(pg[3]); q.push_back(pb[3]);
                q.push_back(UART_P::END_BYTE);
            end
            default: begin                       // TXN_RAW
                foreach (raw[i]) q.push_back(raw[i]);
            end
        endcase
    endfunction

endclass
