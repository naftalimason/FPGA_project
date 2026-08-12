// ---------------------------------------------------------------------------
// MonMsg + Monitor: the PC-side UART receiver model. Deserialises every byte
// on tx_line (start / 8 data LSB-first / parity / stop) at 8 Mbaud, checks the
// parity against the CURRENT runtime mode (SimCfg), assembles messages by the
// structural rule (delimiters at wire positions 0/5/10/15 - pixel bytes may
// legally equal '{'/'}'/','), and mails each complete message to the Checker.
// Line-level problems (bad parity, bad stop, stray bytes, broken skeleton)
// are counted here and folded into the final error total.
// Included by tb_pkg.sv - do not compile standalone.
// ---------------------------------------------------------------------------

class MonMsg;
    byte unsigned b [$];
    realtime      t_first, t_last;
endclass

class Monitor;

    virtual chip_if       vif;
    SimCfg                cfg;
    mailbox #(MonMsg)     mbx;

    int bytes_seen  = 0;
    int msgs_seen   = 0;
    int parity_errs = 0;
    int frame_errs  = 0;
    int stray_bytes = 0;
    int struct_errs = 0;

    function new(virtual chip_if v, SimCfg c, mailbox #(MonMsg) m);
        vif = v; cfg = c; mbx = m;
    endfunction

    function int total_errs();
        return parity_errs + frame_errs + stray_bytes + struct_errs;
    endfunction

    // Receive one UART byte; samples at bit centres from the start edge.
    task automatic recv_byte(output byte unsigned b, output bit ok);
        bit par, stop, expp;
        ok = 1'b1;
        b  = '0;
        @(negedge vif.tx_line);                  // start-bit edge
        #(TB_BIT_T / 2);
        if (vif.tx_line !== 1'b0) begin          // glitch, not a real start
            frame_errs++; ok = 1'b0;
            Logger::log("[ERROR] Monitor: start-bit glitch on tx_line");
            return;
        end
        for (int i = 0; i < 8; i++) begin
            #(TB_BIT_T); b[i] = vif.tx_line;     // LSB first
        end
        #(TB_BIT_T); par  = vif.tx_line;
        #(TB_BIT_T); stop = vif.tx_line;
        expp = cfg.parity_odd ? ~(^b) : (^b);
        if (par !== expp) begin
            parity_errs++; ok = 1'b0;
            Logger::log($sformatf("[ERROR] Monitor: DUT parity error on byte 0x%02h (mode=%s)",
                                 b, cfg.parity_odd ? "odd" : "even"));
        end
        if (stop !== 1'b1) begin
            frame_errs++; ok = 1'b0;
            Logger::log($sformatf("[ERROR] Monitor: DUT stop-bit error on byte 0x%02h", b));
        end
        bytes_seen++;
    endtask

    // Assemble messages forever; delimiters sit at wire positions 0/5(/10/15).
    task automatic run();
        forever begin
            byte unsigned b;
            bit           ok;
            MonMsg        m;
            recv_byte(b, ok);
            if (b !== UART_P::START_BYTE) begin
                stray_bytes++;
                Logger::log($sformatf("[ERROR] Monitor: stray byte 0x%02h outside a message", b));
                continue;
            end
            m = new();
            m.t_first = $realtime;
            m.b.push_back(b);
            repeat (4) begin recv_byte(b, ok); m.b.push_back(b); end
            recv_byte(b, ok); m.b.push_back(b);              // wire byte 5
            if (b != UART_P::END_BYTE) begin                 // 16-byte message
                if (b != UART_P::CHAR_COMMA) begin
                    struct_errs++;
                    Logger::log($sformatf("[ERROR] Monitor: wire byte5 = 0x%02h, expected ',' or '}'", b));
                end
                repeat (10) begin recv_byte(b, ok); m.b.push_back(b); end
                if (m.b[10] != UART_P::CHAR_COMMA) begin
                    struct_errs++;
                    Logger::log($sformatf("[ERROR] Monitor: wire byte10 = 0x%02h, expected ','", m.b[10]));
                end
                if (m.b[15] != UART_P::END_BYTE) begin
                    struct_errs++;
                    Logger::log($sformatf("[ERROR] Monitor: wire byte15 = 0x%02h, expected '}'", m.b[15]));
                end
            end
            m.t_last = $realtime;
            msgs_seen++;
            mbx.put(m);
        end
    endtask

endclass
