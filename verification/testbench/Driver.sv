// ---------------------------------------------------------------------------
// Driver: the PC-side UART transmitter model. Serialises Transaction wire
// bytes onto rx_line at 8 Mbaud (start / 8 data LSB-first / parity / stop),
// with the parity computed from the CURRENT runtime mode (SimCfg). Before
// EVERY byte it honours the DUT's hardware flow control (CTS# low = OK), like
// a real FT2232. Per-byte error injection (bad parity / bad stop / message
// truncation) drives the RX-error / resend verification. Also owns the PC
// RTS# line (the DUT's TX flow control input).
// Included by tb_pkg.sv - do not compile standalone.
// ---------------------------------------------------------------------------

class Driver;

    virtual chip_if vif;
    SimCfg          cfg;

    int bytes_sent      = 0;
    int msgs_sent       = 0;
    int cts_wait_events = 0;   // times a byte had to wait for CTS#

    function new(virtual chip_if v, SimCfg c);
        vif = v; cfg = c;
        vif.rx_line = 1'b1;    // UART line idles high
        vif.rts_n   = 1'b0;    // PC ready to receive by default
    endfunction

    // Block until the DUT is ready to receive (CTS# low). Hard 2 ms guard so
    // the simulation can never hang on a dead handshake.
    task automatic wait_cts(string ctx);
        int t = 0;
        bit waited = 0;
        while (vif.cts_n !== 1'b0) begin
            waited = 1;
            #100ns; t++;
            if (t > 20000) begin
                Logger::log($sformatf("[ERROR] Driver: CTS# timeout after 2 ms (%s)", ctx));
                $fatal(1, "Driver CTS timeout - DUT never became ready");
            end
        end
        if (waited) cts_wait_events++;
    endtask

    // Send one UART byte; optional parity/stop corruption.
    task automatic send_byte(byte unsigned b, bit bad_par = 0, bit bad_stop = 0,
                             string ctx = "");
        bit par;
        wait_cts(ctx);
        par = cfg.parity_odd ? ~(^b) : (^b);
        if (bad_par) par = ~par;
        vif.rx_line = 1'b0;                    #(TB_BIT_T);   // start
        for (int i = 0; i < 8; i++) begin
            vif.rx_line = b[i];                #(TB_BIT_T);   // data, LSB first
        end
        vif.rx_line = par;                     #(TB_BIT_T);   // parity
        vif.rx_line = bad_stop ? 1'b0 : 1'b1;  #(TB_BIT_T);   // stop
        vif.rx_line = 1'b1;                                   // back to idle
        if (bad_stop) #(TB_BIT_T);             // settle after a framing error
        bytes_sent++;
    endtask

    // Send a complete transaction (respecting its injection knobs).
    task automatic send_txn(Transaction t, string ctx = "");
        byte unsigned q [$];
        t.get_bytes(q);
        foreach (q[i]) begin
            if (t.truncate_after >= 0 && i > t.truncate_after) break;
            send_byte(q[i], (i == t.bad_parity_pos), (i == t.bad_stop_pos), ctx);
        end
        msgs_sent++;
    endtask

    // Idle the line high for n bit times (inter-message gaps, RESYNC margin).
    task automatic idle_bits(int n);
        repeat (n) #(TB_BIT_T);
    endtask

    // PC RTS# control: ready=1 -> RTS# low -> DUT may transmit.
    task automatic set_pc_ready(bit ready);
        vif.rts_n = ready ? 1'b0 : 1'b1;
    endtask

endclass
