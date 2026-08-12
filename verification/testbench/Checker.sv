// ---------------------------------------------------------------------------
// Checker (scoreboard): consumes the Monitor's message mailbox and compares
// against expected-result byte queues built by the test program's reference
// model. Owns the test bookkeeping (test/pass/fail/error counters), the
// feature/coverage counters, and the negative check ("expect silence"). Every
// comparison logs expected vs observed so the run is fully readable from the
// log alone. Included by tb_pkg.sv - do not compile standalone.
// ---------------------------------------------------------------------------

class Checker;

    mailbox #(MonMsg) mbx;

    // ---- test bookkeeping ----
    int    tests_run     = 0;
    int    tests_pass    = 0;
    int    tests_fail    = 0;
    int    total_errors  = 0;
    int    cur_test_errs = 0;
    string cur_test      = "";

    // ---- feature / coverage counters ----
    int cov_reg_write    = 0;   // register writes driven
    int cov_reg_read     = 0;   // register reads + byte-exact reply checks
    int cov_img_write    = 0;   // full image-write bursts completed
    int cov_img_read     = 0;   // full image-read bursts completed
    int cov_payload_rx   = 0;   // burst payload replies checked byte-exact
    int cov_e_replies    = 0;   // {E} resend requests checked byte-exact
    int cov_err_inject   = 0;   // deliberate error injections driven
    int cov_random_ops   = 0;   // constrained-random operations completed
    int cov_cts_pauses   = 0;   // PC-RTS# pause windows exercised
    int cov_parity_switch= 0;   // runtime parity-mode switches exercised
    int cov_sram_words   = 0;   // SRAM words compared OK (hierarchical peek)
    int cov_silence      = 0;   // expect-silence (negative) checks passed

    function new(mailbox #(MonMsg) m);
        mbx = m;
    endfunction

    // ---- test markers ----
    task automatic test_begin(string name, string purpose);
        cur_test      = name;
        cur_test_errs = 0;
        tests_run++;
        Logger::log("");
        Logger::log($sformatf("[TEST START] %s", name));
        Logger::log($sformatf("[TEST INFO] purpose: %s", purpose));
    endtask

    task automatic test_end();
        if (cur_test_errs == 0) begin
            tests_pass++;
            Logger::log($sformatf("[TEST PASS] %s", cur_test));
        end
        else begin
            tests_fail++;
            Logger::log($sformatf("[TEST FAIL] %s (%0d error(s))", cur_test, cur_test_errs));
        end
    endtask

    // ---- error / info primitives ----
    function void err(string s);
        total_errors++;
        cur_test_errs++;
        Logger::log({"[ERROR] ", s});
    endfunction

    function void info(string s);
        Logger::log({"[TEST INFO] ", s});
    endfunction

    function void check_eq32(bit [31:0] obs, bit [31:0] exp, string what);
        if (obs === exp)
            info($sformatf("%s: observed 0x%08h == expected 0x%08h  (check ok)", what, obs, exp));
        else
            err($sformatf("%s: observed 0x%08h != expected 0x%08h", what, obs, exp));
    endfunction

    function void check_true(bit cond, string what);
        if (cond) info($sformatf("%s  (check ok)", what));
        else      err(what);
    endfunction

    // ---- message expectation: wait for ONE DUT message, compare byte-exact ----
    task automatic expect_msg(input byte unsigned exp [$], input string what,
                              input realtime timeout = 500us);
        MonMsg   m;
        realtime t0 = $realtime;
        bit      mismatch = 0;
        while (!mbx.try_get(m)) begin
            #1us;
            if (($realtime - t0) > timeout) begin
                err($sformatf("%s: TIMEOUT waiting for DUT message; expected: %s",
                              what, hexq(exp)));
                return;
            end
        end
        if (m.b.size() != exp.size()) mismatch = 1;
        else foreach (exp[i]) if (m.b[i] !== exp[i]) mismatch = 1;
        if (mismatch)
            err($sformatf("%s: message mismatch  expected: %s | observed: %s",
                          what, hexq(exp), hexq(m.b)));
        else
            info($sformatf("%s: %0d-byte reply matches: %s (check ok)",
                           what, exp.size(), hexq(exp)));
    endtask

    // ---- negative check: NO message may arrive inside the window ----
    task automatic expect_silence(input realtime window, input string what);
        MonMsg   m;
        realtime t0 = $realtime;
        while (($realtime - t0) < window) begin
            #5us;
            if (mbx.try_get(m)) begin
                err($sformatf("%s: unexpected DUT message during silence window: %s",
                              what, hexq(m.b)));
                return;
            end
        end
        info($sformatf("%s: no DUT message within %0t (check ok)", what, window));
        cov_silence++;
    endtask

    // ---- end-of-run: any unconsumed message is an error ----
    task automatic drain_check();
        MonMsg m;
        while (mbx.try_get(m))
            err($sformatf("unconsumed DUT message at end of run: %s", hexq(m.b)));
    endtask

endclass
