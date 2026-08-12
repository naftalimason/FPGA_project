`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: tb_top
// Description: Self-checking class-based verification top for chiptop
//   (uartBurst_ahbSingle). The TB is a PC model on the real board pins: it
//   drives/receives 8 Mbaud UART with RTS#/CTS# hardware flow control, builds
//   every stimulus and every expected reply from message_format_v1, and
//   verifies registers, image storage (hierarchical SRAM peek), image
//   readback, all three error/resend paths, runtime parity, FIFO status and
//   flow control. Deterministic PASS/FAIL; full log mirrored to tb_sim.log;
//   waves to dump.fst. Default build uses PROG_W=7 (8x8 image, 16 payloads)
//   via +define+SIM_SMALL_IMAGE; without the define it scales to the full
//   256x256 golden image files.
//////////////////////////////////////////////////////////////////////////////////

module tb_top;

    import UART_P::*;
    import tb_pkg::*;

    // =========================================================================
    // Parameters / image geometry
    // =========================================================================
`ifdef SIM_SMALL_IMAGE
    localparam int unsigned TB_PROG_W = 7;                 // 2^(7-1) = 64 pixels
`else
    localparam int unsigned TB_PROG_W = IMG_PROG_W;        // full 256x256
`endif
    localparam int unsigned N_PIX    = 1 << (TB_PROG_W - 1);
    localparam int unsigned N_WORDS  = N_PIX / 4;          // words/channel = payload frames
    localparam int unsigned IMG_DIM  = 1 << ((TB_PROG_W - 1) / 2);   // 8 or 256

    localparam realtime WATCHDOG_T = (TB_PROG_W <= 8) ? 20ms : 3s;

    // register map (address_map_v1 / Excel tables)
    localparam byte unsigned A2_IMG  = 8'h04;
    localparam byte unsigned A2_UART = 8'h08;
    localparam byte unsigned A2_FIFO = 8'h0C;
    localparam bit [15:0] IMG_STATUS_OFF = 16'h0000, IMG_RXMON_OFF = 16'h0004,
                          IMG_TXMON_OFF  = 16'h0008, IMG_CTRL_OFF  = 16'h000C,
                          IMG_CHK_OFF    = 16'h0010;
    localparam bit [15:0] UREG_OFF = 16'h0000, UCFG_OFF = 16'h0004,
                          UCHK_OFF = 16'h0008, CLKST_OFF = 16'h000C;
    localparam bit [15:0] FIFO_CHK_OFF = 16'h0018;

    // expected constant register values
    localparam bit [31:0] FIFO_IDLE_STAT =
        (32'(AFIFO_AE_THRESH) << 10) | (32'(AFIFO_AF_THRESH) << 6) | (1 << 3) | (1 << 2);
    localparam bit [31:0] IMG_STATUS_LOADED =
        (32'h1 << 18) | (32'(IMG_DIM) << 9) | 32'(IMG_DIM);
    localparam bit [31:0] MON_COMPLETE = 32'h0001_0000;    // packed monitor bit16

    localparam int PAUSE_SAMPLES = 68;                     // ~5 byte times @100ns

    // =========================================================================
    // Clock / reset / DUT / interface
    // =========================================================================
    logic clk100 = 1'b0;
    always #5 clk100 = ~clk100;                            // 100 MHz board clock

    logic rstn = 1'b0;                                     // CPU_RESETN

    chip_if cif (clk100);

    chiptop #(.PROG_W(TB_PROG_W)) dut (
        .CLK100MHZ   (clk100),
        .CPU_RESETN  (rstn),
        .UART_TXD_IN (cif.rx_line),
        .UART_RTS    (cif.rts_n),
        .UART_RXD_OUT(cif.tx_line),
        .UART_CTS    (cif.cts_n)
    );

    // =========================================================================
    // Golden reference image + running reference model
    // =========================================================================
    logic [31:0] ref_r [0:N_WORDS-1];
    logic [31:0] ref_g [0:N_WORDS-1];
    logic [31:0] ref_b [0:N_WORDS-1];
    bit          img_inverted = 1'b0;    // current DUT image polarity vs ref

    bit [31:0] model_img_chk  = '0;      // RW checker-register reference model
    bit [31:0] model_uart_chk = '0;
    bit [31:0] model_fifo_chk = '0;

    // =========================================================================
    // Verification environment
    // =========================================================================
    SimCfg            cfg;
    Driver            drv;
    Monitor           mon;
    Checker           chk;
    mailbox #(MonMsg) mbx;
    bit               env_built = 1'b0;

    // monitor runs in its OWN process (never disturbed by test-side forks)
    initial begin
        wait (env_built);
        mon.run();
    end

    // passive coverage: DUT RX flow control observed deasserting (CTS# high)
    int cov_dut_cts_deasserts = 0;
    always @(posedge cif.cts_n) if (rstn) cov_dut_cts_deasserts++;

    // passive spies on the DUT error/resend events (read-only hierarchical
    // taps; every event is timestamped so error episodes are fully traceable)
    int spy_phy_errs = 0, spy_mac_errs = 0, spy_clsf_errs = 0, spy_flushes = 0;
    always @(posedge dut.mmcm_clk) begin
        if (rstn && dut.gclk) begin
            if (dut.phy_error) begin
                spy_phy_errs++;
                Logger::log($sformatf("[TEST INFO] (spy) DUT phy_error event #%0d @%0t", spy_phy_errs, $realtime));
            end
            if (dut.mac_err) begin
                spy_mac_errs++;
                Logger::log($sformatf("[TEST INFO] (spy) DUT mac_err event #%0d @%0t", spy_mac_errs, $realtime));
            end
            if (dut.classifier_err) begin
                spy_clsf_errs++;
                Logger::log($sformatf("[TEST INFO] (spy) DUT classifier_err event #%0d @%0t", spy_clsf_errs, $realtime));
            end
            if (dut.flush_rx) begin
                spy_flushes++;
                Logger::log($sformatf("[TEST INFO] (spy) DUT flush_rx pulse #%0d @%0t", spy_flushes, $realtime));
            end
        end
    end

    // =========================================================================
    // Global watchdog - the simulation can never hang
    // =========================================================================
    initial begin
        #(WATCHDOG_T);
        Logger::log($sformatf("[ERROR] GLOBAL WATCHDOG: simulation exceeded %0t", WATCHDOG_T));
        Logger::log("[FINAL SUMMARY] OVERALL RESULT: FAIL (watchdog timeout)");
        Logger::close();
        $fatal(1, "global watchdog timeout");
    end

    // =========================================================================
    // Helper functions
    // =========================================================================
    function automatic byte unsigned ref_pix(int ch, int p);
        logic [31:0] w;
        case (ch)
            0:       w = ref_r[p >> 2];
            1:       w = ref_g[p >> 2];
            default: w = ref_b[p >> 2];
        endcase
        case (p & 3)
            0:       return w[31:24];
            1:       return w[23:16];
            2:       return w[15:8];
            default: return w[7:0];
        endcase
    endfunction

    function automatic byte unsigned img_pix(int ch, int p);   // expected in DUT
        return img_inverted ? ~ref_pix(ch, p) : ref_pix(ch, p);
    endfunction

    // hierarchical SRAM peek (generate scope needs constant indices)
    function automatic logic [31:0] sram_word(int ch, int w);
        case (ch)
            0:       return dut.g_bank[0].u_sram.mem[w];
            1:       return dut.g_bank[1].u_sram.mem[w];
            default: return dut.g_bank[2].u_sram.mem[w];
        endcase
    endfunction

    function automatic void make_reg_reply(input byte unsigned a2, input bit [15:0] addr,
                                           input bit [31:0] d, output byte unsigned q [$]);
        q.delete();
        q.push_back(START_BYTE); q.push_back(CHAR_R);
        q.push_back(a2); q.push_back(addr[15:8]); q.push_back(addr[7:0]);
        q.push_back(CHAR_COMMA);
        q.push_back(CHAR_V); q.push_back(8'h00); q.push_back(d[31:24]); q.push_back(d[23:16]);
        q.push_back(CHAR_COMMA);
        q.push_back(CHAR_V); q.push_back(8'h00); q.push_back(d[15:8]); q.push_back(d[7:0]);
        q.push_back(END_BYTE);
    endfunction

    function automatic void make_payload(input int f, output byte unsigned q [$]);
        int p = 4 * f;
        q.delete();
        q.push_back(START_BYTE);
        q.push_back(img_pix(0, p));   q.push_back(img_pix(1, p));
        q.push_back(img_pix(2, p));   q.push_back(img_pix(0, p+1));
        q.push_back(CHAR_COMMA);
        q.push_back(img_pix(1, p+1)); q.push_back(img_pix(2, p+1));
        q.push_back(img_pix(0, p+2)); q.push_back(img_pix(1, p+2));
        q.push_back(CHAR_COMMA);
        q.push_back(img_pix(2, p+2)); q.push_back(img_pix(0, p+3));
        q.push_back(img_pix(1, p+3)); q.push_back(img_pix(2, p+3));
        q.push_back(END_BYTE);
    endfunction

    function automatic void make_emsg(output byte unsigned q [$]);
        q.delete();
        q.push_back(START_BYTE); q.push_back(CH_E);
        q.push_back(8'h00); q.push_back(8'h00); q.push_back(8'h00);
        q.push_back(END_BYTE);
    endfunction

    // =========================================================================
    // Stimulus / check tasks (reference model included)
    // =========================================================================
    task automatic reg_write(byte unsigned a2, bit [15:0] addr, bit [31:0] d, string ctx);
        Transaction t = new(TXN_REG_WRITE);
        t.a2 = a2; t.addr = addr; t.data = d;
        chk.info($sformatf("%s: REG_WRITE A2=0x%02h addr=0x%04h data=0x%08h", ctx, a2, addr, d));
        drv.send_txn(t, ctx);
        chk.cov_reg_write++;
        drv.idle_bits(8);                       // APB write completes in << 1 byte time
    endtask

    task automatic reg_read_expect(byte unsigned a2, bit [15:0] addr, bit [31:0] expd,
                                   string ctx);
        Transaction   t = new(TXN_REG_READ);
        byte unsigned e [$];
        t.a2 = a2; t.addr = addr;
        chk.info($sformatf("%s: REG_READ A2=0x%02h addr=0x%04h expecting 0x%08h",
                           ctx, a2, addr, expd));
        drv.send_txn(t, ctx);
        make_reg_reply(a2, addr, expd, e);
        chk.expect_msg(e, ctx);
        chk.cov_reg_read++;
        drv.idle_bits(4);
    endtask

    // RW checker registers (24-bit fields): write + model update / read-vs-model
    task automatic write_checker(int which, bit [31:0] d, string ctx);
        case (which)
            0: begin reg_write(A2_IMG,  IMG_CHK_OFF,  d, ctx); model_img_chk  = d & 32'h00FFFFFF; end
            1: begin reg_write(A2_UART, UCHK_OFF,     d, ctx); model_uart_chk = d & 32'h00FFFFFF; end
            default: begin reg_write(A2_FIFO, FIFO_CHK_OFF, d, ctx); model_fifo_chk = d & 32'h00FFFFFF; end
        endcase
    endtask

    task automatic read_checker(int which, string ctx);
        case (which)
            0:       reg_read_expect(A2_IMG,  IMG_CHK_OFF,  model_img_chk,  ctx);
            1:       reg_read_expect(A2_UART, UCHK_OFF,     model_uart_chk, ctx);
            default: reg_read_expect(A2_FIFO, FIFO_CHK_OFF, model_fifo_chk, ctx);
        endcase
    endtask

    // deliberate bad frame -> expect {E} resend request, then RESYNC idle gap
    task automatic inject_expect_E(Transaction t, string what);
        byte unsigned e [$];
        byte unsigned raw [$];
        t.get_bytes(raw);
        chk.info($sformatf("%s: injecting error stimulus, wire bytes: %s%s", what, hexq(raw),
                           (t.bad_parity_pos >= 0) ? $sformatf(" [bad parity on byte %0d]", t.bad_parity_pos) :
                           (t.bad_stop_pos   >= 0) ? $sformatf(" [bad stop on byte %0d]",   t.bad_stop_pos) : ""));
        drv.send_txn(t, what);
        make_emsg(e);
        chk.expect_msg(e, {what, " -> {E} resend request"});
        chk.cov_e_replies++;
        chk.cov_err_inject++;
        drv.idle_bits(30);                      // > 11-bit RESYNC re-arm window
    endtask

    // full image write burst (header + N_WORDS payloads); optional mid-burst error
    task automatic image_write(bit inverted, int err_at, string ctx);
        Transaction   t;
        byte unsigned e [$];
        t = new(TXN_IMG_HDR);
        t.height = 24'(IMG_DIM); t.width = 24'(IMG_DIM);
        chk.info($sformatf("%s: IMG_WRITE_HDR H=W=%0d, then %0d pixel payloads%s",
                           ctx, IMG_DIM, N_WORDS,
                           (err_at >= 0) ? $sformatf(" (illegal frame injected before payload %0d)", err_at) : ""));
        drv.send_txn(t, {ctx, " header"});
        drv.idle_bits(6);
        for (int f = 0; f < N_WORDS; f++) begin
            if (f == err_at) begin
                Transaction bad = new(TXN_REG_READ);   // 6-byte frame: illegal in burst
                bad.a2 = A2_IMG; bad.addr = '0;
                inject_expect_E(bad, {ctx, " mid-burst illegal 6-byte frame"});
            end
            t = new(TXN_PIXEL);
            for (int k = 0; k < 4; k++) begin
                t.pr[k] = inverted ? ~ref_pix(0, 4*f + k) : ref_pix(0, 4*f + k);
                t.pg[k] = inverted ? ~ref_pix(1, 4*f + k) : ref_pix(1, 4*f + k);
                t.pb[k] = inverted ? ~ref_pix(2, 4*f + k) : ref_pix(2, 4*f + k);
            end
            drv.send_txn(t, $sformatf("%s payload %0d", ctx, f));
        end
        img_inverted = inverted;
        chk.cov_img_write++;
        drv.idle_bits(60);                      // drain to SRAM + monitor sync settle
    endtask

    // full image read burst; every payload checked byte-exact, optional CTS pause
    task automatic image_read(bit cts_pause, string ctx);
        Transaction   t = new(TXN_IMG_RD_REQ);
        byte unsigned e [$];
        t.height = 24'(IMG_DIM); t.width = 24'(IMG_DIM);
        chk.info($sformatf("%s: IMG_READ_REQ H=W=%0d, expecting %0d payload replies%s",
                           ctx, IMG_DIM, N_WORDS, cts_pause ? " (with PC-RTS# pause)" : ""));
        drv.send_txn(t, ctx);
        for (int f = 0; f < N_WORDS; f++) begin
            make_payload(f, e);
            chk.expect_msg(e, $sformatf("%s payload %0d/%0d", ctx, f, N_WORDS-1), 1ms);
            chk.cov_payload_rx++;
            if (cts_pause && f == 3) do_cts_pause();
        end
        chk.cov_img_read++;
        drv.idle_bits(10);
    endtask

    // deassert PC RTS# mid-stream: DUT must finish in-flight byte(s), then hold
    task automatic do_cts_pause();
        bit line_stayed_idle = 1'b1;
        chk.info("CTS pause: deasserting PC RTS# mid burst-read stream");
        drv.set_pc_ready(0);
        #(3 * TB_BYTE_T);                       // grace: in-flight byte(s) finish
        for (int i = 0; i < PAUSE_SAMPLES; i++) begin
            #100ns;
            if (cif.tx_line !== 1'b1) line_stayed_idle = 1'b0;
        end
        chk.check_true(line_stayed_idle,
                       "no DUT TX activity while PC RTS# deasserted (flow-control hold)");
        drv.set_pc_ready(1);
        chk.cov_cts_pauses++;
    endtask

    // hierarchical SRAM compare against the reference image
    task automatic check_sram(string ctx);
        logic [31:0] expw, obsw;
        int          bad = 0;
        for (int ch = 0; ch < 3; ch++) begin
            for (int w = 0; w < N_WORDS; w++) begin
                case (ch)
                    0:       expw = img_inverted ? ~ref_r[w] : ref_r[w];
                    1:       expw = img_inverted ? ~ref_g[w] : ref_g[w];
                    default: expw = img_inverted ? ~ref_b[w] : ref_b[w];
                endcase
                obsw = sram_word(ch, w);
                if (obsw !== expw) begin
                    chk.err($sformatf("%s: SRAM[%s] word %0d = 0x%08h, expected 0x%08h",
                                      ctx, (ch==0) ? "R" : (ch==1) ? "G" : "B", w, obsw, expw));
                    bad++;
                end
                else chk.cov_sram_words++;
            end
        end
        if (bad == 0)
            chk.info($sformatf("%s: all 3x%0d SRAM words match the reference image (check ok)",
                               ctx, N_WORDS));
    endtask

    task automatic check_fifo_regs_idle(string ctx);
        reg_read_expect(A2_FIFO, 16'h0000, FIFO_IDLE_STAT, {ctx, " fifo_rx_r"});
        reg_read_expect(A2_FIFO, 16'h0004, FIFO_IDLE_STAT, {ctx, " fifo_rx_g"});
        reg_read_expect(A2_FIFO, 16'h0008, FIFO_IDLE_STAT, {ctx, " fifo_rx_b"});
        reg_read_expect(A2_FIFO, 16'h000C, FIFO_IDLE_STAT, {ctx, " fifo_tx_r"});
        reg_read_expect(A2_FIFO, 16'h0010, FIFO_IDLE_STAT, {ctx, " fifo_tx_g"});
        reg_read_expect(A2_FIFO, 16'h0014, FIFO_IDLE_STAT, {ctx, " fifo_tx_b"});
    endtask

    // =========================================================================
    // Final summary
    // =========================================================================
    task automatic final_summary();
        int mon_errs = mon.total_errs();
        int grand    = chk.total_errors + mon_errs;
        chk.drain_check();
        grand = chk.total_errors + mon_errs;
        Logger::log("");
        Logger::log("[FINAL SUMMARY] ============================================================");
        Logger::log($sformatf("[FINAL SUMMARY] Tests executed        : %0d", chk.tests_run));
        Logger::log($sformatf("[FINAL SUMMARY] Tests passed          : %0d", chk.tests_pass));
        Logger::log($sformatf("[FINAL SUMMARY] Tests failed          : %0d", chk.tests_fail));
        Logger::log($sformatf("[FINAL SUMMARY] Total detected errors : %0d (scoreboard %0d + UART line %0d)",
                             grand, chk.total_errors, mon_errs));
        Logger::log("[FINAL SUMMARY] Feature / coverage counters:");
        Logger::log($sformatf("[FINAL SUMMARY]   reg writes driven          : %0d", chk.cov_reg_write));
        Logger::log($sformatf("[FINAL SUMMARY]   reg reads checked          : %0d", chk.cov_reg_read));
        Logger::log($sformatf("[FINAL SUMMARY]   image write bursts         : %0d", chk.cov_img_write));
        Logger::log($sformatf("[FINAL SUMMARY]   image read bursts          : %0d", chk.cov_img_read));
        Logger::log($sformatf("[FINAL SUMMARY]   payload replies checked    : %0d", chk.cov_payload_rx));
        Logger::log($sformatf("[FINAL SUMMARY]   error injections           : %0d", chk.cov_err_inject));
        Logger::log($sformatf("[FINAL SUMMARY]   {E} resend replies checked : %0d", chk.cov_e_replies));
        Logger::log($sformatf("[FINAL SUMMARY]   SRAM words verified        : %0d", chk.cov_sram_words));
        Logger::log($sformatf("[FINAL SUMMARY]   silence (negative) checks  : %0d", chk.cov_silence));
        Logger::log($sformatf("[FINAL SUMMARY]   parity-mode switches       : %0d", chk.cov_parity_switch));
        Logger::log($sformatf("[FINAL SUMMARY]   PC-RTS# pause windows      : %0d", chk.cov_cts_pauses));
        Logger::log($sformatf("[FINAL SUMMARY]   random operations          : %0d", chk.cov_random_ops));
        Logger::log($sformatf("[FINAL SUMMARY]   DUT CTS# deasserts seen    : %0d", cov_dut_cts_deasserts));
        Logger::log($sformatf("[FINAL SUMMARY]   PC bytes sent / DUT bytes received back : %0d / %0d",
                             drv.bytes_sent, mon.bytes_seen));
        Logger::log($sformatf("[FINAL SUMMARY]   driver waits on DUT CTS#   : %0d", drv.cts_wait_events));
        if (grand == 0 && chk.tests_fail == 0) begin
            Logger::log("[FINAL SUMMARY] OVERALL RESULT: PASS");
            Logger::close();
            $finish;
        end
        else begin
            Logger::log("[FINAL SUMMARY] OVERALL RESULT: FAIL");
            Logger::close();
            $fatal(1, "verification FAILED");
        end
    endtask

    // =========================================================================
    // Main test program
    // =========================================================================
    initial begin : main
        Transaction t;
        Transaction bad;
        byte unsigned e [$];
        int n_rand_illegal;

        $timeformat(-9, 0, " ns", 10);
        Logger::open("tb_sim.log");
        $dumpfile("dump.fst");
        $dumpvars(0, tb_top);

        Logger::log("[TEST INFO] ================================================================");
        Logger::log("[TEST INFO] uartBurst_ahbSingle chiptop verification");
        Logger::log($sformatf("[TEST INFO] PROG_W=%0d -> image %0dx%0d (%0d pixels, %0d payload frames)",
                             TB_PROG_W, IMG_DIM, IMG_DIM, N_PIX, N_WORDS));
        Logger::log("[TEST INFO] ================================================================");

        // golden image
`ifdef SIM_SMALL_IMAGE
        $readmemh("tb_red_8x8.mem",   ref_r);
        $readmemh("tb_green_8x8.mem", ref_g);
        $readmemh("tb_blue_8x8.mem",  ref_b);
`else
        $readmemh("red_hex.mem",   ref_r);
        $readmemh("green_hex.mem", ref_g);
        $readmemh("blue_hex.mem",  ref_b);
`endif

        // environment
        cfg = new();
        mbx = new();
        drv = new(cif, cfg);
        mon = new(cif, cfg, mbx);
        chk = new(mbx);
        env_built = 1'b1;

        // reset sequence
        rstn = 1'b0;
        #1us;
        rstn = 1'b1;
        #5us;                                   // MMCM lock + both reset syncs + gclk

        // ---------------------------------------------------------------------
        chk.test_begin("T00_reset_idle",
            "reset behaviour: UART line idles high, DUT RX flow control asserted");
        chk.check_true(cif.tx_line === 1'b1, "UART_RXD_OUT idles high after reset");
        chk.check_true(cif.cts_n  === 1'b0, "UART_CTS asserted (DUT ready to receive)");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T01_reset_register_values",
            "read every register over APB and compare against the documented reset values");
        reg_read_expect(A2_IMG,  IMG_STATUS_OFF, 32'h0,       "img_status reset");
        reg_read_expect(A2_IMG,  IMG_RXMON_OFF,  32'h0,       "img_rx_monitor reset");
        reg_read_expect(A2_IMG,  IMG_TXMON_OFF,  32'h0,       "img_tx_monitor reset");
        reg_read_expect(A2_IMG,  IMG_CTRL_OFF,   32'h0,       "img_ctrl reset");
        reg_read_expect(A2_IMG,  IMG_CHK_OFF,    32'h0,       "img_checker reset");
        reg_read_expect(A2_UART, UREG_OFF,       32'h0,       "uart_reg reset (R0C read)");
        reg_read_expect(A2_UART, UCFG_OFF,       32'h0,       "uart_config is WO - reads 0");
        reg_read_expect(A2_UART, UCHK_OFF,       32'h0,       "uart_checker reset");
        reg_read_expect(A2_UART, CLKST_OFF,      32'h1,       "clk_status: fast clock selected");
        check_fifo_regs_idle("reset");
        reg_read_expect(A2_FIFO, FIFO_CHK_OFF,   32'h0,       "fifo_checker reset");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T02_register_write_read",
            "RW checker registers on all 3 RGFs: min/max/boundary data, back-to-back ops, RO/unmapped/aliased addresses");
        write_checker(0, 32'hFFFF_FFFF, "img_checker max");     // 24-bit field clip
        read_checker (0, "img_checker readback (0x00FFFFFF)");
        write_checker(0, 32'h0000_0000, "img_checker min");
        read_checker (0, "img_checker readback (0)");
        write_checker(1, 32'hA5C3_F0E1, "uart_checker pattern");
        read_checker (1, "uart_checker readback");
        write_checker(2, 32'h1234_5678, "fifo_checker pattern");
        read_checker (2, "fifo_checker readback");
        // back-to-back: two writes then two reads at full message rate
        write_checker(0, 32'h0011_1111, "img_checker b2b write 1");
        write_checker(0, 32'h0022_2222, "img_checker b2b write 2");
        read_checker (0, "img_checker b2b read 1");
        read_checker (0, "img_checker b2b read 2");
        // write to a read-only register: must be ignored
        reg_write(A2_IMG, IMG_STATUS_OFF, 32'hDEAD_BEEF, "write attempt to RO img_status");
        reg_read_expect(A2_IMG, IMG_STATUS_OFF, 32'h0, "RO img_status unchanged");
        // unmapped local offset: write ignored, read returns 0
        reg_write(A2_IMG, 16'h0100, 32'h55AA_55AA, "write to unmapped offset 0x0100");
        reg_read_expect(A2_IMG, 16'h0100, 32'h0, "unmapped offset reads 0");
        read_checker (0, "img_checker unaffected by unmapped access");
        // unmapped BAR A2: as-built aliasing (BAR default rgf_sel=0 -> img_rgf)
        reg_read_expect(8'h10, IMG_STATUS_OFF, 32'h0,
                        "unmapped A2=0x10 aliases to img_rgf (as-built BAR default)");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T03_read_refusal_no_image",
            "IMG_READ_REQ while ready_to_read=0 must be refused SILENTLY (no reply), system stays alive");
        t = new(TXN_IMG_RD_REQ);
        t.height = 24'(IMG_DIM); t.width = 24'(IMG_DIM);
        drv.send_txn(t, "read request before any image");
        chk.expect_silence(300us, "silent refusal of premature image read");
        read_checker(1, "aliveness read after refusal");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T04_image_write",
            "full image write burst: header + pixel payloads -> SRAM; verify status, monitor, SRAM contents, FIFO health");
        image_write(1'b0, -1, "image write #1");
        reg_read_expect(A2_IMG, IMG_STATUS_OFF, IMG_STATUS_LOADED,
                        "img_status: ready_to_read + H/W loaded");
        reg_read_expect(A2_IMG, IMG_RXMON_OFF, MON_COMPLETE,
                        "img_rx_monitor: write_complete set, counters wrapped");
        reg_read_expect(A2_IMG, IMG_CTRL_OFF, 32'h0, "img_ctrl pulses all returned low");
        check_sram("image write #1");
        check_fifo_regs_idle("post image write");   // incl. sticky ovf/unf = 0
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T05_image_read",
            "full image read burst: every payload reply checked byte-exact against the reference image");
        image_read(1'b0, "image read #1");
        reg_read_expect(A2_IMG, IMG_TXMON_OFF, MON_COMPLETE,
                        "img_tx_monitor: read_complete set");
        reg_read_expect(A2_IMG, IMG_STATUS_OFF, IMG_STATUS_LOADED,
                        "img_status: image still valid after read");
        check_fifo_regs_idle("post image read");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T06_error_paths",
            "all three error sources (phy parity/framing, mac structure, classifier content) -> {E} resend + R0C counters + recovery");
        // (a) parity error -> phy_err
        t = new(TXN_REG_WRITE);
        t.a2 = A2_UART; t.addr = UCHK_OFF; t.data = 32'h0BAD_0BAD;
        t.bad_parity_pos = 2; t.truncate_after = 2;
        inject_expect_E(t, "phy parity error");
        write_checker(1, 32'h00BE_EF11, "recovery write after parity error");
        read_checker (1, "recovery readback after parity error");
        // (b) stop-bit (framing) error -> phy_err
        t = new(TXN_RAW);
        t.raw.push_back(8'h55);
        t.bad_stop_pos = 0;
        inject_expect_E(t, "phy stop-bit framing error");
        // (c) frame-structure error -> mac_err (boundary byte neither ',' nor '}')
        t = new(TXN_RAW);
        t.raw.push_back(START_BYTE); t.raw.push_back(CHAR_R);
        t.raw.push_back(8'h01); t.raw.push_back(8'h02); t.raw.push_back(8'h03);
        t.raw.push_back(8'h51);                        // 'Q' at boundary slot
        inject_expect_E(t, "mac frame-structure error");
        // (d) illegal opcode, legal skeleton -> classifier_err
        t = new(TXN_RAW);
        t.raw.push_back(START_BYTE); t.raw.push_back(8'h51);   // 'Q' opcode
        t.raw.push_back(8'h00); t.raw.push_back(8'h00); t.raw.push_back(8'h00);
        t.raw.push_back(END_BYTE);
        inject_expect_E(t, "classifier illegal opcode (6-byte)");
        // (e) reserved 11-byte / 2-submessage frame -> classifier_err
        t = new(TXN_RAW);
        t.raw.push_back(START_BYTE); t.raw.push_back(CHAR_W);
        t.raw.push_back(8'h01); t.raw.push_back(8'h02); t.raw.push_back(8'h03);
        t.raw.push_back(CHAR_COMMA);
        t.raw.push_back(CHAR_V); t.raw.push_back(8'h04); t.raw.push_back(8'h05);
        t.raw.push_back(8'h06);
        t.raw.push_back(END_BYTE);
        inject_expect_E(t, "reserved 11-byte frame");
        // (f) pixel payload without a header (normal mode, no opcode match)
        t = new(TXN_PIXEL);                            // all-zero pixels
        inject_expect_E(t, "pixel payload without burst header");
        // counters: phy=2, mac=1, clsf=3; R0C read returns then clears
        reg_read_expect(A2_UART, UREG_OFF, 32'h0003_0102,
                        "uart_reg counters {clsf=3,mac=1,phy=2} (R0C read)");
        reg_read_expect(A2_UART, UREG_OFF, 32'h0,
                        "uart_reg cleared by previous read (R0C)");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T07_image_overwrite_with_midburst_error",
            "second image session (inverted data), illegal frame mid-burst -> {E} + resend, burst resumes seamlessly");
        image_write(1'b1, 7, "image write #2 (inverted)");
        reg_read_expect(A2_IMG, IMG_STATUS_OFF, IMG_STATUS_LOADED,
                        "img_status: ready again after overwrite");
        reg_read_expect(A2_IMG, IMG_RXMON_OFF, MON_COMPLETE,
                        "img_rx_monitor: write_complete for session 2");
        check_sram("image overwrite (inverted)");
        reg_read_expect(A2_UART, UREG_OFF, 32'h0001_0000,
                        "uart_reg: exactly one classifier error from the mid-burst inject");
        reg_read_expect(A2_UART, UREG_OFF, 32'h0, "uart_reg cleared again (R0C)");
        chk.info({"NOTE: the first payload below guards the tx_afifo session-flush ",
                  "race (BUG-1, fixed in seq_top_fsm v2.4: the setup states hold the ",
                  "img_rgf grant and arm the FSMs only SRST_SETTLE ticks after the ",
                  "trio flush). Before the fix this reply carried the PREVIOUS ",
                  "image's word 0. See tb_features.txt."});
        image_read(1'b0, "image read #2 (inverted data)");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T08_runtime_parity",
            "uart_config runtime parity: switch to EVEN, run traffic both directions, violate it, switch back to ODD");
        reg_write(A2_UART, UCFG_OFF, 32'h0, "uart_config <= even parity");
        drv.idle_bits(30);
        cfg.parity_odd = 1'b0;                  // PC follows the DUT's new mode
        chk.cov_parity_switch++;
        chk.info("PC parity mode switched to EVEN");
        write_checker(0, 32'h0077_7777, "checker write under EVEN parity");
        read_checker (0, "checker readback under EVEN parity");
        // a byte whose parity is wrong for the CURRENT (even) mode
        t = new(TXN_RAW);
        t.raw.push_back(8'h33);
        t.bad_parity_pos = 0;
        inject_expect_E(t, "parity violation under EVEN mode");
        reg_read_expect(A2_UART, UREG_OFF, 32'h0000_0001,
                        "uart_reg: one phy error under EVEN mode (R0C)");
        reg_write(A2_UART, UCFG_OFF, 32'h1, "uart_config <= odd parity (restore)");
        drv.idle_bits(30);
        cfg.parity_odd = 1'b1;
        chk.cov_parity_switch++;
        chk.info("PC parity mode switched back to ODD");
        read_checker(0, "checker readback back under ODD parity");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T09_txflow_control_pause",
            "repeat full image read with a PC-RTS# pause mid-stream: DUT must hold at a byte boundary and resume losslessly");
        image_read(1'b1, "image read #3 (with CTS pause)");
        reg_read_expect(A2_IMG, IMG_TXMON_OFF, MON_COMPLETE,
                        "img_tx_monitor: read_complete after paused read");
        chk.test_end();

        // ---------------------------------------------------------------------
        chk.test_begin("T10_constrained_random",
            "constrained-random register traffic vs reference model, with random illegal frames folded in");
        n_rand_illegal = 0;
        for (int i = 0; i < 16; i++) begin
            t = new();
            if (!t.randomize()) begin           // fallback if solver unavailable
                t.kind = (($urandom & 1) != 0) ? TXN_REG_WRITE : TXN_REG_READ;
                case ($urandom_range(0, 2))
                    0: begin t.a2 = A2_IMG;  t.addr = IMG_CHK_OFF;  end
                    1: begin t.a2 = A2_UART; t.addr = UCHK_OFF;     end
                    default: begin t.a2 = A2_FIFO; t.addr = FIFO_CHK_OFF; end
                endcase
                t.data = $urandom;
            end
            if (t.kind == TXN_REG_WRITE) begin
                case (t.a2)
                    A2_IMG:  begin write_checker(0, t.data, $sformatf("random op %0d: write img_checker", i));  end
                    A2_UART: begin write_checker(1, t.data, $sformatf("random op %0d: write uart_checker", i)); end
                    default: begin write_checker(2, t.data, $sformatf("random op %0d: write fifo_checker", i)); end
                endcase
            end
            else begin
                case (t.a2)
                    A2_IMG:  read_checker(0, $sformatf("random op %0d: read img_checker", i));
                    A2_UART: read_checker(1, $sformatf("random op %0d: read uart_checker", i));
                    default: read_checker(2, $sformatf("random op %0d: read fifo_checker", i));
                endcase
            end
            chk.cov_random_ops++;
            if ((i % 5) == 4) begin             // fold in a random illegal frame
                bad = new(TXN_RAW);             // fresh object each pass (the
                                                //   block is static - a declared
                                                //   initializer would run once)
                bad.raw.push_back(START_BYTE);
                bad.raw.push_back(8'h58);       // 'X' - never a legal opcode
                bad.raw.push_back(byte'($urandom)); bad.raw.push_back(byte'($urandom));
                bad.raw.push_back(byte'($urandom));
                bad.raw.push_back(END_BYTE);
                inject_expect_E(bad, $sformatf("random illegal frame after op %0d", i));
                n_rand_illegal++;
            end
        end
        reg_read_expect(A2_UART, UREG_OFF, 32'(n_rand_illegal) << 16,
                        "uart_reg: classifier count matches random illegal frames (R0C)");
        reg_read_expect(A2_UART, UREG_OFF, 32'h0, "uart_reg cleared (R0C)");
        chk.test_end();

        // ---------------------------------------------------------------------
        final_summary();
    end

endmodule
