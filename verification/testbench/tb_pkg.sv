`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Package: tb_pkg
// Description: Verification package for the uartBurst_ahbSingle chiptop TB.
//   Holds the shared timing constants, the Logger (terminal + persistent log
//   file), the runtime SimCfg (parity mode), and - via `include - the class
//   library: Transaction, MonMsg/Monitor, Driver, Checker.
//////////////////////////////////////////////////////////////////////////////////

package tb_pkg;

    import UART_P::*;

    // ---- global TB timing constants (8 Mbaud UART, matches UART_P::BAUD_RATE) ----
    localparam realtime TB_BIT_T  = 125ns;              // one UART bit
    localparam realtime TB_BYTE_T = 11 * TB_BIT_T;      // start+8+parity+stop

    localparam byte unsigned CH_E = 8'h45;              // 'E' (not in UART_P)

    // ---- helper: format a byte queue as hex ----
    function automatic string hexq(byte unsigned q [$]);
        string s = "";
        foreach (q[i]) s = {s, $sformatf("%02h ", q[i])};
        return s;
    endfunction

    // ---- Logger: everything goes to the terminal AND tb_sim.log ----
    class Logger;
        static int fd = 0;
        static function void open(string name);
            fd = $fopen(name, "w");
            if (fd == 0) $display("[WARNING] Logger: could not open %s", name);
        endfunction
        static function void log(string s);
            $display("%s", s);
            if (fd) $fdisplay(fd, "%s", s);
        endfunction
        static function void close();
            if (fd) $fclose(fd);
            fd = 0;
        endfunction
    endclass

    // ---- shared runtime configuration (uart_rgf.uart_config mirror) ----
    class SimCfg;
        bit parity_odd = 1'b1;      // reset default = odd (UART_P::PARITY_ODD)
    endclass

    `include "Transaction.sv"
    `include "Monitor.sv"
    `include "Driver.sv"
    `include "Checker.sv"

endpackage
