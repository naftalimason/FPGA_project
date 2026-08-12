`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: chip_if
// Description: UART + modem-line interface between the PC-model testbench and
//   chiptop. Directions are from the TB (PC) point of view; modem lines are
//   active-low exactly like the FT2232 pads on the Nexys A7:
//     rx_line -> chiptop.UART_TXD_IN   (PC -> DUT serial)
//     tx_line <- chiptop.UART_RXD_OUT  (DUT -> PC serial)
//     rts_n   -> chiptop.UART_RTS      (PC RTS#: 0 = PC ready to RECEIVE)
//     cts_n   <- chiptop.UART_CTS      (DUT CTS#: 0 = DUT ready to RECEIVE)
//////////////////////////////////////////////////////////////////////////////////

interface chip_if (input logic clk100);

    logic rx_line;   // PC -> DUT serial
    logic tx_line;   // DUT -> PC serial
    logic rts_n;     // PC RTS# (DUT TX flow control input)
    logic cts_n;     // DUT CTS# (PC TX flow control; sample before each byte)

    modport tb  (input clk100, tx_line, cts_n, output rx_line, rts_n);
    modport dut (input rx_line, rts_n, output tx_line, cts_n);

endinterface
