`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 04/19/2026 07:54:53 PM
// Design Name:
// Module Name: UART_package
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//   Shared UART/system parameters.
//
//   IMPORTANT:
//   SYS_CLK_FREQ and BAUD_RATE are compile-time constants.
//   The multiplication/division below is done at elaboration/synthesis constant
//   evaluation time. It does NOT infer runtime multiplier/divider hardware.
//
// Dependencies:
//
// Revision:
// Revision 0.03 - Reduced-ratio UART RX 16x timing
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

package UART_P;

    // -------------------------------------------------------------------------
    // Constant compile-time helper functions
    // -------------------------------------------------------------------------
    function automatic int unsigned gcd_u(
        input int unsigned a,
        input int unsigned b
    );
        int unsigned t;
        begin
            while (b != 0) begin
                t = b;
                b = a % b;
                a = t;
            end
            return a;
        end
    endfunction


    // -------------------------------------------------------------------------
    // System and UART clock frequencies
    //
    // Lab 10 now has two clock domains:
    //   SYS_CLK_FREQ  = original board/system clock domain (RGF, ROM, FIFO write side)
    //   UART_CLK_FREQ = selected UART/MMCM clock domain (UART RX/TX, FIFO read side)
    //
    // Do not change SYS_CLK_FREQ when changing the MMCM UART clock.
    // Change UART_CLK_FREQ to match clk_wiz_0 clk_out1.
    // -------------------------------------------------------------------------
    localparam int unsigned SYS_CLK_FREQ  = 100_000_000; // board/system clock
    localparam int unsigned UART_CLK_FREQ = 280_000_000; // selected UART/MMCM clock
    localparam int unsigned BAUD_RATE     = 8_000_000;   // UART baud rate; keep Python baudrate equal

    // -------------------------------------------------------------------------
    // Seven segment display parameters
    // -------------------------------------------------------------------------
    localparam int unsigned DIGITS   = 8;
    localparam int unsigned DIGIT    = 4;
    localparam int unsigned SEG      = 7;
    localparam int unsigned ANODES   = 8;
    localparam int unsigned AN_SEL   = $clog2(ANODES);
    localparam int unsigned REFRESH_VAL   = 31249;
    localparam int unsigned REFRESH_WIDTH = 15;

    // -------------------------------------------------------------------------
    // UART protocol/data parameters
    // -------------------------------------------------------------------------
    localparam int unsigned D_SIZE   = 8;
    localparam int unsigned SQR_SIZE = 8;

    // -------------------------------------------------------------------------
    // TX baud timing parameters
    //
    // The TX timer produces one pulse every UART bit period.
    // If UART_CLK_FREQ / BAUD_RATE is not an integer, the timer alternates between
    // BASE_DIV and BASE_DIV+1 clock periods using a small remainder accumulator.
    //
    // Example:
    //   UART_CLK_FREQ = 100 MHz, BAUD_RATE = 115200
    //   BASE_DIV     = 868
    //   some periods = 868 clocks, some = 869 clocks
    //
    // Runtime hardware required:
    //   - clock counter
    //   - small remainder accumulator
    //   - constant compare/add/sub only when a baud tick is generated
    // -------------------------------------------------------------------------
    localparam int unsigned UART_TX_TICK_RATE = BAUD_RATE;

    localparam int unsigned UART_TX_BASE_DIV =
        (UART_TX_TICK_RATE == 0) ? 1 : (UART_CLK_FREQ / UART_TX_TICK_RATE);

    localparam int unsigned UART_TX_REMAINDER =
        (UART_TX_TICK_RATE == 0) ? 0 : (UART_CLK_FREQ - (UART_TX_BASE_DIV * UART_TX_TICK_RATE));

    localparam int unsigned UART_TX_FRAC_THRESHOLD =
        (UART_TX_REMAINDER == 0) ? UART_TX_TICK_RATE : (UART_TX_TICK_RATE - UART_TX_REMAINDER);

    // Terminal counter values. A terminal count of N means the period is N+1 clocks.
    localparam int unsigned UART_TX_BASE_TERM =
        (UART_TX_BASE_DIV > 0) ? (UART_TX_BASE_DIV - 1) : 0;

    localparam int unsigned UART_TX_EXT_TERM =
        UART_TX_BASE_DIV;

    localparam int unsigned UART_TX_MAX_TERM =
        (UART_TX_REMAINDER == 0) ? UART_TX_BASE_TERM : UART_TX_EXT_TERM;

    // Kept for compatibility with existing code names.
    localparam int unsigned UART_BIT_TIME  = UART_TX_BASE_DIV;
    localparam int unsigned BIT_TIME_WIDTH =
        (UART_TX_MAX_TERM < 2) ? 1 : $clog2(UART_TX_MAX_TERM + 1);

    localparam int unsigned UART_TX_FRAC_WIDTH =
        (UART_TX_TICK_RATE < 2) ? 1 : $clog2(UART_TX_TICK_RATE + 1);

    // -------------------------------------------------------------------------
    // RX 16x oversampling timing parameters
    //
    // The RX PHY needs BAUD_RATE*16 sample ticks.
    //
    // Instead of using a wide fixed-point fractional accumulator, the package
    // reduces the required ratio at compile time:
    //
    //      UART_RX_TICK_RATE / UART_CLK_FREQ = BAUD_16_NUM / BAUD_16_DEN
    //
    // The runtime generator therefore only needs a reduced-size accumulator that
    // produces BAUD_16_NUM enable pulses every BAUD_16_DEN UART clock cycles.
    // This keeps the baud generator modular while avoiding a large timing path.
    //
    // Requirement: UART_RX_TICK_RATE must be less than or equal to UART_CLK_FREQ.
    // -------------------------------------------------------------------------
    localparam int unsigned UART_RX_OVERSAMPLE = 16;
    localparam int unsigned UART_RX_TICK_RATE  = BAUD_RATE * UART_RX_OVERSAMPLE;

    localparam int unsigned BAUD_16_GCD =
        (UART_RX_TICK_RATE == 0) ? 1 : gcd_u(UART_CLK_FREQ, UART_RX_TICK_RATE);

    localparam int unsigned BAUD_16_NUM =
        (UART_RX_TICK_RATE == 0) ? 0 : (UART_RX_TICK_RATE / BAUD_16_GCD);

    localparam int unsigned BAUD_16_DEN =
        (UART_RX_TICK_RATE == 0) ? 1 : (UART_CLK_FREQ / BAUD_16_GCD);

    localparam int unsigned BAUD_16_ACC_WIDTH =
        ((BAUD_16_DEN + BAUD_16_NUM) < 2) ? 1 : $clog2(BAUD_16_DEN + BAUD_16_NUM + 1);

    // Kept for compatibility with existing code names.
    localparam int unsigned BAUD_16_BASE_DIV =
        (UART_RX_TICK_RATE == 0) ? 1 : (UART_CLK_FREQ / UART_RX_TICK_RATE);

    localparam int unsigned BAUD_16_REMAINDER =
        (UART_RX_TICK_RATE == 0) ? 0 : (UART_CLK_FREQ - (BAUD_16_BASE_DIV * UART_RX_TICK_RATE));

    localparam int unsigned BAUD_16_FRAC_THRESHOLD =
        (BAUD_16_REMAINDER == 0) ? UART_RX_TICK_RATE : (UART_RX_TICK_RATE - BAUD_16_REMAINDER);

    localparam int unsigned BAUD_16_BASE_TERM =
        (BAUD_16_BASE_DIV > 0) ? (BAUD_16_BASE_DIV - 1) : 0;

    localparam int unsigned BAUD_16_EXT_TERM =
        BAUD_16_BASE_DIV;

    localparam int unsigned BAUD_16_MAX_TERM =
        (BAUD_16_REMAINDER == 0) ? BAUD_16_BASE_TERM : BAUD_16_EXT_TERM;

    localparam int unsigned BAUD_16_LO    = BAUD_16_BASE_DIV;
    localparam int unsigned BAUD_16_HI    = BAUD_16_BASE_DIV + ((BAUD_16_REMAINDER != 0) ? 1 : 0);
    localparam int unsigned BAUD_16_WIDTH =
        (BAUD_16_MAX_TERM < 2) ? 1 : $clog2(BAUD_16_MAX_TERM + 1);

    localparam int unsigned BAUD_16_FRAC_WIDTH = BAUD_16_ACC_WIDTH;

    // -------------------------------------------------------------------------
    // UART parity mode (shared by rx_phy and tx_phy - MUST match on both sides)
    // 1 = ODD parity (project decision; rx_phy_spec_v1 sec 3/8, tx_phy_spec_v1)
    // -------------------------------------------------------------------------
    localparam logic PARITY_ODD = 1'b1;

    // -------------------------------------------------------------------------
    // RX sampling thresholds
    // -------------------------------------------------------------------------
    localparam logic [3:0] RX_START_THRESHOLD   = 4'd7;
    localparam logic [3:0] RX_FULL_THRESHOLD    = 4'd15;
    // Short CLEANUP tail: byte_done must fire (and the FSM reach IDLE) before the
    // NEXT back-to-back byte's start edge (~1-2 ticks after the stop cell ends).
    // 6 left the FSM in CLEANUP when that edge arrived -> start missed; 3 gives
    // a 2-3 tick margin (rx_phy_fsm also catches a start edge inside CLEANUP).
    localparam logic [3:0] RX_CLEANUP_THRESHOLD = 4'd3;
    localparam int unsigned RX_MIDPOINT_WIDTH   = 4;

    // -------------------------------------------------------------------------
    // RX post-flush re-sync (error_resend_flow): after flush_rx the receiver
    // re-arms only once the line has been continuously idle-high for one full
    // character time (11 bits x 16 ticks). This skips any tail bytes of the
    // errored frame still arriving (full-duplex: {E} completes before the PC's
    // frame ends), preventing mid-byte misframing cascades. The PC cannot
    // resend before receiving the 6-byte {E}, so the quiet gap is guaranteed.
    // -------------------------------------------------------------------------
    localparam int unsigned RX_RESYNC_TICKS = 11 * 16;   // one character time
    localparam int unsigned RX_RESYNC_CNT_W = 8;

    // -------------------------------------------------------------------------
    // Timer and delay parameters
    // -------------------------------------------------------------------------
    localparam int unsigned LOG2_HOLD_TIME = 25;
    localparam logic [24:0] DELAY_MODE_0 = 25'd2;
    localparam logic [24:0] DELAY_MODE_1 = 25'd5_000_000;
    localparam logic [24:0] DELAY_MODE_2 = 25'd10_000_000;
    localparam logic [24:0] DELAY_MODE_3 = 25'd20_000_000;

    // -------------------------------------------------------------------------
    // Matrix size thresholds
    // -------------------------------------------------------------------------
    localparam logic [7:0] SIZE_VAL_0 = 8'd0;
    localparam logic [7:0] SIZE_VAL_1 = 8'd31;
    localparam logic [7:0] SIZE_VAL_2 = 8'd127;
    localparam logic [7:0] SIZE_VAL_3 = 8'd255;

    // -------------------------------------------------------------------------
    // Push button parameters
    // -------------------------------------------------------------------------
    localparam int unsigned PB_TIME = 21;
    localparam logic [20:0] PB_COUNT_LIMIT = 21'd1_999_999;

    // -------------------------------------------------------------------------
    // Message and protocol characters
    // -------------------------------------------------------------------------
    localparam int unsigned MSG_LEN = 16;
    localparam int unsigned MSG_WIDTH = MSG_LEN * D_SIZE;
    localparam logic [7:0] START_BYTE = 8'h7B; // '{'
    localparam logic [7:0] END_BYTE   = 8'h7D; // '}'
    localparam logic [7:0] CHAR_COMMA = 8'h2C; // ','
    localparam logic [7:0] CHAR_SPACE = 8'h20; // ' '
    localparam logic [7:0] CHAR_LF    = 8'h0A; // '\n'
    localparam logic [7:0] CHAR_CR    = 8'h0D; // '\r'

    // -------------------------------------------------------------------------
    // Message opcodes
    // -------------------------------------------------------------------------
    localparam logic [7:0] CHAR_R = 8'h52; // 'R'
    localparam logic [7:0] CHAR_C = 8'h43; // 'C'
    localparam logic [7:0] CHAR_V = 8'h56; // 'V'
    localparam logic [7:0] CHAR_G = 8'h47; // 'G'
    localparam logic [7:0] CHAR_B = 8'h42; // 'B'
    localparam logic [7:0] CHAR_L = 8'h4C; // 'L'
    localparam logic [7:0] CHAR_P = 8'h50; // 'P'
    localparam logic [7:0] CHAR_S = 8'h53; // 'S'
    localparam logic [7:0] CHAR_W = 8'h57; // 'W' - RGF write command / Width opcode
    localparam logic [7:0] CHAR_I = 8'h49; // 'I' - Image-burst opcode
    localparam logic [7:0] CHAR_H = 8'h48; // 'H' - Height opcode

    // -------------------------------------------------------------------------
    // ASCII validation ranges
    // -------------------------------------------------------------------------
    localparam logic [7:0] BCD_ZERO = 8'h30;
    localparam logic [7:0] BCD_NINE = 8'h39;
    localparam logic [7:0] UPPER_A  = 8'h41;
    localparam logic [7:0] UPPER_Z  = 8'h5A;

    // -------------------------------------------------------------------------
    // RX message classification (shared by rx_parser and rx_classifier)
    // See parser_spec_v1 / classifier_spec_v1 and message_format_lab12_v1.
    // -------------------------------------------------------------------------
    localparam int unsigned LOCAL_ADDR_WIDTH = 16; // {A1,A0} register-file offset
    localparam int unsigned BAR_ADDR_WIDTH   = 8;  // A2 -> chiptop BAR / master_sel
    localparam int unsigned WDATA_WIDTH      = 32; // {DH1,DH0,DL1,DL0}
    localparam int unsigned DIM_WIDTH        = 24; // image height / width
    localparam int unsigned CHAN_WIDTH       = 32; // per-colour burst word (4 bytes)
    localparam int unsigned PIXELS_PER_MSG   = 4;  // pixels carried per payload frame
    localparam int unsigned PAY_BYTES        = 12; // payload bytes (4 delimiters dropped)
    // Burst frame counter must hold ceil(H*W / 4); size it for full H*W.
    localparam int unsigned BURST_CNT_WIDTH  = 2*DIM_WIDTH;

    // -------------------------------------------------------------------------
    // Burst pixel counters / AHB address model (address_map_v1: fixed 256x256,
    // full-image only from address 0; counter and address both step +4/frame)
    // -------------------------------------------------------------------------
    localparam int unsigned PIX_CNT_W  = 17; // holds pixel_target = H*W = 65,536
    localparam int unsigned PIX_STRIDE = 4;  // pixels per frame = address stride
    localparam int unsigned AHB_ADDR_W = 17; // pixel-stride address 0..65,532

    // -------------------------------------------------------------------------
    // APB / register-file / BAR parameters (apb_spec_v1, register_files_v1,
    // address_map_v1). All registers are exactly 32 bits (= one SRAM word).
    // -------------------------------------------------------------------------
    localparam int unsigned N_RGF      = 3;              // img, uart, fifo
    localparam int unsigned RGF_SEL_W  = 2;              // $clog2(N_RGF)
    localparam int unsigned RGF_ADDR_W = 16;             // FULL {A1,A0} byte-address
                                                         //   decode (Excel offsets,
                                                         //   stride 4)
    localparam int unsigned DIM_W      = 12;             // H/W field width (256 fits)
    // BAR decode map (address_map_v1 sec 2): A2 -> chip selects
    localparam logic [7:0] SRAM_BASE_A2 = 8'h00;         // RGB SRAM bank
    localparam logic [7:0] IMG_RGF_A2   = 8'h04;         // rgf_sel = 0
    localparam logic [7:0] UART_RGF_A2  = 8'h08;         // rgf_sel = 1
    localparam logic [7:0] FIFO_RGF_A2  = 8'h0C;         // rgf_sel = 2

    // -------------------------------------------------------------------------
    // Packed image-progress counter {complete[16], row[15:8], col[7:0]}
    // (+4 per RGB word group; bit16 = end of the fixed 256x256 image).
    // IMG_PROG_W is the STRUCTURAL width (17). Modules take it as a parameter;
    // overriding it smaller is permitted ONLY in testbenches (small-image runs).
    // -------------------------------------------------------------------------
    localparam int unsigned IMG_PROG_W  = 17;
    localparam int unsigned IMG_DIM_F_W = 9;   // img_status height/width fields
                                               //   (9 bits: holds 256 directly)

    // -------------------------------------------------------------------------
    // Async FIFO geometry (rx_afifo / tx_afifo x3 each, async_fifo_spec_v1)
    // -------------------------------------------------------------------------
    localparam int unsigned AFIFO_WIDTH     = 32; // one 32-bit SRAM row
    localparam int unsigned AFIFO_DEPTH     = 16; // power of two
    localparam int unsigned AFIFO_AF_THRESH = AFIFO_DEPTH - 3; // Excel: (DEPTH-1)-2
    localparam int unsigned AFIFO_AE_THRESH = 0;  // Excel: 0 (empty governs the
                                                  //   drain; almost_empty advisory)

    // Message-type codes driven by rx_parser (parser_spec_v1 sec. 3).
    typedef enum logic [2:0] {
        MSG_REG_WRITE     = 3'b000, // {W<A>,V<..>,V<..>}   len 16
        MSG_REG_READ_REQ  = 3'b001, // {R<A>}               len  6
        MSG_IMG_WRITE_HDR = 3'b010, // {I<0>,H<..>,W<..>}   len 16 (arms burst)
        MSG_IMG_READ_REQ  = 3'b011, // {R<A>,H<..>,W<..>}   len 16
        MSG_PIXEL_PAYLOAD = 3'b100, // {<..>,<..>,<..>}     len 16 (opcode-bypass)
        MSG_NONE          = 3'b111  // illegal / no match (msg_legal = 0)
    } msg_type_t;

endpackage : UART_P
