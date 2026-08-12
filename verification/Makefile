# ============================================================
# uartBurst_ahbSingle Verilator Makefile
# Folder layout:
#   FinalProject_UARTburst_AHBsingle/
#     sources_and_constraints/  -> DUT RTL files (+ board XDC, unused in sim)
#     verification/
#       Makefile                -> this file (run make from verification/)
#       testbench/              -> tb_top + verification classes + sim-only files + .mem
# ============================================================

VERILATOR ?= verilator
TOP       ?= tb_top
NPROC     ?= 4

SRC_DIR   := ../sources_and_constraints
TB_DIR    := testbench
SIM_NAME  ?= $(TOP)
# GNU Make / Verilator cannot build object files inside a directory whose path
# contains spaces. If the project root path has spaces, fall back to a
# deterministic build directory under /tmp (standard on macOS/Linux; not
# machine-specific). Run outputs (tb_sim.log, dump.fst) still land in the
# project root either way. Override from the command line if desired.
empty :=
space := $(empty) $(empty)
ifneq ($(findstring $(space),$(CURDIR)),)
SIM_DIR   ?= /tmp/uartburst_ahbsingle_$(SIM_NAME)-sim
else
SIM_DIR   ?= $(SIM_NAME)-sim
endif

# Short simulation image unless overridden from command line.
# Example override: make simulate SIM_DEFINES="+define+SIM_SMALL_IMAGE +define+SOME_OTHER_DEFINE"
SIM_DEFINES ?= +define+SIM_SMALL_IMAGE

# Verilator 5.x FST tracing links against liblz4/zlib. On Homebrew systems the
# headers/libs live under the brew prefix, which is not on the default compiler
# search path; pass them through to the generated makefile when brew exists
# (no effect on systems where the toolchain already finds them).
BREW_PREFIX := $(shell brew --prefix 2>/dev/null)
ifneq ($(BREW_PREFIX),)
SIM_CPPFLAGS := -I$(BREW_PREFIX)/include
SIM_LDLIBS   := -L$(BREW_PREFIX)/lib -llz4 -lz
else
SIM_CPPFLAGS :=
SIM_LDLIBS   := -llz4 -lz
endif

# Include/search paths. The -y paths let Verilator find modules by name if needed.
INC_DIRS := +incdir+$(SRC_DIR) +incdir+$(TB_DIR)
LIB_DIRS := -y $(SRC_DIR) -y $(TB_DIR)

# Testbench files first where needed: interface/classes/stub.
# (Transaction/Driver/Monitor/Checker are `include'd by tb_pkg.sv via +incdir.)
TB_SRCS = \
  $(TB_DIR)/chip_if.sv \
  $(TB_DIR)/tb_pkg.sv \
  $(TB_DIR)/clk_wiz_0_sim.sv \
  $(TB_DIR)/BUFGMUX_sim.sv

# DUT RTL files. Keep packages first.
RTL_SRCS = \
  $(SRC_DIR)/UART_package.sv \
  $(SRC_DIR)/master_reset_sync.sv \
  $(SRC_DIR)/glitchless_mux.sv \
  $(SRC_DIR)/baud_rate_16th.sv \
  $(SRC_DIR)/clock_gen.sv \
  $(SRC_DIR)/bus_sync_2ff.sv \
  $(SRC_DIR)/differentiator.sv \
  $(SRC_DIR)/baud_ctr.sv \
  $(SRC_DIR)/rx_shift_reg.sv \
  $(SRC_DIR)/rx_phy_fsm.sv \
  $(SRC_DIR)/rx_phy.sv \
  $(SRC_DIR)/rx_mac_fsm.sv \
  $(SRC_DIR)/rx_mac.sv \
  $(SRC_DIR)/uart_rx.sv \
  $(SRC_DIR)/parser.sv \
  $(SRC_DIR)/classifier.sv \
  $(SRC_DIR)/bar.sv \
  $(SRC_DIR)/rx_resend_ctrl.sv \
  $(SRC_DIR)/fifo_mem.sv \
  $(SRC_DIR)/fifo_ptr.sv \
  $(SRC_DIR)/async_fifo.sv \
  $(SRC_DIR)/rx_producer_fsm.sv \
  $(SRC_DIR)/rx_ahb_master_mac.sv \
  $(SRC_DIR)/rx_sequencer.sv \
  $(SRC_DIR)/tx_ahb_master_mac.sv \
  $(SRC_DIR)/tx_consumer_fsm.sv \
  $(SRC_DIR)/tx_sequencer.sv \
  $(SRC_DIR)/ahb_master_phy.sv \
  $(SRC_DIR)/ahb_slave.sv \
  $(SRC_DIR)/sram.sv \
  $(SRC_DIR)/apb_master_phy.sv \
  $(SRC_DIR)/apb_master_mac.sv \
  $(SRC_DIR)/apb_master.sv \
  $(SRC_DIR)/apb_interconnect.sv \
  $(SRC_DIR)/apb_slave_envelope.sv \
  $(SRC_DIR)/img_rgf.sv \
  $(SRC_DIR)/uart_rgf.sv \
  $(SRC_DIR)/fifo_rgf.sv \
  $(SRC_DIR)/seq_top_fsm.sv \
  $(SRC_DIR)/sequencer_top.sv \
  $(SRC_DIR)/tx_msg_arbiter.sv \
  $(SRC_DIR)/composer.sv \
  $(SRC_DIR)/tx_mac.sv \
  $(SRC_DIR)/tx_phy.sv \
  $(SRC_DIR)/chiptop.sv

# Top-level TB last.
TOP_SRC = $(TB_DIR)/tb_top.sv

SRCS = $(RTL_SRCS) $(TB_SRCS) $(TOP_SRC)

# Verilator warnings: keep useful checks but avoid known noisy warnings from class-based SV / RTL width cleanup.
VFLAGS = \
  --cc \
  --exe \
  --main \
  --top-module $(TOP) \
  --timing \
  --trace-fst \
  --no-sched-zero-delay \
  -Wno-fatal \
  -Wno-WIDTHTRUNC \
  -Wno-WIDTHEXPAND \
  -Wno-PINCONNECTEMPTY \
  -Wno-DECLFILENAME \
  -Wno-UNOPTFLAT \
  -Wno-UNUSEDSIGNAL \
  -Wno-UNUSEDPARAM \
  -Wno-TIMESCALEMOD

# Copy memory files to the run directory root because $readmemh("tb_red_8x8.mem")
# resolves relative to where the simulator is launched, not relative to sources/testbench.
# NOTE: the small TB images are tb_*_8x8.mem; the project's golden 256x256
# red_hex.mem/green_hex.mem/blue_hex.mem already live at the root and are NEVER
# copied over or removed by this Makefile.
MEMS := $(wildcard $(TB_DIR)/*.mem)

all: $(SIM_DIR)/$(SIM_NAME).test

prep_mem:
	@echo "[0/4] Copying simulation .mem files from $(TB_DIR)/ to project root..."
	@for f in $(MEMS); do cp $$f .; done

$(SIM_DIR)/$(SIM_NAME).mk: prep_mem $(SRCS)
	@echo "[1/4] Verilating uartBurst_ahbSingle testbench..."
	$(VERILATOR) \
		$(VFLAGS) \
		-Mdir $(SIM_DIR) \
		--output-groups $(NPROC) \
		--prefix $(SIM_NAME) -o $(SIM_NAME).test \
		$(LIB_DIRS) \
		$(INC_DIRS) \
		$(SIM_DEFINES) \
		$(SRCS)

$(SIM_DIR)/$(SIM_NAME).test: $(SIM_DIR)/$(SIM_NAME).mk
	@echo "[2/4] Building simulator..."
	$(MAKE) -j$(NPROC) -C $(SIM_DIR) -f $(SIM_NAME).mk \
		USER_CPPFLAGS="$(SIM_CPPFLAGS)" USER_LDLIBS="$(SIM_LDLIBS)"

simulate: $(SIM_DIR)/$(SIM_NAME).test
	@echo "[3/4] Running uartBurst_ahbSingle verification simulation..."
	$(SIM_DIR)/$(SIM_NAME).test
	@echo "[4/4] Done."

clean:
	rm -rf $(SIM_DIR)
	rm -f dump.fst tb_sim.log
	rm -f tb_red_8x8.mem tb_green_8x8.mem tb_blue_8x8.mem

.PHONY: all simulate clean prep_mem
