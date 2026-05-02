# Simulation Makefile — ModelSim 2024.2
#
# Requires ModelSim 2024.2 installed at:
#   /tools/mentor/ModelSim/2024.2/modelsim_dlx/bin/
#
# Usage:
#   make pu_tb         # compile + run pixel_unit_tb
#   make chain_tb      # compile + run systolic_chain_tb
#   make disp_tb       # compile + run triangle_dispatcher_tb
#   make fifo_tb       # compile + run triangle_fifo_tb
#   make top_tb        # compile + run rasterizer_top_tb
#   make all           # run pu/chain/disp/fifo
#   make clean         # delete build dir and stamp files

MSIM_BIN := /tools/mentor/ModelSim/2024.2/modelsim_dlx/bin
VLIB     := $(MSIM_BIN)/vlib
VMAP     := $(MSIM_BIN)/vmap
VLOG     := $(MSIM_BIN)/vlog
VSIM     := $(MSIM_BIN)/vsim

HW_DIR  := ../hardware
SIM_DIR := .
BUILD   := build
WORK    := $(BUILD)/work

# -sv enables SystemVerilog; +incdir resolves `include "triangle_packet.svh"`
VLOGFLAGS := -sv -work $(WORK) +incdir+$(HW_DIR)
# -c = batch/command-line mode (no GUI); exit 0 on success, non-zero on $fatal
VSIMFLAGS := -c -lib $(WORK) -do "run -all; quit"

$(WORK):
	$(VLIB) $(WORK)
	$(VMAP) work $(WORK)

# ---------------- stamp-based compile targets ----------------
# Stamps live in BUILD/ so clean removes them with the library.

$(BUILD)/.stamp_pu_tb: $(WORK) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv
	$(VLOG) $(VLOGFLAGS) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv
	@touch $@

$(BUILD)/.stamp_chain_tb: $(WORK) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv
	$(VLOG) $(VLOGFLAGS) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv
	@touch $@

$(BUILD)/.stamp_disp_tb: $(WORK) \
        $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
        $(SIM_DIR)/triangle_dispatcher_tb.sv
	$(VLOG) $(VLOGFLAGS) \
	    $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
	    $(SIM_DIR)/triangle_dispatcher_tb.sv

	@touch $@

$(BUILD)/.stamp_fifo_tb: $(WORK) $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv
	$(VLOG) $(VLOGFLAGS) $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv
	@touch $@

# framebuffer module lives in framebuffer_new.sv
$(BUILD)/.stamp_top_tb: $(WORK) \
        $(HW_DIR)/rasterizer_top.sv $(HW_DIR)/pixel_unit.sv \
        $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
        $(HW_DIR)/vga_framebuffer.sv $(HW_DIR)/avalon_interface.sv \
        $(HW_DIR)/framebuffer_new.sv $(SIM_DIR)/rasterizer_top_tb.sv
	$(VLOG) $(VLOGFLAGS) \
	    $(HW_DIR)/rasterizer_top.sv $(HW_DIR)/pixel_unit.sv \
	    $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
	    $(HW_DIR)/vga_framebuffer.sv $(HW_DIR)/avalon_interface.sv \
	    $(HW_DIR)/framebuffer_new.sv $(SIM_DIR)/rasterizer_top_tb.sv
	@touch $@

# ---------------- run targets ----------------

pu_tb: $(BUILD)/.stamp_pu_tb
	$(VSIM) $(VSIMFLAGS) pixel_unit_tb

chain_tb: $(BUILD)/.stamp_chain_tb
	$(VSIM) $(VSIMFLAGS) systolic_chain_tb

disp_tb: $(BUILD)/.stamp_disp_tb
	$(VSIM) $(VSIMFLAGS) triangle_dispatcher_tb

fifo_tb: $(BUILD)/.stamp_fifo_tb
	$(VSIM) $(VSIMFLAGS) triangle_fifo_tb

top_tb: $(BUILD)/.stamp_top_tb
	$(VSIM) $(VSIMFLAGS) rasterizer_top_tb

all: pu_tb chain_tb disp_tb fifo_tb

clean:
	rm -rf $(BUILD)

.PHONY: all clean pu_tb chain_tb disp_tb fifo_tb top_tb
