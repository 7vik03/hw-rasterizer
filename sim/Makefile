# Simulation Makefile — ModelSim 2024.2

MSIM_BIN := /tools/mentor/ModelSim/2024.2/modelsim_dlx/bin
VLIB     := $(MSIM_BIN)/vlib
VMAP     := $(MSIM_BIN)/vmap
VLOG     := $(MSIM_BIN)/vlog
VSIM     := $(MSIM_BIN)/vsim

HW_DIR  := ../hardware
SIM_DIR := .
BUILD   := build
WORK    := $(BUILD)/work

VLOGFLAGS := -sv -work $(WORK) +incdir+$(HW_DIR)
VSIMFLAGS := -c -lib $(WORK) -do "run -all; quit"

$(WORK):
	$(VLIB) $(WORK)
	$(VMAP) work $(WORK)

# ---------------- compile targets ----------------

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

$(BUILD)/.stamp_top_tb: $(WORK) \
        $(HW_DIR)/rasterizer_top.sv \
        $(HW_DIR)/pixel_unit.sv \
        $(HW_DIR)/triangle_dispatcher.sv \
        $(HW_DIR)/triangle_fifo.sv \
        $(HW_DIR)/vga_framebuffer.sv \
        $(HW_DIR)/avalon_interface.sv \
        $(SIM_DIR)/rasterizer_top_tb.sv
	$(VLOG) $(VLOGFLAGS) \
	    $(HW_DIR)/pixel_unit.sv \
	    $(HW_DIR)/triangle_fifo.sv \
	    $(HW_DIR)/triangle_dispatcher.sv \
	    $(HW_DIR)/vga_framebuffer.sv \
	    $(HW_DIR)/avalon_interface.sv \
	    $(HW_DIR)/rasterizer_top.sv \
	    $(SIM_DIR)/rasterizer_top_tb.sv
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

all: pu_tb chain_tb disp_tb fifo_tb top_tb

clean:
	rm -rf $(BUILD)

.PHONY: all clean pu_tb chain_tb disp_tb fifo_tb top_tb
