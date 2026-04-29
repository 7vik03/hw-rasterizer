# Simulation Makefile.
#
# Defaults to iverilog (Icarus); set SIM=vsim for ModelSim or override
# the *_CMD variables below. All targets compile from sources in
# ../hardware and the testbench source under sim/.
#
# Usage:
#   make pu_tb         # build + run pixel_unit_tb
#   make chain_tb      # build + run systolic_chain_tb
#   make disp_tb       # build + run triangle_dispatcher_tb
#   make fifo_tb       # build + run triangle_fifo_tb
#   make top_tb        # build + run rasterizer_top_tb (smoke)
#   make all           # build + run every TB above
#   make clean         # delete build dir

HW_DIR  := ../hardware
SIM_DIR := .
BUILD   := build

# the +include path lets `include "triangle_packet.svh"` resolve
INCFLAGS := -I$(HW_DIR)
IVFLAGS  := -g2012 -Wall $(INCFLAGS)

# common HDL files (modules used as DUTs or dependencies)
HW_COMMON := \
    $(HW_DIR)/pixel_unit.sv \
    $(HW_DIR)/triangle_fifo.sv \
    $(HW_DIR)/triangle_dispatcher.sv \
    $(HW_DIR)/vga_framebuffer.sv

# ---------------- per-testbench builds ----------------

$(BUILD):
	mkdir -p $(BUILD)

# pixel_unit_tb: only needs pixel_unit.sv + the TB
$(BUILD)/pu_tb: $(BUILD) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv
	iverilog $(IVFLAGS) -o $@ \
	    $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv

# systolic_chain_tb: 4 chained pixel_units
$(BUILD)/chain_tb: $(BUILD) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv
	iverilog $(IVFLAGS) -o $@ \
	    $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv

# triangle_dispatcher_tb: dispatcher + fifo
$(BUILD)/disp_tb: $(BUILD) \
        $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
        $(SIM_DIR)/triangle_dispatcher_tb.sv
	iverilog $(IVFLAGS) -o $@ \
	    $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
	    $(SIM_DIR)/triangle_dispatcher_tb.sv

# triangle_fifo_tb
$(BUILD)/fifo_tb: $(BUILD) $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv
	iverilog $(IVFLAGS) -o $@ \
	    $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv

# rasterizer_top_tb: pulls in everything (smoke test)
$(BUILD)/top_tb: $(BUILD) \
        $(HW_DIR)/rasterizer_top.sv $(HW_COMMON) \
        $(HW_DIR)/avalon_interface.sv $(HW_DIR)/framebuffer.sv \
        $(SIM_DIR)/rasterizer_top_tb.sv
	iverilog $(IVFLAGS) -o $@ \
	    $(HW_DIR)/rasterizer_top.sv $(HW_COMMON) \
	    $(HW_DIR)/avalon_interface.sv $(HW_DIR)/framebuffer.sv \
	    $(SIM_DIR)/rasterizer_top_tb.sv

# ---------------- run targets ----------------

pu_tb: $(BUILD)/pu_tb
	vvp $(BUILD)/pu_tb

chain_tb: $(BUILD)/chain_tb
	vvp $(BUILD)/chain_tb

disp_tb: $(BUILD)/disp_tb
	vvp $(BUILD)/disp_tb

fifo_tb: $(BUILD)/fifo_tb
	vvp $(BUILD)/fifo_tb

top_tb: $(BUILD)/top_tb
	vvp $(BUILD)/top_tb

all: pu_tb chain_tb disp_tb fifo_tb

clean:
	rm -rf $(BUILD)

.PHONY: all clean pu_tb chain_tb disp_tb fifo_tb top_tb
