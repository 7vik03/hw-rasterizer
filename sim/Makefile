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
# -onfinish exit makes $finish return cleanly; quit -code propagates exit status
VSIMFLAGS := -c -lib $(WORK) -onfinish exit \
             -do "onerror {quit -code 1}; run -all; quit -code 0"

# ---------------- library setup (order-only prereq) ----------------
$(WORK):
	mkdir -p $(BUILD)
	$(VLIB) $(WORK)
	$(VMAP) work $(WORK)

# ---------------- compile stamps ----------------
# `| $(WORK)` makes the work lib an order-only prereq so its mtime
# (which vlog updates) doesn't trigger spurious recompiles.

$(BUILD)/.stamp_pu_tb: $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv | $(WORK)
	$(VLOG) $(VLOGFLAGS) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv
	@touch $@

$(BUILD)/.stamp_chain_tb: $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv | $(WORK)
	$(VLOG) $(VLOGFLAGS) $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv
	@touch $@

$(BUILD)/.stamp_disp_tb: $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
                         $(SIM_DIR)/triangle_dispatcher_tb.sv | $(WORK)
	$(VLOG) $(VLOGFLAGS) \
	    $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
	    $(SIM_DIR)/triangle_dispatcher_tb.sv
	@touch $@

$(BUILD)/.stamp_fifo_tb: $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv | $(WORK)
	$(VLOG) $(VLOGFLAGS) $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv
	@touch $@

# rasterizer_top pulls in everything; note no framebuffer_new.sv (you don't have it)
$(BUILD)/.stamp_top_tb: $(HW_DIR)/rasterizer_top.sv $(HW_DIR)/pixel_unit.sv \
                        $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
                        $(HW_DIR)/vga_framebuffer.sv $(HW_DIR)/avalon_interface.sv \
                        $(SIM_DIR)/rasterizer_top_tb.sv | $(WORK)
	$(VLOG) $(VLOGFLAGS) \
	    $(HW_DIR)/rasterizer_top.sv $(HW_DIR)/pixel_unit.sv \
	    $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
	    $(HW_DIR)/vga_framebuffer.sv $(HW_DIR)/avalon_interface.sv \
	    $(SIM_DIR)/rasterizer_top_tb.sv
	@touch $@

# ---------------- run targets ----------------
# tee to a log AND grep for PASS so a TB that prints "FAIL" but exits 0
# still fails the make target. Adjust the grep pattern per TB if needed.

pu_tb: $(BUILD)/.stamp_pu_tb
	$(VSIM) $(VSIMFLAGS) pixel_unit_tb 2>&1 | tee $(BUILD)/pu_tb.log
	@grep -q "^PASS" $(BUILD)/pu_tb.log || (echo ">>> pu_tb FAILED"; exit 1)

chain_tb: $(BUILD)/.stamp_chain_tb
	$(VSIM) $(VSIMFLAGS) systolic_chain_tb 2>&1 | tee $(BUILD)/chain_tb.log
	@grep -q "^PASS" $(BUILD)/chain_tb.log || (echo ">>> chain_tb FAILED"; exit 1)

disp_tb: $(BUILD)/.stamp_disp_tb
	$(VSIM) $(VSIMFLAGS) triangle_dispatcher_tb 2>&1 | tee $(BUILD)/disp_tb.log
	@grep -q "^PASS" $(BUILD)/disp_tb.log || (echo ">>> disp_tb FAILED"; exit 1)

fifo_tb: $(BUILD)/.stamp_fifo_tb
	$(VSIM) $(VSIMFLAGS) triangle_fifo_tb 2>&1 | tee $(BUILD)/fifo_tb.log
	@grep -q "^PASS" $(BUILD)/fifo_tb.log || (echo ">>> fifo_tb FAILED"; exit 1)

top_tb: $(BUILD)/.stamp_top_tb
	$(VSIM) $(VSIMFLAGS) rasterizer_top_tb 2>&1 | tee $(BUILD)/top_tb.log
	@grep -q "^PASS" $(BUILD)/top_tb.log || (echo ">>> top_tb FAILED"; exit 1)

all: pu_tb chain_tb disp_tb fifo_tb

clean:
	rm -rf $(BUILD) modelsim.ini transcript *.wlf

.PHONY: all clean pu_tb chain_tb disp_tb fifo_tb top_tb