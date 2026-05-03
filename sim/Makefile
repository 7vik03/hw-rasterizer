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
VSIMFLAGS := -c -lib $(WORK) -onfinish exit \
             -do "onerror {quit -code 1}; run -all; quit -code 0"

$(WORK):
	mkdir -p $(BUILD)
	$(VLIB) $(WORK)
	$(VMAP) work $(WORK)

# ---------------- compile stamps ----------------

$(BUILD)/.stamp_avalon_tb: $(HW_DIR)/avalon_interface.sv $(HW_DIR)/triangle_fifo.sv \
                           $(SIM_DIR)/avalon_interface_tb.sv | $(WORK)
	$(VLOG) $(VLOGFLAGS) \
	    $(HW_DIR)/avalon_interface.sv $(HW_DIR)/triangle_fifo.sv \
	    $(SIM_DIR)/avalon_interface_tb.sv
	@touch $@

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

$(BUILD)/.stamp_vga_tb: $(HW_DIR)/vga_framebuffer.sv \
                        $(SIM_DIR)/vga_framebuffer_tb.sv | $(WORK)
	$(VLOG) $(VLOGFLAGS) \
	    $(HW_DIR)/vga_framebuffer.sv \
	    $(SIM_DIR)/vga_framebuffer_tb.sv
	@touch $@

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
avalon_tb: $(BUILD)/.stamp_avalon_tb
	$(VSIM) $(VSIMFLAGS) avalon_interface_tb 2>&1 | tee $(BUILD)/avalon_tb.log
	@grep -Eq '^(PASS|# ?PASS)' $(BUILD)/avalon_tb.log || (echo ">>> avalon_tb FAILED"; exit 1)
	@echo ">>> avalon_tb PASSED"

pu_tb: $(BUILD)/.stamp_pu_tb
	$(VSIM) $(VSIMFLAGS) pixel_unit_tb 2>&1 | tee $(BUILD)/pu_tb.log
	@grep -Eq '^(PASS|# ?PASS)' $(BUILD)/pu_tb.log || (echo ">>> pu_tb FAILED"; exit 1)
	@echo ">>> pu_tb PASSED"

chain_tb: $(BUILD)/.stamp_chain_tb
	$(VSIM) $(VSIMFLAGS) systolic_chain_tb 2>&1 | tee $(BUILD)/chain_tb.log
	@grep -Eq '^(PASS|# ?PASS)' $(BUILD)/chain_tb.log || (echo ">>> chain_tb FAILED"; exit 1)
	@echo ">>> chain_tb PASSED"

disp_tb: $(BUILD)/.stamp_disp_tb
	$(VSIM) $(VSIMFLAGS) triangle_dispatcher_tb 2>&1 | tee $(BUILD)/disp_tb.log
	@grep -Eq '^(PASS|# ?PASS)' $(BUILD)/disp_tb.log || (echo ">>> disp_tb FAILED"; exit 1)
	@echo ">>> disp_tb PASSED"

fifo_tb: $(BUILD)/.stamp_fifo_tb
	$(VSIM) $(VSIMFLAGS) triangle_fifo_tb 2>&1 | tee $(BUILD)/fifo_tb.log
	@grep -Eq '^(PASS|# ?PASS)' $(BUILD)/fifo_tb.log || (echo ">>> fifo_tb FAILED"; exit 1)
	@echo ">>> fifo_tb PASSED"

vga_tb: $(BUILD)/.stamp_vga_tb
	$(VSIM) $(VSIMFLAGS) vga_framebuffer_tb 2>&1 | tee $(BUILD)/vga_tb.log
	@grep -Eq '^(PASS|# ?PASS)' $(BUILD)/vga_tb.log || (echo ">>> vga_tb FAILED"; exit 1)
	@echo ">>> vga_tb PASSED"

top_tb: $(BUILD)/.stamp_top_tb
	$(VSIM) $(VSIMFLAGS) rasterizer_top_tb 2>&1 | tee $(BUILD)/top_tb.log
	@grep -Eq '^(PASS|# ?PASS)' $(BUILD)/top_tb.log || (echo ">>> top_tb FAILED"; exit 1)
	@echo ">>> top_tb PASSED"

all: pu_tb chain_tb disp_tb fifo_tb vga_tb avalon_tb top_tb

clean:
	rm -rf $(BUILD) modelsim.ini transcript *.wlf

.PHONY: all clean pu_tb chain_tb disp_tb fifo_tb vga_tb avalon_tb top_tb
