# Simulation Makefile (Verilator-based).
#
# Requires Verilator 5.x for --binary and --timing. On the school server
# verify with `verilator --version`; if it's <5.0 ask for a newer module.
#
# Usage:
#   make pu_tb         # build + run pixel_unit_tb
#   make chain_tb      # build + run systolic_chain_tb
#   make disp_tb       # build + run triangle_dispatcher_tb
#   make fifo_tb       # build + run triangle_fifo_tb
#   make top_tb        # build + run rasterizer_top_tb (currently broken;
#                      #   avalon_interface.sv is a stub without endmodule)
#   make all           # build + run pu/chain/disp/fifo
#   make clean         # delete build dir
#
# Tracing: set TRACE=1 to enable FST waveform dump (slower).
#   make TRACE=1 chain_tb
# The .fst lands at obj_dir/<tb>/dump.fst by default if your TB uses $dumpfile.

HW_DIR  := ../hardware
SIM_DIR := .
OBJDIR  := obj_dir

# Common Verilator flags.
#   --binary       build a runnable simulator (Verilator 5.x)
#   --timing       support delays, @(posedge), wait(), etc.
#   --public-flat-rw
#                  keep all RTL signals readable/writable, so hierarchical
#                  refs in the TB (dut.z_mem[i] = ..., dut.col_base_q,
#                  g_pu[3].u_pu.z_mem[i]) survive Verilator's optimizer
#   -Wno-fatal     do not promote warnings to errors
#   -Wno-WIDTHEXPAND, -Wno-WIDTHTRUNC
#                  silence width warnings on int<->logic compares in the
#                  TB's $sformatf paths
#   -Wno-UNUSED*   silence warnings for ports the TB ties off / leaves open
#   --x-assign 0, --x-initial 0
#                  flush Xs to 0 on init so reads of uninitialized memory
#                  are deterministic (otherwise z_mem reads return X and
#                  the z compare goes wrong)
VFLAGS := --binary --timing \
          --public-flat-rw \
          -Wno-fatal \
          -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
          -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
          --x-assign 0 --x-initial 0 \
          -I$(HW_DIR)

ifdef TRACE
VFLAGS += --trace-fst
endif

# Per-TB Verilator subdir keeps build artifacts (Vmodel.cpp etc.) from
# colliding when you build several testbenches in the same checkout.
define VRUN
verilator $(VFLAGS) --Mdir $(OBJDIR)/$(1) --top $(2) -o $(1) $(3)
endef

# ---------------- per-testbench builds ----------------

$(OBJDIR)/pu_tb/pu_tb: $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv
	$(call VRUN,pu_tb,pixel_unit_tb,\
	    $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/pixel_unit_tb.sv)

$(OBJDIR)/chain_tb/chain_tb: $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv
	$(call VRUN,chain_tb,systolic_chain_tb,\
	    $(HW_DIR)/pixel_unit.sv $(SIM_DIR)/systolic_chain_tb.sv)

$(OBJDIR)/disp_tb/disp_tb: $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_dispatcher_tb.sv
	$(call VRUN,disp_tb,triangle_dispatcher_tb,\
	    $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
	    $(SIM_DIR)/triangle_dispatcher_tb.sv)

$(OBJDIR)/fifo_tb/fifo_tb: $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv
	$(call VRUN,fifo_tb,triangle_fifo_tb,\
	    $(HW_DIR)/triangle_fifo.sv $(SIM_DIR)/triangle_fifo_tb.sv)

$(OBJDIR)/top_tb/top_tb: $(HW_DIR)/rasterizer_top.sv $(HW_DIR)/pixel_unit.sv \
                        $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
                        $(HW_DIR)/vga_framebuffer.sv $(HW_DIR)/avalon_interface.sv \
                        $(HW_DIR)/framebuffer.sv $(SIM_DIR)/rasterizer_top_tb.sv
	$(call VRUN,top_tb,rasterizer_top_tb,\
	    $(HW_DIR)/rasterizer_top.sv $(HW_DIR)/pixel_unit.sv \
	    $(HW_DIR)/triangle_dispatcher.sv $(HW_DIR)/triangle_fifo.sv \
	    $(HW_DIR)/vga_framebuffer.sv $(HW_DIR)/avalon_interface.sv \
	    $(HW_DIR)/framebuffer.sv $(SIM_DIR)/rasterizer_top_tb.sv)

# ---------------- run targets ----------------

pu_tb: $(OBJDIR)/pu_tb/pu_tb
	$(OBJDIR)/pu_tb/pu_tb

chain_tb: $(OBJDIR)/chain_tb/chain_tb
	$(OBJDIR)/chain_tb/chain_tb

disp_tb: $(OBJDIR)/disp_tb/disp_tb
	$(OBJDIR)/disp_tb/disp_tb

fifo_tb: $(OBJDIR)/fifo_tb/fifo_tb
	$(OBJDIR)/fifo_tb/fifo_tb

top_tb: $(OBJDIR)/top_tb/top_tb
	$(OBJDIR)/top_tb/top_tb

all: pu_tb chain_tb disp_tb fifo_tb

clean:
	rm -rf $(OBJDIR)

.PHONY: all clean pu_tb chain_tb disp_tb fifo_tb top_tb
