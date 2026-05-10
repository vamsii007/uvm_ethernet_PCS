#!/bin/bash
rm -rf simv simv.daidir csrc ucli.key *.log *.vpd *.fsdb
vcs -sverilog -ntb_opts uvm \
  -timescale=1ns/1ps \
  +incdir+TB \
  +incdir+reference_model \
  TB/top.sv \
  reference_model/pcs_tx_rm_dut0_dpi.c \
  -CFLAGS "-std=c99" \
  -debug_access+all \
  -l compile.log

./simv +UVM_TESTNAME=pcs_test -l sim.log
