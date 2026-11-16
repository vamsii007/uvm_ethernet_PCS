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

TESTS=${@:-"pcs_test pcs_test_min pcs_test_max pcs_test_all_zeros pcs_test_all_ones \
pcs_test_walking_ones pcs_test_walking_zeros pcs_test_alt_55_aa pcs_test_same_byte pcs_test_btb"}

for t in $TESTS; do
  ./simv +UVM_TESTNAME=$t +ntb_random_seed_automatic -l sim_$t.log
  echo "$t: $(grep -E '^UVM_(ERROR|FATAL) :' sim_$t.log | tr '\n' ' ')"
done
