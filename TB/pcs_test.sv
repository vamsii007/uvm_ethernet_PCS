`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_test extends uvm_test;
  `uvm_component_utils(pcs_test)

  pcs_env env;
  pcs_sequence seq;

  function new(string name="pcs_test", uvm_component parent=null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = pcs_env::type_id::create("env", this);
  endfunction

  task run_phase(uvm_phase phase);

  pcs_random_reset_vseq vseq;

  phase.raise_objection(this);

  for (int i = 0; i < env.vseqr.num_duts; i++) begin
    vseq = pcs_random_reset_vseq::type_id::create($sformatf("vseq_%0d", i));

    vseq.dut_id = i;

    `uvm_info(get_type_name(),
      $sformatf("Starting random reset vseq for DUT[%0d]", i),
      UVM_LOW)

    vseq.start(env.vseqr);
    end
    #1000ns;

    phase.drop_objection(this);
  endtask

endclass
