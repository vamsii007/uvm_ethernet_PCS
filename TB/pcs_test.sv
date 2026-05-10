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
    phase.raise_objection(this);

    seq = pcs_sequence::type_id::create("seq");
    seq.start(env.agent[0].seqr);

    #1000ns;

    phase.drop_objection(this);
  endtask

endclass