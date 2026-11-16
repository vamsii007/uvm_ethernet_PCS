`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_env extends uvm_env;
  `uvm_component_utils(pcs_env)

   int unsigned num_duts = 1;

  pcs_agent      agent[];
  pcs_scoreboard sb[];

    pcs_virtual_sequencer vseqr;


  function new(string name="pcs_env", uvm_component parent=null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

      if (!uvm_config_db#(int unsigned)::get(this, "", "num_duts", num_duts)) begin
      num_duts = 1;
    end

    `uvm_info("ENV", $sformatf("Creating %0d PCS agents", num_duts), UVM_LOW)

    agent = new[num_duts];
    sb    = new[num_duts];

    vseqr = pcs_virtual_sequencer::type_id::create("vseqr", this);

     for (int i = 0; i < num_duts; i++) begin
      agent[i] = pcs_agent::type_id::create($sformatf("agent_%0d", i), this);
      sb[i]    = pcs_scoreboard::type_id::create($sformatf("sb_%0d", i), this);

      uvm_config_db#(int)::set(this, agent[i].get_name(), "pcs_id", i);
      uvm_config_db#(int)::set(this, sb[i].get_name(),    "pcs_id", i);
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);

    vseqr.num_duts = num_duts;
    vseqr.seqr = new[num_duts];
    vseqr.vif  = new[num_duts];

    for (int i = 0; i < num_duts; i++) begin
      agent[i].mon.mon_ap.connect(sb[i].imp);

      vseqr.seqr[i] = agent[i].seqr;
      vseqr.vif[i]  = agent[i].vif;
    end
  endfunction

endclass
