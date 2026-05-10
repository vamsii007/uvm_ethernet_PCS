`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_env extends uvm_env;
  `uvm_component_utils(pcs_env)

  pcs_agent      agent[10];
  pcs_scoreboard sb[10];

  function new(string name="pcs_env", uvm_component parent=null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    for (int i=0; i<1; i++) begin
      agent[i] = pcs_agent::type_id::create($sformatf("agent[%0d]", i), this);
      sb[i] = pcs_scoreboard::type_id::create($sformatf("sb[%0d]", i), this);
      uvm_config_db#(int)::set(this, agent[i].get_name(), "pcs_id", i);
      uvm_config_db#(int)::set(this, sb[i].get_name(), "pcs_id", i);
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);

    for (int i=0; i<1; i++) begin
      agent[i].mon.mon_ap.connect(sb[i].imp);
    end
  endfunction
endclass