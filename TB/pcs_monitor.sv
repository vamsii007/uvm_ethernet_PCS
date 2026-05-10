import uvm_pkg::*;
 `include "uvm_macros.svh"

class pcs_monitor extends uvm_monitor;
 `uvm_component_utils(pcs_monitor)

virtual pcs_if vif;

uvm_analysis_port#(pcs_monitem) mon_ap;


function new(string name= "pcs_monitor", uvm_component parent= null);
 super.new(name,parent);
 mon_ap=new("mon_ap", this);
endfunction

function void build_phase(uvm_phase phase);
 super.build_phase(phase);

  if(!uvm_config_db#(virtual pcs_if)::get(this,"","vif",vif)) begin
     `uvm_fatal(get_type_name(), "failed to get virtual interface")
  end

endfunction

virtual task run_phase(uvm_phase phase);
  pcs_monitem monitem;

  super.run_phase(phase);

  forever begin
    @(posedge vif.Clk);

    monitem = pcs_monitem::type_id::create("monitem");

    monitem.rst_n = vif.rst_n;
    monitem.Din   = vif.Din;
    monitem.TX_EN = vif.TX_EN;
    monitem.Dout  = vif.Dout;
    mon_ap.write(monitem);
  end
endtask

endclass


     
