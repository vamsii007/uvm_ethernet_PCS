import uvm_pkg::*;
`include "uvm_macros.svh"

class pcs_driver extends uvm_driver #(pcs_item);
  `uvm_component_utils(pcs_driver)

  virtual pcs_if vif;

  function new(string name="pcs_driver", uvm_component parent=null);
    super.new(name,parent);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    if(!uvm_config_db#(virtual pcs_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal(get_type_name(), "failed to get handle to virtual interface")
    end
  endfunction

  virtual task run_phase(uvm_phase phase);
    pcs_item my_item;
    vif.TX_EN <= 1'b0;
    vif.Din   <= 8'd0;

    wait(vif.rst_n == 1'b1);

    forever begin
      `uvm_info(get_type_name(), "waiting for data from sequencer", UVM_MEDIUM)
      seq_item_port.get_next_item(my_item);
      drive_packet(my_item);
      seq_item_port.item_done();
    end
  endtask

  task drive_packet(pcs_item my_item);

    foreach(my_item.data[i]) begin
      @(vif.cb_dr);
      vif.cb_dr.TX_EN <= 1'b1;
      vif.cb_dr.Din   <= my_item.data[i];
    end

    repeat(my_item.gap_incycles) begin
      @(vif.cb_dr);
      vif.cb_dr.TX_EN <= 1'b0;
      vif.cb_dr.Din   <= 8'd0;
    end

  endtask

endclass