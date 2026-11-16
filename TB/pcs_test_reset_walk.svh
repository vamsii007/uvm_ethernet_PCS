`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_test_reset_walk extends pcs_test;
  `uvm_component_utils(pcs_test_reset_walk)

  virtual pcs_if vif;

  // Match the enum order in dut.sv:
  // s_reset=0, s_send_idle=1, s_SDD1=2, s_SDD2=3, s_Transmit_data=4,
  // s_CSR1=5, s_CSR2=6, s_ESD1=7, s_ESD2=8
  localparam logic [3:0] ST_RESET     = 4'd0;
  localparam logic [3:0] ST_IDLE      = 4'd1;
  localparam logic [3:0] ST_SDD2      = 4'd3;
  localparam logic [3:0] ST_TX_DATA   = 4'd4;
  localparam logic [3:0] ST_CSR2      = 4'd6;
  localparam logic [3:0] ST_ESD1      = 4'd7;
  localparam logic [3:0] ST_ESD2      = 4'd8;

  function new(string name="pcs_test_reset_walk", uvm_component parent=null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual pcs_if)::get(this, "", "vif", vif))
      `uvm_fatal(get_type_name(), "no vif")
  endfunction

  task pulse_reset(string label);
    `uvm_info(get_type_name(), $sformatf("INJECTING RESET DURING %s", label), UVM_LOW)
    @(posedge vif.Clk);
    vif.rst_n <= 0;
    repeat (3) @(posedge vif.Clk);
    vif.rst_n <= 1;
    repeat (5) @(posedge vif.Clk);
  endtask

  task wait_for_state(logic [3:0] target, string label);
    int unsigned cycles = 0;
    int unsigned timeout = 100000;
    while (top.dut.cstate !== target && cycles < timeout) begin
      @(posedge vif.Clk);
      cycles++;
    end
    if (cycles >= timeout)
      `uvm_error(get_type_name(), $sformatf("TIMEOUT waiting for %s", label))
    else
      `uvm_info(get_type_name(), $sformatf("Reached %s after %0d cycles", label, cycles), UVM_LOW)
  endtask

  task run_phase(uvm_phase phase);
    pcs_sequence seq;
    phase.raise_objection(this);

    fork
      begin : stimulus
        forever begin
          seq = pcs_sequence::type_id::create("seq");
          seq.start(env.agent[0].seqr);
        end
      end

      begin : reset_walker
        `uvm_info(get_type_name(), "reset walker started", UVM_LOW)
        repeat (10) @(posedge vif.Clk);

        wait_for_state(ST_IDLE,    "s_send_idle");
        pulse_reset("s_send_idle");

        wait_for_state(ST_SDD2,    "s_SDD2");
        pulse_reset("s_SDD2");

        wait_for_state(ST_TX_DATA, "s_Transmit_data");
        pulse_reset("s_Transmit_data");

        wait_for_state(ST_CSR2,    "s_CSR2");
        pulse_reset("s_CSR2");

        wait_for_state(ST_ESD1,    "s_ESD1");
        pulse_reset("s_ESD1");

        wait_for_state(ST_ESD2,    "s_ESD2");
        pulse_reset("s_ESD2");

        repeat (50) @(posedge vif.Clk);
        `uvm_info(get_type_name(), "reset walker complete", UVM_LOW)
      end
    join_any
    disable fork;

    phase.drop_objection(this);
  endtask

endclass
