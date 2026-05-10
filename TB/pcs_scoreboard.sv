`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

import pcs_tx_rm_dpi_pkg::*;

class pcs_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(pcs_scoreboard)

  uvm_analysis_imp #(pcs_monitem, pcs_scoreboard) imp;

  int pcs_id;
  int pass_count;
  int fail_count;

  function new(string name="pcs_scoreboard", uvm_component parent=null);
    super.new(name, parent);
    imp = new("imp", this);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    if (!uvm_config_db#(int)::get(this, "", "pcs_id", pcs_id))
      pcs_id = -1;
  endfunction

  function void write(pcs_monitem tr);
    logic [3:0][2:0] exp_dout;

    // Active-high reset
    if (tr.rst_n) begin
      pcs_rm_reset();
      `uvm_info("PCS_SB", $sformatf("PCS[%0d] reference model reset", pcs_id), UVM_LOW)
      return;
    end

    // Call C reference model
    exp_dout = pcs_rm_step(tr.Din, tr.TX_EN);

    // Compare DUT output with reference model output
    if (tr.Dout !== exp_dout) begin
      fail_count++;

      `uvm_error("PCS_SB", $sformatf(
        "PCS[%0d] MISMATCH: Din=0x%02h TX_EN=%0b expected=%012b got=%012b",
        pcs_id,
        tr.Din,
        tr.TX_EN,
        exp_dout,
        tr.Dout
      ))
    end
    else begin
      pass_count++;

      `uvm_info("PCS_SB", $sformatf(
        "PCS[%0d] PASS: Din=0x%02h TX_EN=%0b Dout=%012b",
        pcs_id,
        tr.Din,
        tr.TX_EN,
        tr.Dout
      ), UVM_HIGH)
    end
  endfunction

  function void report_phase(uvm_phase phase);
    super.report_phase(phase);

    `uvm_info("PCS_SB", $sformatf(
      "PCS[%0d] SCOREBOARD RESULT: PASS=%0d FAIL=%0d",
      pcs_id, pass_count, fail_count
    ), UVM_LOW)
  endfunction

endclass