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

  bit       prev_valid;
  bit [7:0] prev_Din;
  bit       prev_TX_EN;

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

    if (!tr.rst_n) begin
      pcs_rm_reset();
      `uvm_info("PCS_SB", $sformatf(
    "PCS[%0d] RESET ACTIVE: Din=0x%02h TX_EN=%0b Dout=%012b, ref symbols=%0s, dut symbols=%0s",
    pcs_id,
    tr.Din,
    tr.TX_EN,
    tr.Dout,
    symbols_to_string(exp_dout), symbols_to_string(tr.Dout)), UVM_LOW)
   return;
    end

    if (!prev_valid) begin
      prev_Din   = tr.Din;
      prev_TX_EN = tr.TX_EN;
      prev_valid = 1;
      return;
    end

    exp_dout = pcs_rm_step(prev_Din, prev_TX_EN);

    if (tr.Dout !== exp_dout) begin
      fail_count++;
      `uvm_error("PCS_SB", $sformatf(
        "PCS[%0d] MISMATCH: Din=0x%02h TX_EN=%0b expected=%003h got=%003h, ref symbols=%0s, dut symbols=%0s",
        pcs_id, prev_Din, prev_TX_EN, exp_dout, tr.Dout, symbols_to_string(exp_dout), symbols_to_string(tr.Dout)))
    end
    else begin
      pass_count++;
      `uvm_info("PCS_SB", $sformatf(
        "PCS[%0d] PASS: Din=0x%02h TX_EN=%003h Dout=%003hb, symbols=%0s",
        pcs_id, prev_Din, prev_TX_EN, tr.Dout,symbols_to_string(tr.Dout) ), UVM_HIGH)
    end

    prev_Din   = tr.Din;
    prev_TX_EN = tr.TX_EN;
  endfunction

   function logic signed [2:0] unpack_symbol(input logic [2:0] z);
  case (z)
    3'b000: return  3'sd0;
    3'b001: return  3'sd1;
    3'b010: return  3'sd2;
    3'b111: return -3'sd1;
    3'b110: return -3'sd2;
    default: return 3'sd0;
  endcase
endfunction

  // ------------------------------------------------------------
  // Helper: readable symbol print
  // ------------------------------------------------------------
  function string symbols_to_string(input logic [11:0] packed_d);
    logic signed [2:0] a;
    logic signed [2:0] b;
    logic signed [2:0] c;
    logic signed [2:0] d;

    a = unpack_symbol(packed_d[11:9]);
    b = unpack_symbol(packed_d[8:6]);
    c = unpack_symbol(packed_d[5:3]);
    d = unpack_symbol(packed_d[2:0]);

    return $sformatf("(%0d,%0d,%0d,%0d)", a, b, c, d);
  endfunction


  function void report_phase(uvm_phase phase);
    super.report_phase(phase);
    `uvm_info("PCS_SB", $sformatf(
      "PCS[%0d] SCOREBOARD RESULT: PASS=%0d FAIL=%0d",
      pcs_id, pass_count, fail_count), UVM_LOW)
  endfunction

endclass
