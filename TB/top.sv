`timescale 1ns/1ps

`include "uvm_macros.svh"
import uvm_pkg::*;

`include "../dut/dut.sv"

`include "../reference_model/pcs_tx_rm_dpi_pkg.sv"

`include "pcs_if.sv"
`include "pcs_item.sv"
`include "pcs_monitem.sv"
`include "pcs_sequencer.sv"
`include "pcs_driver.sv"
`include "pcs_monitor.sv"
`include "pcs_scoreboard.sv"
`include "pcs_agent.sv"
`include "pcs_env.sv"
`include "pcs_sequence.sv"
`include "pcs_test.sv"

module top;

  logic clk;

  pcs_if vif(clk);

  DUTS26_0 dut (
    .Clk   (clk),
    .Reset (~vif.rst_n),
    .Din   (vif.Din),
    .TX_EN (vif.TX_EN),
    .Dout  (vif.Dout)
  );

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    vif.rst_n  = 0;
    vif.Din   = 8'h0;
    vif.TX_EN = 1'b0;

    repeat (5) @(posedge clk);

    vif.rst_n = 1;
  end

  initial begin
    uvm_config_db#(virtual pcs_if)::set(
      null,
      "uvm_test_top.env.agent[0]",
      "vif",
      vif
    );

    run_test("pcs_test");
  end

endmodule