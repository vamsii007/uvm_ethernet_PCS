`timescale 1ns/1ps

`include "uvm_macros.svh"
import uvm_pkg::*;

`include "../dut/dut.sv"
`include "../reference_model/pcs_tx_rm_dpi_pkg.sv"

`include "pcs_if.sv"
`include "pcs_item.sv"
`include "pcs_monitem.sv"
`include "pcs_sequencer.sv"
`include "pcs_virtual_sequencer.sv"
`include "pcs_driver.sv"
`include "pcs_monitor.sv"
`include "pcs_scoreboard.sv"
`include "pcs_agent.sv"
`include "pcs_env.sv"
`include "pcs_sequence.sv"
`include "pcs_random_reset_vseq.sv"
`include "pcs_test.sv"


module top;


    localparam int NUM_DUTS = 1;

  logic clk;

  pcs_if vif[NUM_DUTS](clk);

  genvar g;

 generate
  for(g = 0; g < NUM_DUTS; g++) begin
  DUTS26_0 dut (
    .Clk   (clk),
    .Reset (~vif[g].rst_n),
    .Din   (vif[g].Din),
    .TX_EN (vif[g].TX_EN),
    .Dout (vif[g].Dout)
  );
   initial begin
    vif[g].rst_n = 1'b0;
    vif[g].Din   = 8'h00;
    vif[g].TX_EN = 1'b0;

    uvm_config_db#(virtual pcs_if)::set(
      null,
     $sformatf("uvm_test_top.env.agent_%0d", g),
      "vif",
      vif[g]
    );
   end
  end
 endgenerate


  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end


  initial begin
    uvm_config_db#(int unsigned)::set(
      null,
      "uvm_test_top.env",
      "num_duts",
      NUM_DUTS
    );

    run_test("pcs_test");
  end

endmodule
