package pcs_tx_rm_dpi_pkg;
  // Single-instance DPI wrapper for the professor DUTS26_0-compatible C reference model.
  // Call pcs_rm_reset() whenever the DUT reset is asserted.
  
  // Remember to do " import pcs_tx_rm_dpi_pkg::*; " in the scoreboard

  import "DPI-C" context function void pcs_dpi_dut0_reset();
  import "DPI-C" context function int unsigned pcs_dpi_dut0_step(
      input int unsigned din,
      input int unsigned tx_en
  );

  import "DPI-C" context function void pcs_dpi_dut0_step_debug(
      input  int unsigned din,
      input  int unsigned tx_en,
      output int unsigned dout12,
      output int unsigned sc,
      output int unsigned sd,
      output int unsigned condition,
      output int unsigned state
  );

  function automatic void pcs_rm_reset();
    pcs_dpi_dut0_reset();
  endfunction

  function automatic logic [11:0] pcs_rm_step12(input logic [7:0] din,
                                                input logic       tx_en);
    int unsigned raw;
    raw = pcs_dpi_dut0_step({24'd0, din}, {31'd0, tx_en});
    pcs_rm_step12 = raw[11:0];
  endfunction

  function automatic logic [3:0][2:0] pcs_rm_step(input logic [7:0] din,
                                                  input logic       tx_en);
    logic [11:0] raw12;
    raw12 = pcs_rm_step12(din, tx_en);
    pcs_rm_step = raw12;
  endfunction

endpackage : pcs_tx_rm_dpi_pkg
