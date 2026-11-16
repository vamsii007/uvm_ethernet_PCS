
`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_virtual_sequencer extends uvm_sequencer #(pcs_item);

  `uvm_component_utils(pcs_virtual_sequencer)

  int unsigned num_duts;

  pcs_sequencer seqr[];
  virtual pcs_if vif[];

  function new(string name = "pcs_virtual_sequencer",
               uvm_component parent = null);
    super.new(name, parent);
  endfunction

endclass

