`timescale 1ns/1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_item extends uvm_sequence_item;
 

  rand bit [7:0] data[];
  rand int gap_incycles;

   constraint packet_size {
    data.size() inside {[64:1518]};
   }

   constraint gap_between_packets {
    gap_incycles inside {[12:50]};
   }

   
  `uvm_object_utils_begin(pcs_item)
    `uvm_field_array_int(data, UVM_ALL_ON)
    `uvm_field_int      (gap_incycles, UVM_ALL_ON)
  `uvm_object_utils_end
  

  function new(string name="pcs_item");
    super.new(name);
  endfunction

endclass
