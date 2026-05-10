`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_monitem extends uvm_sequence_item;

 bit rst_n;
 bit [7:0] Din;
 bit TX_EN;
 bit [3:0][2:0] Dout;

`uvm_object_utils_begin(pcs_monitem)
 `uvm_field_int(rst_n,UVM_ALL_ON)
 `uvm_field_int(enc_in,UVM_ALL_ON)
 `uvm_field_int(enc_out,UVM_ALL_ON)
`uvm_object_utils_end

 function new(string name="pcs_monitem");
  super.new(name);
 endfunction

endclass 

