import uvm_pkg::*;
`include "uvm_macros.svh"

class pcs_sequence extends uvm_sequence#(pcs_item);
    `uvm_object_utils(pcs_sequence)


 rand int packet_num;

 constraint num_packets_c {
    packet_num inside {[60:70]};
  }

function new(string name = "pcs_sequence");
 super.new(name);
endfunction


virtual task body();
  pcs_item my_item;

  if(!this.randomize())
       `uvm_error("SEQ_RAND", "pcs_sequence randomization failed")

  repeat(packet_num) begin 
    my_item = pcs_item::type_id::create("my_item");

     start_item(my_item);

      if(!my_item.randomize())
        `uvm_error("ITEM_RAND", "pcs_item randomization failed")

     finish_item(my_item);
  end 

endtask

endclass
