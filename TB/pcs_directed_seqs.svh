`include "uvm_macros.svh"
import uvm_pkg::*;

// =============================================================
// Min-size packet: exactly 64 bytes
// =============================================================
class pcs_seq_min_packet extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_min_packet)
  function new(string name="pcs_seq_min_packet"); super.new(name); endfunction

  task body();
    pcs_item item;
    repeat (5) begin
      item = pcs_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        data.size() == 64;
        gap_incycles == 12;
      }) `uvm_error("RAND", "min packet randomize failed")
      finish_item(item);
    end
  endtask
endclass

// =============================================================
// Max-size packet: exactly 1518 bytes
// =============================================================
class pcs_seq_max_packet extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_max_packet)
  function new(string name="pcs_seq_max_packet"); super.new(name); endfunction

  task body();
    pcs_item item;
    repeat (3) begin
      item = pcs_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        data.size() == 1518;
        gap_incycles == 12;
      }) `uvm_error("RAND", "max packet randomize failed")
      finish_item(item);
    end
  endtask
endclass

// =============================================================
// All zeros payload: stresses the scrambler in isolation
// =============================================================
class pcs_seq_all_zeros extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_all_zeros)
  function new(string name="pcs_seq_all_zeros"); super.new(name); endfunction

  task body();
    pcs_item item;
    repeat (3) begin
      item = pcs_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        data.size() == 200;
        foreach (data[i]) data[i] == 8'h00;
        gap_incycles == 12;
      }) `uvm_error("RAND", "all-zeros randomize failed")
      finish_item(item);
    end
  endtask
endclass

// =============================================================
// All ones payload: complement of all-zeros
// =============================================================
class pcs_seq_all_ones extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_all_ones)
  function new(string name="pcs_seq_all_ones"); super.new(name); endfunction

  task body();
    pcs_item item;
    repeat (3) begin
      item = pcs_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        data.size() == 200;
        foreach (data[i]) data[i] == 8'hFF;
        gap_incycles == 12;
      }) `uvm_error("RAND", "all-ones randomize failed")
      finish_item(item);
    end
  endtask
endclass

// =============================================================
// Walking ones: byte 0=0x01, byte 1=0x02, byte 2=0x04, ..., wrap at bit 7
// Exercises every bit position in the data path one at a time
// =============================================================
class pcs_seq_walking_ones extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_walking_ones)
  function new(string name="pcs_seq_walking_ones"); super.new(name); endfunction

  task body();
    pcs_item item = pcs_item::type_id::create("item");
    start_item(item);
    if (!item.randomize() with {
      data.size() == 64;
      foreach (data[i]) data[i] == (8'h01 << (i % 8));
      gap_incycles == 12;
    }) `uvm_error("RAND", "walking-ones randomize failed")
    finish_item(item);
  endtask
endclass

// =============================================================
// Walking zeros: byte 0=0xFE, byte 1=0xFD, byte 2=0xFB, ...
// =============================================================
class pcs_seq_walking_zeros extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_walking_zeros)
  function new(string name="pcs_seq_walking_zeros"); super.new(name); endfunction

  task body();
    pcs_item item = pcs_item::type_id::create("item");
    start_item(item);
    if (!item.randomize() with {
      data.size() == 64;
      foreach (data[i]) data[i] == ~(8'h01 << (i % 8));
      gap_incycles == 12;
    }) `uvm_error("RAND", "walking-zeros randomize failed")
    finish_item(item);
  endtask
endclass

// =============================================================
// Alternating 0x55/0xAA: maximum bit transitions
// =============================================================
class pcs_seq_alt_55_aa extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_alt_55_aa)
  function new(string name="pcs_seq_alt_55_aa"); super.new(name); endfunction

  task body();
    pcs_item item = pcs_item::type_id::create("item");
    start_item(item);
    if (!item.randomize() with {
      data.size() == 200;
      foreach (data[i]) data[i] == ((i % 2 == 0) ? 8'h55 : 8'hAA);
      gap_incycles == 12;
    }) `uvm_error("RAND", "alt 55/AA randomize failed")
    finish_item(item);
  endtask
endclass

// =============================================================
// Same-byte packet: every byte = 0xA5 (mixed 1s and 0s)
// =============================================================
class pcs_seq_same_byte extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_same_byte)
  function new(string name="pcs_seq_same_byte"); super.new(name); endfunction

  task body();
    pcs_item item = pcs_item::type_id::create("item");
    start_item(item);
    if (!item.randomize() with {
      data.size() == 200;
      foreach (data[i]) data[i] == 8'hA5;
      gap_incycles == 12;
    }) `uvm_error("RAND", "same-byte randomize failed")
    finish_item(item);
  endtask
endclass

// =============================================================
// Back-to-back: many packets with minimum gap
// =============================================================
class pcs_seq_btb extends uvm_sequence#(pcs_item);
  `uvm_object_utils(pcs_seq_btb)
  function new(string name="pcs_seq_btb"); super.new(name); endfunction

  task body();
    pcs_item item;
    repeat (10) begin
      item = pcs_item::type_id::create("item");
      start_item(item);
      if (!item.randomize() with {
        data.size() inside {[64:128]};
        gap_incycles == 12;
      }) `uvm_error("RAND", "btb randomize failed")
      finish_item(item);
    end
  endtask
endclass
