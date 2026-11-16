`include "uvm_macros.svh"
import uvm_pkg::*;

// =============================================================
// Min-size packet test
// =============================================================
class pcs_test_min extends pcs_test;
  `uvm_component_utils(pcs_test_min)
  function new(string name="pcs_test_min", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_min_packet s;
    phase.raise_objection(this);
    s = pcs_seq_min_packet::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// Max-size packet test
// =============================================================
class pcs_test_max extends pcs_test;
  `uvm_component_utils(pcs_test_max)
  function new(string name="pcs_test_max", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_max_packet s;
    phase.raise_objection(this);
    s = pcs_seq_max_packet::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// All-zeros test
// =============================================================
class pcs_test_all_zeros extends pcs_test;
  `uvm_component_utils(pcs_test_all_zeros)
  function new(string name="pcs_test_all_zeros", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_all_zeros s;
    phase.raise_objection(this);
    s = pcs_seq_all_zeros::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// All-ones test
// =============================================================
class pcs_test_all_ones extends pcs_test;
  `uvm_component_utils(pcs_test_all_ones)
  function new(string name="pcs_test_all_ones", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_all_ones s;
    phase.raise_objection(this);
    s = pcs_seq_all_ones::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// Walking ones test
// =============================================================
class pcs_test_walking_ones extends pcs_test;
  `uvm_component_utils(pcs_test_walking_ones)
  function new(string name="pcs_test_walking_ones", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_walking_ones s;
    phase.raise_objection(this);
    s = pcs_seq_walking_ones::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// Walking zeros test
// =============================================================
class pcs_test_walking_zeros extends pcs_test;
  `uvm_component_utils(pcs_test_walking_zeros)
  function new(string name="pcs_test_walking_zeros", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_walking_zeros s;
    phase.raise_objection(this);
    s = pcs_seq_walking_zeros::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// Alternating 0x55/0xAA test
// =============================================================
class pcs_test_alt_55_aa extends pcs_test;
  `uvm_component_utils(pcs_test_alt_55_aa)
  function new(string name="pcs_test_alt_55_aa", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_alt_55_aa s;
    phase.raise_objection(this);
    s = pcs_seq_alt_55_aa::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// Same-byte test
// =============================================================
class pcs_test_same_byte extends pcs_test;
  `uvm_component_utils(pcs_test_same_byte)
  function new(string name="pcs_test_same_byte", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_same_byte s;
    phase.raise_objection(this);
    s = pcs_seq_same_byte::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass

// =============================================================
// Back-to-back test
// =============================================================
class pcs_test_btb extends pcs_test;
  `uvm_component_utils(pcs_test_btb)
  function new(string name="pcs_test_btb", uvm_component parent=null);
    super.new(name, parent); endfunction
  task run_phase(uvm_phase phase);
    pcs_seq_btb s;
    phase.raise_objection(this);
    s = pcs_seq_btb::type_id::create("s");
    s.start(env.agent[0].seqr);
    #500ns;
    phase.drop_objection(this);
  endtask
endclass
