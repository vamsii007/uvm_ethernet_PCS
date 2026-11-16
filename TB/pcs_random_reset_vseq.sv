`include "uvm_macros.svh"
import uvm_pkg::*;

class pcs_random_reset_vseq extends uvm_sequence #(pcs_item);

  `uvm_object_utils(pcs_random_reset_vseq)
  `uvm_declare_p_sequencer(pcs_virtual_sequencer)

  int unsigned dut_id = 0;

  rand int unsigned num_sequences;
  rand int unsigned num_resets;
  rand int unsigned min_reset_gap_cycles;
  rand int unsigned max_reset_gap_cycles;
  rand int unsigned min_reset_width_cycles;
  rand int unsigned max_reset_width_cycles;

  constraint reset_cfg_c {
    num_sequences inside {[2:5]};
    num_resets    inside {[50:150]};

    min_reset_gap_cycles inside {[1:5]};
    max_reset_gap_cycles inside {[6:40]};

    min_reset_width_cycles inside {[1:3]};
    max_reset_width_cycles inside {[4:10]};

    min_reset_gap_cycles < max_reset_gap_cycles;
    min_reset_width_cycles < max_reset_width_cycles;
  }

  function new(string name = "pcs_random_reset_vseq");
    super.new(name);
  endfunction

  task apply_initial_reset();

    `uvm_info(get_type_name(), "Applying initial reset", UVM_LOW)

    p_sequencer.vif[dut_id].cb_dr.rst_n <= 1'b0;

    repeat (5) @(p_sequencer.vif[dut_id].cb_dr);

    p_sequencer.vif[dut_id].cb_dr.rst_n <= 1'b1;

    repeat (5) @(p_sequencer.vif[dut_id].cb_dr);

    `uvm_info(get_type_name(), "Initial reset released", UVM_LOW)

  endtask

  task traffic_thread();

    pcs_sequence seq;

    for (int i = 0; i < num_sequences; i++) begin
      seq = pcs_sequence::type_id::create($sformatf("seq_%0d", i));

      `uvm_info(get_type_name(),
        $sformatf("Starting randomized traffic sequence %0d", i),
        UVM_MEDIUM)

      seq.start(p_sequencer.seqr[dut_id]);
    end

  endtask

  task random_reset_thread();

    int unsigned gap_cycles;
    int unsigned width_cycles;

    for (int r = 0; r < num_resets; r++) begin

      gap_cycles   = $urandom_range(max_reset_gap_cycles, min_reset_gap_cycles);
      width_cycles = $urandom_range(max_reset_width_cycles, min_reset_width_cycles);

      repeat (gap_cycles) @(p_sequencer.vif[dut_id].cb_dr);

      `uvm_info(get_type_name(),
        $sformatf("Random reset %0d: assert for %0d cycles after gap %0d cycles",
                  r, width_cycles, gap_cycles),
        UVM_LOW)

      p_sequencer.vif[dut_id].cb_dr.rst_n <= 1'b0;

      repeat (width_cycles) @(p_sequencer.vif[dut_id].cb_dr);

      p_sequencer.vif[dut_id].cb_dr.rst_n <= 1'b1;

      `uvm_info(get_type_name(),
        $sformatf("Random reset %0d: released", r),
        UVM_LOW)

      repeat (2) @(p_sequencer.vif[dut_id].cb_dr);
    end

  endtask

  task body();

    if (p_sequencer == null) begin
      `uvm_fatal(get_type_name(), "p_sequencer is null")
    end

    if (dut_id >= p_sequencer.num_duts) begin
      `uvm_fatal(get_type_name(),
        $sformatf("dut_id=%0d out of range num_duts=%0d",
                  dut_id, p_sequencer.num_duts))
    end

    if (p_sequencer.vif[dut_id] == null) begin
      `uvm_fatal(get_type_name(), "vif handle is null")
    end

    if (p_sequencer.seqr[dut_id] == null) begin
      `uvm_fatal(get_type_name(), "sequencer handle is null")
    end

    if (!this.randomize()) begin
      `uvm_fatal(get_type_name(), "pcs_random_reset_vseq randomization failed")
    end

    `uvm_info(get_type_name(),
      $sformatf("Random reset config: DUT=%0d num_sequences=%0d num_resets=%0d gap=[%0d:%0d] width=[%0d:%0d]",
                dut_id,
                num_sequences,
                num_resets,
                min_reset_gap_cycles,
                max_reset_gap_cycles,
                min_reset_width_cycles,
                max_reset_width_cycles),
      UVM_LOW)

    apply_initial_reset();

    fork
      traffic_thread();
      random_reset_thread();
    join

    repeat (50) @(p_sequencer.vif[dut_id].cb_dr);

  endtask

endclass
