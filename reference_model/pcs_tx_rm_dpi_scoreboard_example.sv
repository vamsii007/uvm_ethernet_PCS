// Example usage inside a scoreboard or monitor-side checker.
// Adjust signal names to your virtual interface.

import pcs_tx_rm_dpi_pkg::*;

always_ff @(posedge vif.Clk or posedge vif.Reset) begin
  if (vif.Reset) begin
    pcs_rm_reset();
  end else begin
    logic [3:0][2:0] exp_dout;

    // Call once per DUT clock, using the same Din/TX_EN values seen by the DUT
    // for this clock edge.  The C model returns the expected registered output
    // after this edge.
    exp_dout = pcs_rm_step(vif.Din, vif.TX_EN);

    if (vif.Dout !== exp_dout) begin
      `uvm_error("PCS_SB", $sformatf(
        "Mismatch Din=0x%02h TX_EN=%0b exp=%012b got=%012b",
        vif.Din, vif.TX_EN, exp_dout, vif.Dout))
    end
  end
end
