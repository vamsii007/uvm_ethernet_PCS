interface pcs_if(input logic Clk);

  logic rst_n;
  logic [7:0] Din;
  logic TX_EN;
  logic [3:0][2:0] Dout;

  clocking cb_dr @(posedge Clk);
    output rst_n;
    output Din;
    output TX_EN;
    input  Dout;
  endclocking

  clocking cb_mon @(posedge Clk);
    input rst_n;
    input Din;
    input TX_EN;
    input Dout;
  endclocking

  modport DRV(clocking cb_dr, input Clk);
  modport MON(clocking cb_mon, input Clk);

  modport DUT(
    input  Clk,
    input  rst_n,
    input  Din,
    input  TX_EN,
    output Dout
  );

endinterface