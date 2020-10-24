// MACRO for toggling PLL on/off for respectively real world h/w vs simulation of POST for example
//`define SIM


`ifdef SIM
  //`include "submodules/one-shots.v" // in the subdir, it is not automatically included, so we can an include statement universally
  `define TEST_SMALL_DATA
  `define USE_SIM_CLK_NOT_PLL
`endif // ifdef SIM
