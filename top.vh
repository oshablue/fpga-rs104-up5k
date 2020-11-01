// MACRO for toggling PLL on/off for respectively real world h/w vs simulation of POST for example
//`define SIM


`ifdef SIM
  //`include "submodules/one-shots.v" // in the subdir, it is not automatically included, so we can an include statement universally
  `define TEST_SMALL_DATA
  `define USE_SIM_CLK_NOT_PLL
`endif // ifdef SIM





// For turning on the selfread so that after capture, data is dumped in parallel
// out to some external fifo for example
// Tested - so far yes still works as well with manually clocked out data from
// nano program arduino with this macro turned off
`define SELFREAD 1

// For use with nano arduino clocked read sample output and also read into
// the external fifo (write into it really):
//`define SYNC_SELFREAD_WITH_CLOCKED_OUT 1

// For external fifo - note that documentation for the FT2232 chip shows different
// implementations in different areas.  However, currently for the FT2232H
// datasheet, the wr# is, as it is named with the "#", active low, thus idle high
// Table 3.7 in DS_FT2232H
// See Figure 4.6 in DS_FT2232H - this matches idle high, active low strobe
// with the write-into-2232-fifo happening on the falling edge
// As long as TXE# is not high
`define EXT_FIFO_WR_ACTIVE  1'b 0
`define EXT_FIFO_WR_IDLE    1'b 1


// Here's the deal with PLL implementation
// Ok - here are the refs:
// Err... see the Readme
// https://github.com/mystorm-org/BlackIce-II/wiki/PLLs-Advanced
// https://github.com/YosysHQ/arachne-pnr/issues/64
// You see my friend, apparently, when the PLLSOURCE pin (external)
// is IOB bottom Bank 1 (for example we were/are using in our design G3/IOB_25b)
// this compiles fine using SB_PLL40_CORE
// HOWEVER
// When CLK input goes to eg Pin 35 which is IOT_46B_G0 (Top, Bank 0)
// the build breaks and arachne-pnr can't place the PLL so we need to use
// PLL 2 which allows both an external pin to be used both to drive the PLL
// and internally on the FPGA (as an output from the PLL block)
// Defining the item below is like setting CLK (in from external oscillator)
// to Pin 35 like for use with the Lattice Eval Board (UltraPlus Breakout Board)
// Whereas undefining it is like using our planned implementation and setting the
// CLK input pin to be eg Pin 20
// Using Pin defs that don't match the imlementation and not using the correct
// PLL (for example allowing the item below to be defined, but keeping CLK on
// pin 20) will break the build.
//
// Uncomment to use breakout board with Pin 35 as the clock input (external)
// Comment it to use our pin 20 implementation in the hardware HDL-0108-RSCPT
// COMMENT OUT NEXT define LINE FOR HDL HARDWARE
// UNCOMMENT NEXT define LINE FOR LATTICE SEMI DEMO BOARD
//`define USE_LATTICE_BREAKOUT_DEMO
// Thus below, not defined for HDL-0108-RSCPT config/hw
`ifdef USE_LATTICE_BREAKOUT_DEMO
  `define PLLSOURCE_BOTTOM_BANK
`endif
// CAUTION: Have not yet tested with the above commented out and then swapping
// PCF file for the target build
// THOUGH: Tested with not defined USE_LATTICE_BREAKOUT_DEMO and using
// pins-up5k-sg48.pcf for pins as implemented on the HDL-0108-RSCPT R01 A1





// Choose between using the read_mem_in implementation or otherwise
// using the read_mem_in line instead as rx-delay-bit selection
// Could also otherwise implement the rx-delay as a single line with external
// control whose state simply defines the rx-delay after capture init
// Use either both of the top two, or just the 3rd item
// This implementation could obviously be cleaned up - just looking to retain
// functionality during development.
// These also relate to the SELFREAD - so please coordinate the defines.
// They are more granular here for dev reasons.
//`define USE_READ_MEM_IN_AS_READ_MEM_IN
//`define USE_CAPT_DONE_AS_CAPT_DONE
`define USE_RX_DELAY_CTRL_2BIT


// TODO - noticed some potential noise at 0 Rx delay with updated
// multi rx delay implemented - esp. at zero points
// Tested various combos of reverting back to no rx delay and
// moving capture (from data_in) timing to be based on posedge sclk for example
// and all effect jitter or adc settling noise, it seems.
// For now, implementation is ok - however possible improvement by
// creating a delayed clock for the data_in grab that is sync'd to a delay
// from the capture signal by some small amount
// Thus perhaps running internally at 160MHz if possible (or similar)
// And using the same SAMPLECLK (80MHz) for the main timing functions,
// the divided by 2 clock (sclk) for the adc, and then uncomment the
// block below, swapping it, for a slightly delayed grab from data_in such
// that it is basically the only thing running at a slightly delayed clock edge,
// yet still within the timing needed before next capture and data shifts, etc.

// Confirmations that indeed we are running at 40MHz sample capture (encode)
// clock and that the returned samples are correct in time, showing the
// correct waveform in time, eg thickness as calculated from the return
// in real life is correct


// UART output for e.g. RS-485/422 style conversion output drive in our particular
// system
`define USE_UART // 1



// Testing 0 - 9 ascii output at 1 second intervals:
//`define UART_1SEC_OUTPUT_TEST

// Include / use the module that encapsulates a 1-second interval 9600
// digit transmit
//`define USE_UART_TEST_MODULE



// Testing clock output to a pin
`define TEST_TIMING_TO_PIN_OUTPUT



// Testing without using BRAMs generated
//`define TEST_NO_BRAM




// TESTING - Test increment data values into memory instead of capture data
//`define DATA_TEST_INCREMENT_VALUES




// Testing for non-brams timing, using small data bufs
//`define TEST_SMALL_DATA





// Idle state for the uart_send signal
`define UART_SEND_IDLE    1'b 1











// For parallel data output
// Default: Not used for UART / RS-485
//`define USE_DATA_PORT



// For TESTING on the LAttice Demo Board
// Using leds
//`define FLASH_LEDS 1





// For TESTING with Multiple Pulse Patterns
//`define DEMO_VAR_FREQ_PULSE






//`define SEQ_ID_IS_BASE_1
`define SEQ_ID_IS_BASE_0




`ifndef USE_UART
  // TODO - Implemented TXE# (fifo_txe)

  // divide_by_n HDL-0108-RSCPT implementation used asymmetric divide_by_n
  // where the division, except for divide by 2, kept constant pulse width
  // for logic high and simply spaced these further apart with longer logic
  // low with increasing N
  `define DIVIDE_BY_N_ASYMMETRIC

`endif



`define CLK_2MBPS_UART_40MHZ_SAMPLE_RATE

// TODO Only intermittently worked? Maybe something with UART trigger/counter issue ?
//`define CLK_1MBPS_UART_20MHZ_SAMPLE_RATE
