// top.v
//
// 2020-09-Sep-03
//
// Top.v:
//
//
// For:
//  HDL-0104-RS104 using UART => RS-485 board-level IO
//
//
// Derived from:
//  top-switched-back-to-HDL-but-has-UART-testing-base.v.bak
//
//
//
//
// Implementation:
// 80MHz output PLL from 16MHz oscillator input (CLK)
// Sampling at 40MHz with divide by 2 sample clock
// and the use of sync for all async signals ends up dividing by 2
// So you see some code like monostable vibrator for example running
// with twice the counter length
//
//
// Status:
// vbak.good02: Looks functional, async issues addressed - continuous
// sweep input signals look intact at the read output from arduino test code
// and capturing indeed the 4096 samples at 40MHz (have tested eg 400 samples
// of a 100kHz waveform at 40MHz and indeed this shows 1 full cycle on the
// plotter - indicating correct sampling in time and sample storage)
//
// Issue remaining:
// - is the always vertical, sometimes occurring spike on the eg 100kHz wf
//   due to timing/memory/async issue or just actual noise/bit issue in the
//   prototype setup

// Notes:
// All Lattice devices have weak pull up resistors on IO

// Refs:
// https://stackoverflow.com/questions/38030768/icestick-yosys-using-the-global-set-reset-gsr
// http://www.clifford.at/yosys/files/yosys_appnote_011_design_investigation.pdf
// https://en.wikibooks.org/wiki/Programmable_Logic/Verilog_RTL_Coding_Guidelines
// https://electronics.stackexchange.com/questions/38645/why-are-inferred-latches-bad
// https://www.doulos.com/knowhow/fpga/why-should-i-care-about-transparent-latches/
// https://electronics.stackexchange.com/questions/236845/register-with-enable-signal-problem-of-understanding-simulation-results
// https://stackoverflow.com/questions/38030768/icestick-yosys-using-the-global-set-reset-gsr

// Reminders:
// - (* keep *) works for SIM synth in POST to prevent pruning a signal however
//   it may not be desirable as pruning happens typically for a good reason
// - Yosys, rightly imho, doesn't like the // synthesis translate_off / translate_on directives

// This is only needed if command line yosys is used
// Or you can just add the one-shots.v to the command line command
// If you include here, you will get a redefinition error because the file
// is automatically included from within the top directory
// Only use include for files not in the top level directory here
`include "submodules/one-shots.v"

// However, the header (.vh) files are not automatically included in the compilation
// when they live in the same root directory
`include "top.vh"


`include "submodules/dividers.v"
`include "submodules/uart.v"





/*


   >=>       >=>     >===>            >======>   >=> >==>    >=>   >=>>=>
   >=>      >=>    >=>    >=>         >=>    >=> >=> >> >=>  >=> >=>    >=>
   >=>     >=>   >=>        >=>       >=>    >=> >=> >=> >=> >=>  >=>
   >=>    >=>    >=>        >=>       >======>   >=> >=>  >=>>=>    >=>
   >=>   >=>     >=>        >=>       >=>        >=> >=>   > >=>       >=>
   >=>  >=>        >=>     >=>        >=>        >=> >=>    >>=> >=>    >=>
   >=> >=>           >===>            >=>        >=> >=>     >=>   >=>>=>


*/

module top (
    input CLK,                  // 16MHz clock
    input RST,                  // RST for PLL?
    input ext_trig_in,              // trigger a capture event sequence from comms device
    //input read_mem_in,             // read strobe from comms device
    // repurpose read_mem_in as input for rx_delay_ctrl_b0:
    //input rx_delay_ctrl_b0,
    //input rx_delay_ctrl_b1,
    input [1:0] rx_delay_ctrl,
    input [7:0] data_in,

  `ifndef USE_UART
    input fifo_txe,             // FIFO ok to write (buffer empty/write to FIFO enabled)
  `endif

    //output LED,                 // User/boot LED next to power LED
    //output USBPU,               // USB pull-up resistor
    //output SAMPLECLK,           // capture output from PLL
    //output SAMPLECLK8X,         //
    //output clk_lock,

  `ifdef USE_DATA_PORT
    output [7:0] data_out,      // testing external LED
  `endif

    // re-purpose capt_done from output to input for rx_delay_ctrl_b1
    //output capt_done,           // capture event is complete
    output adc_encode,           // encode signal to ADC to capture a sample
  `ifndef USE_UART
    output fifo_wri,             // FIFO write strobe
  `endif
    output PULSE_NEG,
    output RTZ_NEG,
    output RTZ_POS

  `ifdef USE_LATTICE_BREAKOUT_DEMO  // Not true for HDL-0108-RSCPT or RS104 config/hw
    ,
    output RGB0,                 // status output
    output RGB1,
    output RGB2                   // last LED status indic
  `endif

  `ifdef USE_UART
    ,
    output UART_TX,
    input UART_RX
    //`ifdef TEST_TIMING_TO_PIN_OUTPUT - commented out so TB so instantiate UUT without error on these lines, even though they are not implemented in this case
      ,
      output PDO0,           // goes to JT3A Pin 10 (PDO0)
      output PDO1,           // goes to JT3A Pin 8 (PDO1)
      output PDO2,           // goes to JT3A Pin 6 (PDO2)
      output PDO3            // goes to JT3A Pin 4 (PDO3)
    //`endif
  `endif
);







    `ifndef TEST_SMALL_DATA
      parameter NSAMPLES = 12'b 1111_1111_1111; // 4096
    `else
      parameter NSAMPLES = 12'b 0001_1111; // 1111 = 8
    `endif // ifndef SIM

    // When not using external pins:
    wire SAMPLECLK, SAMPLECLK8X, clk_lock, clk160;

    wire read_mem;

    wire clk_copy;

    reg [7:0] seq_id;  // sequence ID for the frame
    `ifndef USE_UART
      reg [7:0] seq_id_nxt;
    `endif

    reg reset;

    reg [11:0] addr_wr, addr_wr_nxt, addr_rd, addr_rd_nxt;
    reg [7:0] val;

    reg [7:0] mem [0:NSAMPLES];    // Check hardware.out :: yeah, this gens the SB_RAM40_4K blocks







    /*

    >======>   >=>       >=>
    >=>    >=> >=>       >=>
    >=>    >=> >=>       >=>
    >======>   >=>       >=>
    >=>        >=>       >=>
    >=>        >=>       >=>
    >=>        >=======> >=======>

    */



    // Again, below (next) is temp set to ifndef - but this will break actual hardware build
    // within apio
  //`ifndef SYNTHESIS // usually ifdef - but for putting fake clock into SYN (POST) - change to ifndef
  // ? - using the ifndef SYNTH_TEST macro this builds ok in hardware (within apio) mode
  //`ifndef SYNTH_TEST // use this for SIMULATION (post) only
  //`ifdef SYNTHESIS // use this for real hardware to include the PLL
  `ifndef USE_SIM_CLK_NOT_PLL         // this MACRO is toggled by hand here
    `ifndef PLLSOURCE_BOTTOM_BANK // We are here for HDL-0108-RSCPT config/hw
      pllcore p(
          .clk    (CLK),          // 16MHz oscillator input
          //.clkin  (SAMPLECLK8X),  // now a non-pin
          .clkout (clk160), //SAMPLECLK),    // now a non-pin
          .lock   (clk_lock)      // now a non-pin
      );
      assign clk_copy = CLK;
    `else                         // We are here for Lattice breakout
      pll2pad p(
          .clk_in   (CLK),        // Input that is a pad/pin (real) //
          .clkout_a (clk_copy),   // Output that is a copy of the pin in clk
          .clkout_b (clk160),     // Output that is PLL out // was: SAMPLECLK
          .lock     (clk_lock)    // Pin 37 is no longer available as RST (input) when using this PLL version -- see PLL modules
      );
    `endif
  `else // ifndef was SYNTH_TEST / was SYNTHESIS

    // For synthesis (post-syn) simulation - for now, PLL not working for sim
    // so just simulate the clock
    // If yes, SYNTH_TEST (?Is this different from SYNTHESIS?)
    // Naw - still doesn't go here for simulation with ifndef SYNTH_TEST too
    assign clk160 = CLK;

  `endif // ifdef SYNTHESIS


    // Not current:
    // For 40MHz clock out from PLL:
    //assign adc_encode = SAMPLECLK;




    // clk160: 160 MHz
    // SAMPLECLK = 80 MHz
    // sclk = 40 MHz
    wire sclk;

    `ifndef USE_UART
      divide_by_n #(.N(2)) div160to80(clk160, 1'b0, SAMPLECLK);
      divide_by_n #(.N(4)) divsclk(clk160, 1'b0, sclk);
      assign adc_encode = sclk;
    `else
      wire sysclk;
      // Based on 160MHz PLL
      `ifdef CLK_2MBPS_UART_40MHZ_SAMPLE_RATE
        parameter SYSCLK_DIV = 2;
        parameter SAMPLECLK_DIV = 4;
      `endif
      `ifdef CLK_1MBPS_UART_20MHZ_SAMPLE_RATE // Only intermittently worked ... TODO
        parameter SYSCLK_DIV = 4;
        parameter SAMPLECLK_DIV = 8;
      `endif
      divide_by_n #(.N(SYSCLK_DIV)) div160to80(clk160, 1'b0, sysclk); // Usually 2 => 80MHz from 160MHz
      // In SIM, below create a 40 MHz clock but with 160MHz digital high
      // pulse width - which might create problems for the ADC? (not symmetric)
      // With current code implementation - so the sum total of code keeps the
      // origina pulse width and just lengthens the digital low width
      // 80MHz sysclk is however symmetric pulse width
      // In UART implementation, there is no other use of divide_by_n
      divide_by_n #(.N(SAMPLECLK_DIV)) divsclk(clk160, 1'b0, sclk); // Usually 4 => 40MHz from 160MHz

      //divide_by_n #(.N(2)) div160to80(clk160, 1'b0, sysclk);
      //divide_by_n #(.N(4)) divsclk(clk160, 1'b0, sclk);
      assign adc_encode = sclk;
    `endif








      /*

      >======>     >=======>   >=>>=>   >=======> >===>>=====>
      >=>    >=>   >=>       >=>    >=> >=>            >=>
      >=>    >=>   >=>        >=>       >=>            >=>
      >> >==>      >=====>      >=>     >=====>        >=>
      >=>  >=>     >=>             >=>  >=>            >=>
      >=>    >=>   >=>       >=>    >=> >=>            >=>
      >=>      >=> >=======>   >=>>=>   >=======>      >=>

      */
      /***************************************************************************
      //
      /* RESET SIGNAL DEFINITION
      //
      /***************************************************************************/

      // Reset section
      //
      // Yes, reset is mostly just for resetting the sequence count and not a
      // real complete reset signal -- for the per-waveform/read/write reset
      // we use the trig_in family of signals.
      //
      // Assuming yes that we have control of the RST input, from external
      // circuitry - and yes, may be different from power up,
      // For example to reset to initial conditions or reset a sequence
      // ID after a block of captures.
      // Here, we will assume an active low RST signal, as without the function
      // explicitly set up on the HDL MCU or this UP5K device, it is pulled up to
      // the power rail (3.3 here)
      // Async reset, allows either signal to run the block: clock or RST
      // Sync reset - we use this because two conditions can drive into seq_id_nxt
      //wire n_reset;
      //assign n_reset = RST; // not using directly to allow a spot to interject an updated functionality
      // active low means we are in reset condition
      //always @ ( posedge SAMPLECLK or negedge n_reset)  // Async
      // Thus: some hold time is required for the RST line external drive, but not much
      // Moving this section to the block above where these vars are otherwise used for the
      // remaining control flow ...
      // Trying again with a reset pulse
      // See the Lattice ice40 ultraplus Family Datasheet for timing max's
      // We are back to using
      wire reset_wire;
      monostable #(
        // 160MHz/32 = 5 for 32MHz operation of the MCU
        // However in practice it would appear that the reset pulse
        // from the MCU may be longer!
        // Was PW = 3, with CW = 4 - but not long enough! see next below:
        // Really could arrange to trigger the reset signal on the rising edge
        // (end of the reset pulse)
        // From scope measurement, using immediate toggle of the pin, not waiting
        // until the next ISR call to set reset !RST back high again which would
        // otherwise be the length of the ISR interval, currently around 28ms (27ms exact min calcd)
        // we have 108ns approx from 10% of first early drop to low to 10% first
        // early rise back to high 1/dX => 9.3MHz
        // 32/8 = 4 MCU clock cycles
        // 160MHz/8MHz = 20 = N'h 14 = 5 bits minimum if counter synth is understood
        // So we would want a longer PW too ...
        // Add 50% = 30 => N'h 1E => 5 bits minimum
        // And then next trig in wouldn't happen until next ISR cycle which is
        // very far into the future (28ms for UART for example)
        // Or for sysclk we divide by 2 for 15 (still trying 5 bits)
        // NOTE msv checks the trigger signal before incrementing, thus,
        // good ... counting only begins after release of the trigger signal
        .PULSE_WIDTH(15),
        .COUNTER_WIDTH(5)
      ) msv_reset (
          .clk          (sysclk),
          .reset        (1'b0),
          .trigger      (!RST),
          .pulse        (reset_wire)
      );
      //reg reset;
      always @( posedge sysclk ) begin
        reset <= reset_wire;
      end









    //**************************************************************************
    //
    /* LEDS
    //
    /**************************************************************************/

    // Not for HDL-0108-RSCPT or RS104 config/hw
  `ifdef USE_LATTICE_BREAKOUT_DEMO        // Not for HDL-0108-RSCPT or RS104 config/hw
    `ifdef FLASH_LEDS
      // Status LED output (?) ... testing
      wire pretty_slow;
      wire clk_super_slow;
      reg [2:0] ledsreg = 3'b 110;  // LEDs OFF = 1 ... ON = 0
      reg [2:0] leds = 3'b 000;
      reg [2:0] ledson = 3'b 111;
      assign RGB0 = leds[2];  // B
      assign RGB1 = leds[1];  // G
      assign RGB2 = leds[0];  // R
      // instead of RST try: 1'b0
      // yeah if we use RST -- this pin is currently confusing things
      // so for now, just use 1'b0 for the reset instead of the RST
      //divide_by_n #(.N(256)) div_slow(CLK, 1'b0, clk_slow);
      // Yeah testing with 256 is 16MHz / 256 = 62.5 kHz and yes this is the output
      divide_by_n #(.N(65536)) div_pretty_slow(clk_copy, 1'b0, pretty_slow);
      divide_by_n #(.N(32)) div_super_slow(pretty_slow, 1'b0, clk_super_slow);
      // TODO FOR FUN: Change to PWM and Rate MOD
      always @ ( posedge clk_super_slow) begin
        // Or could do with modulus and counter, etc.
        ledsreg <= {ledsreg[1:0], ledsreg[2]};  // left rotate with carry
        leds <= (ledsreg | ledson);             // | not & because 0 = On
        ledson <= ~ledson;                      // alt on / off
      end
    `else // if not define FLASH_LEDS
      // 1'b1 = OFF ... 1'b0 = ON
      // RGB0 = blue
      //assign RGB0 = 1'b1;
      //assign RGB1 = 1'b0; // at the moment, RGB0 flashing only works when RGB1 is on too ... it seems
      //assign RGB2 = 1'b1;
    `endif
  `endif











    /*

      >===>>=====> >======>     >=>    >===>          >=> >==>    >=>
           >=>     >=>    >=>   >=>  >>    >=>        >=> >> >=>  >=>
           >=>     >=>    >=>   >=> >=>               >=> >=> >=> >=>
           >=>     >> >==>      >=> >=>               >=> >=>  >=>>=>
           >=>     >=>  >=>     >=> >=>   >===>       >=> >=>   > >=>
           >=>     >=>    >=>   >=>  >=>    >>        >=> >=>    >>=>
           >=>     >=>      >=> >=>   >====>          >=> >=>     >=>

    */

    //*************************************************************************
    //
    /* TRIGGER INPUT SIGNAL MANAGEMENT AND GENERATION
    //
    /*************************************************************************/

    // https://electronics.stackexchange.com/questions/26502/verilog-check-for-two-negedges-in-always-block
    // TODO !!!: Probably should update this to be USE_UART case-dependent

    wire trig_in_fall, trig_in_rise;
    wire trig_in;
    wire trig_in_rise_pulse;

    monostable #(
      .PULSE_WIDTH(3),              // 160MHz/32 = 5 for 32MHz operation of the MCU
      .COUNTER_WIDTH(4)
    ) msv_trig_in (
      .clk          (sysclk),
      .reset        (1'b 0),
      .trigger      (ext_trig_in),
      .pulse        (trig_in)
    );
    edge_detect edet_trig_in (
      .async_sig  (trig_in),
      .clk        (sysclk), //was sclk //(clk_copy), // so far SAMPLECLK is best //clk160 should give edge lengths of 80MHz?
      .rise       (trig_in_rise),
      .fall       (trig_in_fall)
    );
    monostable #(
      .PULSE_WIDTH(3),
      .COUNTER_WIDTH(4)
    ) msv_trig_in_rise (
      .clk         (sysclk),
      .reset       (1'b 0),
      .trigger     (trig_in_rise),
      .pulse       (trig_in_rise_pulse)
    );
    // This was (above) all just only the external signal just feeding into the
    // edge_detect directly, and using sclk instead of SAMPLECLK









        /*

          >======>     >=>      >=>       >====>     >=======> >=>             >>       >=>      >=>
          >=>    >=>    >=>   >=>         >=>   >=>  >=>       >=>            >>=>       >=>    >=>
          >=>    >=>     >=> >=>          >=>    >=> >=>       >=>           >> >=>       >=> >=>
          >> >==>          >=>            >=>    >=> >=====>   >=>          >=>  >=>        >=>
          >=>  >=>       >=> >=>          >=>    >=> >=>       >=>         >=====>>=>       >=>
          >=>    >=>    >=>   >=>         >=>   >=>  >=>       >=>        >=>      >=>      >=>
          >=>      >=> >=>      >=>       >====>     >=======> >=======> >=>        >=>     >=>

        */

        //**************************************************************************
        //
        /* RX DELAY CONTROL
        //
        /**************************************************************************/

      `ifdef USE_RX_DELAY_CTRL_2BIT



        wire rx_delay;
        reg [13:0] rx_delay_clocks;

        // TODO - check the ifndef USE_UART condition

        `ifndef USE_UART
          parameter RX_DELAY_BASE = 2400; // 80e6 (clk rate) X 30e-6 = 2400 where 30e-6 is increments of 30 microseconds
          always @( posedge SAMPLECLK )
          begin
            if ( trig_in_rise ) begin
              rx_delay_clocks <= ( RX_DELAY_BASE * rx_delay_ctrl ) + 2; // See note below
            end
          end

          // 2 clock offset allows us to use the same !rx_delay signal to capture
          // the waveform even when the input is indicating zero usec delay request.
          // Using just +1 creates some jitter at the converter
          // quite visible - probably too fast a timing somewhere.
          // 2 clocks gets us into the 40MHz timing range, sufficiently low enough
          // for the converter's specs.

          monostable_vpw14b #(
              //.PULSE_WIDTH( RX_DELAY_BASE * rx_delay_ctrl_val ),
              .COUNTER_WIDTH(14)         // 2^14 = 16,384 max - for 3x rx delay base of 2400 => 7200
          ) msv_rx_delay (
              .pulse_width (rx_delay_clocks),
              .clk      (SAMPLECLK),
              .reset    (trig_in_rise),
              .trigger  (trig_in_fall),  // was trig_in_rise
              .pulse    (rx_delay)
          );

          wire rx_delay_rise, rx_delay_fall;
          edge_detect edet_rx_delay (
            .async_sig  (rx_delay),
            .clk        (clk160), //was sclk //(clk_copy), // so far SAMPLECLK is best //clk160 should give edge lengths of 80MHz?
            .rise       (rx_delay_rise),
            .fall       (rx_delay_fall)
          );
        `endif // ifndef USE_UART

        `ifdef USE_UART
          // we are now back to 80MHz
          //parameter RX_DELAY_BASE = 4800; // 80e6 (clk rate) X 30e-6 = 2400 where 30e-6 is increments of 30 microseconds
          parameter RX_DELAY_BASE = 2400;
          // Yes, the updates below fixed the unknown in the rx_delay signal (so far)
          // that was causing issues with damped pulse signal due to RTZ firing at the same
          // time as the pulse - traced back to essentially the rx_delay edge signal usage
          // triggering the delay to rtz / rtz early (same time as pulse!) - scoped output signals
          //always @( posedge sysclk ) begin
          always @ ( posedge trig_in_rise ) begin
            //if ( trig_in_rise ) begin
              rx_delay_clocks <= ( RX_DELAY_BASE * rx_delay_ctrl ) + 4; // See note below
            //end
          end

          // 2 clock offset allows us to use the same !rx_delay signal to capture
          // the waveform even when the input is indicating zero usec delay request.
          // Using just +1 creates some jitter at the converter
          // quite visible - probably too fast a timing somewhere.
          // 2 clocks gets us into the 40MHz timing range, sufficiently low enough
          // for the converter's specs.

          monostable_vpw14b #(
              //.PULSE_WIDTH( RX_DELAY_BASE * rx_delay_ctrl_val ),
              .COUNTER_WIDTH(14)         // 2^14 = 16,384 max - for 3x rx delay base of 2400 => 7200
          ) msv_rx_delay (
              .pulse_width (rx_delay_clocks),
              .clk      (sysclk),
              .reset    (trig_in_rise),
              .trigger  (trig_in_fall),  // was trig_in_rise
              .pulse    (rx_delay)
          );

          wire rx_delay_rise, rx_delay_fall;
          edge_detect edet_rx_delay (
            .async_sig  (rx_delay),
            .clk        (sysclk), //was sclk //(clk_copy), // so far SAMPLECLK is best //clk160 should give edge lengths of 80MHz?
            .rise       (rx_delay_rise),
            .fall       (rx_delay_fall)
          );
        `endif // ifdef USE_UART

      `endif // end USE RX DELAY 2 BIT SELECTOR















    /*

    >=>        >=> >======>     >=> >===>>=====> >=======>       >===>>=====> >======>     >=>    >===>
    >=>        >=> >=>    >=>   >=>      >=>     >=>                  >=>     >=>    >=>   >=>  >>    >=>
    >=>   >>   >=> >=>    >=>   >=>      >=>     >=>                  >=>     >=>    >=>   >=> >=>
    >=>  >=>   >=> >> >==>      >=>      >=>     >=====>              >=>     >> >==>      >=> >=>
    >=> >> >=> >=> >=>  >=>     >=>      >=>     >=>                  >=>     >=>  >=>     >=> >=>   >===>
    >> >>    >===> >=>    >=>   >=>      >=>     >=>                  >=>     >=>    >=>   >=>  >=>    >>
    >=>        >=> >=>      >=> >=>      >=>     >=======>            >=>     >=>      >=> >=>   >====>

    */
    //**************************************************************************
    //
    /* TRIGD signal for CAPTURE DATA (DATAIN)
    //
    /**************************************************************************/

      wire trigd;         // trigd = triggered =

      `ifndef USE_UART
        monostable #(
          .PULSE_WIDTH(NSAMPLES*2),
          .COUNTER_WIDTH(14)
        ) msv_capt(
          .clk          (SAMPLECLK),
          .reset        (trig_in_rise),
          `ifndef USE_RX_DELAY_CTRL_2BIT
            .trigger      (trig_in_rise),
          `endif
          `ifdef USE_RX_DELAY_CTRL_2BIT
            .trigger      (!rx_delay),
          `endif
          .pulse        (trigd)
        );


        `ifdef SELFREAD
          wire trigd_rise, trigd_fall;
          edge_detect edet_trigd (
              .async_sig  (trigd),
              .clk        (SAMPLECLK),
              .rise       (trigd_rise),
              .fall       (trigd_fall)
          );
        `endif
      `endif // ifndef USE_UART

      `ifdef USE_UART
        monostable #(
          .PULSE_WIDTH(NSAMPLES*2), // was 4 when using 160MHz clock // the wrong multiplier will shift the signal/front-face!
          .COUNTER_WIDTH(16)
        ) msv_capt(
          .clk          (sysclk),
          .reset        (trig_in_rise),
          `ifndef USE_RX_DELAY_CTRL_2BIT
            .trigger      (trig_in_rise),
          `endif
          `ifdef USE_RX_DELAY_CTRL_2BIT
            // TODO now that msv is not edge sensitive we have to watch for idle 0 conditions that trigger the trigd pulse
            .trigger      (rx_delay_fall), //(!rx_delay),
          `endif
          .pulse        (trigd)
        );

        `ifdef SELFREAD
          wire trigd_rise, trigd_fall;
          edge_detect edet_trigd (
              .async_sig  (trigd),
              .clk        (sysclk),
              .rise       (trigd_rise),
              .fall       (trigd_fall)
          );
        `endif

      `endif // ifndef USE_UART






      /*

      >======>     >=======>       >>       >====>           >===>>=====> >======>     >=>    >===>
      >=>    >=>   >=>            >>=>      >=>   >=>             >=>     >=>    >=>   >=>  >>    >=>
      >=>    >=>   >=>           >> >=>     >=>    >=>            >=>     >=>    >=>   >=> >=>
      >> >==>      >=====>      >=>  >=>    >=>    >=>            >=>     >> >==>      >=> >=>
      >=>  >=>     >=>         >=====>>=>   >=>    >=>            >=>     >=>  >=>     >=> >=>   >===>
      >=>    >=>   >=>        >=>      >=>  >=>   >=>             >=>     >=>    >=>   >=>  >=>    >>
      >=>      >=> >=======> >=>        >=> >====>                >=>     >=>      >=> >=>   >====>

      */

      //*************************************************************************
      //
      // NEW_CAPT (FRESH DATA READY TO READ OUT)
      //
      //*************************************************************************

      `ifdef SELFREAD
        reg new_capt = 0; // 0 is only for sim purposes, should be cleared on h/w reset

        `ifndef USE_UART
          always @( posedge SAMPLECLK )
          begin
            if ( trigd_fall ) begin
              new_capt <= 1'b 1;
            end
            if ( addr_rd_nxt == NSAMPLES ) begin
              new_capt <= 1'b 0;
            end
          end
        `endif // ifndef USE_UART

        `ifdef USE_UART

          always @( posedge sysclk )
          begin
            if ( trigd_fall ) begin
              new_capt <= 1'b 1;
            end
            //if ( addr_rd_nxt == NSAMPLES ) begin
            // Due to timing for UART, addr_rd lags enough that if we use
            // addr_dr_nxt to read to the end we miss the last sample and get
            // only 4094 it seems always - so we update this to add_rd for the
            // comparison clause and get now consistently 4095
            if ( addr_rd == NSAMPLES ) begin
              new_capt <= 1'b 0;
            end
          end

          // If using new_capt edge signals:

          wire new_capt_rise, new_capt_fall;

          edge_detect edet_new_capt (
            .async_sig  (new_capt),
            .clk        (sysclk),
            .rise       (new_capt_rise),
            .fall       (new_capt_fall)
          );


        `endif // ifdef USE_UART

      `endif // ifdef SELFREAD










    // http://www.patorjk.com/software/taag/#p=display&f=Big&t=READMEM
    // Arrows font
    /***************************************************************************


    >======>     >=======>       >>       >====>     >=>       >=> >=======> >=>       >=>
    >=>    >=>   >=>            >>=>      >=>   >=>  >> >=>   >>=> >=>       >> >=>   >>=>
    >=>    >=>   >=>           >> >=>     >=>    >=> >=> >=> > >=> >=>       >=> >=> > >=>
    >> >==>      >=====>      >=>  >=>    >=>    >=> >=>  >=>  >=> >=====>   >=>  >=>  >=>
    >=>  >=>     >=>         >=====>>=>   >=>    >=> >=>   >>  >=> >=>       >=>   >>  >=>
    >=>    >=>   >=>        >=>      >=>  >=>   >=>  >=>       >=> >=>       >=>       >=>
    >=>      >=> >=======> >=>        >=> >====>     >=>       >=> >=======> >=>       >=>


    //**************************************************************************
    //
    /* READ MEM / DUMPCLK / SELF READ / DATAOUT CLOCK SIGNAL TIMING
    //
    /**************************************************************************/

  `ifdef SELFREAD
    `ifndef USE_UART
      // TODO this output clock is very dirty ... or is it some IO interaction w/ FIFO external?
      // Yes, below, using 32 divider with bit toggle creates
      // a 50% duty clock at about 1.25MHz (complete cycle is ie about 800ns)
      // So this gives plenty of setup time on the WR# signal for the ext fifo
      // TODO the whole duration of the clock pulses is currently 204us which is
      // only 255 samples (hey, at least it's a multiple of 2 ;)
      // So ... next ...
      // Now duration of all write pulses is 3.30 ms
      // and still at 1.25MHz effective clock rate (800ns)
      // So this is: 4125 or yeah, looks like we're basically getting our
      // 4096 sample dump now
      // TODO why are we getting those voltage-truncate pulses now when the
      // 2232 is powered on and running and enabled -- voltage goes from the first
      // few pulses at 3.3V to somewhere around 2.5? volts - number of pulses varies
      // TXE signal is always high
      // TODO unless it's due to the nano always reading the pins too?
      wire dumpclkedges;
      // 12Mbps => FIFO 8-bit at 1.5MB/s (approx)
      // 80 MHz / 1.5 => 53 => 64 divider is the closest
      // 80 MHz / 64 => 1.25MHz
      // However, the dividy_by_n actually is only spikes (mini pulses at the divide interval)
      // So we create a /64 clock by using 32 and then using the toggle below
      // Below in most testing functional using 32
      // To test frequency clock noise issues, slowing this down a lot
      divide_by_n #(.N(32)) divdumpclk(SAMPLECLK, 1'b0, dumpclkedges);
      reg dumpclkreg;
      wire dumpclk;
      assign dumpclk = dumpclkreg;
      always @( posedge SAMPLECLK )
      begin
        if ( trigd | !new_capt ) begin
          dumpclkreg <= `EXT_FIFO_WR_IDLE; //1'b 1 I believe - see MACRO
        end
        if ( dumpclkedges & new_capt ) begin
          dumpclkreg <= ~dumpclkreg;
        end
      end
    `endif
  `endif // ifdef SELFREAD


  // Slower readmem signal for dumping the bytes over serial instead of parallel
  `ifdef SELFREAD
    `ifdef USE_UART
      wire read_mem_uart;
      // Initial drafting:
      // Tried 8 - but this gives more like a 51kHz byte start output rate from the bitstream
      // actually ... probably missing like every other count though
      // 4 gives more like 105kHz rate for start of each byte output measured from the bitstream
      // Theoretical is 80MHz/64/8 = 156,250Hz
      // However measuring the read_mem_uart_rise at the clock output test pin PDO0 for example
      // we get like 305kHz for the read_mem_uart_rise pulses
      // Caution of course: Shorter (too short)
      // creates that byte-wise overlap - unless we change the baud rate (bit width in time)
      // So measured from the read_mem_uart_rise output on scope:
      // Confirms that dumpclk = SAMPLECLK / 32 and then because the code toggles the dumpclkreg
      // we divide by two again - so yes, 80 MHz / 32/ 2
      // And then here we divide by 4 now - so yes this gives 312,500 Hz output - which matches physical measurement on scope
      // For 10 bits (8N1) at nominal 160MHz/42/2 => 1,904,762 (for example) baud clock:
      // 10 bits * 1/(baud) and then 1/x = 190,476 Hz for byte packet frequency
      // So even approximately, this is too fast.
      // Thus let us try 8 again for now (until we implement a more precise clock gen)
      // Also we can't really derive this from the dumpclk because that is a gated signal - see above
      // So let's just move now to a more configurably timed signal
      // Previous simply 2^n divider method, gated was:
      //divide_by_n #(.N(8)) divread_mem_uart(dumpclk, 1'b0, read_mem_uart);
      reg clk_read_mem_uart = 0;
      reg [31:0] cntr_read_mem_uart = 32'b0;
      // 160MHz / eg 190,400Hz = 840.3...
      // Divide by two since we are toggling
      // /2 = 420
      // however we have another divide Hz by 2 below
      // So instead of 840 we start with 840/2 = 420
      // SCOPE: Yes, confirmed - though have to be careful about actual organization
      // and synthesis as depending on the actual verilog statements here, synth
      // may lead to suppression of a cycle or even allowing an extra cycle in
      // depending on clock domains etc.
      // However, note: the UART state machine implemented at this time actually
      // requires more clocks cycles to pump out each byte, perhaps 2 or 3
      // Thus we are looking at like: 13 baud clocks or so per byte?
      // TODOFUTURE: We could rework the UART module to perhaps condense somewhat
      // Anyway: at baud 2Mbps, 13 cycles is: 2Mbps/13 = 153,846 (oy)
      // So: 160MHz / 153k = 1046 => /2 = 523
      // We are back to 80MHz so:
      //parameter period_read_mem_uart = 32'd 523; //32'd 420;

      // Much functional testing on this, works:
      parameter period_read_mem_uart = 32'd 262;

      // Could try to trim down the send byte timing for a little more throughput
      //parameter period_read_mem_uart = 32'd 252;

      //always @ (posedge sysclk) begin
      always @ ( posedge trigd_fall, posedge sysclk ) begin
          if ( trigd_fall ) begin
            cntr_read_mem_uart <= 32'b0;
          end else begin
            cntr_read_mem_uart <= cntr_read_mem_uart + 1;
            if (cntr_read_mem_uart == period_read_mem_uart) begin
                // Another divide Hz by 2
                clk_read_mem_uart <= ~clk_read_mem_uart;
                cntr_read_mem_uart <= 32'b0;
            end
          end
      end
      // Now we need to gate this such that read mem uart signal is only active
      // after the captured data is ready
      // and idles correctly (at not 1 for inactive uart send signal)
      // TODO:
      //reg read_mem_uart_reg;
      //assign read_mem_uart = read_mem_uart_reg;
      // Doesn't work: Maybe because it's combinatorial
      //assign read_mem_uart = (clk_read_mem_uart & new_capt);
      // SCOPE: Yes, from output we have about 189kHz output from this
      // NOTES: 191,000 * 10-bits = 1,910,000 minimum baud rate
      /*always @ ( posedge clk160 ) begin
        if ( trigd | !new_capt ) begin
          read_mem_uart_reg <= `UART_SEND_IDLE;
        end
        if ( clk_read_mem_uart & new_capt ) begin
          // Another divide Hz by 2 (multiply period by 2)
          read_mem_uart_reg <= ~read_mem_uart_reg;
        end //else begin
        //  read_mem_uart_reg <= `UART_SEND_IDLE;
        //end
      end*/
      // new_capt means there is complete data ready to read out that has not yet been completely read out
      assign read_mem_uart = (clk_read_mem_uart & new_capt);

      wire read_mem_uart_rise, read_mem_uart_fall;

      edge_detect edet_self_read_mem_uart (
          .async_sig  (read_mem_uart),
          .clk        (sysclk), // Using SAMPLECLK 40 MHz pulse width created, based on scope measurement of output signal
          .rise       (read_mem_uart_rise),
          .fall       (read_mem_uart_fall)
      );
      // Construct a signal for early rd_addr updating
      wire rd_addr_update_rise, rd_addr_update_fall, rd_addr_update_msv;
      monostable #(
        .PULSE_WIDTH(period_read_mem_uart/2), // yes rounding/truncation will happen - but we just a delayed signal
        .COUNTER_WIDTH(10)                    // 10 wires for up to count of 32
      ) msv_rd_addr_update_msv (
        .clk        (sysclk),
        .reset      (read_mem_uart_rise),
        .trigger    (read_mem_uart_fall),
        .pulse      (rd_addr_update_msv)
      );
      edge_detect edet_rd_addr_update (
          .async_sig  (rd_addr_update_msv),
          .clk        (sysclk),
          .rise       (rd_addr_update_rise),
          .fall       (rd_addr_update_fall)
      );
    `endif // ifdef USE_UART
  `endif // ifdef SELFREAD










    // Regarding initial values and their non-use:
    // https://github.com/YosysHQ/yosys/issues/103


  `ifdef USE_DATA_PORT
    assign data_out = val;
  `endif

  `ifdef USE_CAPT_DONE_AS_CAPT_DONE // currently we're not in here for HDL-0108-RSCPT first fw deployed
    assign capt_done = !trigd;
  `endif






  /*

    >======>   >=>     >=> >=>         >=>>=>   >=======>
    >=>    >=> >=>     >=> >=>       >=>    >=> >=>
    >=>    >=> >=>     >=> >=>        >=>       >=>
    >======>   >=>     >=> >=>          >=>     >=====>
    >=>        >=>     >=> >=>             >=>  >=>
    >=>        >=>     >=> >=>       >=>    >=> >=>
    >=>          >====>    >=======>   >=>>=>   >=======>

  */

  //**************************************************************************
  //
  /* PULSE CONTROL - SINGLE/STANDARD
  //
  /**************************************************************************/

    wire pulse_on, delay_to_rtz, pulse_rtz;

    wire delay_to_capture;        // Rx (receive) signal capture delay

    assign PULSE_NEG = pulse_on;
    assign RTZ_NEG = pulse_rtz;
    assign RTZ_POS = pulse_rtz;

    wire delay_to_rtz_rise, delay_to_rtz_fall;
    wire pulse_ctrl_clk;

    `ifndef USE_UART
      assign pulse_ctrl_clk = SAMPLECLK;
    `else
      assign pulse_ctrl_clk = sysclk;
    `endif



    `ifndef DEMO_VAR_FREQ_PULSE

      // We can definitely parameterize the VAR_FREQ / swept frequency - see DEMO_VAR_FREQ_PULSE defined case

      //
      // First pulse


      //**********************
      //  PULSE
      //
      // Yes scope-verified that with of 12 gives a pulse width that measures as 7MHz
      // 24 width => -140VDC on 3.5MHz Comp under test
      // Slope limit of the voltage with time likely due to damping in system and ohmage
      // versus with 12/12 pulse width and rtz delay time, voltage to -100VDC or so in neg pulse
      // and
      parameter PULSE_ON_WIDTH = 12;
      monostable #(
        .PULSE_WIDTH(PULSE_ON_WIDTH),           // 80MHz / 7MHz (for 2x3.5MHz for half-cycle on hold)
        .COUNTER_WIDTH(5)           // 5 wires for up to count of 32
                                    // Measured (ctrl sig?) pulse width on scope is about 152 ns
      ) msv_pulse_on (
        .clk        (pulse_ctrl_clk),
        .reset      (trig_in_rise), // was trig_in_rise
        .trigger    (trig_in_fall), // was trig_in_rise
        .pulse      (pulse_on)
      );


      //**********************
      // Delay to RTZ
      //
      // Width of 1: Max neg voltage is less for the 3.5MHz XD under test
      // Width of 3: Slightly more negative voltage
      // Width of 5: Yes slight more negative voltage peak further
      // Width of 8: Yes more neg even TxDAC:0x0f on 3.5MHz Comp:
      // Width of 12: Yes, even more so, on target
      parameter DELAY_TO_RTZ_WIDTH = 4;
      monostable #(
        `ifndef DEMO_VAR_FREQ_PULSE
          // was 12 typically
          // Yes, pulses get narrower with decreasing PW
          // Scope shows slightly lesser voltage but much sharper neg peak (so does WF returned)
          // At PW = 5 - single spike vs multiple spikes -
          // At PW = 4 - seems even sharper S/N
          // Indeed lots of impact on receive WF characs with tuned pulse ctrl timing
          // Various values here cause timing issues and things fall apart
          // However: 12 seems slightly better for inductor-shorted-tuned transducer in terms of echo definition
          // However: 4 seems slightly better for non-inductor tuned, just crystal tuned transducers (sharp echo spikes)
          //   Including both composite test transducers here
          // Not yet sure about continuous pulsing (eg current consumption over long term)
          .PULSE_WIDTH( PULSE_ON_WIDTH + DELAY_TO_RTZ_WIDTH ),         // 80MHz / 7MHz (for 2x3.5MHz for half-cycle on hold)
        `endif
        `ifdef DEMO_VAR_FREQ_PULSE
          .PULSE_WIDTH(11),
        `endif
        .COUNTER_WIDTH(5)           // 5 wires for up to count of 32
      ) msv_delay_to_rtz (
        .clk        (pulse_ctrl_clk),
        .reset      (trig_in_rise),
        .trigger    (trig_in_fall),
        .pulse      (delay_to_rtz)
      );

      edge_detect edet_delay_to_rtz (
        .async_sig  (delay_to_rtz),
        .clk        (pulse_ctrl_clk),
        .rise       (delay_to_rtz_rise),
        .fall       (delay_to_rtz_fall)
      );


      //**********************
      // RTZ
      //
      // Indeed on scope, return to zero starts exactly with the application of this pulse
      monostable #(
        `ifndef DEMO_VAR_FREQ_PULSE
          // was 64
          // using up to 240 makes not much difference - except in combination
          // with changing counter width eg to 10 we get timing errors that need to get
          // sorted out - including loss of RTZ altogether
          // Interestingly, NOTE: if RTZ fails or no RTZ or very minimal (like1? maybe zero)
          // the amp saturates and the high frequency ringing early in the return signal after the pulse
          // goes away - so things "look" cleaner on the return before the first front face
          // longer RTZs don't really help all that much -- there are just trade-offs
          // Probably better to simply tune the pulse duration and the delay to RTZ
          // Could sneak away with no RTZ in some situations probably
          .PULSE_WIDTH(64),         // Long (about 4x = 2 cycles) for the RTZ
        `endif
        `ifdef DEMO_VAR_FREQ_PULSE
          .PULSE_WIDTH(10),
        `endif
          // was 7 - 10 created some errors sometimes depending on the pulse width
          // More TODO regarding counter synth and implementation in real h/w
          .COUNTER_WIDTH(7)         // 7 = Max of 128 - if using the 1usec delay before SSR blanking - we have plenty of time
      ) msv_pulse_rtz (
          .clk      (pulse_ctrl_clk),
          .reset    (trig_in_rise),
          .trigger  (delay_to_rtz_fall),  // same as above msv should change if we don't want pre-pulse activation
          .pulse    (pulse_rtz)
      );

      // End of Standard Single Pulse
      //

    `endif // end of NOT using DEMO_VAR_FREQ_PULSE







    // End Pulse Ctrl
    //
    //














  `ifndef SELFREAD
    `ifdef USE_READ_MEM_IN_AS_READ_MEM_IN
      wire read_mem_rise, read_mem_fall;
      assign read_mem = read_mem_in;
      edge_detect edet_read_mem (
          .async_sig  (read_mem_in),
          .clk        (SAMPLECLK),
          .rise       (read_mem_rise),
          .fall       (read_mem_fall)
      );
    `endif
  `endif


  `ifndef USE_UART
    //`ifdef SELFREAD | SYNC_SELFREAD_WITH_CLOCKED_OUT
    // Create this signal regardless, not getting || macro ifdefs to work yet ...
    // At 1.25MHz output write strobe rate, half a cycle is 400ns clock pulse width
    // For an 80MHz clock (12.5ns full cycle), that is:
    // 400ns / 12.5 ns = 32 clocks
    // So yeah, to delay by 32 clocks or so would put us maybe right in the middle of
    // of the strobe
    // Use: 32 NOT SELFREAD ? (using nano)
    // Or 40MHz (data grab rate, 8-bits) / 1.25MHz (data comms out rate, 8-bits)
    // 40/1.25 = 32 ... 1/2 => 16
    // Use: 16 SELFREAD - good enough - still messy at proto wire stages ...
    // ok
    // Current HDL-0108-RSCPT design, max comms throughput in theory is 12MHz (12Mbps)
    // 1.25MHz x 8-bits => 10MHz => 10Mbps throughput
    wire read_mem_shifted;
    shift #(
        .DEPTH(16)
      ) read_mem_shifted_shift (
        .clk        (SAMPLECLK),
        .data_in    (read_mem),
        .data_out   (read_mem_shifted)
    );
    //`endif
  `endif // ifndef USE_UART


    `ifndef SELFREAD
      `ifdef SYNC_SELFREAD_WITH_CLOCKED_OUT
        assign fifo_wri = !read_mem_shifted; // inverted logic
      `endif
    `endif




    `ifdef SELFREAD
      `ifndef USE_UART
        assign read_mem = dumpclk; // & new_capt;
        assign fifo_wri = read_mem_shifted;
        //assign fifo_wri = read_mem;
        //assign read_mem_rise = !trigd & dumpclk & new_capt;
        wire read_mem_rise, read_mem_fall;
        edge_detect edet_self_read_mem (
            .async_sig  (read_mem),
            .clk        (SAMPLECLK),
            .rise       (read_mem_rise),
            .fall       (read_mem_fall)
        );
      `endif // ifndef USE_UART
    `endif








    /*

    >======>     >=======>       >>       >====>                 >>       >====>     >====>     >=>      >=>
    >=>    >=>   >=>            >>=>      >=>   >=>             >>=>      >=>   >=>  >=>   >=>   >=>    >=>
    >=>    >=>   >=>           >> >=>     >=>    >=>           >> >=>     >=>    >=> >=>    >=>   >=> >=>
    >> >==>      >=====>      >=>  >=>    >=>    >=>          >=>  >=>    >=>    >=> >=>    >=>     >=>
    >=>  >=>     >=>         >=====>>=>   >=>    >=>         >=====>>=>   >=>    >=> >=>    >=>     >=>
    >=>    >=>   >=>        >=>      >=>  >=>   >=>         >=>      >=>  >=>   >=>  >=>   >=>      >=>
    >=>      >=> >=======> >=>        >=> >====>           >=>        >=> >====>     >====>         >=>

    */
    //*************************************************************************
    //
    // READ ADDRESS INCREMENT / MGMT
    //
    //*************************************************************************


    `ifndef USE_UART
      always @( posedge SAMPLECLK )
      begin
        if ( trig_in_rise ) begin
          addr_rd_nxt <= 12'b 0000_0000_0000;
        end
        if ( read_mem_rise ) begin
          addr_rd_nxt <= (addr_rd + 1'b 1);
        end
      end
    `endif // ifndef USE_UART

    `ifdef USE_UART
      always @( posedge sysclk )   // Was SAMPLECLK
      begin
        if ( trig_in_rise ) begin     // Was trig_in_rise
          addr_rd_nxt <= 12'b 0000_0000_0000;
        end else if ( read_mem_uart_rise ) begin
          addr_rd_nxt <= (addr_rd + 1'b 1);
        end else begin
          addr_rd_nxt <= addr_rd_nxt; // new
        end
      end
    `endif // ifdef USE_UART








    /*

    >=>        >=> >======>     >=> >===>>=====> >=======>             >>       >====>     >====>     >=>      >=>
    >=>        >=> >=>    >=>   >=>      >=>     >=>                  >>=>      >=>   >=>  >=>   >=>   >=>    >=>
    >=>   >>   >=> >=>    >=>   >=>      >=>     >=>                 >> >=>     >=>    >=> >=>    >=>   >=> >=>
    >=>  >=>   >=> >> >==>      >=>      >=>     >=====>            >=>  >=>    >=>    >=> >=>    >=>     >=>
    >=> >> >=> >=> >=>  >=>     >=>      >=>     >=>               >=====>>=>   >=>    >=> >=>    >=>     >=>
    >> >>    >===> >=>    >=>   >=>      >=>     >=>              >=>      >=>  >=>   >=>  >=>   >=>      >=>
    >=>        >=> >=>      >=> >=>      >=>     >=======>       >=>        >=> >====>     >====>         >=>

    */
    //*************************************************************************
    //
    // WRITE ADDRESS INCREMENT
    //
    //*************************************************************************

    `ifndef USE_UART
      always @( posedge SAMPLECLK )
      begin
        if ( trig_in_rise ) begin
          addr_wr_nxt <= 12'b 0000_0000_0000;
          `ifdef DATA_TEST_INCREMENT_VALUES
            test_val_next <= 12'b 0000_0000_0000;
          `endif // ifdef DATA_TEST_INCREMENT_VALUES
        end else if ( trigd ) begin
          addr_wr_nxt <= (addr_wr + 1'b 1);
          `ifdef DATA_TEST_INCREMENT_VALUES
            test_val_next <= (test_val + 1'b 1);
          `endif // ifdef DATA_TEST_INCREMENT_VALUES
        end
      end
    `endif // ifndef USE_UART

    `ifdef USE_UART

      wire write_addr_incr_clk;
      assign write_addr_incr_clk = ( trigd & adc_encode );

      // Please IEEE 1364.1 5.2.2.x for correct formulation
      // Yosys rightly doesn't support dual async edge-triggered units
      // and with other variants of below you should see build errors
      // https://github.com/YosysHQ/yosys/issues/662
      always @ ( posedge trig_in_rise or posedge write_addr_incr_clk)
      begin
        if ( trig_in_rise ) begin
          addr_wr_nxt <= 12'b 0000_0000_0000;
        end else begin
          addr_wr_nxt <= addr_wr_nxt + 1; //(addr_wr + 1); //1'b 1);
        end
      end

    `endif // ifndef USE_UART






    /*

    >=>        >=> >======>     >=> >===>>=====> >=======>
    >=>        >=> >=>    >=>   >=>      >=>     >=>
    >=>   >>   >=> >=>    >=>   >=>      >=>     >=>
    >=>  >=>   >=> >> >==>      >=>      >=>     >=====>
    >=> >> >=> >=> >=>  >=>     >=>      >=>     >=>
    >> >>    >===> >=>    >=>   >=>      >=>     >=>
    >=>        >=> >=>      >=> >=>      >=>     >=======>

    */
    /***************************************************************************
    //
    /* WRITE TO MEMORY
    //
    /***************************************************************************/


    `ifndef USE_UART

      // WARNING this have been massively changed from before UART implementation!!!

      // This will give a half delay - yes, SIM shows working
      wire delayed_capture_clk;
      // Delay until data good at the converter
      assign delayed_capture_clk = ( ( sclk && ~SAMPLECLK ) ); // && ~clk160 );

      always @( posedge delayed_capture_clk )
      begin

        if ( trigd ) begin
          addr_wr <= addr_wr_nxt;
          mem[addr_wr] <= data_in;
        end

      end
    `endif // ifndef USE_UART


    `ifdef USE_UART

      // This will give a half delay - yes confirmed-ish in SIM
      //(* keep *) wire delayed_capture_clk; // yes this works for POST sim so far
      // interestingly it also effects the PDO output pin presence of this signal too!

      wire delayed_capture_clk; // yes this works for POST sim so far
      // Delay until data good at the converter - yes-ish
      assign delayed_capture_clk = ( ~sclk & sysclk );

      wire write_enable;

      // This create a pulse that is a full 160MHz long (pulse width is logic high length of 80MHz signal)
      //assign write_enable = ( trigd & delayed_capture_clk );

      // This creates a pulse width equal to one half cycle of 40MHz
      assign write_enable = ( trigd & !adc_encode);

      // At the moment, it seems to like this capture delay timing
      reg [7:0] data_in_tmp;
      always @ ( posedge write_enable ) begin
        data_in_tmp <= data_in;
        //data_in_tmp <= data_in_tmp + 1; // Testing sequence count output
      end

      // This is too fast, essentially one full 160MHz or an 80MHz pulse half-cycle width
      // (ie 6.25ns) cycle after the input capture - we get data glitches
      //always @ (negedge write_enable ) begin // was negedge
      always @ ( posedge write_addr_incr_clk ) begin
        `ifndef TEST_NO_BRAM
          mem[addr_wr] <= data_in_tmp;
        `endif // ifndef TEST_NO_BRAM
      end


      // was pose pose
      always @ ( posedge trig_in_rise, negedge write_addr_incr_clk) begin
        if ( trig_in_rise ) begin
          addr_wr <= 0;
        end else begin
          addr_wr <= addr_wr_nxt;
        end
      end

    `endif // ifndef USE_UART






    /*

      >=>>=>       >===>      >=======>
    >=>    >=>   >=>    >=>   >=>
     >=>       >=>        >=> >=>
       >=>     >=>        >=> >=====>
          >=>  >=>        >=> >=>
    >=>    >=>   >=>     >=>  >=>
      >=>>=>       >===>      >=>

    */
    /***************************************************************************
    //
    /* START OF FRAME (Setup/Definition)
    //
    /***************************************************************************/

    // SOF - Start of Frame - Const section construction - always stays the same
    // seq_id Sequence ID ends up being base 1 - that is, first waveform back starts at
    // the number 1 (not zero) - and wraps at 255 - so really only a count range of 255 items before wrap to 1
    // TODO verify behavior after the RST signal, in terms of first WF seq_id
    // Until you update it ...
    // TODO - module?
    // And confirm
    `ifndef TEST_NO_BRAM
      //(* keep *) reg [7:0] sof [0:11];               // 12-byte start of frame
      reg [7:0] sof [0:11];
      // For individual clock cycle timing verifications or auto sync, on the
      // serial comms output end, you may wish to use values like:
      // 0x55 or 0xaa for some sync or SOF bytes
      // However, 0x00 and 0xff so close in time quite definitely exceed the intended
      // allowable signal content and thus are easy to start with for SOF
      // delimiters.
      // aa = 170
      // 55 = 85
      // If not using BRAM, like -nobram for SIM then these are not RAMs
      // and thus init doesn't seem to work - at least array is pruned
      // Though for h/w synth yosys supports putting this into RAM using initial block here

      // Works for real h/w (using BRAM), not for SIM - haven't tried init'ing the
      // missing sof[x]'s though for SIM functionality (filled up auto to zero in BRAM real h/w synth though )
      /*
      initial begin
        sof[0] <= 8'h aa;
        sof[1] <= 8'h 55;
        sof[2] <= 8'h aa;
        sof[3] <= 8'h 55;

        sof[8] <= 8'h ff;
        sof[9] <= 8'h 00;
        sof[10] <= 8'h ff;
        sof[11] <= 8'h 00;
      end
      */

      // Added to try keeping SIM close to h/w synth
      // Yes, this works for SIM - as long as added the missing sof's
      // Also works in real h/w synth - haven't checked yet if it gets replaced
      // by BRAM in real h/w synth yet though - could be
      always @ ( posedge trig_in_rise ) begin
        sof[0] <= 8'h aa;
        sof[1] <= 8'h 55;
        sof[2] <= 8'h aa;
        sof[3] <= 8'h 55;
        sof[4] <= 8'h 00;
        sof[5] <= 8'h 00;
        sof[6] <= 8'h 00;
        sof[7] <= 8'h 00;
        sof[8] <= 8'h ff;
        sof[9] <= 8'h 00;
        sof[10] <= 8'h ff;
        sof[11] <= 8'h 00;
      end
    `endif // TEST_NO_BRAM








    /*

        >===>      >=>     >=> >===>>=====> >======>   >=>     >=> >===>>=====>
      >=>    >=>   >=>     >=>      >=>     >=>    >=> >=>     >=>      >=>
    >=>        >=> >=>     >=>      >=>     >=>    >=> >=>     >=>      >=>
    >=>        >=> >=>     >=>      >=>     >======>   >=>     >=>      >=>
    >=>        >=> >=>     >=>      >=>     >=>        >=>     >=>      >=>
      >=>     >=>  >=>     >=>      >=>     >=>        >=>     >=>      >=>
        >===>        >====>         >=>     >=>          >====>         >=>

    */
    /***************************************************************************
    //
    /* OUTPUT PACKET CONSTRUCTION AND READ MEMORY TO OUTPUT PORT
    //
    /***************************************************************************/

    `ifdef USE_UART
      reg [7:0] uart_txbyte = 0; // To show uncertainty in SIM, remove the init; inits don't work in real h/w
    `endif // ifdef USE_UART


    `ifndef USE_UART
      always @ ( posedge SAMPLECLK ) begin
        if ( read_mem ) begin
          if ( addr_rd == 4 ) begin
            addr_rd <= addr_rd_nxt;
            seq_id <= seq_id_nxt;
            val <= seq_id;
          end else if ( addr_rd < 12 ) begin
            addr_rd <= addr_rd_nxt;
            val <= sof[addr_rd];
          end else begin
            addr_rd <= addr_rd_nxt;
            val <= mem[addr_rd];          // To test correct addr_rd handling: <= addr_rd[7:0] etc.
          end
        end
      end
    `endif // ifndef USE_UART



      // TODO we have not yet extract/compartmentalized the USE_UART
      // version that is currently not cased below:

      `ifdef SEQ_ID_IS_BASE_1

        // Actually as of this writing, the count with MCU timing goes:
        // 1, 2, 3, 0 ... so this is not all quite functional either
        always @ ( posedge reset, posedge trig_in_rise ) begin

          if ( reset ) begin
            seq_id <= 8'h 00;
          end else begin
            // This will always thus start the seq ID number at 1 in the SOF
            seq_id <= seq_id + 1;
          end

        end

      `endif

      `ifdef SEQ_ID_IS_BASE_0

        // RS8 (HDL-0108-RSCPT) uses a base-0 channel/seq number
        // So to match that convention, we increment at the end of the capture
        reg pending_seq_id_reset = 1'b 0;

        always @ ( posedge reset, posedge trig_in_rise ) begin
            // And use trig_in_rise because the reset actually occurs
            // after the last channel WF in a channel series has been
            // initiated
            if ( reset ) begin
              pending_seq_id_reset <= 1'b 1;
            end else begin
              pending_seq_id_reset <= 1'b 0;
            end
        end

        always @ ( posedge new_capt_fall ) begin
          // By now, the seq_id for this WF was already sent
          if ( pending_seq_id_reset ) begin
            seq_id <= 1'b 0;
          end else begin
            seq_id <= seq_id + 1'b 1;
          end
        end

      `endif // ifdef SEQ_ID_IS_BASE_1




      always @ ( posedge trig_in_rise, posedge rd_addr_update_fall ) begin

        if ( trig_in_rise ) begin
          addr_rd <= 12'b 0;
        end else begin
          addr_rd <= addr_rd_nxt;
        end

      end



      // clk160 condition for reading memory seemed to fail and provide blank output
      // using a block read at the slow clock of the read_mem_uart seems to capture
      // correctly the memory address such that the value is readily available for a
      // non-blocking read in the main output section
      // Was blocking (=) ... changed to non-blocking
      reg [7:0] next_byte;
      always @ ( posedge read_mem_uart ) begin
        //next_byte <= next_byte + 1; // works  // mem[addr_rd];
        //next_byte <= addr_rd[7:0]; // works
        next_byte <= mem[addr_rd];
      end



      //always @ ( posedge sysclk ) begin
      always @ ( posedge read_mem_uart_fall ) begin

        //if ( read_mem_uart_fall ) begin

          if ( addr_rd == 4 ) begin
            uart_txbyte <= seq_id;
          end else if ( addr_rd < 12 ) begin

            `ifndef TEST_NO_BRAM
              uart_txbyte <= sof[addr_rd];
            `else // ifndef TEST_NO_BRAM
              uart_txbyte <= 8'h aa;
            `endif

          end else begin

            `ifndef TEST_NO_BRAM
              uart_txbyte <= next_byte;
            `else
              uart_txbyte <= addr_rd[7:0];
            `endif //ifndef TEST_NO_BRAM

          end // addr_rd case ifs

        //end // read_mem_uart_fall

      end // always

    //`endif // ifndef USE_UART



    // Back to the LED flasher test code, left-overs
    // drive USB pull-up resistor to '0' to disable USB
    //assign USBPU = 0;
    // now not using this, to save a public pin
    //assign LED = trig_in;








    /*

      >=>     >=>       >>       >======>     >===>>=====>
      >=>     >=>      >>=>      >=>    >=>        >=>
      >=>     >=>     >> >=>     >=>    >=>        >=>
      >=>     >=>    >=>  >=>    >> >==>           >=>
      >=>     >=>   >=====>>=>   >=>  >=>          >=>
      >=>     >=>  >=>      >=>  >=>    >=>        >=>
        >====>    >=>        >=> >=>      >=>      >=>

    */
    /***************************************************************************
    //
    /* UART IMPLEMENTATION
    //
    /***************************************************************************/

    //
    //
    // UART Implementation - taken from:
    // See module section for sources and examples
    //
    // Testing option / ifdefs included below:
    // Every 1 sec, send a next char over uart
    //

    //`ifdef USE_LATTICE_BREAKOUT_DEMO
    `ifdef USE_UART

      `ifdef UART_1SEC_OUTPUT_TEST
        parameter ASCII_0 = 8'd48; //8'd0; //8'd48;
        parameter ASCII_9 = 8'd57; //8'd255; //8'd57;
      `endif // ifdef UART_1SEC_OUTPUT_TEST

      // UART registers - defined earlier now
      //reg [7:0] uart_txbyte = 0; // = ASCII_0; // init to a value doesn't work on this h/w


      reg uart_send_reg;
      wire uart_send;
      wire uart_txed;



      `ifdef USE_BAUD_115200
        reg clk_baud_115200 = 0;
        reg [31:0] cntr_115200 = 32'b0;
        // 160MHz / 115200 = 1388.8888888
        // Divide by two since we are toggling
        // /2 = 694.44444 => 694 => % error =
        // 160M / 694 / 2 = 115,273.8 - 115200 / 115200 => 73.8 / 115200 => 0.06%
        parameter period_115200 = 694;

        always @ (posedge clk160) begin
            cntr_115200 <= cntr_115200 + 1;
            if (cntr_115200 == period_115200) begin
                clk_baud_115200 <= ~clk_baud_115200;
                cntr_115200 <= 32'b0;
            end
        end
      `endif // ifdef USE_BAUD_115200

      // Standard baud rates:
      // 7.3728 MHz / 2^n
      // Max for currently used dev/test RS-485 to USB/Serial:
      // 7.3728 / 8 = 921.6kHz Baud rate (called 921k)
      `ifdef USE_UART_BAUD_921600
        reg clk_baud_921600 = 0;
        reg [31:0] cntr_921600 = 32'b0;
        parameter period_921600 = 87; // 160MHz/921600 = 173.6111 / 2 = 86.8

        always @ ( posedge clk160 ) begin
          cntr_921600 <= cntr_921600 + 1;
          if ( cntr_921600 == period_921600 ) begin
              clk_baud_921600 <= ~clk_baud_921600;
              cntr_921600 <= 32'b0;
          end
        end
      `endif // ifdef USE_UART_BAUD_921600


      // Non-standard baud rates:
      // eg 10Mbps for FTDI if using baud rate aliasing
      // However, never got this to work (yet?) on Mac OS X
      // And, haven't yet tested with the agnostic D2xx type test code
      // However, using the standard baud rate multiples seems to work,
      // even including 2x 921600 = 1,843,200 entered as a custom value for port rate
      // in CoolTerm
      // And these results are for testing the PLL as on the UP5K Lattice Dev Kit
      // Thus, apparently the real clock rate is 159 MHz
      // Also, with the current UART implementation, the speed seems to max functionally
      // at about 3.57 MHz -- and then even higher than using a divisor of 16
      // or sometimes
      reg clk_baud_fastest = 0;
      //reg [31:0] cntr_fastest = 32'b0;
      //parameter period_fastest = 40;  // for 160MHz clk in  but that's too fast for the larger counters
      reg [7:0] cntr_fastest = 8'b 0;
      parameter period_fastest = 20;  // 2Mbps ish
      // 44: 1,818,181.8...
      // 43: Toggle at 160MHz / 43 => divide once more by two for the rate since
      //     this is a toggle (output test ok)
      //     Gives: 1,860,456.12
      // 42: Gives: 1,904,762 Output blanks out (nothing) at this time - OK using a shortened uart_send MSV pulse seems to help (output test ok with update)
      // 41: Seems to provide output, at (1,951,219) if power cycle and every other request again - might be better now with OK above
      // 40: 2MBps
      // Remember baud rate must work for 10-bits and the read_mem_uart_rise ferquency (interval between bytes transmitted basically)

      //always @ ( posedge sysclk ) begin
      always @ ( posedge trigd_fall, posedge sysclk ) begin

        if ( trigd_fall ) begin
          cntr_fastest <= 8'b0;
        end else begin
          cntr_fastest <= cntr_fastest + 1;         // these were cntr_10M
          if ( cntr_fastest == period_fastest ) begin  // was cntr_10M == period_10M
              clk_baud_fastest <= ~clk_baud_fastest;  // clk_baud_10M
              cntr_fastest <= 8'b0;              // was cntr_10M
          end
        end

      end


      wire clk_baud;
      assign clk_baud = clk_baud_fastest;


      `ifdef UART_1SEC_OUTPUT_TEST
        // 1-sec clock generation setup
        // Simpler earlier using: just the same clk160 block and a period of
        // e.g. 80,000,000 -- however kept running into issues --
        // 8,000,000 was functional -- generally glitchy though through ranges
        // stopped debugging and just reworked as a clk160 divided to clk5
        // and then using smaller numbers for period ()
        reg clk_1 = 1'b0;
        //reg [31:0] cntr_1 = 32'b0;
        // Divide 160MHz by 2 since we are toggling
        //parameter period_1 = 80000000; // For 1 Hz clock: 80,000,000 -- not yet clean // 8,000,000 clean so far is 10 Hz
        //reg [31:0] period_1 = 32'd20000000;
        wire clk5;
        divide_by_n #(.N(32)) div_clk5(clk160, 1'b0, clk5);
        reg [31:0] period_1 = 32'd5000000;
        reg [31:0] cntr_1 = 32'd5000000; //period_1; //32'b0;

        wire clk_uart_trigger_output;
        assign clk_uart_trigger_output = clk_1; //clk_baud_115200; //clk_1;
      `endif

      // ifdef USE_UART_BAUD_921600
      //parameter period_tx_send_pulse = 4 * period_921600;

      // ifdef USE_UART_BAUD_10000000 (10 MHz)
      //parameter period_tx_send_pulse = 4 * period_10M;
      // TODO:
      // If using 4 * period_faster -- we get no output for dumping samples
      // over UART at the first target rate
      // Using period_fastest we do
      // So maybe the signal never changes or is too long - so we need to work with this
      // ALSO TODO:
      // We get about 0x0420 samples (or byte, if real or not) out -- so need to debug that too
      // might just be duration of sapmle dump output versus actual count?
      // Or timing ...

      // There may be clock async for example between uart_send init timing
      // triggered from the read_mem_uart_rise/fall and the baud clock
      // so we should stretch this out in time a touch
      // even with 2x we can just slightly miss the rising edge of the send pulse
      parameter period_tx_send_pulse_mult = 4; // was 3
      parameter period_tx_send_pulse = period_tx_send_pulse_mult*period_fastest;


      `ifdef UART_1SEC_OUTPUT_TEST

        always @ (posedge clk5) begin
            cntr_1 <= cntr_1 - 1;
            if (cntr_1 == 32'b0) begin
                clk_1 <= ~clk_1;
                cntr_1 <= period_1;
            end
        end

        monostable #(
          .PULSE_WIDTH(period_tx_send_pulse),
          .COUNTER_WIDTH(32)
        ) msv_uart_send_pulse (
          .clk          (clk160),
          .reset        (RST),
          .trigger      (clk_uart_trigger_output),
          .pulse        (uart_send)
        );

      `endif // ifdef UART_1SEC_OUTPUT_TEST


      `ifndef UART_1SEC_OUTPUT_TEST
        //wire uart_msv_send_trigger;
        //buf(uart_msv_send_trigger, read_mem_uart_fall); // was rise
        monostable #(
          .PULSE_WIDTH(period_tx_send_pulse),
          .COUNTER_WIDTH(7) // Functional use 32! Even though only 8 bits are needed, in synth uart send break with 8 bits (!)
          // interesting ... values that work:
          // 32, 12, 10, 7, 6
          // 9 => abc error
          // 8 => no response to mcu paq command
          // Yes PW matches expected 3*pd = 60 on SIM WF
          // 60 = 3C = 6 bits minimum required
          // 120 = 7 bits minimum required
        ) msv_uart_send_pulse (
          .clk          (sysclk),
          .reset        (trig_in_rise),
          .trigger      (read_mem_uart_fall), //read_mem_uart_fall? read_mem_uart?
          .pulse        (uart_send)
        );
      `endif // ifndef UART_1SEC_OUTPUT_TEST

      // From test bench:
      // Test clock toggles every 10
      // So 160 MHz = Count of 20
      // read_mem_uart_fall:
      // 356380 - 335420 = 20960 / 20 => 1048
      // 160 MHz / 1048 = 153kHz
      // clk_baud:
      // 346960 - 345320 = 1640 / 20 => 82
      // 160MHz / 82 => 1.95MHz
      // So baud is correctish
      // Read address is changing:
      // 387840 - 345920 = 41920 / 20 => 2096
      // 160 MHz / 2096 = 76.3kHz
      // So - that seems a little slow to get the new read address *** that's the issue

      // From SIM test bench using now the 80MHz sysclk
      // 46140 - 25100 from consecutive uart send pulse rising edges
      // 21040
      // /20 => @160MHz for PLL main TB clk => 1052
      // 1052 160MHz clk cycles => 152091.2548 Hz
      // 1/152091.2548 Hz = 6.575 usec window for each Byte to send out
      // 4096 Bytes => 26.9312 ms

      // For testing - this just continually dumps data even if the value is the same
      //reg always_uart_send = 1'b1;

      // Yes - it likey's the latched value (the uart transmitter does it seems)
      always @ ( posedge sysclk ) begin
          uart_send_reg <= uart_send;
      end

      // TODO yes, when there is no data coming, it is this signal that
      // is not firing (testing on output pin)
      // When it does fire it is about 3.7MHz (3.68MHz)
      // And it is idling at 0
      // OK and yes sometimes it does fire and there is still no uart output

      uart_tx_8n1 transmitter (

          // module input: baud rate clock (init test @ 115200)
          .clk (clk_baud),

          // module input: byte to be transmitted
          .txbyte (uart_txbyte),

          // module input: flag to enable sending data
          //.senddata (uart_send),
          .senddata (uart_send_reg),

          // Custom added OB - needed?
          .reset (trig_in_rise), // ( reset ),

          // output: tx is finished
          .txdone (uart_txed),

          // output: UART tx pin
          .tx (UART_TX)
      );





      `ifndef FLASH_LEDS // WARN: Also needs maybe some portion of the UART_1SEC_OUTPUT_TEST ifdefs
        //assign RGB0 = clk_1; //~uart_send;
        //assign RGB1 = 1'b1;
        //assign RGB2 = 1'b1;
      `endif

      `ifdef UART_1SEC_OUTPUT_TEST
        always @ ( posedge clk_uart_trigger_output )
        begin

            if (uart_txbyte == ASCII_9) begin
                uart_txbyte <= ASCII_0;
            end else begin
                uart_txbyte <= uart_txbyte + 1;
            end

        end
      `endif // ifdef UART_1SEC_OUTPUT_TEST


    `endif    // ifdef USE_UART
    //`endif  // ifdef USE_LATTICE_BREAKOUT_DEMO








        /*

            >===>      >=>     >=> >===>>=====>       >===>>=====> >=======>   >=>>=>   >===>>=====>
          >=>    >=>   >=>     >=>      >=>                >=>     >=>       >=>    >=>      >=>
        >=>        >=> >=>     >=>      >=>                >=>     >=>        >=>            >=>
        >=>        >=> >=>     >=>      >=>                >=>     >=====>      >=>          >=>
        >=>        >=> >=>     >=>      >=>                >=>     >=>             >=>       >=>
          >=>     >=>  >=>     >=>      >=>                >=>     >=>       >=>    >=>      >=>
            >===>        >====>         >=>                >=>     >=======>   >=>>=>        >=>

        */

        `ifdef TEST_TIMING_TO_PIN_OUTPUT
          buf(PDO0, adc_encode);
          buf(PDO1, trigd);
          buf(PDO2, write_enable); //trig_in_rise);
          buf(PDO3, uart_send);
        `endif







endmodule // top


















/*

  >=>       >=>     >===>      >====>     >=>     >=> >=>       >=======>   >=>>=>
  >> >=>   >>=>   >=>    >=>   >=>   >=>  >=>     >=> >=>       >=>       >=>    >=>
  >=> >=> > >=> >=>        >=> >=>    >=> >=>     >=> >=>       >=>        >=>
  >=>  >=>  >=> >=>        >=> >=>    >=> >=>     >=> >=>       >=====>      >=>
  >=>   >>  >=> >=>        >=> >=>    >=> >=>     >=> >=>       >=>             >=>
  >=>       >=>   >=>     >=>  >=>   >=>  >=>     >=> >=>       >=>       >=>    >=>
  >=>       >=>     >===>      >====>       >====>    >=======> >=======>   >=>>=>

*/

// *****************************************************************************
//
// MODULES
//      MODULES
//          MODULES
//              MODULES
//                    MODULES
//
// *****************************************************************************

























// *****************************************************************************
// PLL
// $ cd /Users/myself/.apio/packages/toolchain-icestorm/bin
// $ ./icepll -i 16 -o 80
// 16MHz in and 40MHz out:
// See also: https://github.com/YosysHQ/arachne-pnr/issues/64
// for CLK source pins and needed outputs
// https://github.com/mystorm-org/BlackIce-II/wiki/PLLs-Improved
// https://github.com/mystorm-org/BlackIce-II/wiki/PLLs-Advanced
//
// 160 MHz out below:
module pllcore(input clk, output clkout, output lock); // had also output clkin (?)
	SB_PLL40_CORE #(
		.FEEDBACK_PATH("SIMPLE"),
		.PLLOUT_SELECT("GENCLK"),
		.DIVR(4'b0000),
		.DIVF(7'b0100111),
		.DIVQ(3'b010),        // 40MHz: 3'b100 -- vs -- 80MHz: 3'b011 -- vs -- 160MHz: 3'b010
		.FILTER_RANGE(3'b001)
	) uut (
		.LOCK(lock),
		.RESETB(1'b1), // was 1'b 1
		.BYPASS(1'b0),
		.REFERENCECLK(clk),
		.PLLOUTCORE(clkout)
	);
	//assign clkin = clk;
endmodule
// 80 MHz out below:
/*module pllcore(input clk, output clkin, clkout, lock);
	SB_PLL40_CORE #(
		.FEEDBACK_PATH("SIMPLE"),
		.PLLOUT_SELECT("GENCLK"),
		.DIVR(4'b0000),
		.DIVF(7'b0100111),
		.DIVQ(3'b011),        // 40MHz: 3'b100 -- vs -- 80MHz: 3'b011
		.FILTER_RANGE(3'b001)
	) uut (
		.LOCK(lock),
		.RESETB(1'b1),
		.BYPASS(1'b0),
		.REFERENCECLK(clk),
		.PLLOUTCORE(clkout)
	);
	assign clkin = clk;
endmodule
*/
//
// cd ~/.apio/packages/toolchain-icestorm/bin
// ./icepll -i 12 -o 160
// For 12MHz in: USE_LATTICE_BREAKOUT_DEMO
// 160MHz out: 159MHz real
// 4'b0000
// 7'b0110100
// 3'b010
// F/R: 3'b001
//
// 16MHz in: HDL-0108-RSCPT series
// 4'b0000
// 7'b0100111
// 3'b010
// F/R: 3'b001
//
`ifdef PLLSOURCE_BOTTOM_BANK // for Lattice breakout
  module pll2pad(input clk_in, output clkout_a, clkout_b, lock);
  	SB_PLL40_2_PAD #(
  		.FEEDBACK_PATH("SIMPLE"),
  		.DIVR(4'b0000),
  		.DIVF(7'b0110100),
  		.DIVQ(3'b010),        // 40MHz: 3'b100 -- vs -- 80MHz: 3'b011 -- vs 160MHz: 3'b010 (For 16MHz in)
  		.FILTER_RANGE(3'b001)
  	) uut (
  		.LOCK(lock),
  		.RESETB(1'b1),
  		.BYPASS(1'b0),
  		.PACKAGEPIN(clk_in),
      .PLLOUTGLOBALB(clkout_b),
      //.PLLOUTCOREB(clkout_b),
  		.PLLOUTCOREA(clkout_a)
  	);
  endmodule
`endif




module edge_detect (input async_sig,
                    input clk,
                    output reg rise,
                    output reg fall);
  // Again on the iCE40 init vals don't matter
  // they are likely always zero
  // but we init for sim purposes

  reg [1:3] resync = 0;

  always @(posedge clk)
  begin
    // detect rising and falling edges.
    rise <= resync[2] & !resync[3];
    fall <= resync[3] & !resync[2];
    // update history shifter.
    resync <= {async_sig , resync[1:2]};
  end

endmodule







module shift(
    input clk,
    input data_in,
    output data_out
  );

  parameter DEPTH = 3;
  reg [DEPTH-1:0] holding_register;

  always @ (posedge clk) begin
    holding_register <= {holding_register[DEPTH-2:0], data_in};
  end

  assign data_out = holding_register[DEPTH-1];

endmodule
