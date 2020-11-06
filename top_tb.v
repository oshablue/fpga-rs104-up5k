// top_tb.v

//`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps

// REFS:
// https://verilogguide.readthedocs.io/en/latest/verilog/testbench.html
// https://github.com/YosysHQ/yosys-bench/blob/master/verilog/benchmarks_small/various/pwm256_tb.v
// https://stackoverflow.com/questions/35927650/is-it-possible-to-create-a-simulation-waveform-from-yosys-output
// http://www.clifford.at/yosys/files/yosys_manual.pdf
// https://github.com/cliffordwolf/icotools/blob/master/examples/icezero/testbench.v
// https://stackoverflow.com/questions/45172834/how-to-run-post-synthesis-simulation-with-the-icestorm-ice40-fpga-flow
//
// YOSYS
// http://www.clifford.at/yosys/files/yosys_manual.pdf
//
// HOW TO
// PRE Pre-synthesis sim:
// After creating sim link in /usr/local/lib or bin was it? for ivl directory or whatever to
// the directory in ~/.apio/packages/toolchain-iverilog/etc
// and also a sim link to the bin for vvp in the local directory, the sim lives in the /usr/local/bin or similar
// then
// $ iverilog -v -o test_pre.vvp top.v top_tb.v ~/.apio/packages/toolchain-iverilog/vlib/cells_sim.v
// $ vvp test_pre.vvp
// open /Applications/gtkwave.app test_pre.vcd (making sure this matches the filename in the _tb.v file)
//
// POST
// https://stackoverflow.com/questions/35927650/is-it-possible-to-create-a-simulation-waveform-from-yosys-output
// $ ln -s ~/.apio/packages/toolchain-icestorm/share/yosys /usr/local/share/yosys
// Make sure in test bench _tb file to update any reg inputs/output to work with the POST_SYNTHESIS
// otherwise you'll get errors about not being able to find eg data_in (see below)
// $
// $ yosys -p 'synth_ice40 -top top -blif test.blif' top.v
//   # If we have actual implemented BRAMs that are not pruned that so far seem to break iverilog
//   # just due to the issue of port[0] ... etc individual mapping it would seem ... so far anyway
//   # see the brams_map.v file versus the test_syn.v output file mapping of signals ... actually it seems ok
//   # but somehow iverilog breaks on it with no port - probably missing something here
//   # anyway, can add:
//   # -nobram right after synth_ice40
//   # however note everything will take a whole lot longer as LUTs are used to simulate instead
//   # Also maybe add: | grep -B 3 -A 3 Warning to see extra lines specifically around Warnings
//   # to the end of the first command there to see warnings - A/B flags show lines before/after
// Thus our current SIM command is:
// $ yosys -p 'synth_ice40 -nobram -top top -blif test.blif' top.v
//
// If we are using additional files and not includes (which matches the apio default)
// if the files are in the main directory then we need to capture all the files/modules doing for example:
//
// $ yosys -p 'synth_ice40 -top top -blif test.blif' one-shots.v top.v
//
// Or modify the main SIM macro ifdef case to add the include
// Or we put the file in a folder and then always use include
//
// $ yosys -o test_syn.v test.blif
// $ iverilog -o test_post -D POST_SYNTHESIS top_tb.v test_syn.v `yosys-config --datdir/ice40/cells_sim.v`
// $ ./test_post
//   will dump the data into whatever the named output file - which here confusingly is still test_pre.vcd
//   This will not have named signals for the modules (except user-defined top level names) by default
//   It will also bypass the clk sim for SYNTHESIS - thus if PLL sim doesn't work, this won't
//
// For POST_SYNTHESIS this doesn't work right off the bat for PLL to sim clock changes so far:
// because first top.v is synthesized into for example test.blif
// and then the test.blif is converted back to *.v
// and then this test bench file is used with iverilog on the converted *.v (like test_syn.v)
// because we are in POST -- so the define from the top_tb gets lost, and top.v just uses
// the PLL (not the sim clock) which breaks anyway because we are not using the _tb.v file in the
// first step of sim.
// SO: DO this manually with the SYNTHESIS ifdef <=> ifndef in top.v manually for now when
// doing post-syn sim
//
// Viewing flow graphs of the synthesis:
// - Haven't yet gotten the whole thing or even more than one part to flow graph viz yet
//   Perhaps a limitation, more likely just not full usage understood yet
// - See scripts in script-drafts
// - Can also just use those script examples on the yosys command line
// - Mac OS X, did:
//   - brew install graphviz (installs only the command line items)
//   - brew install xdot (installs a GUI viewer for the dot file)
// - *.dot to png for example, with suppressed output:
//   - $ dot -Tpng top.dot -o top.png >/dev/null




// See above comment - this doesn't really do anything right now
`define TESTBENCH

module top_tb;

  // For USE_UART
  // Ref I/O:
  /*
    module top (
      input CLK,                  // 16MHz clock
      input RST,                  // RST for PLL?
      input ext_trig_in,              // trigger a capture event sequence from comms device
      input [1:0] rx_delay_ctrl,
      input [7:0] data_in,
      output adc_encode,           // encode signal to ADC to capture a sample
      output PULSE_NEG,
      output RTZ_NEG,
      output RTZ_POS
      ,
      output UART_TX,
      input UART_RX
        ,
        output PDO0,           // goes to JT3A Pin 10 (PDO0)
        output PDO1,           // goes to JT3A Pin 8 (PDO1)
        output PDO2            // goes to JT3A Pin 6 (PDO2)
    );
  */


    // https://stackoverflow.com/questions/45172834/how-to-run-post-synthesis-simulation-with-the-icestorm-ice40-fpga-flow
    reg CLK;
    reg RST = 1;     // active low
    reg ext_trig_in = 0;
    reg [1:0] rx_delay_ctrl = 2'b 0;
    reg [7:0] data_in = 8'b 0101_0101;
    reg UART_RX = 1'b 0;
    wire adc_encode, PULSE_NEG, RTZ_NEG, RTZ_POS;
    wire UART_TX;
    wire PDO0, PDO1, PDO2;


    // clock generation
    // vs always #10 CLK=~CLK; // see link above
    // was:
    always #10 CLK = (CLK === 1'b0);
    //always #1 CLK = (CLK === 1'b0);

    // duration for each bit = 20 * timescale = 20 * 1 ns  = 20ns
    //localparam period = 20;

    // If CLK is 160 MHz
    // #5 = 32MHz MCU clock for example

    top UUT (
      .CLK (CLK),                  // 16MHz clock
      .RST (RST),                  // RST for PLL?
      .ext_trig_in (ext_trig_in),              // trigger a capture event sequence from comms device
      `ifndef POST_SYNTHESIS
        .rx_delay_ctrl (rx_delay_ctrl),
        .data_in (data_in),
      `else
        . \rx_delay_ctrl[0] (rx_delay_ctrl[0]),
        . \rx_delay_ctrl[1] (rx_delay_ctrl[1]),
        . \data_in[0] (data_in[0]),
        . \data_in[1] (data_in[1]),
        . \data_in[2] (data_in[2]),
        . \data_in[3] (data_in[3]),
        . \data_in[4] (data_in[4]),
        . \data_in[5] (data_in[5]),
        . \data_in[6] (data_in[6]),
        . \data_in[7] (data_in[7]),
      `endif
      .adc_encode (adc_encode),           // encode signal to ADC to capture a sample
      .PULSE_NEG (PULSE_NEG),
      .RTZ_NEG (RTZ_NEG),
      .RTZ_POS (RTZ_POS),
      .UART_TX (UART_TX),
      .UART_RX (UART_RX),
      // Below depends on MACRO for test timing output
      .PDO0 (PDO0),           // goes to JT3A Pin 10 (PDO0)
      .PDO1 (PDO1),           // goes to JT3A Pin 8 (PDO1)
      .PDO2 (PDO2)            // goes to JT3A Pin 6 (PDO2)
    );

    // OKishTODO update pulse widths and timing to match as measured on scope
    // for actual MCU control!

    // TODO RESET signal for seq_id reset (actually the !RST) signal
    // during a channel scan actually happens probably during the capture
    // thus in earlier implementations the seq_id would get reset prematuraly
    // after the capture trigger but maybe before the output started - thus sending
    // one WF early the zeroed seq_id

    // NOTE: Times below are ADDITIVE! (not absolute time positions on the timeline)

    initial // initial block executes only once
        begin
            $dumpfile("sim_dump.vcd");
            $dumpvars;
            //#100 RST = 0;
            //#150 RST = 1;
            // With 160MHz real PLL and #1 CLK toggles
            // 160MHz is 2 CLKs statements from above
            // Duration of single reset toggle (2 consec C high level MCU instructions)
            // 160MHz = 6.25ns period = 2 CLKs
            // 108 ish ns for reset toggle from MCU => 17.3 periods = #35
            // Or #350 if using #10 CLK toggles
            #100 RST = 0;
            #350 RST = 1;
            //#200 ext_trig_in = 1;
            //#250 ext_trig_in = 0;
            // Trig is same as above - 2 consec MCU high level stmts:
            #100000 ext_trig_in = 1; // typical test was 1000 ...testing longer intervals
            #350 ext_trig_in = 0;
            // Now we test with a 2nd reset - really it would be the only one
            // in a true channel scan series - we put it sometime within the capture
            // period eg 103220 - 100800 = 2420
            #2420 RST = 0;
            #350  RST = 1;
            // Testing about the first PAQ pulse not working
            /*#10000 RST = 0;
            #10050 RST = 1;
            #10200 ext_trig_in = 1;
            #10250 ext_trig_in = 0;
            */

            // What if there is another duplicate trigger?
            #10000 ext_trig_in = 1;
            #350 ext_trig_in = 0;

            #1000000 $finish; // vvp takes about 18 seconds at this 10M for pre_synth or maybe 60 sec for POST
            // 160MHz * 0.04 sec = 6.4M
        end
endmodule


// Versus:
/*repeat (5) @(posedge CLK);
ext_trig_in <= 1;
repeat (5) @(posedge CLK);
ext_trig_in <= 0;
*/
