#!/bin/bash
echo "Make sure to set SIM macro define in the top or top header file."
yosys -p 'synth_ice40 -nobram -top top -blif simtmp/sim.blif' top.v
yosys -o simtmp/sim_synth.v simtmp/sim.blif
iverilog -v -o simtmp/sim_to_waves.s -D POST_SYNTHESIS top_tb.v simtmp/sim_synth.v `yosys-config --datdir/ice40/cells_sim.v`
./simtmp/sim_to_waves.s
# See the waveforms with eg
# open /Applications/gtkwave sim_dump.vcd
echo "To see waves, example: open /Applications/gtkwave.app sim_dump.vcd"
echo "And you can File => Read Save File to reopen wave views from prior saved session."
