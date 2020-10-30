

// For divide by N
`define CLOG2(x) \
   x <= 1  ? 1 : \
   x <= 2	 ? 1 : \
   x <= 4	 ? 2 : \
   x <= 8	 ? 3 : \
   x <= 16	 ? 4 : \
   x <= 32	 ? 5 : \
   x <= 64	 ? 6 : \
   x <= 128	 ? 7 : \
   x <= 256	 ? 8 : \
   x <= 512	 ? 9 : \
   x <= 1024	 ? 10 : \
   x <= 2048	 ? 11 : \
   x <= 4096	 ? 12 : \
   x <= 8192	 ? 13 : \
   x <= 16384	 ? 14 : \
   x <= 32768	 ? 15 : \
   x <= 65536	 ? 16 : \
   -1



`ifdef DIVIDE_BY_N_ASYMMETRIC

  // This module variant, except for N = 2
  // creates high pulses that are 2 clocks that are spread apart by 2*N
  // length of logic low pulses, like:
  //   _                _                _
  // _| |______________| |______________| |________
  // Except for N=2 where it is normal 50% duty cycle clock division by 2
  module divide_by_n(
  	 input clk,
  	 input reset,
  	 output reg out
    );

  	parameter N = 2;

    // Please NOTE:
    //https://github.com/YosysHQ/yosys/issues/103
    // The initialization doesn't really mean anything
    // All regs on iCE40 init to zero ostensibly
    // However, for simulation purposes, we need to define an init value

  	reg [`CLOG2(N)-1:0] counter = 0;

  	always @(posedge clk)
  	begin
  		out <= 0;

  		if (reset)
  			counter <= 0;
  		else
  		if (counter == 0)
  		begin
  			out <= 1;
  			counter <= N - 1;
  		end else
  			counter <= counter - 1;
  	end

  endmodule

`else

  //
  // Symmetric (50% duty cycle) version
  //
  
  module divide_by_n(
     input clk,
     input reset,
     output reg out
    );

    parameter N = 2;

    // Please NOTE:
    //https://github.com/YosysHQ/yosys/issues/103
    // The initialization doesn't really mean anything
    // All regs on iCE40 init to zero ostensibly
    // However, for simulation purposes, we need to define an init value

    reg [`CLOG2(N/2)-1:0] counter = 0;

    always @(posedge clk)
    begin

      if (reset) begin
        counter <= 0;
        out <= 0;
      end else begin
        if (counter == 0) begin
          out <= ~out;
          counter <= (N/2) - 1;
        end else begin
          counter <= counter - 1;
        end
      end

    end

  endmodule

`endif
