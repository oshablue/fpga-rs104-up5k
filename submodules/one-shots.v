// one-shots.v

module monostable (
        input clk,
        input reset,
        input trigger,
        output reg pulse = 0  // output reg pulse = 0
      );

        parameter PULSE_WIDTH = 0;      // e.g. 4096 for 12-bit counter (0 to 4095 + 1)
        parameter COUNTER_WIDTH = 0;     // e.g. 13 to capture extra bit for 4096 sample intervals

        reg [COUNTER_WIDTH-1:0] count = 0; // [COUNTER_WIDTH-1:0] count = 0;
        //reg triggered = 0;

        // Some how this does not synthesize correctly! probably something wrong here
        // but can't find it - some inferred hardware issue probably ...
        // At least for POST synth simulation anyway ...
        // When not working ... the COUNTER_WIDTH was determining the length of the one-shot (!)
        /*always @ (posedge clk) begin
          if ( reset ) begin
            count <= 0;
            pulse <= 1'b 0;
            triggered <= 1'b 0;
          end else begin
            if ( trigger ) begin
              count <= count + 1'b 1;
              pulse <= 1'b 1;
              triggered <= 1'b 1;
            end else if ( (count != PULSE_WIDTH) && (triggered == 1'b 1) ) begin
              count <= count + 1'b 1;
              pulse <= 1'b 1;
              triggered <= 1'b 1; // every path/branch should assign
            end else begin
              pulse <= 1'b 0;
              count <= 0; // make this retriggerable without reset
              triggered <= 1'b 0;
            end
          end
        end*/

        /* looking at options to trigger at the end of the input only
        reg armed = 0;
        always @ ( posedge clk) begin
          if ( reset ) begin
            armed <= 0;
          else if ( trigger ) begin
            armed <= 1;
          end
        end*/

        always @ (posedge clk) begin
          if ( reset | ( count == PULSE_WIDTH) ) begin
            count <= 0;
            pulse <= 1'b 0;
          end else if ( trigger ) begin
            // Here since we have the trigger if statement first
            // We only start counting after the trigger is done
            // Thus real pulse width is trigger length + pulse width
            count <= 1;
            pulse <= 1;
          end else if ( pulse ) begin
            count <= count + 1;
            pulse <= 1;
          end
        end

endmodule





module monostable_vpw14b (
        input [13:0] pulse_width,
        input clk,
        input reset,
        input trigger,
        output reg pulse = 0  // output reg pulse = 0
      );

        //parameter PULSE_WIDTH = 0;      // e.g. 4096 for 12-bit counter (0 to 4095 + 1)
        parameter COUNTER_WIDTH = 0;     // e.g. 13 to capture extra bit for 4096 sample intervals

        reg [COUNTER_WIDTH-1:0] count = 0; // TODO parameterize

        // In this case, we modify to ignore any reset signal
        // We just want trigger and then reset on reach max count

        // Was:
        /*wire count_rst = (count == pulse_width); //reset | (count == PULSE_WIDTH);

        always @ (posedge trigger, posedge count_rst) begin
                if (count_rst) begin
                        pulse <= 1'b0;
                end else begin
                        pulse <= 1'b1;
                end
        end

        always @ (posedge clk, posedge count_rst) begin
                if(count_rst) begin
                        count <= 0;
                end else begin
                        if(pulse) begin
                                count <= count + 1'b1;
                        end
                end
        end
        */

        // New formulation:
        //reg triggered = 0;
        always @ (posedge clk) begin
          if ( reset ) begin
            count <= 0;
            pulse <= 1'b 0;
            //triggered <= 1'b 0;
          end else begin
            if ( trigger ) begin
              count <= count + 1;
              pulse <= 1'b 1;
              //triggered <= 1'b 1;
            end else if ( (count != pulse_width) && pulse ) begin // triggered ) begin
              count <= count + 1;
            end else begin
              pulse <= 1'b 0;
              //triggered <= 1'b 0;
              count <= 0;
            end
          end
        end

endmodule
