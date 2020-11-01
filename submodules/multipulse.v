// multipulse.v
// Not fully implemented - pulled out from top.v from demo of capability and
// related ideas
//
// Needs module wrapper with IO





    //**************************************************************************
    //
    /* PULSE CONTROL - MULTIPLE / DEMO
    //
    /**************************************************************************/

`ifdef DEMO_VAR_FREQ_PULSE

    wire
      pulse_on1, pulse_rtz1,
      pulse_on2, pulse_rtz2, pulse_on3, pulse_rtz3, pulse_on4, pulse_rtz4,
      pulse_on5, pulse_rtz5, pulse_on6, pulse_rtz6, pulse_on7, pulse_rtz7,
      pulse_on8, pulse_rtz8, pulse_on9, pulse_rtz9, pulse_on10, pulse_rtz10,
      pulse_on11, pulse_rtz11, pulse_on12, pulse_rtz12,
      pulse_rtzlast;

    assign pulse_on = (
        pulse_on1 || pulse_on2 || pulse_on3 || pulse_on4
        || pulse_on5 || pulse_on6 || pulse_on7 || pulse_on8
        || pulse_on9 // || pulse_on10 || pulse_on11 || pulse_on12
      );
    assign pulse_rtz = (
        pulse_rtz1 || pulse_rtz2 || pulse_rtz3 || pulse_rtz4 || pulse_rtzlast
        || pulse_rtz5 || pulse_rtz6 || pulse_rtz7 || pulse_rtz8
        || pulse_rtz9 // || pulse_rtz10 || pulse_rtz11 || pulse_rtz12
      );

    // Notes:
    // For demo VAR_FREQ / swept frequency:
    // We use: 2x faster clock and only the pulse and rtz sections of the control

    // Yes, this can definitely be parameterized and for loop equivalent ...
    // This is a very quick dev rev just to test and demo hardware capabilities
    // (ie not a demo of good coding practice - under the gun here that is ...)

    //
    // First pulse for VAR_FREQ demo idea
    monostable #(
      .PULSE_WIDTH(100), // was 24
      .COUNTER_WIDTH(8) // was 5
    ) msv_pulse_on1 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (trig_in_rise),
      .pulse      (pulse_on1)
    );
    monostable #(
        .PULSE_WIDTH(70),   // was 23
        .COUNTER_WIDTH(8)   // was 5
    ) msv_pulse_rtz1 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on1),
        .pulse    (pulse_rtz1)
    );
    // End of First Pulse
    //


    //
    // Second Pulse
    monostable #(
      .PULSE_WIDTH(50),     // was 22
      .COUNTER_WIDTH(7)     // was 5
    ) msv_pulse_on2 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz1),
      .pulse      (pulse_on2)
    );
    monostable #(
        .PULSE_WIDTH(40),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz2 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on2),
        .pulse    (pulse_rtz2)
    );
    // End of Second Pulse
    //



    //
    // Third Pulse
    monostable #(
      .PULSE_WIDTH(35),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on3 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz2),
      .pulse      (pulse_on3)
    );
    monostable #(
        .PULSE_WIDTH(30),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz3 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on3),
        .pulse    (pulse_rtz3)
    );
    // End of Third Pulse
    //


    //
    // Pulse 4
    monostable #(
      .PULSE_WIDTH(26),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on4 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz3),
      .pulse      (pulse_on4)
    );
    monostable #(
        .PULSE_WIDTH(22),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz4 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on4),
        .pulse    (pulse_rtz4)
    );
    // End of Pulse 4
    //


    //
    // Pulse 5
    monostable #(
      .PULSE_WIDTH(18),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on5 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz4),
      .pulse      (pulse_on5)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz5 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on5),
        .pulse    (pulse_rtz5)
    );
    // End of Pulse 5
    //



    //
    // Pulse 6
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(7)
    ) msv_pulse_on6 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz5),
      .pulse      (pulse_on6)
    );
    monostable #(
        .PULSE_WIDTH(14),
        .COUNTER_WIDTH(7)
    ) msv_pulse_rtz6 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on6),
        .pulse    (pulse_rtz6)
    );
    // End of Pulse 6
    //




    //
    // Pulse 7
    monostable #(
      .PULSE_WIDTH(14),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on7 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz6),
      .pulse      (pulse_on7)
    );
    monostable #(
        .PULSE_WIDTH(12),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz7 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on7),
        .pulse    (pulse_rtz7)
    );
    // End of Pulse 7
    //




    //
    // Pulse 8
    monostable #(
      .PULSE_WIDTH(10),
      .COUNTER_WIDTH(4)
    ) msv_pulse_on8 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz7),
      .pulse      (pulse_on8)
    );
    monostable #(
        .PULSE_WIDTH(10),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz8 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on8),
        .pulse    (pulse_rtz8)
    );
    // End of Pulse 8
    //



    //
    // Pulse 9
    monostable #(
      .PULSE_WIDTH(8),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on9 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz8),
      .pulse      (pulse_on9)
    );
    monostable #(
        .PULSE_WIDTH(8),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz9 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on9),
        .pulse    (pulse_rtz9)
    );
    // End of Pulse 9
    //


    //
    // Around here we need to update analog components on the board that
    // are in place to limit slope because some manufacturers make transducers
    // with inductor tuning that requires heavy damping to control the load
    // if we want to keep amplitude (just due to the intentionally limited)
    // slope
    //


    /*
    //
    // Pulse 10
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on10 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz9),
      .pulse      (pulse_on10)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz10 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on10),
        .pulse    (pulse_rtz10)
    );
    // End of Pulse 10
    //




    //
    // Pulse 11
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on11 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz10),
      .pulse      (pulse_on11)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz11 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on11),
        .pulse    (pulse_rtz11)
    );
    // End of Pulse 11
    //




    //
    // Pulse 12
    monostable #(
      .PULSE_WIDTH(16),
      .COUNTER_WIDTH(5)
    ) msv_pulse_on12 (
      .clk        (clk160),
      .reset      (trig_in_rise),
      .trigger    (!pulse_rtz11),
      .pulse      (pulse_on12)
    );
    monostable #(
        .PULSE_WIDTH(16),
        .COUNTER_WIDTH(5)
    ) msv_pulse_rtz12 (
        .clk      (clk160),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on12),
        .pulse    (pulse_rtz12)
    );
    // End of Pulse 12
    //
    */


    //
    // Final RTZ
    monostable #(
        .PULSE_WIDTH(64),
        .COUNTER_WIDTH(7)
    ) msv_lastpulse_rtz (
        .clk      (SAMPLECLK),
        .reset    (trig_in_rise),
        .trigger  (!pulse_on9),     // Yes, we'll just overlap the above for now
        .pulse    (pulse_rtzlast)
    );
    //
    //



`endif // End of DEMO_VAR_FREQ_PULSE
