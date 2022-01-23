`timescale 1 ns/10 ps  // time-unit = 1 ns, precision = 10 ps

module PWM_ADC_tb;
	 reg clk;
    reg PWM_in;
    wire PWM_out;



    PWM_ADC1 u (
	 .clk(clk), 
	 .PWM_in(PWM_in), 
	 .PWM_out(PWM_out)
	 );
	 

always 
begin
    #20 clk = ~clk; 
    
end

always @(posedge clk)
begin
    // values for a and b
    PWM_in = 16;
    
    #100000; // wait for period 
    

    $stop;   // end of simulation
end
endmodule