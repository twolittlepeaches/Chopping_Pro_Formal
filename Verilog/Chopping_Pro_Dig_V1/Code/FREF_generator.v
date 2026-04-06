`timescale 1ps/1fs

module FREF_generator #(parameter Fr=32e6)
	(fref,rst_n);

output fref;
output rst_n;

reg fref;
reg rst_n;

real jitter_prev,jitter;
real jitter_tot;
real dith_delay;
real dith_prev;
real TREF = 1.0/Fr*1e12;
real TREF_2 = 0.5/Fr*1e12;
integer seed;
integer fp1;
parameter J_Tr = 0.1;

initial begin
	seed = 13;
	fref = 1'b0;
	jitter_prev = 0;
	fp1 = $fopen("./fref.txt","w");
	#(TREF_2) fref <= 1'b1;
end

always @(posedge fref) begin
	//jitter = J_Tr * (0.01*$dist_uniform(seed,-50,50));
	jitter = J_Tr * $dist_normal(seed,0,100,1000)*0.01;
	jitter_tot = jitter-jitter_prev;
	#TREF_2 fref <= 1'b0;
	#(TREF_2+jitter_tot) fref <= 1'b1;
	jitter_prev = jitter;
end

always @(posedge fref) begin
	//$fstrope(fp1,"%3.13e \n",$realtime);
	$fwrite(fp1,"%3.13e \n",$realtime);
end

/*
initial begin
	rst_n <= 1'b1;
	#560 rst_n <= 1'b0;
	#80123 rst_n <= 1'b1;
	#500_000_000 $finish();
end
*/

endmodule
