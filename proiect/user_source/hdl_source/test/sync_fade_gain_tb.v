`timescale 1ns/1ps
module sync_fade_gain_tb;
 reg clk,rst_n,sample_valid_in; reg signed [15:0] pcm_sample_in; reg [7:0] alpha_in; wire signed [15:0] pcm_sample_out; wire sample_valid_out; integer errors;
 sync_fade_gain dut(.clk(clk),.rst_n(rst_n),.pcm_sample_in(pcm_sample_in),.sample_valid_in(sample_valid_in),.alpha_in(alpha_in),.pcm_sample_out(pcm_sample_out),.sample_valid_out(sample_valid_out));
 always #5 clk=~clk;
 task automatic check_sample; input signed [15:0] sample; input [7:0] alpha; input signed [15:0] expected; begin
   alpha_in=alpha; repeat(3) @(posedge clk); @(negedge clk); pcm_sample_in=sample; sample_valid_in=1'b1; repeat(4) @(posedge clk); #1;
   if (!sample_valid_out || pcm_sample_out !== expected) begin $display("FAIL alpha=%0d sample=%0d expected=%0d got=%0d valid=%b",alpha,sample,expected,pcm_sample_out,sample_valid_out); errors=errors+1; end
   @(negedge clk); sample_valid_in=1'b0; repeat(4) @(posedge clk); #1; if(sample_valid_out!==1'b0) begin $display("FAIL valid"); errors=errors+1; end
 end endtask
 initial begin clk=0; rst_n=0; pcm_sample_in=0; sample_valid_in=0; alpha_in=255; errors=0; repeat(3) @(posedge clk); rst_n=1;
   check_sample(16'sd12345,8'd255,16'sd12345); check_sample(-16'sd12345,8'd255,-16'sd12345); check_sample(16'sd12345,8'd0,16'sd0); check_sample(-16'sd12345,8'd0,16'sd0); check_sample(16'sd10000,8'd128,16'sd5019); check_sample(-16'sd10000,8'd128,-16'sd5019); check_sample(16'sh7fff,8'd255,16'sh7fff); check_sample(16'sh8000,8'd255,16'sh8000);
   if(errors==0) $display("PASS sync_fade_gain_tb"); else begin $display("FAIL sync_fade_gain_tb errors=%0d",errors); $fatal(1); end $finish; end
 initial begin #100000; $fatal(1,"FAIL timeout"); end
endmodule
