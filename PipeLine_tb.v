`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/01/08 10:35:23
// Design Name: 
// Module Name: PipeLine_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module PipeLine_tb();
reg clk,reset;
PipeLine uut(.clk(clk),.reset(reset));
initial begin
    clk=0;#40;    
    reset=1;#20;
    reset=0;
end  
always begin
    #50; clk=~clk;
end
endmodule
