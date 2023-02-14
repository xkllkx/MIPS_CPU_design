`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/12/23 20:13:00
// Design Name: 
// Module Name: SingleCycle_tb
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

module SingleCycle_tb;
reg clk,reset;
SingleCycle uut(.clk(clk),.reset(reset));
initial begin
    clk=0;#40;    
    reset=1;#20;
    reset=0;
end  

always begin
    #50; clk=~clk;
end
endmodule

