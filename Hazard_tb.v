`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   2022/01/12 22:38:23
// Design Name:   
// Module Name:   Hazard_tb
// Project Name: 
// Target Device:  
// Tool versions:  
// Description: 
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module CPU_NoHazard_tb;
reg CLK;
reg RST;
wire [31:0] PC_OUT;
CPU_NoHazard uut (.PC_OUT(PC_OUT), .CLK(CLK), .RST(RST));
initial begin
	CLK = 1'b0;
	forever #10 CLK = ~CLK;
end 
initial begin
// Initialize Inputs
	RST = 1'b0;
// Wait 100 ns for global reset to finish
	#100;
// Add stimulus here
	RST = 1'b1;
	#15 RST = 1'b0;
end
endmodule