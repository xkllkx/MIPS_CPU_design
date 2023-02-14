`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/12/23 20:06:00
// Design Name: 
// Module Name: SingleCycle
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

module SingleCycle(clk,reset);
wire [31:0] Instruction;
wire [4:0] Writereg;
wire [31:0] PC_in;
wire [31:0] WriteData;
wire [31:0] Readdata1,Readdata2;
wire [31:0] PC_out1;
wire [31:0]signout;
wire [31:0]ALUIn2;
wire RegWrite,RegDST,Jump,Branch,MemRead,MemtoReg,MemWrite,ALUSrc;
wire [1:0] ALUOp;
wire [3:0] ALUfunc;
wire [31:0]ALUresult1,ALUresult;
wire [31:0]MUX_out_0;
wire Zero,MUX_and;
wire [31:0] Read_data;
wire [31:0] PC_add4,Instruction3,Instruction2;
wire [27:0] Instruction1;
input clk,reset;
    PC PC(reset,clk, PC_out1,PC_in);
    regis regis(Writereg,Readdata1,Readdata2,clk,reset,RegWrite,clk,WriteData,Instruction);
    muxleft muxleft(RegDST,Instruction,Writereg);
    ins ins(PC_out1,Instruction,clk,reset);
    Control Control(RegDST,Jump,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,ALUOp,Instruction);
    muxmiddle muxmiddle(ALUSrc,ALUIn2,Readdata2,signout);
    signextension signextension(Instruction,signout);
    ALUcontrol ALUcontrol(ALUOp,Instruction,ALUfunc);
    ALU ALU(ALUfunc,Readdata1,ALUIn2,Zero,ALUresult);
    DataMemory DataMemory(reset, clk, ALUresult, Readdata2, Read_data, MemRead, MemWrite);
    muxright muxright(MemtoReg,ALUresult,Read_data,WriteData);
    and a1(MUX_and,Branch,Zero);
    Mux_1 Mux_1(MUX_out_0,MUX_and,PC_add4,ALUresult1);
    Add Add(ALUresult1,PC_add4,Instruction3);
    Shiftleft2_32_32 Shiftleft2_32_32(Instruction3,signout);
    Shiftleft2_26_28 Shiftleft2_26_28(Instruction1,Instruction);
    Mux_2 Mux_2(PC_in,Jump,MUX_out_0,Instruction1,PC_add4);
    add4 add4(PC_out1,PC_add4);
endmodule

module add4(PC_out1,PC_add4);
    input [31:0] PC_out1;
    output [31:0] PC_add4;
    assign PC_add4=PC_out1+4;
endmodule

module Mux_2(PC_in,Jump,MUX_out_0,Instruction1,PC_add4);
input Jump;   
input [27:0] Instruction1;
input [31:0] MUX_out_0,PC_add4;
output [31:0] PC_in;
reg [31:0] PC_in;
	always@(*) begin 
	   if (Jump==0)
			PC_in=MUX_out_0;
	   else if (Jump==1)
			PC_in={PC_add4[31:28],Instruction1};	 
    end
endmodule

module Shiftleft2_26_28(Instruction1,Instruction);
input [31:0]Instruction;
output [27:0]Instruction1;
wire [27:0]Instruction1;
    assign Instruction1 ={Instruction[25:0],2'b00};
endmodule

module Shiftleft2_32_32(Instruction3,Instruction2);
input [31:0]Instruction2;
output [31:0]Instruction3;
wire [31:0]Instruction3;
    assign Instruction3 ={Instruction2[29:0],2'b00};
endmodule

module Add(ALUresult,PC_add4,Instruction3);
input [31:0] PC_add4,Instruction3;
output [31:0] ALUresult;
wire [31:0] ALUresult;
    assign ALUresult=PC_add4+Instruction3;
endmodule

module Mux_1(MUX_out_0,MUX_and,PC_add4,ALUresult);
input MUX_and;
input [31:0] PC_add4,ALUresult;
output [31:0] MUX_out_0;
reg [31:0]  MUX_out_0;
	always@(*) begin 
		 if (MUX_and==0)
			MUX_out_0=PC_add4;
		 else if (MUX_and==1)
			MUX_out_0=ALUresult;	 
	end
endmodule

module muxright(MemtoReg,ALUresult,Read_data,WriteData);
    input MemtoReg;
    input [31:0]ALUresult;
    input [31:0]Read_data;
    output reg[31:0]WriteData;
    always@(*) begin
        if(MemtoReg==0) WriteData=ALUresult;
        else WriteData=Read_data;
    end
endmodule

module ALUcontrol(ALUOp,Instruction,ALUfunc);
input [1:0]ALUOp;
input [31:0]Instruction;
output reg [3:0]ALUfunc;
wire [5:0]funct;
    assign funct=Instruction[5:0];
    always@(*) begin
        case(ALUOp)
            2'b00: ALUfunc=4'b0010;//lw,sw-add
            2'b01: ALUfunc=4'b0110;//beq-sub
            2'b10: begin
                if(funct==6'b100000)//R-type
                    ALUfunc=4'b0010;
                else if(funct==6'b100010)
                    ALUfunc=4'b0110;   
                else if(funct==6'b100100) 
                    ALUfunc=4'b0000;  
                else if(funct==6'b100101) 
                    ALUfunc=4'b0001;
                else if(funct==6'b101010) 
                    ALUfunc=4'b0111;    
            end        
            default: ALUfunc=funct;
        endcase
    end
endmodule

module Control(RegDST,Jump,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,ALUOp,Instruction);
input [31:0] Instruction;
output reg RegDST,Jump,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
output reg [1:0] ALUOp;
    always@(*)begin
        case(Instruction[31:26])
            6'b000000 : begin  // R-Format
               RegDST = 1'b1 ;
               Jump = 1'b0 ;
               Branch = 1'b0 ;
               MemRead = 1'b0 ;
               MemtoReg = 1'b0 ;
               MemWrite = 1'b0 ;
               ALUSrc = 1'b0 ;
               RegWrite = 1'b1 ;
               ALUOp = 2'b10 ;
            end
             6'b100011 : begin  // loadword
               RegDST = 1'b0 ;
               Jump = 1'b0 ;
               Branch = 1'b0 ;
               MemRead = 1'b1 ;
               MemtoReg = 1'b1 ;
               MemWrite = 1'b0 ;
               ALUSrc = 1'b1 ;
               RegWrite = 1'b1 ;
               ALUOp = 2'b00 ;
            end
             6'b101011 : begin  // saveword
               RegDST = 1'b0 ;
               Jump = 1'b0 ;
               Branch = 1'b0 ;
               MemRead = 1'b0 ;
               MemtoReg = 1'b0 ;
               MemWrite = 1'b1 ;
               ALUSrc = 1'b1 ;
               RegWrite = 1'b0 ;
               ALUOp = 2'b00 ;
            end
             6'b000100 : begin  //branch
               RegDST = 1'b0 ;
               Jump = 1'b0 ;
               Branch = 1'b1 ;
               MemRead = 1'b0 ;
               MemtoReg = 1'b0 ;
               MemWrite = 1'b0 ;
               ALUSrc = 1'b0 ;
               RegWrite = 1'b0 ;
               ALUOp = 2'b01 ;
            end
             6'b000010 : begin  //jump
               RegDST = 1'b0 ;
               Jump = 1'b1 ;
               Branch = 1'b0 ;
               MemRead = 1'b0 ;
               MemtoReg = 1'b0 ;
               MemWrite = 1'b0 ;
               ALUSrc = 1'b0 ;
               RegWrite = 1'b0 ;
               ALUOp = 2'b00 ;
            end
            6'b001000 : begin  //addi
               RegDST = 1'b0 ;
               Jump = 1'b0 ;
               Branch = 1'b0 ;
               MemRead = 1'b0 ;
               MemtoReg = 1'b0 ;
               MemWrite = 1'b0 ;
               ALUSrc = 1'b1 ;
               RegWrite = 1'b1 ;
               ALUOp = 2'b00 ;
            end
            default:begin 
               RegDST = 1'b0 ;
               Jump = 1'b0 ;
               Branch = 1'b0 ;
               MemRead = 1'b0 ;
               MemtoReg = 1'b0 ;
               MemWrite = 1'b0 ;
               ALUSrc = 1'b0 ;
               RegWrite = 1'b0 ;
               ALUOp = 2'b00 ;
            end
        endcase
    end   
endmodule

module PC(reset,Clk, PC_out1,PC_in);
output reg   [31:0] PC_out1;
input reset,Clk;
input [31:0] PC_in;
reg flag;
    always@(posedge Clk or posedge reset)begin
        if(reset) flag<=1'b0;
        else flag<=1'b1;
    end
    always@(posedge Clk or posedge reset)begin
        if(reset) PC_out1<=32'b0;
        else if(flag) PC_out1<=PC_in;
        else PC_out1<=32'b0;
    end    
endmodule

module ins(PC_out1,Instruction,clk,reset);
input clk,reset;
input [31:0]PC_out1;
output [31:0]Instruction;  
reg [31:0]Instr_memory [31:0]; 
integer i;
assign Instruction=Instr_memory[PC_out1/4];
initial begin
    Instr_memory[0]<=32'b001000_00000_10000_0000000000001010; //addi $s0,$zero,10
    Instr_memory[1]<=32'b001000_10000_10001_0000000000000010; //addi $s1,$s0,2
    Instr_memory[2]<=32'b101011_10010_10000_0000000000000010; //sw $s0,4($s2)
    Instr_memory[3]<=32'b001000_00000_10010_0000000000001010; //addi $s2,$zero,10
    Instr_memory[4]<=32'b000100_10010_10000_0000000000000001; //beq $s0,$s2,LABEL1
    Instr_memory[5]<=32'b000000_10001_10000_10011_00000_100010; //sub $s3,$s1,$s0
    Instr_memory[6]<= 32'b000010_00000000000000000000000111; //LABEL1:j EXIT
    Instr_memory[7]<= 32'b000000_00000_00000_00000_00000_000000; //EXIT: none
	for(i=2;i<256;i=i+1)  Instr_memory[i] = 32'h00000000;
end
endmodule

module regis(Writereg,Readdata1,Readdata2,clk,reset,RegWrite,clk,WriteData,Instruction);
input  clk,reset,RegWrite;
output  [31:0] Readdata1,Readdata2;
input [31:0]Instruction;
input [4:0]Writereg;
input[31:0] WriteData;
reg [31:0] register[31:0];
integer i=0;
    initial begin
        for(i=0;i<32;i=i+1)begin
            register[i]=32'd0;
        end
    end

    assign Readdata1=register[Instruction[25:21]]; 
    assign Readdata2=register[Instruction[20:16]];

    always@(posedge clk or posedge reset)
    begin
        if(reset) register[Writereg]<=32'd0;
        else if(RegWrite) register[Writereg]<=WriteData;
        else register[Writereg]<=register[Writereg];       
    end
endmodule

module muxleft(RegDST,Instruction,Writereg);
    input RegDST;
    input [31:0]Instruction;
    output reg[4:0]Writereg;
    always@(*)begin//R-type or load mux
        if(RegDST==0) Writereg=Instruction[20:16];//load
        else Writereg=Instruction[15:11];//R-type
    end
endmodule

module muxmiddle(ALUSrc,ALUIn2,Readdata2,signout);
input ALUSrc;
input [31:0]Readdata2;
input [31:0]signout; 
output  reg[31:0]ALUIn2;
    always@(*) begin
        if(ALUSrc==0) ALUIn2=Readdata2;//load
        else ALUIn2=signout;//R-type
    end
endmodule

module signextension(Instruction,signout);
input [31:0]Instruction;
output reg[31:0] signout;
wire[15:0] signin;
    assign signin=Instruction[15:0];
    always@(*) begin
        signout={{16{signin[15]}},signin[15:0]};
    end
endmodule

module ALU(ALUfunc,ALUIn1,ALUIn2,Zero,ALUresult);
input [3:0]ALUfunc;
input [31:0]ALUIn1,ALUIn2;
output reg [31:0]ALUresult;
output reg Zero=0;
    always@(*)
    begin
        case(ALUfunc)
        4'b0000: ALUresult=ALUIn1&ALUIn2;
        4'b0001: ALUresult=ALUIn1|ALUIn2;
        4'b0010: ALUresult=ALUIn1+ALUIn2;
        4'b0110: begin
            ALUresult=ALUIn1-ALUIn2;
            if(ALUresult==0) Zero=1;
            else Zero=0;
        end
        default: ALUresult=(ALUIn1<ALUIn2)?32'd1:32'd0;
    endcase
end
endmodule

module DataMemory(reset, clk, ALUresult, Readdata2, Read_data, MemRead, MemWrite);
input reset, clk;
input [31:0] ALUresult, Readdata2;
input MemRead, MemWrite;
wire [5:0]address=ALUresult[5:0];
reg [31:0] RAM_data[63:0];
output  [31:0] Read_data;
assign Read_data = MemRead? RAM_data[address]: 32'h00000000;
integer i;
    initial begin
        for(i=0;i<64;i=i+1)begin
            RAM_data[i] <= 32'h00000000;
        end
    end
    always@(negedge clk or posedge reset) begin
        if(reset)
            begin
            for(i=0;i<64;i=i+1)begin
                 RAM_data[i] <= 32'h00000000;
                 end
            end
        else if(MemWrite) RAM_data[address]<=Readdata2;
        else RAM_data[address]<=RAM_data[address];       
    end
endmodule