`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/01/07 13:30:05
// Design Name: 
// Module Name: PipeLine
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

module PipeLine(clk,reset);
wire [31:0] Instruction;
wire [4:0] Writereg;
input clk,reset;
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
wire REGWRITE_ID_EX, MEMTOREG_ID_EX, BRANCH_ID_EX, MEMREAD_ID_EX, MEMWRITE_ID_EX, ALUSRC_ID_EX, REGDST_ID_EX ;
wire [1:0]ALUOP_ID_EX ;
wire [31:0]PC_4_ID_EX, READ_DATA_1_ID_EX, READ_DATA_2_ID_EX, SE_ID_EX,INSTRUCTION_ID_EX;
wire [4:0]RD_ID_EX, RT_ID_EX ;
wire [31:0]INSTRUCTION_IF_ID, PC_4_IF_ID ;
wire REGWRITE_EX_MEM, MEMTOREG_EX_MEM, BRANCH_EX_MEM, MEMREAD_EX_MEM, MEMWRITE_EX_MEM, ZERO_EX_MEM ;
wire [31:0] ADD_RESULT_EX_MEM, ALU_RESULT_EX_MEM, READ_DATA_2_EX_MEM ;
wire [4:0] WRITE_ADDR_EX_MEM ;
wire REGWRITE_MEM_WB, MemToReg_MEM_WB ;
wire [31:0]DataMemory_Output_MEM_WB, ALU_result_MEM_WB ;
wire [4:0]WRITE_ADDR_MEM_WB ;

    PC PC(reset,clk, PC_out1,PC_in);
    IF_ID IF_ID_Reg(PC_4_IF_ID, INSTRUCTION_IF_ID, PC_add4, Instruction,clk,reset);//ok
    ID_EX ID_EX(REGWRITE_ID_EX, MEMTOREG_ID_EX, BRANCH_ID_EX, MEMREAD_ID_EX, MEMWRITE_ID_EX,ALUSRC_ID_EX, REGDST_ID_EX, ALUOP_ID_EX, 
	   PC_4_ID_EX, READ_DATA_1_ID_EX, READ_DATA_2_ID_EX, SE_ID_EX, RD_ID_EX, RT_ID_EX,
	   RegWrite, MemtoReg, Branch, MemRead, MemWrite, RegDST, ALUSrc, ALUOp, 
	   PC_4_IF_ID, Readdata1, Readdata2, signout, INSTRUCTION_IF_ID[15:11], INSTRUCTION_IF_ID[20:16], clk,reset);
    EX_MEM EX_MEM(REGWRITE_EX_MEM, MEMTOREG_EX_MEM, BRANCH_EX_MEM, MEMREAD_EX_MEM, MEMWRITE_EX_MEM,
	   ADD_RESULT_EX_MEM, ALU_RESULT_EX_MEM, READ_DATA_2_EX_MEM, WRITE_ADDR_EX_MEM, ZERO_EX_MEM,
	   clk, reset, REGWRITE_ID_EX, MEMTOREG_ID_EX, BRANCH_ID_EX, MEMREAD_ID_EX, MEMWRITE_ID_EX,
	   ALUresult1, ALUresult, READ_DATA_2_ID_EX, Writereg, Zero);
    MEM_WB MEM_WB(WRITE_ADDR_MEM_WB, ALU_result_MEM_WB, DataMemory_Output_MEM_WB, REGWRITE_MEM_WB, MemToReg_MEM_WB, 
	   WRITE_ADDR_EX_MEM, ALU_RESULT_EX_MEM, Read_data, REGWRITE_EX_MEM, MEMTOREG_EX_MEM, clk,reset);
    regis regis(WRITE_ADDR_MEM_WB,Readdata1,Readdata2,clk,reset,REGWRITE_MEM_WB,clk,WriteData,INSTRUCTION_IF_ID);
    muxleft muxleft(REGDST_ID_EX,RD_ID_EX, RT_ID_EX,Writereg);//ok
    ins ins(PC_out1,Instruction,clk,reset);//ok
    Control Control(RegDST,Jump,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,ALUOp,INSTRUCTION_IF_ID);
    muxmiddle muxmiddle(ALUSRC_ID_EX,ALUIn2,READ_DATA_2_ID_EX,SE_ID_EX);
    signextension signextension(INSTRUCTION_IF_ID[15:0],signout);//ok
    ALUcontrol ALUcontrol(ALUOP_ID_EX,SE_ID_EX[5:0],ALUfunc);//ok
    ALU ALU(ALUfunc,READ_DATA_1_ID_EX,ALUIn2,Zero,ALUresult);//ok
    DataMemory DataMemory(reset, clk, ALU_RESULT_EX_MEM, READ_DATA_2_EX_MEM, Read_data, MEMREAD_EX_MEM, MEMWRITE_EX_MEM);
    muxright muxright(MemToReg_MEM_WB,ALU_result_MEM_WB,DataMemory_Output_MEM_WB,WriteData);
    and a1(MUX_and,BRANCH_EX_MEM,ZERO_EX_MEM);
    Add Add(ALUresult1,PC_4_ID_EX,Instruction3);//ok
    Shiftleft2_32_32 Shiftleft2_32_32(Instruction3,SE_ID_EX);//ok
    Mux_2 Mux_2(PC_in,MUX_and,PC_add4,ADD_RESULT_EX_MEM);//ok
    add4 add4(PC_out1,PC_add4);//ok
endmodule

module add4(PC_out1,PC_add4);
input [31:0] PC_out1;
output [31:0] PC_add4;
    assign PC_add4=PC_out1+4;
endmodule

module Mux_2(PC_in,select,MUX_out_0,Add_result_out);
input select;   
input [31:0] MUX_out_0,Add_result_out;
output [31:0] PC_in;
reg [31:0] PC_in;
	always@(*) begin 
	   if (select==0) PC_in=MUX_out_0;
	   else if (select==1) PC_in=Add_result_out;	 
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

module Add(ALUresult,PC_add4,ShiftLeft2out);
input [31:0] PC_add4,ShiftLeft2out;
output [31:0] ALUresult;
wire [31:0] ALUresult;
    assign ALUresult=PC_add4+ShiftLeft2out;
endmodule

module Mux_1(MUX_out_0,MUX_and,PC_add4,ALUresult);
input MUX_and;
input [31:0] PC_add4,ALUresult;
output [31:0] MUX_out_0;
reg [31:0]  MUX_out_0;
	always@(*) begin 
	   if (MUX_and==0) MUX_out_0=PC_add4;
	   else if (MUX_and==1) MUX_out_0=ALUresult;	 
	end
endmodule

module muxright(MemtoReg,ALUresult,Read_data,WriteData);
input MemtoReg;
input [31:0]ALUresult;
input [31:0]Read_data;
output reg[31:0]WriteData;
    always@(*)//reg write data choose alu or mem ªºmux
    begin
        if(MemtoReg==0) WriteData=ALUresult;//alu
        else WriteData=Read_data;//mem
    end
endmodule

module ALUcontrol(ALUOp,Instruction,ALUfunc);
input [1:0]ALUOp;
wire [5:0]funct;
input [31:0]Instruction;
output reg [3:0]ALUfunc;
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
output reg [31:0] PC_out1;
input reset,Clk;
input [31:0] PC_in;    
    always@(posedge Clk or posedge reset)begin
        if(reset) PC_out1<=32'b0;
        else PC_out1<=PC_in;
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
    always@(*) begin
        if(RegWrite) register[Writereg]=WriteData;     
    end
endmodule

module muxleft(RegDST,RD_ID_EX, RT_ID_EX,Writereg);
input RegDST;
input [4:0]RD_ID_EX, RT_ID_EX;
output reg[4:0]Writereg;
    always@(*)begin//R-type or load mux
        if(RegDST==0)
            Writereg=RT_ID_EX;//load
        else
            Writereg=RD_ID_EX;//R-type
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

module signextension(signin,signout);
input[15:0] signin;
output reg[31:0] signout;
    always@(*) begin
        signout={{16{signin[15]}},signin[15:0]};
    end
endmodule

module ALU(ALUfunc,ALUIn1,ALUIn2,Zero,ALUresult);
input [3:0]ALUfunc;
input [31:0]ALUIn1,ALUIn2;
output reg [31:0]ALUresult;
output reg Zero=0;
    always@(*) begin
        case(ALUfunc)
        4'b0000: ALUresult=ALUIn1&ALUIn2;
        4'b0001: ALUresult=ALUIn1|ALUIn2;
        4'b0010: ALUresult=ALUIn1+ALUIn2;
        4'b0110: 
            begin
            ALUresult=ALUIn1-ALUIn2;
            if(ALUresult==0)
                Zero=1;
            else
                Zero=0;
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
    always@(*) begin
        if(MemWrite) RAM_data[address]=Readdata2;    
    end	
    
endmodule

module EX_MEM(REGWRITE_out, MEMTOREG_out, BRANCH_out, MEMREAD_out, MEMWRITE_out,
	ADD_RESULT_out, ALU_RESULT_out, READ_DATA_2_out, WRITE_ADDR_out, ZERO_out,
	CLK, RST, REGWRITE, MEMTOREG, BRANCH, MEMREAD, MEMWRITE,
	ADD_RESULT, ALU_RESULT, READ_DATA_2, WRITE_ADDR, ZERO);
output reg REGWRITE_out, MEMTOREG_out; //WB
output reg BRANCH_out, MEMREAD_out, MEMWRITE_out; //M
output reg [31:0] ADD_RESULT_out, ALU_RESULT_out, READ_DATA_2_out;
output reg [4:0] WRITE_ADDR_out ;
output reg ZERO_out ;
input CLK, RST;
input REGWRITE, MEMTOREG; //WB
input BRANCH, MEMREAD, MEMWRITE; //M
input [31:0] ADD_RESULT, ALU_RESULT, READ_DATA_2;
input [4:0] WRITE_ADDR ;
input ZERO ;

always@(posedge CLK or posedge RST)
	if(RST)begin
		REGWRITE_out<=1'b0;
		MEMTOREG_out<=1'b0;
		BRANCH_out<=1'b0;
		MEMREAD_out<=1'b0;
		MEMWRITE_out<=1'b0;

		ADD_RESULT_out <= 32'b0 ;
		ALU_RESULT_out <= 32'b0 ;
		READ_DATA_2_out <= 32'b0 ;
		WRITE_ADDR_out <= 5'b0 ;
		ZERO_out <= 1'b0 ;
	end
	else begin
		REGWRITE_out<=REGWRITE;
		MEMTOREG_out<=MEMTOREG;
		BRANCH_out<=BRANCH;
		MEMREAD_out<=MEMREAD;
		MEMWRITE_out<=MEMWRITE;

		ADD_RESULT_out <= ADD_RESULT ;
		ALU_RESULT_out <= ALU_RESULT ;
		READ_DATA_2_out <= READ_DATA_2 ;
		WRITE_ADDR_out <= WRITE_ADDR ;
		ZERO_out <= ZERO ;
	end
endmodule

module MEM_WB (WRITE_ADDR_out, ALU_result_out, READ_DATA_out, WB_RegWrite_out, WB_MemToReg_out, 
	WRITE_ADDR, ALU_result, READ_DATA, WB_RegWrite, WB_MemToReg, CLK, RST);
output reg [31:0] ALU_result_out, READ_DATA_out;
output reg [4:0] WRITE_ADDR_out ;
output reg WB_RegWrite_out, WB_MemToReg_out;
input [31:0] ALU_result, READ_DATA;
input [4:0]  WRITE_ADDR ;
input WB_RegWrite, WB_MemToReg ;
input CLK,RST;

always@(posedge CLK or posedge RST) begin
	if(RST) begin
		WRITE_ADDR_out <= 32'b0;
		ALU_result_out <= 32'b0;
		READ_DATA_out <= 32'b0;
		WB_RegWrite_out <= 1'b0;
		WB_MemToReg_out <= 1'b0;
	end
	else begin
		WRITE_ADDR_out <= WRITE_ADDR;
		ALU_result_out <= ALU_result;
		READ_DATA_out <= READ_DATA;
		WB_RegWrite_out <= WB_RegWrite ;
		WB_MemToReg_out <= WB_MemToReg ;
	end
end
endmodule

module ID_EX(REGWRITE_out, MEMTOREG_out, BRANCH_out, MEMREAD_out, MEMWRITE_out,ALUSRC_out, REGDST_out, ALUOP_out, 
	PC_4_out, READ_DATA_1_out, READ_DATA_2_out, SE_REG_out, RD_out, RT_out,
	REGWRITE, MEMTOREG, BRANCH, MEMREAD, MEMWRITE, REGDST, ALUSRC, ALUOP, 
	PC_4, READ_DATA_1, READ_DATA_2, SE_REG, RD, RT, CLK, RST);
output reg REGWRITE_out, MEMTOREG_out; //WB
output reg BRANCH_out, MEMREAD_out, MEMWRITE_out; //M
output reg ALUSRC_out, REGDST_out;//EX
output reg [1:0] ALUOP_out; //EX
output reg [31:0] PC_4_out, READ_DATA_1_out, READ_DATA_2_out, SE_REG_out;
output reg [4:0] RD_out, RT_out;
input CLK, RST;
input REGWRITE, MEMTOREG; //WB
input BRANCH, MEMREAD, MEMWRITE; //M
input REGDST, ALUSRC ;//EX
input [1:0] ALUOP; //EX
input [31:0] PC_4, READ_DATA_1, READ_DATA_2, SE_REG;
input [4:0] RD, RT;

always@(posedge CLK or posedge RST)
	if(RST)begin
		REGWRITE_out<=1'b0;
		MEMTOREG_out<=1'b0;
		BRANCH_out<=1'b0;
		MEMREAD_out<=1'b0;
		MEMWRITE_out<=1'b0;
		REGDST_out<=1'b0;
		ALUSRC_out<=1'b0;
		ALUOP_out<=2'b00;
		PC_4_out<=32'b0;
		READ_DATA_1_out<=32'b0; 
		READ_DATA_2_out<=32'b0;
		SE_REG_out<=32'b0;
		RD_out<=5'b0; 
		RT_out<=5'b0;
	end
	else begin
		REGWRITE_out<=REGWRITE;
		MEMTOREG_out<=MEMTOREG;
		BRANCH_out<=BRANCH;
		MEMREAD_out<=MEMREAD;
		MEMWRITE_out<=MEMWRITE;
		REGDST_out<=REGDST;
		ALUSRC_out<=ALUSRC;
		ALUOP_out<=ALUOP;
		PC_4_out<=PC_4;
		READ_DATA_1_out<=READ_DATA_1; 
		READ_DATA_2_out<=READ_DATA_2;
		SE_REG_out<=SE_REG;
		RD_out<=RD; 
		RT_out<=RT;
	end
endmodule

module IF_ID (PC_4_out, IM_out, PC_4, IM, CLK, RST);
output reg [31:0] PC_4_out, IM_out;
input [31:0] PC_4, IM;
input CLK,RST;

    always@(posedge CLK or posedge RST) begin
	   if(RST) begin
		  PC_4_out <= 32'b0;
		  IM_out <= 32'b0;
	   end
	   else begin
		  PC_4_out <= PC_4;
		  IM_out <= IM;
	   end
    end
endmodule