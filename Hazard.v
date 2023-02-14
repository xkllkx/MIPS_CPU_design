`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/01/11 17:34:51
// Design Name: 
// Module Name: Hazard
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

module Hazard(PC_OUT, CLK, RST) ;
	output [31:0]PC_OUT ;
	input CLK, RST ;
	wire [31:0]PC_OUT, PC_IN, INSTRUCTION, READ_DATA_1, READ_DATA_2, SE_Out, ALU_IN1, ALU_result, DataMemory_Output, WRITE_DATA,
		PC_4, SFL32_Output, PC_Branch, branch_result ;  // 32bits
	wire [27:0]SFL28_Output ;
	wire [4:0]WRITE_ADDR ; // 5bits
	wire [3:0]ALU_CONTROL ; // 4bits
	wire [1:0]ALUOP ; // 2bits 
	wire REGDST, ALUSRC, MEMTOREG, REGWRITE, MEMREAD, MEMWRITE, BRANCH, ZERO, branch_select ;  // 1bits

//////////////////////////////////////////////////////////////////////////////////////////////////////

	wire [31:0]INSTRUCTION_IF_ID, PC_4_IF_ID ;

	wire REGWRITE_ID_EX, MEMTOREG_ID_EX, BRANCH_ID_EX, MEMREAD_ID_EX, MEMWRITE_ID_EX, ALUSRC_ID_EX, REGDST_ID_EX ; 
	wire [1:0]ALUOP_ID_EX ;
	wire [31:0]PC_4_ID_EX, READ_DATA_1_ID_EX, READ_DATA_2_ID_EX, SE_ID_EX;
	wire [4:0]RD_ID_EX, RT_ID_EX ;


	wire REGWRITE_EX_MEM, MEMTOREG_EX_MEM, BRANCH_EX_MEM, MEMREAD_EX_MEM, MEMWRITE_EX_MEM, ZERO_EX_MEM ;
	wire [31:0] ADD_RESULT_EX_MEM, ALU_RESULT_EX_MEM, READ_DATA_2_EX_MEM ;
	wire [4:0] WRITE_ADDR_EX_MEM ;

	wire REGWRITE_MEM_WB, MemToReg_MEM_WB ;
	wire [31:0]DataMemory_Output_MEM_WB, ALU_result_MEM_WB ;
	wire [4:0]WRITE_ADDR_MEM_WB ;


	wire PCWrite ;
	PC Program_Counter(.PC_OUT(PC_OUT), .PC_IN(PC_IN), .PCWrite(PCWrite), .CLK(CLK), .RST(RST));
//	InstructionMemory IM(.INSTRUCTION(INSTRUCTION), .ADDR(PC_OUT), .RST(RST)) ;
    ins ins(PC_OUT,INSTRUCTION,CLK,RST);
	wire [31:0]constant  ;
	assign constant = 32'd4 ;

	Add Add_1(.out(PC_4), .in_0(PC_OUT), .in_1(constant) ) ;

	wire IF_ID_WRTIE ;
	IF_ID IF_ID_Reg(.PC_4_out(PC_4_IF_ID), .IM_out(INSTRUCTION_IF_ID), .PC_4(PC_4), .IM(INSTRUCTION), .IF_ID_WRTIE(IF_ID_WRTIE), .CLK(CLK), .RST(RST));
    
	wire ctr ; 
    Control Control ( REGDST, ALUSRC, MEMTOREG, REGWRITE, MEMREAD, MEMWRITE, BRANCH, ALUOP,INSTRUCTION_IF_ID, ctr ) ;  
	Register_file RF(.READ_DATA_1(READ_DATA_1), .READ_DATA_2(READ_DATA_2), .READ_ADDR_1(INSTRUCTION_IF_ID[25:21]), .READ_ADDR_2(INSTRUCTION_IF_ID[20:16]), 
    .WRITE_ADDR(WRITE_ADDR_MEM_WB), .WRITE_DATA(WRITE_DATA), .REGWRITE(REGWRITE_MEM_WB), .CLK(CLK),.RST(RST) );   
 
    signextension signextension(INSTRUCTION_IF_ID,SE_Out);
	
	wire [4:0]RS_ID_EX ;
	ID_EX ID_EX_Reg(.REGWRITE_out(REGWRITE_ID_EX), .MEMTOREG_out(MEMTOREG_ID_EX), .BRANCH_out(BRANCH_ID_EX), .MEMREAD_out(MEMREAD_ID_EX),
	 .MEMWRITE_out(MEMWRITE_ID_EX), .ALUSRC_out(ALUSRC_ID_EX), .REGDST_out(REGDST_ID_EX), .ALUOP_out(ALUOP_ID_EX), 
	 .PC_4_out(PC_4_ID_EX), .READ_DATA_1_out(READ_DATA_1_ID_EX), .READ_DATA_2_out(READ_DATA_2_ID_EX), .SE_REG_out(SE_ID_EX), .RD_out(RD_ID_EX), .RT_out(RT_ID_EX), .RS_out(RS_ID_EX),
	.REGWRITE(REGWRITE), .MEMTOREG(MEMTOREG), .BRANCH(BRANCH), .MEMREAD(MEMREAD), .MEMWRITE(MEMWRITE), .REGDST(REGDST), .ALUSRC(ALUSRC), .ALUOP(ALUOP), 
	.PC_4(PC_4_IF_ID), .READ_DATA_1(READ_DATA_1), .READ_DATA_2(READ_DATA_2), .SE_REG(SE_Out), 
	.RD(INSTRUCTION_IF_ID[15:11]), .RT(INSTRUCTION_IF_ID[20:16]), .RS(INSTRUCTION_IF_ID[25:21]), .CLK(CLK), .RST(RST) );


	Mux_5Bits Mux5_1(.OUT(WRITE_ADDR), .IN0(RT_ID_EX), .IN1(RD_ID_EX), .SEL(REGDST_ID_EX)) ; 
	ALU_Controller ALUCTLER(.ALU_func(ALU_CONTROL), .ALU_OP(ALUOP_ID_EX), .FUNCT(SE_ID_EX[5:0]) ) ; 
	Shift_Left_2_32BitsTo32Bits SFL32_1( .Out(SFL32_Output), .In(SE_ID_EX) ) ;
	Add Add_2(.out(PC_Branch), .in_0(PC_4_ID_EX), .in_1(SFL32_Output) ) ;
	
	wire [1:0]FORWARDA, FORWARDB ; 
	wire [31:0]ALU_IN0 ;
	Mux_32Bits_3input Mux_32Bits_3input_1(.OUT(ALU_IN0), .IN0(READ_DATA_1_ID_EX), .IN1(WRITE_DATA), .IN2(ALU_RESULT_EX_MEM), .SEL(FORWARDA) ) ;   
 	
	wire [31:0]Mux32_1_IN0 ;
 	Mux_32Bits_3input Mux_32Bits_3input_2(.OUT(Mux32_1_IN0), .IN0(READ_DATA_2_ID_EX), .IN1(WRITE_DATA), .IN2(ALU_RESULT_EX_MEM), .SEL(FORWARDB) ) ;   

	
	Mux_32Bits Mux1(.out(ALU_IN1), .In0(Mux32_1_IN0), .In1(SE_ID_EX), .ctr2(ALUSRC_ID_EX) ) ;
		
	//ALU ALU_1(.OUT(ALU_result), .ZERO(ZERO), .IN0(ALU_IN0), .IN1(ALU_IN1), .ALU_CONTROL(ALU_CONTROL) ) ; 
    ALU ALU(ALU_CONTROL,ALU_IN0,ALU_IN1,ZERO,ALU_result);

	Forward Forward(.ForwardA(FORWARDA), .ForwardB(FORWARDB), .RS_in(RS_ID_EX) , .RT_in(RT_ID_EX), .RD_EX_MEM(WRITE_ADDR_EX_MEM), .RD_MEM_WB(WRITE_ADDR_MEM_WB), .Regwrite_EX_MEM(REGWRITE_EX_MEM), .Regwrite_MEM_WB(REGWRITE_MEM_WB) );   
	
	EX_MEM EX_MEM_Reg(.REGWRITE_out(REGWRITE_EX_MEM), .MEMTOREG_out(MEMTOREG_EX_MEM), .BRANCH_out(BRANCH_EX_MEM), .MEMREAD_out(MEMREAD_EX_MEM), .MEMWRITE_out(MEMWRITE_EX_MEM),
	.ADD_RESULT_out(ADD_RESULT_EX_MEM), .ALU_RESULT_out(ALU_RESULT_EX_MEM), .READ_DATA_2_out(READ_DATA_2_EX_MEM), .WRITE_ADDR_out(WRITE_ADDR_EX_MEM), .ZERO_out(ZERO_EX_MEM),
	.CLK(CLK), .RST(RST), .REGWRITE(REGWRITE_ID_EX), .MEMTOREG(MEMTOREG_ID_EX), .BRANCH(BRANCH_ID_EX), .MEMREAD(MEMREAD_ID_EX), .MEMWRITE(MEMWRITE_ID_EX),
	.ADD_RESULT(PC_Branch), .ALU_RESULT(ALU_result), .READ_DATA_2(READ_DATA_2_ID_EX), .WRITE_ADDR(WRITE_ADDR), .ZERO(ZERO));


	and and1(branch_select, BRANCH_EX_MEM, ZERO_EX_MEM) ;

    DataMemory DataMemory(RST, CLK,ALU_RESULT_EX_MEM, READ_DATA_2_EX_MEM,MEMREAD_EX_MEM, DataMemory_Output, MEMWRITE_EX_MEM);
	MEM_WB MEM_WB_Reg(.WRITE_ADDR_out(WRITE_ADDR_MEM_WB), .ALU_result_out(ALU_result_MEM_WB), .READ_DATA_out(DataMemory_Output_MEM_WB), .WB_RegWrite_out(REGWRITE_MEM_WB), .WB_MemToReg_out(MemToReg_MEM_WB), 
	.WRITE_ADDR(WRITE_ADDR_EX_MEM), .ALU_result(ALU_RESULT_EX_MEM), .READ_DATA(DataMemory_Output), .WB_RegWrite(REGWRITE_EX_MEM), .WB_MemToReg(MEMTOREG_EX_MEM), .CLK(CLK), .RST(RST));

	Mux_32Bits Mux2(.out(WRITE_DATA), .In0(DataMemory_Output_MEM_WB), .In1(ALU_result_MEM_WB), .ctr2(MemToReg_MEM_WB) ) ; 

	Mux_32Bits Mux3(.out(PC_IN), .In0(PC_4), .In1(ADD_RESULT_EX_MEM), .ctr2(branch_select) ) ;

	Hazard_d Hazard_d(.PCWRITE(PCWrite), .IF_ID_WRITE(IF_ID_WRTIE), .CTRL_SEL(ctr), 
		.RS_IF_ID(INSTRUCTION_IF_ID[25:21]), .RT_IF_ID(INSTRUCTION_IF_ID[20:16]), .RT_ID_EX(RT_ID_EX), .MEMREAD_ID_EX(MEMREAD_ID_EX) );

endmodule 

module regis(Writereg,Readdata1,Readdata2,clk,reset,RegWrite,WriteData,Instruction);
input  clk,reset,RegWrite;
output  [31:0] Readdata1,Readdata2;
input [31:0]Instruction;
input [4:0]Writereg;
input[31:0] WriteData;
reg [31:0] register[31:0];
integer i=0;
initial
begin
    for(i=0;i<32;i=i+1)begin
            register[i]=32'd0;
     end
end

assign Readdata1=register[Instruction[25:21]]; 
assign Readdata2=register[Instruction[20:16]];

always@(*)begin
 if(RegWrite==1)
        register[Writereg]=WriteData;
end
endmodule


module signextension(Instruction,signout);
    input [31:0]Instruction;
    wire[15:0] signin;
    output reg[31:0] signout;
    assign signin=Instruction[15:0];
    always@(*)
    begin
        signout={{16{signin[15]}},signin[15:0]};
    end
endmodule

module Add(out, in_0, in_1) ;

	output [31:0]out ;
	input [31:0]in_0, in_1 ; 

	assign out= in_0 + in_1 ;

endmodule



module ALU1(OUT, ZERO, IN0, IN1, ALU_CONTROL) ; 

	output reg signed [31:0]OUT ;
	output ZERO ;
	input signed [31:0]IN0, IN1 ;
	input [3:0]ALU_CONTROL ;

	always @(IN0 or IN1 or ALU_CONTROL) begin
		case(ALU_CONTROL) 
		4'b0000 : OUT = IN0 & IN1 ;
		4'b0001 : OUT = IN0 | IN1 ;
		4'b0010 : OUT = IN0 + IN1 ;
		4'b0110 : OUT = IN0 - IN1 ;
		4'b0111 : OUT = ( IN0 < IN1 ) ? 1 : 0 ; 
		4'b1100 : OUT = ~(IN0 | IN1) ; 
		default : OUT = 32'd0 ; 
		endcase
	end

	assign ZERO = ( OUT == 0 ) ; 

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

module ALU_Controller( ALU_func, ALU_OP, FUNCT ) ; 

	output reg [3:0]ALU_func ; 
	input [1:0] ALU_OP ;
	input [5:0] FUNCT; 

	always @(ALU_OP or FUNCT) begin
		if ( ALU_OP == 2'b00 )       // add
			ALU_func = 4'b0010 ;
		else if ( ALU_OP == 2'b01 )  // subtract
			ALU_func = 4'b0110 ;
		else if ( ALU_OP == 2'b10 && FUNCT == 6'b100000 ) // add
			ALU_func = 4'b0010 ; 
		else if ( ALU_OP == 2'b10 && FUNCT == 6'b100010 ) // subtract
			ALU_func = 4'b0110 ; 
		else if ( ALU_OP == 2'b10 && FUNCT == 6'b100100 ) // AND
			ALU_func = 4'b0000 ; 
		else if ( ALU_OP == 2'b10 && FUNCT == 6'b100101 ) // OR
			ALU_func = 4'b0001 ; 
		else if ( ALU_OP == 2'b10 && FUNCT == 6'b101010 ) // set on less than
			ALU_func = 4'b0111 ; 
		else  // Error !!! 
			ALU_func = 4'b1111 ; 
	end

endmodule

//OUT = ( IN0 < IN1 ) ? 1 : 0 ; 



module Control ( REGDST, ALUSRC, MEMTOREG, REGWRITE, MEMREAD, MEMWRITE, BRANCH, ALUOP,INSTRUCTION_IF_ID, ctr ) ;

	output reg REGDST, ALUSRC, MEMTOREG, REGWRITE, MEMREAD, MEMWRITE, BRANCH ; 
	output reg [1:0]ALUOP ;
	input [31:0]INSTRUCTION_IF_ID ;
	input ctr ; 


	always @(ctr,INSTRUCTION_IF_ID[31:26]) begin
		case ( {ctr,INSTRUCTION_IF_ID[31:26]} )    
			7'b1000000 : begin  // R-Format
				REGDST = 1'b1 ;
				ALUSRC = 1'b0 ;
				MEMTOREG = 1'b1 ;
				REGWRITE = 1'b1 ;
				MEMREAD = 1'b0 ;
				MEMWRITE = 1'b0 ;
				BRANCH = 1'b0 ;
				ALUOP = 2'b10 ;
			end
			7'b1100011 : begin  // lw
				REGDST = 1'b0 ;
				ALUSRC = 1'b1 ;
				MEMTOREG = 1'b0 ;
				REGWRITE = 1'b1 ;
				MEMREAD = 1'b1 ;
				MEMWRITE = 1'b0 ;
				BRANCH = 1'b0 ;
				ALUOP = 2'b00 ;
			end
			7'b1101011 : begin  // sw
				REGDST = 1'b0 ;   // X
				ALUSRC = 1'b1 ;
				MEMTOREG = 1'b1 ; // X
				REGWRITE = 1'b0 ;
				MEMREAD = 1'b0 ;
				MEMWRITE = 1'b1 ;
				BRANCH = 1'b0 ;
				ALUOP = 2'b00 ;
			end
			7'b1001000 : begin  // addi
				REGDST = 1'b0 ;  
				ALUSRC = 1'b1 ;
				MEMTOREG = 1'b1 ; 
				REGWRITE = 1'b1 ;
				MEMREAD = 1'b0 ;
				MEMWRITE = 1'b1 ;
				BRANCH = 1'b0 ;
				ALUOP = 2'b00 ;  
			end 

			7'b1000100 : begin  // beq
				REGDST = 1'b0 ; // X
				ALUSRC = 1'b0 ;
				MEMTOREG = 1'b1 ; // X
				REGWRITE = 1'b0 ;
				MEMREAD = 1'b0 ;
				MEMWRITE = 1'b0 ;
				BRANCH = 1'b1 ;
				ALUOP = 2'b01 ;
			end
			default : begin  //  Sel == 1'b0  !!   or    Error !!! 
				REGDST = 1'b0 ; // X
				ALUSRC = 1'b0 ; // X
				MEMTOREG = 1'b0 ; // X
				REGWRITE = 1'b0 ; // X
				MEMREAD = 1'b0 ; // X
				MEMWRITE = 1'b0 ; // X
				BRANCH = 1'b0 ; // X
				ALUOP = 2'b00 ; // X
			end
		endcase
	end

endmodule



module Controller ( RegDST, ALUsrc, Memtoreg, RegWrite, MemRead, MemWrite, Branch, ALUOp, INSTRUCTION_IF_ID, ctr ) ;

	output reg RegDST, ALUsrc, Memtoreg, RegWrite, MemRead, MemWrite, Branch;
	output reg [1:0] ALUOp ;
	input [31:0] INSTRUCTION_IF_ID;
	input ctr  ; 


	always @(ctr,INSTRUCTION_IF_ID[31:26]) begin
		case ( {ctr,INSTRUCTION_IF_ID[31:26]} )    
			7'b1000000 : begin  // R-Format
				RegDST = 1'b1 ;
				ALUsrc = 1'b0 ;
				Memtoreg = 1'b1 ;
				RegWrite = 1'b1 ;
				MemRead = 1'b0 ;
				MemWrite = 1'b0 ;
				Branch = 1'b0 ;
				ALUOp = 2'b10 ;
			end
			7'b1100011 : begin  // lw
				RegDST = 1'b0 ;
				ALUsrc= 1'b1 ;
				Memtoreg = 1'b0 ;
				RegWrite = 1'b1 ;
				MemRead = 1'b1 ;
				MemWrite = 1'b0 ;
				Branch = 1'b0 ;
				ALUOp = 2'b00 ;
			end
			7'b1101011 : begin  // sw
				RegDST = 1'b0 ;   // X
				ALUsrc = 1'b1 ;
				Memtoreg = 1'b1 ; // X
				RegWrite = 1'b0 ;
				MemRead = 1'b0 ;
				MemWrite = 1'b1 ;
				Branch = 1'b0 ;
				ALUOp = 2'b00 ;
			end
			7'b1001000 : begin  // addi
				RegDST = 1'b0 ;  
				ALUsrc = 1'b1 ;
				Memtoreg = 1'b1 ; 
				RegWrite = 1'b1 ;
				MemRead = 1'b0 ;
				MemWrite = 1'b1 ;
				Branch = 1'b0 ;
				ALUOp = 2'b00 ;  
			end 

			7'b1000100 : begin  // beq
				RegDST = 1'b0 ; // X
				ALUsrc = 1'b0 ;
				Memtoreg = 1'b1 ; // X
				RegWrite = 1'b0 ;
				MemRead = 1'b0 ;
				MemWrite = 1'b0 ;
				Branch = 1'b1 ;
				ALUOp = 2'b01 ;
			end
			default : begin  //  Sel == 1'b0  !!   or    Error !!! 
                RegDST = 1'b0 ; // X
			    ALUsrc = 1'b0 ; // X
				Memtoreg = 1'b0 ; // X
				RegWrite = 1'b0 ; // X
				MemRead = 1'b0 ; // X
				MemWrite = 1'b0 ; // X
				Branch = 1'b0 ; // X
				ALUOp = 2'b00 ; // X
			end
		endcase
	end

endmodule



module DataMemory(reset, clk, ALUresult, WriteData, Read_data, MemRead, MemWrite);
	input reset, clk;
	input [31:0] ALUresult, WriteData;
	input MemRead, MemWrite;
//	output reg [31:0] Read_data;
	wire [5:0]address=ALUresult[5:0];

	reg [31:0] RAM_data[63:0];
//	assign Read_data=RAM_data[address];
	output  [31:0] Read_data;
	assign Read_data = MemRead? RAM_data[address]: 32'h00000000;

    integer i;
    always@(negedge clk or posedge reset)
    begin
        if(reset)
            begin
            for(i=0;i<64;i=i+1)begin
                 RAM_data[i] <= 32'h00000000;
                 end
            end
        else if(MemWrite)
            RAM_data[address]<=WriteData;
        else
            RAM_data[address]<=RAM_data[address];       
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




module Forward(ForwardA, ForwardB, RS_in , RT_in, RD_EX_MEM, RD_MEM_WB, Regwrite_EX_MEM, Regwrite_MEM_WB);

	input [4:0] RS_in , RT_in, RD_EX_MEM, RD_MEM_WB; 
	input Regwrite_EX_MEM, Regwrite_MEM_WB;
    output reg [1:0] ForwardA, ForwardB;


	always@(*)
	begin

		ForwardA=2'b00;
		ForwardB=2'b00;

		// EX_MEM
		if(Regwrite_EX_MEM && (RD_EX_MEM!=0) && (RD_EX_MEM==RS_in))
			ForwardA=2'b10;
		if(Regwrite_EX_MEM && (RD_EX_MEM!=0) && (RD_EX_MEM==RT_in))
			ForwardB=2'b10;

		// MEM_WB
		if ( Regwrite_MEM_WB && (RD_MEM_WB!=0) && (RD_MEM_WB==RS_in) && ( ~(  (Regwrite_EX_MEM) && (RD_EX_MEM!=0) && (RD_EX_MEM == RS_in) ) ) )
			ForwardA=2'b01;
		if ( Regwrite_MEM_WB && (RD_MEM_WB!=0) && (RD_MEM_WB==RT_in) && ( ~(  (Regwrite_EX_MEM) && (RD_EX_MEM!=0) && (RD_EX_MEM == RT_in) ) ) )
			ForwardB=2'b01;

	end

endmodule




module Hazard_d(PCWRITE, IF_ID_WRITE, CTRL_SEL, RS_IF_ID, RT_IF_ID, RT_ID_EX, MEMREAD_ID_EX);

	output reg PCWRITE, IF_ID_WRITE, CTRL_SEL;

	input [4:0] RS_IF_ID, RT_IF_ID, RT_ID_EX;
	input MEMREAD_ID_EX;

	always@(*)begin

		if( MEMREAD_ID_EX && ((RT_ID_EX==RS_IF_ID) || (RT_ID_EX==RT_IF_ID)) )
		begin
			PCWRITE=1'b0;
			IF_ID_WRITE=1'b0;
			CTRL_SEL=1'b0;	
		end
		
		else begin
			PCWRITE=1'b1;
			IF_ID_WRITE=1'b1;
			CTRL_SEL=1'b1;
		end

	end

endmodule


module ID_EX(REGWRITE_out, MEMTOREG_out, BRANCH_out, MEMREAD_out, MEMWRITE_out,ALUSRC_out, REGDST_out, ALUOP_out, 
	PC_4_out, READ_DATA_1_out, READ_DATA_2_out, SE_REG_out, RD_out, RT_out, RS_out,
	REGWRITE, MEMTOREG, BRANCH, MEMREAD, MEMWRITE, REGDST, ALUSRC, ALUOP, 
	PC_4, READ_DATA_1, READ_DATA_2, SE_REG, RD, RT, RS, CLK, RST);

output reg REGWRITE_out, MEMTOREG_out; //WB
output reg BRANCH_out, MEMREAD_out, MEMWRITE_out; //M
output reg ALUSRC_out, REGDST_out;//EX
output reg [1:0] ALUOP_out; //EX
output reg [31:0] PC_4_out, READ_DATA_1_out, READ_DATA_2_out, SE_REG_out;
output reg [4:0] RD_out, RT_out, RS_out;

input CLK, RST;
input REGWRITE, MEMTOREG; //WB
input BRANCH, MEMREAD, MEMWRITE; //M
input REGDST, ALUSRC ;//EX
input [1:0] ALUOP; //EX
input [31:0] PC_4, READ_DATA_1, READ_DATA_2, SE_REG;
input [4:0] RD, RT, RS;




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
		RS_out<=5'b0;
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
		RS_out<=RS;
	end
endmodule

module IF_ID (PC_4_out, IM_out, PC_4, IM, IF_ID_WRTIE, CLK, RST);

output reg [31:0] PC_4_out, IM_out;
input [31:0] PC_4, IM;
input CLK,RST, IF_ID_WRTIE;

always@(posedge CLK or posedge RST) begin
	if(RST) begin
		PC_4_out <= 32'b0;
		IM_out <= 32'b0;
	end
	else begin
		if (IF_ID_WRTIE == 1'b0) begin
			PC_4_out <= PC_4_out ;
			IM_out <= IM_out ;
		end
		else begin
			PC_4_out <= PC_4;
			IM_out <= IM;
		end
	end
end

endmodule

module ins(PC_out1,Instruction,clk,reset);
 
input clk,reset;
input [31:0]PC_out1;
output [31:0]Instruction;  
reg [31:0]Instr_memory [31:0]; 
integer i;
assign Instruction=Instr_memory[PC_out1/4];  //除以4才是實際位置

//10101100 00001000 00000000 00001000
initial
begin
        Instr_memory[0] <= 32'b001000_00000_01000_0000000000000010;
		Instr_memory[1] <= 32'b001000_00000_01001_0000000000000100;
		Instr_memory[2] <= 32'b001000_00000_01010_0000000000000001;
		Instr_memory[3] <= 32'b001000_00000_01011_0000000000000011;
		Instr_memory[4] <= 32'b000000_01000_01001_00011_00000_100000;
		
		Instr_memory[5] <= 32'b000000_00011_01010_01100_00000_100101;
		
    	Instr_memory[6] <= 32'b000000_01011_00011_01101_00000_100010;
		
		Instr_memory[7] <= 32'b000000_01010_00011_01111_00000_100100;
		Instr_memory[8] <= 32'b101011_00011_01110_0000000000011110;


	for(i=2;i<256;i=i+1)begin
                 Instr_memory[i] = 32'h00000000;
            end
end
endmodule

module InstructionMemory(INSTRUCTION, ADDR, RST) ;

	output [31:0]INSTRUCTION ;
	input [31:0]ADDR ;
	input RST ;

	reg [31:0]Instr_memory[31:0] ;
	
	//assign INSTRUCTION = memory[ADDR] ; 

//////////////////////////////////////////////////////////////////////////////////////////////////

	wire [31:0] shifted_read_addr;
	assign shifted_read_addr = { 2'b00,ADDR[31:2] };
	assign INSTRUCTION = Instr_memory[shifted_read_addr];


	integer k;
	// I-MEM in this case is addressed by word, not by byte
	

	always @(posedge RST)
	begin
	
		for (k=9; k<64; k=k+1) begin  // here Ou changes k=0 to k=16
			Instr_memory[k] <= 32'b0;
		end
	
        Instr_memory[0] <= 32'b001000_00000_01000_0000000000000010;
		Instr_memory[1] <= 32'b001000_00000_01001_0000000000000100;
		Instr_memory[2] <= 32'b001000_00000_01010_0000000000000001;
		Instr_memory[3] <= 32'b001000_00000_01011_0000000000000011;
		Instr_memory[4] <= 32'b000000_01000_01001_00011_00000_100000;
		
		Instr_memory[5] <= 32'b000000_00011_01010_01100_00000_100101;
		
    	Instr_memory[6] <= 32'b000000_01011_00011_01101_00000_100010;
		
		Instr_memory[7] <= 32'b000000_01010_00011_01111_00000_100100;
		Instr_memory[8] <= 32'b101011_00011_01110_0000000000011110;

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



module Mux_5Bits(OUT, IN0, IN1, SEL) ; 

	output [4:0]OUT ;
	input [4:0]IN0, IN1 ;
	input SEL;

	assign OUT = (SEL) ? IN1 : IN0 ;

endmodule



module Mux_32Bits(out, In0, In1, ctr2) ; 

	output [31:0] out;
	input [31:0]In0, In1 ;
	input ctr2;

	assign out = ( ctr2) ?In1 : In0 ;

endmodule



module Mux_32Bits_3input(OUT, IN0, IN1, IN2, SEL) ; 

	output reg [31:0]OUT ;
	input [31:0]IN0, IN1 ,IN2;
	input [1:0]SEL;

	always@(*)begin
		if (SEL == 2'b00)
			OUT = IN0;
		else if (SEL == 2'b01)
			OUT = IN1;
		else if (SEL == 2'b10)
			OUT = IN2;
		else   // Error
			OUT = IN0;
	end
endmodule


module PC(PC_OUT, PC_IN, PCWrite, CLK, RST);
output reg [31:0] PC_OUT;
input [31:0] PC_IN;
input PCWrite;
input CLK, RST;

always @(posedge CLK or posedge RST) begin
	if(RST)
		PC_OUT<=1'b0;

	else begin

		if(PCWrite == 1'b1)
			PC_OUT<=PC_IN;
		else 
			PC_OUT<=PC_OUT;

	end
end

endmodule


module Register_file(
READ_DATA_1, READ_DATA_2,READ_ADDR_1, READ_ADDR_2, WRITE_ADDR, WRITE_DATA,
REGWRITE, CLK,RST
);
output [31:0] READ_DATA_1, READ_DATA_2;
input [31:0] WRITE_DATA;
input [4:0] READ_ADDR_1, READ_ADDR_2, WRITE_ADDR;
input REGWRITE, CLK,RST;

reg [31:0] Regfile[31:0];
integer i;


assign READ_DATA_1 =  Regfile[READ_ADDR_1];
assign READ_DATA_2 =  Regfile[READ_ADDR_2];

always@(negedge CLK or posedge RST)
begin
	if(RST)begin
		for(i=0; i<32; i=i+1)
			Regfile[i] <= 32'b0;
	end
	else begin
		if (REGWRITE==1'b1 && WRITE_ADDR!=0)
			Regfile[WRITE_ADDR] <= WRITE_DATA;
		else
			Regfile[WRITE_ADDR] <= Regfile[WRITE_ADDR];
	end

end

endmodule

/*
always@(negedge CLK or posedge RST)
begin
	if(RST)begin
		for(i=0; i<32; i=i+1)
			Regfile[i] <= 32'b0;
	end

	else begin
		if (REGWRITE==1'b1 && WRITE_ADDR!=0)
			Regfile[WRITE_ADDR] <= WRITE_DATA;
	end

end

always@(posedge CLK)
begin
	READ_DATA_1 <= Regfile[READ_ADDR_1];
	READ_DATA_2 <= Regfile[READ_ADDR_2];
end
*/



module Shift_Left_2_26BitsTo28Bits ( Out,In ) ;
	
	output [27:0]Out ;
	input [25:0]In ;

	assign Out = { In[25:0], 2'b00 } ;

endmodule



module Shift_Left_2_32BitsTo32Bits ( Out,In ) ;
	
	output [31:0]Out ;
	input [31:0]In ;

	assign Out = { In[29:0], 2'b00 } ;

endmodule



module Sign_Extend(OUT_32BITS, IN_16BITS) ;

	output reg [31:0]OUT_32BITS ;
	input [15:0]IN_16BITS ;

	always @(IN_16BITS) begin
		if ( IN_16BITS[15] == 1'b0 ) 
			OUT_32BITS = { { 16{1'b0} } ,IN_16BITS} ;
		else 
			OUT_32BITS = { { 16{1'b1} } ,IN_16BITS} ;
	end

endmodule
