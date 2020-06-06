// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;


//=========================================
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	

	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule

module RISCV_Pipeline(	clk,
						rst_n,
//----------I cache interface-------		
						ICACHE_ren,
						ICACHE_wen,
						ICACHE_addr,  
						ICACHE_wdata,   
						ICACHE_stall,  
						ICACHE_rdata,   
//----------D cache interface-------
						DCACHE_ren,    
						DCACHE_wen,    
						DCACHE_addr,   
						DCACHE_wdata,  
						DCACHE_stall,   
						DCACHE_rdata,   
					);
//----------Input Output Declaration
input clk,rst_n;
input DCACHE_stall,ICACHE_stall;
input [31:0] DCACHE_rdata,ICACHE_rdata;
output DCACHE_ren,ICACHE_ren;
output DCACHE_wen,ICACHE_wen;
output [29:0] DCACHE_addr,ICACHE_addr;
output [31:0] DCACHE_wdata,ICACHE_wdata;
endmodule
//Submodules
//-----------README------------
//Status: Finish = Not debug
//Status: Correct
//Status: None = Still doing
//-----------------------------

module MainControl();
endmodule

module ALUControl();
endmodule

module ImmGenerator();
endmodule

module FowardingUnit();
endmodule

module ALU();
endmodule

module MainRegister();
endmodule

module PC();
endmodule

module PCRegister();
endmodule

//pipeline registers
//Status : Finish
module IFID(clk,rst_n,PC,Inst,nxt_PC,nxt_Inst);
	input clk;
	input rst_n;
	output reg [31:0] PC;
	output reg [31:0] Inst;
	input [31:0] nxt_PC;
	input [31:0] nxt_Inst;

	reg [31:0] nxtr_PC;
	reg [31:0] nxtr_Inst;

	always@(*) begin
		nxtr_PC = rst_n ? 32'b0 : PC;
		nxtr_Inst = rst_n ? 32'b0 : Inst;
	end
	always@(posedge clk) begin
		PC <= nxtr_PC;
		Inst <= nxtr_Inst;
	end
endmodule
//Status : Finish
module IDEX(clk,rst_n,Jtype,RS1_D,RS2IMM_D,RS1_A,RS2_A,RD_A,ALUOp,MemtoReg,RegWrite,MemRead,MemWrite,ALUsrc,Jctrl;
nxt_Jtype,nxt_RS1_D,nxt_RS2IMM_D,nxt_RS1_A,nxt_RS2_A,nxt_RD_A,nxt_ALUOp,nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_ALUsrc,nxt_Jctrl);
	input clk,rst_n;
	output reg MemtoReg,RegWrite,MemRead,MemWrite,ALUsrc,Jctrl;
	output reg [3:0] ALUOp;
	output reg [4:0] RS1_A,RS2_A,RD_A;
	output reg [31:0] RS1_D,RS2IMM_D,Jtype;
	input nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_ALUsrc,nxt_Jctrl;
	input [3:0] nxt_ALUOp;
	input [4:0] nxt_RS1_A,nxt_RS2_A,nxt_RD_A;
	input [31:0] nxt_RS1_D,nxt_RS2IMM_D,nxt_Jtype;

	reg nxtr_MemtoReg,nxtr_RegWrite,nxtr_MemRead,nxtr_MemWrite,nxtr_ALUsrc,nxtr_Jctrl;
	reg [3:0] nxtr_ALUOp;
	reg [4:0] nxtr_RS1_A,nxtr_RS2_A,nxtr_RD_A;
	reg [31:0] nxtr_RS1_D,nxtr_RS2IMM_D,nxtr_Jtype;

	always@(*) begin
		nxtr_Jtype = rst_n ? 32'b0 : nxt_Jtype;
		nxtr_RS1_D = rst_n ? 32'b0 : nxt_RS1_D;
		nxtr_RS2IMM_D = rst_n ? 32'b0 : nxt_RS2IMM_D;
		nxtr_RS1_A = rst_n ? 5'b0 : nxt_RS1_A;
		nxtr_RS2_A = rst_n ? 5'b0 : nxt_RS2_A;
		nxtr_RD_A = rst_n ? 5'b0 : nxt_RD_A;
		nxtr_ALUOp = rst_n ? 4'b0 : nxt_ALUOp;
		nxtr_MemtoReg = rst_n ? 1'b0 : nxt_MemtoReg;
		nxtr_RegWrite = rst_n ? 1'b0 : nxt_RegWrite;
		nxtr_MemRead = rst_n ? 1'b0 : nxt_MemRead;
		nxtr_MemWrite = rst_n ? 1'b0 : nxt_MemWrite;
		nxtr_ALUsrc = rst_n ? 1'b0 : nxt_ALUsrc;
		nxtr_Jctrl = rst_n ? 1'b0 : nxt_Jctrl;
	end
	always@(posedge clk) begin
		{Jtype,RS1_D,RS2IMM_D,RS1_A,RS2_A,RD_A,ALUOp,MemtoReg,RegWrite,MemRead,MemWrite,ALUsrc,Jctrl} <= {nxtr_Jtype,nxtr_RS1_D,nxtr_RS2IMM_D,nxtr_RS1_A,nxtr_RS2_A,nxtr_RD_A,nxtr_ALUOp,nxtr_MemtoReg,nxtr_RegWrite,nxtr_MemRead,nxtr_MemWrite,nxtr_ALUsrc,nxtr_Jctrl};	
	end
endmodule
//Status : Finish
module EXMEM(clk,rst_n,Jtype,ALU,RS2IMM,RD_A,MemtoReg,RegWrite,MemRead,MemWrite,Jctrl,
nxt_Jtype,nxt_ALU,nxt_RS2IMM,nxt_RD_A,nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_Jctrl);
	input clk,rst_n;
	output reg [31:0] Jtype,ALU,RS2IMM;
	output reg [4:0] RD_A;
	output reg MemtoReg,RegWrite,MemRead,MemWrite,Jctrl;
	input [31:0] nxt_Jtype,nxt_ALU,nxt_RS2IMM;
	input [4:0] nxt_RD_A;
	input nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_Jctrl;

	reg [31:0] nxtr_Jtype,nxtr_ALU,nxtr_RS2IMM;
	reg [4:0] nxtr_RD_A;
	reg nxtr_MemtoReg,nxtr_RegWrite,nxtr_MemRead,nxtr_MemWrite,nxtr_Jctrl;

	always@(*) begin
		nxtr_Jtype = rst_n ? 32'b0 : nxt_Jtype;
		nxtr_ALU = rst_n ? 32'b0 : nxt_ALU;
		nxtr_RS2IMM = rst_n ? 32'b0 : nxt_RS2IMM;
		nxtr_RD_A = rst_n ? 5'b0 : nxt_RD_A;
		nxtr_MemtoReg = rst_n ? 1'b0 : nxt_MemtoReg;
		nxtr_RegWrite = rst_n ? 1'b0 : nxt_RegWrite;
		nxtr_MemRead = rst_n ? 1'b0 : nxt_MemRead;
		nxtr_MemWrite = rst_n ? 1'b0 : nxt_MemWrite;
		nxtr_Jctrl = rst_n ? 1'b0 : nxt_Jctrl;
	end

	always@(posedge clk) begin
		Jtype <= nxtr_Jtype;
		ALU <= nxtr_ALU;
		RS2IMM <= nxtr_RS2IMM;
		RD_A <= nxtr_RD_A;
		MemtoReg <= nxtr_MemtoReg;
		RegWrite <= nxtr_RegWrite;
		MemRead <= nxtr_MemRead;
		MemWrite <= nxtr_MemWrite;
		Jctrl <= nxtr_Jctrl;
	end

endmodule
//Status : Finish
module MEMWB(clk,rst_n,Jtype,ALU,MEM,RD_A,MemtoReg,RegWrite,Jctrl,nxt_Jtype,nxt_ALU,nxt_MEM,nxt_RD_A,nxt_MemtoReg,nxt_RegWrite,nxt_Jctrl);
	input clk,rst_n;
	output reg [31:0] Jtype,ALU,MEM;
	output reg [4:0] RD_A;
	output reg MemtoReg,RegWrite,Jctrl;
	input [31:0] nxt_Jtype,nxt_ALU,nxt_MEM;
	input [4:0] nxt_RD_A;
	input nxt_MemtoReg,nxt_RegWrite,nxt_Jctrl;

	reg [31:0] nxtr_Jtype,nxtr_ALU,nxtr_MEM;
	reg [4:0] nxtr_RD_A;
	reg nxtr_MemtoReg,nxtr_RegWrite,nxtr_Jctrl;

	always@(*)begin
		nxtr_Jtype = rst_n ? 32'b0 : nxt_Jtype;
		nxtr_ALU = rst_n ? 32'b0 : nxt_ALU; 
		nxtr_MEM = rst_n ? 32'b0 : nxt_MEM;
		nxtr_RD_A = rst_n ? 5'b0 : nxt_RD_A;
		nxtr_MemtoReg = rst_n ? 1'b0 : nxt_MemtoReg;
		nxtr_RegWrite = rst_n ? 1'b0 : nxt_RegWrite;
		nxtr_Jctrl = rst_n ? 1'b0 : nxt_Jctrl;
	end

	always@(posedge clk)begin
		Jtype <= nxtr_Jtype;
		ALU <= nxtr_ALU; 
		MEM <= nxtr_MEM;
		RD_A <= nxtr_RD_A;
		MemtoReg <= nxtr_MemtoReg;
		RegWrite <= nxtr_RegWrite;
		Jctrl <= nxtr_Jctrl;
	end

endmodule

