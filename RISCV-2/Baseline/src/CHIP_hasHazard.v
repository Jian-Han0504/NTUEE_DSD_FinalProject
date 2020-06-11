// Top module of your design, you cannot modify this module!!
`include "./cache.v"
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
	input clk, rst_n;
	input DCACHE_stall, ICACHE_stall;
	input [31:0] DCACHE_rdata, ICACHE_rdata;
	output DCACHE_ren, ICACHE_ren;
	output DCACHE_wen, ICACHE_wen;
	output [29:0] DCACHE_addr,ICACHE_addr;
	output [31:0] DCACHE_wdata,ICACHE_wdata;
	
	// ============================================
	// ==                  WIRE                  ==
	// ============================================
	wire stall, flush, pcwrite, if_id_write, PC_stall, IF_ID_stall;
	assign stall = ICACHE_stall | DCACHE_stall;
	assign PC_stall = stall | pcwrite;
	assign IF_ID_stall = stall | if_id_write;
	// IF
	wire [31:0] IF_inst;
	wire [31:0] IF_pc;
	wire [31:0] IF_pc_plus4;
	wire [31:0] IF_nxt_pc; // next pc to PC register
	
	// ID
	wire [31:0] ID_inst;
	wire [31:0] ID_pc;
	wire [31:0] ID_pc_plus4;
	wire  [5:0] ctrl_inst; // inst for control unit
	wire  [4:0] alu_ctrl_inst; // inst for ALU control signals
	wire [31:0] ID_immediate;

	wire  [4:0] ID_rs1_addr, ID_rs2_addr; // forwarding to EXE stage
	wire  [4:0] ID_writeb_addr;
	wire [31:0] ID_rs1_data, ID_rs2_data; // output from register file

	// ID Control
	wire ID_jalr, ID_jal, ID_bne, ID_beq, 
		 ID_mem_read, ID_mem_write, ID_mem2reg, 
		 ID_alu_src, ID_reg_write, ID_is_jump;
	wire  [3:0] ID_alu_op;

	// EX
	
	wire [31:0] EX_pc_plus4, nxt_EX_J_StorePC;
	wire  [4:0] EX_rs1_addr, EX_rs2_addr;
	wire [31:0] EX_rs1_data, EX_rs2_data;
	wire [31:0] EX_immediate;
	wire [31:0] EX_Fwdrs2_data, nxt_EX_MemWrt_data;
	wire  [3:0] EX_alu_op;
	wire [31:0] EX_alu_inA, EX_alu_inB;
	wire [31:0] EX_alu_result, nxt_EX_ALU_result;
	wire  [4:0] EX_writeb_addr, nxt_EX_WB_addr;
	wire EX_mem_read, nxt_EX_MemRead, EX_mem_write, nxt_EX_MemWrite, EX_mem2reg, nxt_EX_mem2reg, 
		 EX_alu_src, EX_reg_write, nxt_EX_RegWrite, EX_is_jump,nxt_EX_is_Jump, control_zero;
	assign nxt_EX_J_StorePC = control_zero ? 0 : EX_pc_plus4;
	assign nxt_EX_MemWrt_data = control_zero ? 0 : EX_Fwdrs2_data;
	assign nxt_EX_ALU_result = control_zero ? 0 : EX_alu_result;
	assign nxt_EX_WB_addr = control_zero ? 0 : EX_writeb_addr;
	assign nxt_EX_MemRead  = control_zero ? 0 : EX_mem_read;
	assign nxt_EX_MemWrite = control_zero ? 0 : EX_mem_write;
	assign nxt_EX_mem2reg = control_zero ? 0 : EX_mem2reg;
	assign nxt_EX_RegWrite = control_zero ? 0 : EX_reg_write;
	assign nxt_EX_is_Jump = control_zero ? 0 : EX_is_jump;
	// MEM 
	wire [31:0] ME_pc_plus4;
	wire [31:0] ME_alu_result, ME_memWrt_data;
	wire 		ME_mem_read, ME_mem_write, ME_mem2reg, ME_reg_write, ME_is_jump;
	wire [31:0] ME_memRd_data;
	wire [31:0] ME_writeb_data;
	wire  [4:0] ME_writeb_addr;
	
	// WB
	wire [31:0] WB_pc_plus4;
	wire [31:0] WB_alu_result;
	wire 		WB_mem2reg, WB_reg_write, WB_is_jump;
	wire [31:0] WB_memRd_data;
	wire [31:0] WB_writeb_data;
	wire  [4:0] WB_writeb_addr;
	

	wire 		fwd1A, fwd1B, fwd2A, fwd2B;  // forwarding control

	// ============================================
	// ==               ASSIGNMENT               ==
	// ============================================
	// Basic Outputs
	assign ICACHE_ren = rst_n ? 1'b1 : 1'b0;
	assign ICACHE_wen = 1'b0;
	assign ICACHE_addr = IF_pc[31:2];
	assign ICACHE_wdata = 32'b0;
	assign DCACHE_ren = ME_mem_read;
	assign DCACHE_wen = ME_mem_write;
	assign DCACHE_addr = ME_alu_result[31:2];
	assign DCACHE_wdata = ME_memWrt_data;

	// Endian Little -> Big
	assign IF_inst = {ICACHE_rdata[7:0], ICACHE_rdata[15:8], ICACHE_rdata[23:16], ICACHE_rdata[31:24]};

	// Instruction part
	assign ID_rs1_addr = ID_inst[19:15];
	assign ID_rs2_addr = ID_inst[24:20];
	assign ID_writeb_addr = ID_inst[11:7];

	// RS1 & RS2 mux
	assign EX_alu_inA = fwd1A ? ME_alu_result  :
						fwd2A ? WB_writeb_data : EX_rs1_data;
	assign EX_Fwdrs2_data = fwd1B ? ME_alu_result :
							fwd2B ? WB_writeb_data : EX_rs2_data;
	assign EX_alu_inB = EX_alu_src ? EX_immediate : EX_Fwdrs2_data;
	
	assign ME_memRd_data = DCACHE_rdata;
	assign ME_writeb_data = ME_alu_result;

	// IF/ID
	IFID IFID0 (.clk (clk), 
				.rst_n (rst_n), 
				.isStall (stall),
				.isFlush (flush),
				.nxt_PC (IF_pc),
				.nxt_PCplus4 (IF_pc_plus4),
				.nxt_Inst (IF_inst),
				.PC (ID_pc),
				.PCplus4 (ID_pc_plus4),
				.Inst (ID_inst) 
	);

	// control unit
	assign ctrl_inst = {ID_inst[12], ID_inst[6:2]};
	assign ID_is_jump = ID_jalr | ID_jal;
	MainControl Ctrl (.Inst (ctrl_inst),
					  .Jalr (ID_jalr),
					  .Jal (ID_jal),
					  .BNE (ID_bne),
					  .BEQ (ID_beq),
					  .MemRead (ID_mem_read),
					  .MemWrite (ID_mem_write),
					  .MemtoReg (ID_mem2reg),
					  .ALUSrc (ID_alu_src),
					  .RegWrite (ID_reg_write),
					  .is_Jump (ID_is_jump)
	);

	// ALU control
	assign alu_ctrl_inst = {ID_inst[30], ID_inst[14:12], ID_inst[4]};
	ALUControl ALUCtrl (.Inst (alu_ctrl_inst), .ALUCtrl (ID_alu_op));

	// immediate gen
	ImmGenerator IG0 (.Inst(ID_inst), .Immediate (ID_immediate));

	// register file
	MainRegister MR (.clk(clk),
					.rst_n(rst_n),
					.WB_data (WB_writeb_data),
					.RS1_addr (ID_rs1_addr),
					.RS2_addr (ID_rs2_addr),
					.WB_addr (WB_writeb_addr),
					.RegWrite (WB_reg_write),
					.RS1_data (ID_rs1_data),
					.RS2_data (ID_rs2_data)
	);

	// ID/EXE
	IDEX IDEX0 (.clk (clk),
				.rst_n (rst_n),
				.isStall (stall),
				// pipelined
				.nxt_RS1_data (ID_rs1_data),
				.nxt_RS2_data (ID_rs2_data),
				.nxt_Immediate (ID_immediate),
				.nxt_WB_addr (ID_writeb_addr),
				.nxt_J_StorePC (ID_pc_plus4),
				.nxt_is_Jump (ID_is_jump),
				.RS1_data (EX_rs1_data),
				.RS2_data (EX_rs2_data),
				.Immediate (EX_immediate),
				.WB_addr (EX_writeb_addr),
				.J_StorePC (EX_pc_plus4),
				.is_Jump (EX_is_jump),
				// Control Signals
				.nxt_ALUop (ID_alu_op),
				.nxt_MemtoReg (ID_mem2reg),
				.nxt_RegWrite (ID_reg_write),
				.nxt_MemRead (ID_mem_read),
				.nxt_MemWrite (ID_mem_write),
				.ALUop (EX_alu_op),
				.MemtoReg (EX_mem2reg),
				.RegWrite (EX_reg_write),
				.MemRead (EX_mem_read),
				.MemWrite (EX_mem_write),
				// forwarding
				.nxt_RS1_addr (ID_rs1_addr),
				.nxt_RS2_addr (ID_rs2_addr),
				.nxt_ALUsrc (ID_alu_src),
				.RS1_addr (EX_rs1_addr),
				.RS2_addr (EX_rs2_addr),
				.ALUsrc (EX_alu_src)
	);

	// ============================================
	// ==                   EX                   ==
	// ============================================
	// ALU
	ALU ALU0 (.inA (EX_alu_inA),
			.inB (EX_alu_inB),
			.ctrl (EX_alu_op),
			.out (EX_alu_result)
	);
	
	// EX / MEM
	EXMEM EXME0(.clk (clk),
				.rst_n (rst_n),
				.isStall (stall),
				.nxt_ALU_result (nxt_EX_ALU_result),   // ALU result -> mem write addr
				.nxt_MemWrt_data (nxt_EX_MemWrt_data), // ALU inB -> mem write data
				.nxt_WB_addr (nxt_EX_WB_addr),     // write-back register addr
				.nxt_J_StorePC (nxt_EX_J_StorePC),
				.nxt_is_Jump (nxt_EX_is_Jump),
				.ALU_result (ME_alu_result),
				.MemWrt_data (ME_memWrt_data),
				.WB_addr (ME_writeb_addr),
				.J_StorePC (ME_pc_plus4),
				.is_Jump (ME_is_jump),
				// Control Signals
				.nxt_MemtoReg (nxt_EX_mem2reg),
				.nxt_RegWrite (nxt_EX_RegWrite),
				.nxt_MemRead (nxt_EX_MemRead),
				.nxt_MemWrite (nxt_EX_MemWrite),
				.MemtoReg (ME_mem2reg),
				.RegWrite (ME_reg_write),
				.MemRead (ME_mem_read),
				.MemWrite (ME_mem_write)
	); 

	// ============================================
	// ==                   ME                   ==
	// ============================================
	// Note: Endian Transform defined in EXME module
	MEMWB MEWB0(.clk (clk),
				.rst_n (rst_n),
				.isStall (stall),
				// pipelined
				.nxt_ALU_result (ME_alu_result),
				.nxt_MemRd_data (ME_memRd_data),
				.nxt_WB_addr (ME_writeb_addr),
				.nxt_J_StorePC (ME_pc_plus4),
				.nxt_is_Jump (ME_is_jump),
				.ALU_result (WB_alu_result),
				.MemRd_data (WB_memRd_data),
				.WB_addr (WB_writeb_addr),
				.J_StorePC (WB_pc_plus4),
				.is_Jump (WB_is_jump),
				// Control Signals
				.nxt_MemtoReg (ME_mem2reg),
				.nxt_RegWrite (ME_reg_write),
				.MemtoReg (WB_mem2reg),
				.RegWrite (WB_reg_write)
	);

	// ============================================
	// ==                   WB                   ==
	// ============================================
	assign WB_writeb_data = WB_mem2reg ? WB_memRd_data : WB_alu_result;

	// PC
	wire Reg_eq;
	PC PC0 (.clk (clk),
			.rst_n (rst_n),
			.isStall (stall),
			.Jal (ID_jal),
			.Jalr (ID_jalr),
			.BEQ (ID_beq),
			.BNE (ID_bne),
			.RS1_data (ID_rs1_data),
			.RS2_data (ID_rs2_data),
			.RS1_addr (ID_rs1_addr),
			.Writeb_addr (WB_writeb_addr),
			.Writeb_data (WB_pc_plus4),
			.Writeb_is_Jump (WB_is_jump),
			.Immediate (ID_immediate),
			.ID_PC (ID_pc), // input
			.IF_PC (IF_pc),
			.IF_PCplus4 (IF_pc_plus4),
			.IF_nxt_PC (IF_nxt_pc),
			.isRegEq(Reg_eq)
	);

	// Forwarding Unit
	FowardingUnit FU_DATA  (.RS1_addr (EX_rs1_addr),
							.RS2_addr (EX_rs2_addr),
							.EXME_WB_addr (ME_writeb_addr),
							.MEWB_WB_addr (WB_writeb_addr),
							.RegWrite (EX_reg_write),
							.Fr1A (fwd1A),
							.Fr1B (fwd1B),
							.Fr2A (fwd2A),
							.Fr2B (fwd2B)
	);
	// hazard unit
	hazard_detect HZ(
				.inst_rs1(ID_inst[19:15]),
				.inst_rs2(ID_inst[24:20]),
				.inst_op(ID_inst[6:0]),
				.inst_funct3(ID_inst[14:12]),
				.ex_rd(EX_writeb_addr),
				.wb_rd(WB_writeb_addr),
				.id_memread(ID_mem_read),
				.ex_memread(EX_mem_read),
				.wb_regwrite(WB_reg_write),
				.branch_compare(Reg_eq),
				//branch_true,
				//jal_true,
				//jalr_true,
				.pcwrite(pcwrite),
				.if_id_write(if_id_write),
				.if_flush(flush),
				.control_zero(control_zero)
				);
endmodule

// =========================================
// ==             Sub Modules             ==
// =========================================
module MainControl (input  	   [5:0] Inst, // funct3[0] OP[6:2] (INST[12] INST[6:2])
					output reg       Jalr,
					output reg       Jal,
					output reg 		 BNE,
					output reg 		 BEQ,
					output reg 		 MemRead,
					output reg 		 MemWrite,
					output reg 		 MemtoReg,
					output reg 		 ALUSrc,
					output reg 		 RegWrite,
					output reg 		 is_Jump
	);
	always@(*) begin
		case (Inst[4:0])
			5'b01100: begin // R type
				Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b1;
			end
			5'b00000: begin // lw
				Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'b1; MemWrite = 1'b0; MemtoReg = 1'b1; ALUSrc = 1'b1; RegWrite=1'b1;
			end
			5'b01000: begin // sw
				Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'b0; MemWrite = 1'b1; MemtoReg = 1'bx; ALUSrc = 1'b1; RegWrite=1'b0;
			end
			5'b11000: begin // beq bne
				Jalr = 1'b0; Jal = 1'b0; BEQ = ~Inst[5]; BNE = Inst[5]; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'bx; ALUSrc = 1'b0; RegWrite=1'b0;
			end
			5'b00100: begin // I type
				Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b1; RegWrite=1'b1;
			end
			5'b11011: begin // jal
				Jalr = 1'b0; Jal = 1'b1; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'bx; ALUSrc = 1'bx; RegWrite=1'b1;
			end
			5'b11001: begin // jalr
				Jalr = 1'b1; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'bx; ALUSrc = 1'bx; RegWrite=1'b1;
			end  
			default: begin
				Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b0;
			end
		endcase
		is_Jump = Jalr | Jal;
	end
endmodule

module ALUControl (input  [4:0] Inst, // funct7[5], funct3, OP[4]
				   output [3:0] ALUCtrl
	);
	assign ALUCtrl = Inst[0] ? Inst[4:1] : 4'b0000;
endmodule
// R & I (except Jalr) type: op[4] = 1, otherwise 0
// For B, Jal, Jalr, no need ALU -> default = ADD
// For LW, SW, ADD
// R, I, according to functions
// ALU OP List
// AND = 0111 |  OR = 0110 | ADD = 0000 | SUB = 1000 |
// SLT = 0010 | XOR = 0100 | SRA = 1101 | SLL = 0001 |
// SRL = 0101 |
// NOP = ADD x0, x0, x0

module ImmGenerator (input      [31:0] Inst,
					 output reg [31:0] Immediate
	);
	wire [3:0] Typectrl;
	wire [2:0] Funct3;
	//0000/1101:I 0100:S 1100:B 1111:J
	wire [31:0] I,S,B,J;
	wire [31:0] ISHAMT;
	assign Funct3 = Inst[14:12];
	assign Typectrl = {Inst[6:5],Inst[3:2]};
	assign I = {{21{Inst[31]}},Inst[30:20]};
	assign ISHAMT = {27'b0,Inst[24:20]};
	assign S = {{21{Inst[31]}},Inst[30:25],Inst[11:7]};
	assign B = {{20{Inst[31]}},Inst[7],Inst[30:25],Inst[11:8],1'b0};
	assign J = {{12{Inst[31]}},Inst[19:12],Inst[20],Inst[30:21],1'b0};
	always @(*) begin
		Immediate = 32'b0;
		case (Typectrl)
			4'b0000: Immediate = (Funct3==3'b101) ? ISHAMT : I;
			4'b1101: Immediate = I;
			4'b0100: Immediate = S;
			4'b1100: Immediate = B;
			4'b1111: Immediate = J; 
		endcase
	end
endmodule

module FowardingUnit (input  [4:0] RS1_addr,
					  input  [4:0] RS2_addr,
					  input  [4:0] EXME_WB_addr, // EX/ME write back address
					  input  [4:0] MEWB_WB_addr, // MEM/WB write back address
					  input        RegWrite,
					  output       Fr1A,
					  output       Fr1B,
					  output       Fr2A,
					  output       Fr2B);
	assign Fr1A = ((RS1_addr==EXME_WB_addr) && RegWrite) ? 1'b1 : 1'b0;
	assign Fr1B = ((RS2_addr==MEWB_WB_addr) && RegWrite) ? 1'b1 : 1'b0;
	assign Fr2A = ((RS1_addr==EXME_WB_addr) && RegWrite) ? 1'b1 : 1'b0; 
	assign Fr2B = ((RS2_addr==MEWB_WB_addr) && RegWrite) ? 1'b1 : 1'b0;
endmodule

//Status : Finish
module ALU (input  [31:0] inA,  // rst1
			input  [31:0] inB,  // rst2 or imm
			input   [3:0] ctrl, // ALU op
			output [31:0] out
	);
	reg [32:0] result;
	always @(*) begin
		result = 33'b0;
		case (ctrl[2:0])
			3'b000: begin
				if (ctrl[3]) result = $signed(inA) - $signed(inB);
				else		 result = $signed(inA) + $signed(inB);
			end
			3'b001: result = inA << inB; // SLL
			3'b010: result = (($signed(inA) < $signed(inB)) ? 1 : 0 ); // SLT
			3'b011: result = 32'b0; // NOP
			3'b100: result = inA ^ inB;
			3'b101: begin
				if (ctrl[3]) result = $signed(inA) >>> $signed(inB); // SRA
				else result = inA >> inB; // SRL
			end 
			3'b110: result = inA | inB;
			3'b111: result = inA & inB;
		endcase
	end
	assign out = result[31:0];
endmodule

//Status : Finish
module MainRegister (input clk,
					 input rst_n,
					 input  [31:0] WB_data,
					 input   [4:0] RS1_addr,
					 input   [4:0] RS2_addr,
					 input   [4:0] WB_addr,
					 input         RegWrite, // RegWrite=1 if write
					 output [31:0] RS1_data,
					 output [31:0] RS2_data
	);  
	integer i;
	reg [31:0] r32 [0:31];
	reg [31:0] nxt_r32 [0:31];

	assign RS1_data = r32[RS1_addr];
	assign RS2_data = r32[RS2_addr];
	always @(*) begin
		nxt_r32[0] = 32'b0;
		for (i=1; i<=31; i=i+1)
			nxt_r32[i] = r32[i];
		
		for (i=1; i<32; i=i+1)begin
			nxt_r32[i] = (RegWrite && (WB_addr == i)) ? WB_data : r32[i];
		end
	end    

	always @(posedge clk) begin
		if (!rst_n) begin
			for (i=0; i<=31; i=i+1)
				r32[i] <= 32'b0;
		end else begin
			for (i=0; i<=31; i=i+1)
				r32[i] <= nxt_r32[i];
		end
	end
endmodule

module JtypeForwardingUnit (input    [4:0] Writeb_addr,
							input   [31:0] Writeb_data,
							input    [4:0] RS1_addr,
							input   [31:0] RS1_data,
							input          Writeb_is_Jump, // whether the write back signal is jump
							output  [31:0] Link_data);
	assign Link_data = ((Writeb_addr == RS1_addr) && Writeb_is_Jump) ? Writeb_data : RS1_data;
endmodule

//Status : Finish
module PC (input  clk,
		   input  rst_n,
		   input  Jal,
		   input  Jalr,
		   input  BEQ,
		   input  BNE,
		   input  isStall,
		   input  [31:0] RS1_data,
		   input  [31:0] RS2_data,
		   input   [4:0] RS1_addr,
		   input   [4:0] Writeb_addr,
		   input  [31:0] Writeb_data,
		   input         Writeb_is_Jump,
		   input  [31:0] Immediate,
		   input  [31:0] ID_PC,
		   output reg [31:0] IF_PC,
		   output reg [31:0] IF_PCplus4,
		   output reg [31:0] IF_nxt_PC, // back to instruction memory
		   output isRegEq
	);
	wire JalB; // JalB: Jal | B
	wire [31:0] Imm_plus_RS1, Imm_plus_PC;
	wire [31:0] j_link_data;

	JtypeForwardingUnit JFWD (.Writeb_addr (Writeb_addr),
							  .Writeb_data (Writeb_data),
							  .RS1_addr (RS1_addr),
							  .RS1_data (RS1_data),
							  .Writeb_is_Jump (Writeb_is_Jump),
							  .Link_data (j_link_data)
	);
	assign isRegEq = (RS1_data == RS2_data) ? 1'b1 : 1'b0;
	assign JalB = (isRegEq & BEQ) | (~isRegEq & BNE) | Jal;
	assign Imm_plus_RS1 = Immediate + j_link_data;
	assign Imm_plus_PC  = Immediate + ID_PC;
	
	always@ (*) begin
		IF_PCplus4 = IF_PC + 4;
		if (isStall) begin 
			IF_nxt_PC = IF_PC;
		end else begin
			IF_nxt_PC = Jalr ? Imm_plus_RS1 :
					 	JalB ? Imm_plus_PC : IF_PCplus4;
		end
	end

	always@ (posedge clk) begin
		if (!rst_n)	IF_PC <= 32'b0;
		else IF_PC <= IF_nxt_PC;
	end
endmodule


// =========================================
// ==         Pipelined Registers         ==
// =========================================
module IFID (input             clk,
			 input             rst_n,
			 input             isStall,
			 input             isFlush,
			 input      [31:0] nxt_PC,   // next PC from IF stage
			 input      [31:0] nxt_PCplus4,
			 input      [31:0] nxt_Inst, // next Instructions from IF stage
			 output reg [31:0] PC,       // PC get into ID stage 
			 output reg [31:0] PCplus4,
			 output reg [31:0] Inst      // Instructions get into ID stage
	);

	always@ (posedge clk) begin
		if (!rst_n) begin
			PC <= 32'b0;
			PCplus4 <= 32'b0;
			Inst <= 32'b000000000000_00000_000_00000_0010011;
		end else if (isFlush) begin
			PC <= nxt_PC;
			PCplus4 <= nxt_PCplus4;
			Inst <= 32'b000000000000_00000_000_00000_0010011; // NOP
		end else if (isStall) begin
			PC <= PC;
			PCplus4 <= PCplus4;
			Inst <= Inst;
		end else begin
			PC <= nxt_PC;
			PCplus4 <= nxt_PCplus4;
			Inst <= nxt_Inst;
		end
	end
endmodule

module IDEX (input clk,
			 input rst_n,
			 input isStall,
			 input      [31:0] nxt_RS1_data,
			 input      [31:0] nxt_RS2_data,
			 input      [31:0] nxt_Immediate,
			 input       [4:0] nxt_WB_addr,
			 input      [31:0] nxt_J_StorePC,
			 input             nxt_is_Jump,
			 output reg [31:0] RS1_data,
			 output reg [31:0] RS2_data,    // imm or rs2 according to ALUsrc
			 output reg [31:0] Immediate,
			 output reg  [4:0] WB_addr,     // write back to what register
			 output reg [31:0] J_StorePC,   // PC+4 stored to reg (J-type)
			 output reg        is_Jump,     // whether need write back PC+4 (if Jal | Jalr)
			 // Control Signals
			 input      [3:0]  nxt_ALUop,
			 input             nxt_MemtoReg,
			 input             nxt_RegWrite,
			 input             nxt_MemRead,
			 input             nxt_MemWrite,
			 output reg [3:0]  ALUop,
			 output reg        MemtoReg,
			 output reg        RegWrite,
			 output reg        MemRead,
			 output reg        MemWrite,
 			 // Forwarding
			 input       [4:0] nxt_RS1_addr,
			 input       [4:0] nxt_RS2_addr,
			 input             nxt_ALUsrc, // determine rd == rs2
			 output reg  [4:0] RS1_addr,
			 output reg  [4:0] RS2_addr,
			 output reg        ALUsrc // determine rd == rs2
	);
	always@ (posedge clk) begin
		if (!rst_n) begin
			RS1_data    <= 32'b0;
			RS2_data    <= 32'b0;
			Immediate   <= 32'b0;
			WB_addr     <= 5'b0;  
			J_StorePC   <= 32'b0;
			is_Jump     <= 1'b0;
			ALUop       <= 1'b0;
			MemtoReg    <= 1'b0;
			RegWrite    <= 1'b0;
			MemRead     <= 1'b0;
			MemWrite    <= 1'b0;
			RS1_addr    <= 5'b0;
			RS2_addr    <= 5'b0;
			ALUsrc      <= 1'b0;
		end else if (isStall) begin
			RS1_data    <= RS1_data;
			RS2_data    <= RS2_data;
			Immediate   <= Immediate;
			WB_addr     <= WB_addr;
			J_StorePC   <= J_StorePC;
			is_Jump     <= is_Jump;
			ALUop       <= ALUop;
			MemtoReg    <= MemtoReg;
			RegWrite    <= RegWrite;
			MemRead     <= MemRead;
			MemWrite    <= MemWrite;
			RS1_addr    <= RS1_addr;
			RS2_addr    <= RS2_addr;
			ALUsrc      <= ALUsrc;
		end else begin
			RS1_data    <= nxt_RS1_data;
			RS2_data    <= nxt_RS2_data;
			Immediate   <= nxt_Immediate;
			WB_addr     <= nxt_WB_addr;
			J_StorePC   <= nxt_J_StorePC;
			is_Jump     <= nxt_is_Jump;
			ALUop       <= nxt_ALUop;
			MemtoReg    <= nxt_MemtoReg;
			RegWrite    <= nxt_RegWrite;
			MemRead     <= nxt_MemRead;
			MemWrite    <= nxt_MemWrite;
			RS1_addr    <= nxt_RS1_addr;
			RS2_addr    <= nxt_RS2_addr;
			ALUsrc      <= nxt_ALUsrc;
		end
	end
endmodule

module EXMEM (input clk,
			  input rst_n,
			  input isStall,
			  input      [31:0] nxt_ALU_result,
			  input      [31:0] nxt_MemWrt_data,
			  input       [4:0] nxt_WB_addr,
			  input      [31:0] nxt_J_StorePC,
			  input             nxt_is_Jump,
			  output reg [31:0] ALU_result,
			  output reg [31:0] MemWrt_data,
			  output reg  [4:0] WB_addr,
			  output reg [31:0] J_StorePC,   // PC+4 stored to reg (J-type)
			  output reg        is_Jump,     // whether need write back PC+4 (if Jal | Jalr)
		      // Control Signals
			  input      nxt_MemtoReg,
			  input      nxt_RegWrite,
			  input      nxt_MemRead,
			  input      nxt_MemWrite,
			  output reg MemtoReg,
			  output reg RegWrite,
			  output reg MemRead,
			  output reg MemWrite
			  // Forwarding is WB_addr
	);
	always@ (posedge clk) begin
		if (!rst_n) begin
			ALU_result  <= 32'b0;
			MemWrt_data <= 32'b0;
			WB_addr     <= 5'b0;
			J_StorePC   <= 32'b0;
			is_Jump     <= 1'b0;
			MemtoReg    <= 1'b0;
			RegWrite    <= 1'b0;
			MemRead     <= 1'b0;
			MemWrite    <= 1'b0;
		end else if (isStall) begin
			ALU_result  <= ALU_result;
			MemWrt_data <= MemWrt_data;
			WB_addr     <= WB_addr;  
			J_StorePC   <= J_StorePC;
			is_Jump     <= is_Jump;    
			MemtoReg    <= MemtoReg;    
			RegWrite    <= RegWrite;    
			MemRead     <= MemRead;     
			MemWrite    <= MemWrite;       
		end else begin
			ALU_result  <= nxt_ALU_result;
			MemWrt_data <= {nxt_MemWrt_data[7:0], nxt_MemWrt_data[15:8], nxt_MemWrt_data[23:16], nxt_MemWrt_data[31:24]}; // Endian
			WB_addr     <= nxt_WB_addr;  
			J_StorePC   <= nxt_J_StorePC;
			is_Jump     <= nxt_is_Jump;    
			MemtoReg    <= nxt_MemtoReg;    
			RegWrite    <= nxt_RegWrite;    
			MemRead     <= nxt_MemRead;     
			MemWrite    <= nxt_MemWrite;       
		end
	end	
endmodule

module MEMWB (input clk,
			  input rst_n,
			  input isStall,
			  input       [31:0] nxt_ALU_result,
			  input       [31:0] nxt_MemRd_data,
			  input        [4:0] nxt_WB_addr,
			  input       [31:0] nxt_J_StorePC,
			  input              nxt_is_Jump,
			  output reg  [31:0] ALU_result,
			  output reg  [31:0] MemRd_data,
			  output reg   [4:0] WB_addr,
			  output reg  [31:0] J_StorePC,
			  output reg         is_Jump,
			  // Control Signals
			  input      nxt_MemtoReg,
			  input      nxt_RegWrite,
			  output reg MemtoReg,
			  output reg RegWrite
			  // Forwarding is WB_addr
	);
	always@ (posedge clk) begin
		if (!rst_n) begin
				ALU_result <= 32'b0;
				MemRd_data <= 32'b0;
				WB_addr    <= 5'b0;
				J_StorePC  <= 32'b0;
				is_Jump    <= 1'b0;
				MemtoReg   <= 1'b0;
				RegWrite   <= 1'b0;
		end else if (isStall) begin
				ALU_result <= ALU_result;
				MemRd_data <= MemRd_data;
				WB_addr    <= WB_addr;   
				J_StorePC  <= J_StorePC;
				is_Jump    <= is_Jump;   
				MemtoReg   <= MemtoReg;  
				RegWrite   <= RegWrite;    
		end else begin
				ALU_result <= nxt_ALU_result;
				MemRd_data <= {nxt_MemRd_data[7:0], nxt_MemRd_data[15:8], nxt_MemRd_data[23:16], nxt_MemRd_data[31:24]};
				WB_addr    <= nxt_WB_addr;   
				J_StorePC  <= nxt_J_StorePC;
				is_Jump    <= nxt_is_Jump;   
				MemtoReg   <= nxt_MemtoReg;  
				RegWrite   <= nxt_RegWrite;    
		end
	end
endmodule

module hazard_detect(
				inst_rs1,
				inst_rs2,
				inst_op,
				inst_funct3,
				ex_rd,
				wb_rd,
				id_memread,
				ex_memread,
				wb_regwrite,
				branch_compare,
				//branch_true,
				//jal_true,
				//jalr_true,
				pcwrite,
				if_id_write,
				if_flush,
				control_zero
				);
				
	input [4:0] inst_rs1, inst_rs2;
	input [6:0] inst_op;
	input [2:0] inst_funct3;
	input [4:0] ex_rd, wb_rd; //rd in ex stage or wb stage
	input id_memread, ex_memread, wb_regwrite, branch_compare; // branch_compare means if rs1 = rs2 in branch instruction
	//output branch_true, jal_true, jalr_true; // if we need to take branch step or jump step
	output pcwrite, if_id_write, if_flush, control_zero; //control_zero = 1 when all control signals need to be zero
	
	//reg branch_true, jal_true, jalr_true;
	reg pcwrite, if_id_write, if_flush, control_zero;
	//Load-use hazard and write-read register hazard//
	always @(*) begin
		pcwrite = 1'b0;
		if_id_write = 1'b0;
		control_zero = 1'b0;
		if(ex_memread & ((ex_rd == inst_rs1) | (ex_rd == inst_rs2))) begin // load-use hazard
			pcwrite = 1'b1;
			if_id_write = 1'b1;
			control_zero = 1'b1;
		end
		else if(wb_regwrite & ((wb_rd == inst_rs1) | (wb_rd == inst_rs2))) begin // write-read hazard
			pcwrite = 1'b1;
			if_id_write = 1'b1;
			control_zero = 1'b1;
		end
		else if(inst_op == 7'b1100111 & wb_regwrite & (wb_rd == inst_rs1)) begin
			pcwrite = 1'b1;
			if_id_write = 1'b1;
			control_zero = 1'b1;
		end
	end
	////////////////////////////////////////////
	//branch or jump hazard
	always @(*) begin
		//branch_true = 1'b0;
		//jal_true = 1'b0;
		//jalr_true = 1'b0;
		if_flush = 1'b0;
		if((inst_op == 7'b1100011) & (inst_funct3 == 3'b000) & branch_compare) begin
			//branch_true = 1'b1;
			if_flush = 1'b1;
		end
		else if((inst_op == 7'b1100011) & (inst_funct3 == 3'b001) & !branch_compare) begin
			//branch_true = 1'b1;
			if_flush = 1'b1;
		end
		else if(inst_op == 7'b1101111) begin
			//jal_true = 1'b1;
			if_flush = 1'b1;
		end
		else if(inst_op == 7'b1100111) begin
			//jalr_true = 1'b1;
			if_flush = 1'b1;
		end
	end
endmodule