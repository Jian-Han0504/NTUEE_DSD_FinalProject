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
module Generator(clk,rst_n,stall,flush,raw_inst,decode_inst,select,is_16,reserve,r_stall);
	input clk,rst_n;
	input stall,flush;
	input [31:0] raw_inst;
	output reg [31:0] decode_inst;
	output [1:0] select;
	output is_16;
	output [1:0] reserve;
	output r_stall;

	reg [15:0] buffer,nxt_buffer;
	reg [1:0] reserve,nxt_reserve;
	reg r_stall,nxt_r_stall;

	wire [15:0] inst_16;
	wire [31:0] transform;
	
	assign is_16 = (select==2'b01) ? ~(raw_inst[16]&raw_inst[17]) : ~(raw_inst[0]&raw_inst[1]);
	assign inst_16 = (select==2'b01) ? raw_inst[31:16] : raw_inst[15:0];
	Selector Selector0(.clk(clk),.rst_n(rst_n&~flush),.flush(flush),.is_16(is_16),.cache_stall(stall),.Sel(select));
	Decompressor Decompressor0(.inst_16(inst_16),.inst_32(transform));
	always @(*) begin
		nxt_r_stall = stall;
		nxt_reserve = stall ? reserve : select;
		decode_inst = raw_inst;
		nxt_buffer = buffer;
		case(select)
			2'b00: begin
				if (is_16) decode_inst = transform;
			end
			2'b01: begin
				if (is_16) decode_inst = transform;
				else if (~stall) nxt_buffer = inst_16;
			end
			2'b11: begin
				decode_inst = {raw_inst[15:0],buffer};
				if (~is_16&~stall) begin
					nxt_buffer = inst_16;
				end
			end
			default: begin 
				decode_inst = raw_inst;
				nxt_buffer = buffer;
			end
		endcase
	end

	always @(posedge clk) begin
		buffer <= nxt_buffer;
		reserve <= nxt_reserve;
		r_stall <= nxt_r_stall;
	end
endmodule

module Selector(clk,rst_n,is_16,flush,cache_stall,Sel);
    input clk,rst_n;
    input is_16,flush,cache_stall;
    output [1:0] Sel;

    reg [1:0] state;
    reg [1:0] nxt_state;
    assign Sel = state;
    always@(*)begin
		case(state)
			2'b00: nxt_state = rst_n ? (cache_stall ? 2'b00 : is_16 ? 2'b01 : 2'b00) : 2'b00;
			2'b01: nxt_state = rst_n ? (cache_stall ? 2'b01 : is_16 ? 2'b00 : 2'b11) : 2'b00;
			2'b11: nxt_state = rst_n ? (cache_stall ? 2'b11 : 2'b01) : 2'b00;
			default nxt_state = 2'b00;
		endcase
    end 
    /////Sequential
    always @(posedge clk) begin
        state <= nxt_state;
    end
endmodule
module Decompressor(inst_16,inst_32);
	input [15:0] inst_16;
	output reg[31:0] inst_32;
	wire [4:0] funct;

	assign funct = {inst_16[15:13],inst_16[1:0]};
	always @(*) begin
		inst_32 = {27'b0,5'b10011};
		case (funct)
			//C.ADD & C.MV & C.JR & C.JALR
			5'b10010: begin
				inst_32 = {7'b0000000,inst_16[6:2],inst_16[11:7],3'b000,12'b000000000000};
				if(inst_16[6:2]==5'b00000) begin //C.JR & C.JALR rs1=0
					inst_32[11:0] = {4'b000,inst_16[12],7'b1100111};
				end
				else begin //C.ADD & C.MV
					inst_32[11:0] = {inst_16[11:7],7'b0110011};
					if(~inst_16[12]) begin //C.MV 
						inst_32[19:15] = 5'b00000;
					end
				end
			end
			//C.ADDI
			5'b00001: inst_32 = {{7{inst_16[12]}},inst_16[6:2],inst_16[11:7],3'b000,inst_16[11:7],7'b0010011};//12+5+3+5+7
			//C.SLLI
			5'b00010: inst_32 = {7'b0000000,inst_16[6:2],inst_16[11:7],3'b001,inst_16[11:7],7'b0010011};//7+5+5+3+5+7
			//C.ANDI & C.SRLI & C.SRRI 
			5'b10001: begin
				case(inst_16[11:10])
					//C.ANDI
					2'b10: inst_32 = {{7{inst_16[12]}},inst_16[6:2],2'b01,inst_16[9:7],5'b11101,inst_16[9:7],7'b0010011};
					//C.SRLI
					2'b00: inst_32 = {7'b0000000,inst_16[6:2],2'b01,inst_16[9:7],5'b10101,inst_16[9:7],7'b0010011};
					//C.SRRI
					2'b01: inst_32 = {7'b0100000,inst_16[6:2],2'b01,inst_16[9:7],5'b10101,inst_16[9:7],7'b0010011};
					default: inst_32 = 32'bx;
				endcase
			end
			//C.LW
			5'b01000: inst_32 = {5'b00000,inst_16[5],inst_16[12:10],inst_16[6],4'b0001,inst_16[9:7],5'b01001,inst_16[4:2],7'b0000011};
			//C.SW
			5'b11000: inst_32 = {5'b00000,inst_16[5],inst_16[12],2'b01,inst_16[4:2],2'b01,inst_16[9:7],3'b010,inst_16[11:10],inst_16[6],9'b000100011};
			//C.BEQZ
			5'b11001: inst_32 = {{4{inst_16[12]}},inst_16[6:5],inst_16[2],7'b0000001,inst_16[9:7],3'b000,inst_16[11:10],inst_16[4:3],inst_16[12],7'b1100011};
			//C.BNEZ
			5'b11101: inst_32 = {{4{inst_16[12]}},inst_16[6:5],inst_16[2],7'b0000001,inst_16[9:7],3'b001,inst_16[11:10],inst_16[4:3],inst_16[12],7'b1100011};
			//C.J
			5'b10101: inst_32 = {inst_16[12],inst_16[8],inst_16[10:9],inst_16[6],inst_16[7],inst_16[2],inst_16[11],inst_16[5:3],{9{inst_16[12]}},12'b000001101111};
			//C.Jal
			5'b00101: inst_32 = {inst_16[12],inst_16[8],inst_16[10:9],inst_16[6],inst_16[7],inst_16[2],inst_16[11],inst_16[5:3],{9{inst_16[12]}},12'b000011101111};
			//C.NOP
			5'b00001: inst_32 = {27'b0,5'b10011};
			default: inst_32 = {27'b0,5'b10011};
		endcase
	end
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
	wire cache_stall, hazard_stall;
	wire IFID_flush, IDEX_flush;
	
	// IF
	wire [31:0] IF_inst;
	wire [31:0] IF_pc;
	wire [31:0] IF_pc_plus4;
	wire [31:0] IF_nxt_pc; // next pc to PC register
	
	// ID
	wire [31:0] ID_inst;
	wire  [5:0] ctrl_inst; // inst for control unit
	wire  [5:0] alu_ctrl_inst; // inst for ALU control signals
	wire [31:0] ID_pc;
	wire [31:0] ID_pc_plus4;
	wire [31:0] ID_immediate;
	wire  [4:0] ID_rs1_addr, ID_rs2_addr; // forwarding to EXE stage
	wire  [4:0] ID_writeb_addr;
	wire [31:0] ID_rs1_data, ID_rs2_data; // output from register file
	wire        ID_is_reg_eqaul; // used for hazard detection

	// ID Control
	wire ID_jalr, ID_jal, ID_bne, ID_beq, 
		 ID_mem_read, ID_mem_write, ID_mem2reg, 
		 ID_alu_src, ID_reg_write, ID_is_jump;
	wire  [3:0] ID_alu_op;
	
	// EX
	wire [31:0] EX_pc_plus4;
	wire [31:0] EX_immediate;
	wire  [4:0] EX_rs1_addr, EX_rs2_addr;
	wire [31:0] EX_rs1_data, EX_rs2_data;
	wire [31:0] EX_fwdrs2_data; // rs2_data with forwarding
	wire  [3:0] EX_alu_op;
	wire [31:0] EX_alu_inA, EX_alu_inB;
	wire [31:0] EX_alu_result;
	wire  [4:0] EX_writeb_addr;
	wire EX_mem_read, EX_mem_write, EX_mem2reg, 
		 EX_alu_src, EX_reg_write, EX_is_jump, Flush_IDEX;

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

	// Stall Control
	assign cache_stall = ICACHE_stall | DCACHE_stall;

	// IF
	// Endian Little -> Big
	assign IF_inst = {ICACHE_rdata[7:0], ICACHE_rdata[15:8], ICACHE_rdata[23:16], ICACHE_rdata[31:24]};
	
	// ID
	assign ctrl_inst = {ID_inst[12], ID_inst[6:2]};
	assign alu_ctrl_inst = {ID_inst[30], ID_inst[14:12], ID_inst[5:4]};
	assign ID_rs1_addr = ID_inst[19:15];
	assign ID_rs2_addr = ID_inst[24:20];
	assign ID_writeb_addr = ID_inst[11:7];

	// EXE
	// RS1 & RS2 mux
	assign EX_alu_inA = fwd1A ? ME_alu_result  :
						fwd2A ? WB_writeb_data : EX_rs1_data;
	assign EX_fwdrs2_data = fwd1B ? ME_alu_result :
							fwd2B ? WB_writeb_data : EX_rs2_data;
	assign EX_alu_inB = EX_alu_src ? EX_immediate : EX_fwdrs2_data;
	
	// ME Input
	assign ME_memRd_data = DCACHE_rdata;
	assign ME_writeb_data = ME_alu_result;

	// WB
	assign WB_writeb_data = WB_is_jump ? WB_pc_plus4 :
							WB_mem2reg ? WB_memRd_data : WB_alu_result; 


	// ======================================
	// ==             PIPELINED            ==
	// ======================================
	wire [31:0] IF_16decode;
	wire [1:0] C_select,C_reserve;
	wire C_is_16,IF_C_plus2,ID_C_plus2,C_r_stall;
	wire C_PC16_stall,C_32buf_stall;
	wire ID_C_plus21;
	assign C_PC16_stall = (~(C_select==2'b01))&C_is_16;
	assign C_32buf_stall = (C_select==2'b01)&(~C_is_16);
	assign IF_C_plus2 = (C_select==2'b01) ? 1:0;  
	assign ID_C_plus21 = (C_r_stall) ? (C_reserve==2'b01) : ID_C_plus2;
	Generator GN (.clk(clk),.rst_n(rst_n),.stall(cache_stall|hazard_stall),.flush(IFID_flush),.raw_inst(IF_inst),.decode_inst(IF_16decode),.select(C_select),.is_16(C_is_16),.reserve(C_reserve),.r_stall(C_r_stall));
	// IF/ID
	IFID IFID0 (.clk (clk), 
				.rst_n (rst_n), 
				.Cache_Stall (cache_stall),
				.Hazard_Stall (hazard_stall),
				.Compression_Stall (C_32buf_stall),
				.is_Flush (IFID_flush),
				.nxt_CPCplus2 (IF_C_plus2),
				.nxt_PC (IF_pc),
				.nxt_PCplus4 (IF_pc_plus4),
				.nxt_Inst (IF_16decode),
				.PC (ID_pc),
				.PCplus4 (ID_pc_plus4),
				.Inst (ID_inst), 
				.CPCplus2(ID_C_plus2)
	);

	// Control Unit
	MainControl Ctrl (.Inst (ctrl_inst),
					  .Jalr (ID_jalr),
					  .Jal (ID_jal),
					  .Bne (ID_bne),
					  .Beq (ID_beq),
					  .MemRead (ID_mem_read),
					  .MemWrite (ID_mem_write),
					  .MemtoReg (ID_mem2reg),
					  .ALUSrc (ID_alu_src),
					  .RegWrite (ID_reg_write),
					  .is_Jump (ID_is_jump)
	);

	// ALU control
	ALUControl ALUCtrl (.Inst (alu_ctrl_inst), .ALUCtrl (ID_alu_op));

	// immediate gen
	ImmGenerator IG0 (.Inst(ID_inst), .Immediate (ID_immediate));

	// register file
	MainRegister MR (.clk(clk),
					.rst_n(rst_n),
					.WrtBack_data (WB_writeb_data),
					.RS1_addr (ID_rs1_addr),
					.RS2_addr (ID_rs2_addr),
					.WrtBack_addr (WB_writeb_addr),
					.RegWrite (WB_reg_write),
					.RS1_data (ID_rs1_data),
					.RS2_data (ID_rs2_data)
	);

	// ID/EXE
	IDEX IDEX0 (.clk (clk),
				.rst_n (rst_n),
				.Cache_Stall (cache_stall),
				.Compression_Stall (C_32buf_stall),
				.Hazard_Flush (IDEX_flush),
				// pipelined
				.nxt_RS1_data (ID_rs1_data),
				.nxt_RS2_data (ID_rs2_data),
				.nxt_Immediate (ID_immediate),
				.nxt_WrtBack_addr (ID_writeb_addr),
				.nxt_J_StorePC (ID_pc_plus4),
				.nxt_is_Jump (ID_is_jump),
				.RS1_data (EX_rs1_data),
				.RS2_data (EX_rs2_data),
				.Immediate (EX_immediate),
				.WrtBack_addr (EX_writeb_addr),
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
				.nxt_ALUSrc (ID_alu_src),
				.RS1_addr (EX_rs1_addr),
				.RS2_addr (EX_rs2_addr),
				.ALUSrc (EX_alu_src)
	);

	// ALU
	ALU ALU0 (.InA (EX_alu_inA),
			.InB (EX_alu_inB),
			.Ctrl (EX_alu_op),
			.Out (EX_alu_result)
	);
	
	// EX/MEM
	EXMEM EXME0(.clk (clk),
				.rst_n (rst_n),
				.Cache_Stall (cache_stall),
				.Compression_Stall (C_32buf_stall),
				.nxt_ALU_result (EX_alu_result),    // ALU result -> mem write addr
				.nxt_MemWrt_data (EX_fwdrs2_data),  // ALU InB -> mem write data
				.nxt_WrtBack_addr (EX_writeb_addr), // write-back register addr
				.nxt_J_StorePC (EX_pc_plus4),
				.nxt_is_Jump (EX_is_jump),
				.ALU_result (ME_alu_result),
				.MemWrt_data (ME_memWrt_data),
				.WrtBack_addr (ME_writeb_addr),
				.J_StorePC (ME_pc_plus4),
				.is_Jump (ME_is_jump),
				// Control Signals
				.nxt_MemtoReg (EX_mem2reg),
				.nxt_RegWrite (EX_reg_write),
				.nxt_MemRead (EX_mem_read),
				.nxt_MemWrite (EX_mem_write),
				.MemtoReg (ME_mem2reg),
				.RegWrite (ME_reg_write),
				.MemRead (ME_mem_read),
				.MemWrite (ME_mem_write)
	); 

	// ME/WB
	// Note: Endian Transform defined in EXME module
	MEMWB MEWB0(.clk (clk),
				.rst_n (rst_n),
				.Cache_Stall (cache_stall),
				.Compression_Stall (C_32buf_stall),
				// pipelined
				.nxt_ALU_result (ME_alu_result),
				.nxt_MemRd_data (ME_memRd_data),
				.nxt_WrtBack_addr (ME_writeb_addr),
				.nxt_J_StorePC (ME_pc_plus4),
				.nxt_is_Jump (ME_is_jump),
				.ALU_result (WB_alu_result),
				.MemRd_data (WB_memRd_data),
				.WrtBack_addr (WB_writeb_addr),
				.J_StorePC (WB_pc_plus4),
				.is_Jump (WB_is_jump),
				// Control Signals
				.nxt_MemtoReg (ME_mem2reg),
				.nxt_RegWrite (ME_reg_write),
				.MemtoReg (WB_mem2reg),
				.RegWrite (WB_reg_write)
	);

	// ======================================
	// ==           FLOW CONTROL           ==
	// ======================================
	// PC
	PC PC0 (.clk (clk),
			.rst_n (rst_n),
			.Jal (ID_jal),
			.Jalr (ID_jalr),
			.Beq (ID_beq),
			.Bne (ID_bne),
			.Cache_Stall (cache_stall),
			.Hazard_Stall (hazard_stall),
			.Comp_16_Stall (C_PC16_stall),
			.state (ID_C_plus21),
			.ID_PC (ID_pc),
			.Immediate (ID_immediate),
			.ID_RS1_data (ID_rs1_data),
			.ID_RS2_data (ID_rs2_data),
			// J-type Forwarding
			.ID_RS1_addr (ID_rs1_addr),
			.ID_RS2_addr (ID_rs2_addr),
			.ME_WrtBack_addr(ME_writeb_addr),
		    .ME_WrtBack_data(ME_alu_result),
		    .WB_WrtBack_addr(WB_writeb_addr),
		    .WB_WrtBack_data(WB_writeb_data),
			.EX_WrtBack_addr(EX_writeb_addr),
		    .EX_WrtBack_data(ME_alu_result),
		    .WB_RegWrite(WB_reg_write),
		    .ME_RegWrite(ME_reg_write), 
			// end of J-type Forwarding
			.IF_PC (IF_pc),
			.IF_PCplus4 (IF_pc_plus4),
			.IF_nxt_PC (IF_nxt_pc),
			.is_RegEq (ID_is_reg_eqaul)
	);

	// Forwarding Unit
	FowardingUnit FU_DATA  (.EX_RS1_addr (EX_rs1_addr),
							.EX_RS2_addr (EX_rs2_addr),
							.ME_WrtBack_addr (ME_writeb_addr),
							.WB_WrtBack_addr (WB_writeb_addr),
							.ME_RegWrite (ME_reg_write),
							.WB_RegWrite (WB_reg_write),
							.Fr1A (fwd1A),
							.Fr1B (fwd1B),
							.Fr2A (fwd2A),
							.Fr2B (fwd2B)
	);
	// hazard unit
	Hazard_Detect HZ (.RS1_addr (ID_rs1_addr),
					  .RS2_addr (ID_rs2_addr),
					  .Jalr (ID_jalr),
					  .Jal (ID_jal),
					  .Bne (ID_bne),
					  .Beq (ID_beq),
					  .is_RegEq (ID_is_reg_eqaul),
					  .EX_WrtBack_addr (EX_writeb_addr),
					  .EX_MemRead (EX_mem_read),
					  .EX_RegWrite (EX_reg_write),
					  .Hazard_Stall (hazard_stall),
					  .Flush_IFID (IFID_flush),
					  .Flush_IDEX (IDEX_flush)
	);
endmodule

// =========================================
// ==             Sub Modules             ==
// =========================================
module MainControl (input  	   [5:0] Inst, // funct3[0] OP[6:2] (INST[12] INST[6:2])
					output reg       Jalr,
					output reg       Jal,
					output reg 		 Bne,
					output reg 		 Beq,
					output reg 		 MemRead,
					output reg 		 MemWrite,
					output reg 		 MemtoReg,
					output reg 		 ALUSrc,
					output reg 		 RegWrite,
					output   		 is_Jump);
	assign is_Jump = Jalr | Jal;
	always@(*) begin
		case (Inst[4:0])
			5'b01100: begin // R type
				Jalr = 1'b0; Jal = 1'b0; Beq = 1'b0; Bne = 1'b0; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b1;
			end
			5'b00000: begin // lw
				Jalr = 1'b0; Jal = 1'b0; Beq = 1'b0; Bne = 1'b0; MemRead = 1'b1; MemWrite = 1'b0; MemtoReg = 1'b1; ALUSrc = 1'b1; RegWrite=1'b1;
			end
			5'b01000: begin // sw
				Jalr = 1'b0; Jal = 1'b0; Beq = 1'b0; Bne = 1'b0; MemRead = 1'b0; MemWrite = 1'b1; MemtoReg = 1'b0; ALUSrc = 1'b1; RegWrite=1'b0;
			end
			5'b11000: begin // Beq bne
				Jalr = 1'b0; Jal = 1'b0; Beq = ~Inst[5]; Bne = Inst[5]; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b0;
			end
			5'b00100: begin // I type
				Jalr = 1'b0; Jal = 1'b0; Beq = 1'b0; Bne = 1'b0; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b1; RegWrite=1'b1;
			end
			5'b11011: begin // jal
				Jalr = 1'b0; Jal = 1'b1; Beq = 1'b0; Bne = 1'b0; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b1;
			end
			5'b11001: begin // jalr
				Jalr = 1'b1; Jal = 1'b0; Beq = 1'b0; Bne = 1'b0; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b1;
			end  
			default: begin
				Jalr = 1'b0; Jal = 1'b0; Beq = 1'b0; Bne = 1'b0; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b0;
			end
		endcase
	end
endmodule

module ALUControl (input  [5:0] Inst, // funct7[5], funct3, OP[5:4]
				   output [3:0] ALUCtrl
	);
	// if op[4] == 1 -> R(op[5]=1) or I(op[5]=0) type
	// if not R or I -> use ADD(0000)
	// else if R -> use func7[5] + func3
	// else (I)  -> use 0 + func3
	assign ALUCtrl = ~Inst[0] ? 4'b0000 :
					 (Inst[1] | (Inst[4] & ~Inst[3] & Inst[2])) ? Inst[5:2]:
					 {1'b0, Inst[4:2]};
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

//Status : Finish
module ALU (input  [31:0] InA,  // rst1
			input  [31:0] InB,  // rst2 or imm
			input   [3:0] Ctrl, // ALU op
			output [31:0] Out
	);
	reg [32:0] result;
	always @(*) begin
		result = 33'b0;
		case (Ctrl[2:0])
			3'b000: begin
				if (Ctrl[3]) result = $signed(InA) - $signed(InB);
				else		 result = $signed(InA) + $signed(InB);
			end
			3'b001: result = InA << InB; // SLL
			3'b010: result = (($signed(InA) < $signed(InB)) ? 1 : 0 ); // SLT
			3'b011: result = 32'b0; // NOP
			3'b100: result = InA ^ InB;
			3'b101: begin
				if (Ctrl[3]) result = $signed(InA) >>> $signed(InB); // SRA
				else result = InA >> InB; // SRL
			end 
			3'b110: result = InA | InB;
			3'b111: result = InA & InB;
		endcase
	end
	assign Out = result[31:0];
endmodule

//Status : Finish
module MainRegister (input clk,
					 input rst_n,
					 input  [31:0] WrtBack_data,
					 input   [4:0] RS1_addr,
					 input   [4:0] RS2_addr,
					 input   [4:0] WrtBack_addr,
					 input         RegWrite, // RegWrite=1 if write
					 output [31:0] RS1_data,
					 output [31:0] RS2_data
	);  
	integer i;
	reg [31:0] r32 [0:31];
	reg [31:0] nxt_r32 [0:31];

	assign RS1_data = (RegWrite & (WrtBack_addr == RS1_addr) & (|WrtBack_addr)) ? WrtBack_data : r32[RS1_addr];
	assign RS2_data = (RegWrite & (WrtBack_addr == RS2_addr) & (|WrtBack_addr)) ? WrtBack_data : r32[RS2_addr];
	always @(*) begin
		nxt_r32[0] = 32'b0;
		for (i=1; i<=31; i=i+1)
			nxt_r32[i] = r32[i];
		
		for (i=1; i<32; i=i+1)begin
			nxt_r32[i] = (RegWrite && (WrtBack_addr == i)) ? WrtBack_data : r32[i];
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

// data (EX stage) forwarding from ME/WB stage
module FowardingUnit (input  [4:0] EX_RS1_addr,
					  input  [4:0] EX_RS2_addr,
					  input  [4:0] ME_WrtBack_addr, // EX/ME write back address
					  input  [4:0] WB_WrtBack_addr, // ME/WB write back address
					  input        ME_RegWrite,
					  input        WB_RegWrite,
					  output       Fr1A,
					  output       Fr1B,
					  output       Fr2A,
					  output       Fr2B);
	// A: alu inA, B: alu inB
	// 1: forwarding from ME, 2: forwarding from WB
	assign Fr1A = ((EX_RS1_addr==ME_WrtBack_addr) && ME_RegWrite && ME_WrtBack_addr) ? 1'b1 : 1'b0;
	assign Fr1B = ((EX_RS2_addr==ME_WrtBack_addr) && ME_RegWrite && ME_WrtBack_addr) ? 1'b1 : 1'b0;
	assign Fr2A = ((EX_RS1_addr==WB_WrtBack_addr) && WB_RegWrite && WB_WrtBack_addr) ? 1'b1 : 1'b0; 
	assign Fr2B = ((EX_RS2_addr==WB_WrtBack_addr) && WB_RegWrite && WB_WrtBack_addr) ? 1'b1 : 1'b0;
endmodule

// jal and jalr (ID stage) forwarding from ME/WB stage
module JtypeForwardingUnit (input   [4:0] WB_WrtBack_addr,
							input   [4:0] ME_WrtBack_addr,
							input	[4:0] EX_WrtBack_addr,
							input  [31:0] WB_WrtBack_data,
							input  [31:0] ME_WrtBack_data,
							input  [31:0] EX_WrtBack_data,
							input   [4:0] ID_RS1_addr,
							input  [31:0] ID_RS1_data,
							input   [4:0] ID_RS2_addr,
							input  [31:0] ID_RS2_data,
							input         ME_RegWrite,
							input         WB_RegWrite, // whether the write back signal is jump
							output [31:0] Link_RS1_data, // for jalr link rs1 and branch
							output [31:0] Link_RS2_data  // for branch
	);
	assign Link_RS1_data = ((ME_WrtBack_addr == ID_RS1_addr) & ME_RegWrite) ? ME_WrtBack_data :
					   	   ((WB_WrtBack_addr == ID_RS1_addr) & WB_RegWrite) ? WB_WrtBack_data : 
					       (EX_WrtBack_addr == ID_RS1_addr) ? EX_WrtBack_data :
						   ID_RS1_data;
	assign Link_RS2_data = ((ME_WrtBack_addr == ID_RS2_addr) & ME_RegWrite) ? ME_WrtBack_data :
					   	   ((WB_WrtBack_addr == ID_RS2_addr) & WB_RegWrite) ? WB_WrtBack_data : 
					       ID_RS2_data;
endmodule

//Status : Finish
module PC (input  clk,
		   input  rst_n,
		   input  Jal,
		   input  Jalr,
		   input  Beq,
		   input  Bne,
		   input  Cache_Stall,
		   input  Hazard_Stall,
		   input  state,	
		   input  Comp_16_Stall,   		   
		   input  [31:0] ID_PC,
		   input  [31:0] Immediate,
		   input  [31:0] ID_RS1_data,
		   input  [31:0] ID_RS2_data,
		   // for J-type and B-type Forwarding
		   input   [4:0] ID_RS1_addr,
		   input   [4:0] ID_RS2_addr,
		   input   [4:0] ME_WrtBack_addr,
		   input  [31:0] ME_WrtBack_data,
		   input   [4:0] WB_WrtBack_addr,
		   input  [31:0] WB_WrtBack_data,
		   input   [4:0] EX_WrtBack_addr,
		   input  [31:0] EX_WrtBack_data,
		   input         WB_RegWrite,
		   input         ME_RegWrite,
		   // end of J-type Forwarding
		   output reg [31:0] IF_PC,
		   output reg [31:0] IF_PCplus4,
		   output reg [31:0] IF_nxt_PC, // back to instruction memory
		   output            is_RegEq); // used for hazard detection 
	wire is_Stall;

	wire JalB; // JalB: Jal | B
	wire [31:0] imm_plus_RS1, imm_plus_PC;
	wire [31:0] link_rs1_data, link_rs2_data;

	assign is_Stall = Cache_Stall | Hazard_Stall;

	JtypeForwardingUnit JFWD (.WB_WrtBack_addr (WB_WrtBack_addr),
							  .ME_WrtBack_addr (ME_WrtBack_addr),
							  .EX_WrtBack_addr (EX_WrtBack_addr),
							  .WB_WrtBack_data (WB_WrtBack_data),
							  .ME_WrtBack_data (ME_WrtBack_data),
							  .EX_WrtBack_data (EX_WrtBack_data),
							  .ID_RS1_addr (ID_RS1_addr),
							  .ID_RS1_data (ID_RS1_data),
							  .ID_RS2_addr (ID_RS2_addr),
							  .ID_RS2_data (ID_RS2_data),
							  .ME_RegWrite (ME_RegWrite),
							  .WB_RegWrite (WB_RegWrite),
							  .Link_RS1_data (link_rs1_data), // with forwarding
							  .Link_RS2_data (link_rs2_data)  // with forwarding 
	);

	assign is_RegEq = (link_rs1_data == link_rs2_data) ? 1'b1 : 1'b0;
	assign JalB = (is_RegEq & Beq) | (~is_RegEq & Bne) | Jal;
	assign imm_plus_RS1 = $signed(Immediate) + $signed(link_rs1_data);
	assign imm_plus_PC  = $signed(Immediate) + $signed(ID_PC); 
	
	always@ (*) begin
		IF_PCplus4 = IF_PC + 4;
		if (is_Stall) begin 
			IF_nxt_PC = IF_PC;
		end else begin
			IF_nxt_PC = Jalr ? imm_plus_RS1 :
					 	JalB ? (state ? (imm_plus_PC + 2) : imm_plus_PC) : 
						Comp_16_Stall ? IF_PC : IF_PCplus4; // Hank
		end
	end

	always@ (posedge clk) begin
		if (~rst_n)	IF_PC <= 32'b0;
		else IF_PC <= IF_nxt_PC;
	end
endmodule

module Hazard_Detect( // in ID stage
				RS1_addr,
				RS2_addr,
				Jalr,
				Jal,
				Bne,
				Beq,
				is_RegEq,
				EX_WrtBack_addr, // load use hazard, jalr hazard
				EX_MemRead,
				EX_RegWrite,
				Hazard_Stall,
				Flush_IFID,
				Flush_IDEX
	);			
	input [4:0] RS1_addr, RS2_addr;
	input       Jalr, Jal, Bne, Beq, is_RegEq;
	input [4:0] EX_WrtBack_addr; // rd in ex stage
	input 		EX_MemRead, EX_RegWrite;

	// Flush_IDEX = 1 when all control signals need to be zero
	output reg Hazard_Stall, Flush_IDEX;
	output     Flush_IFID;

	assign Flush_IFID = (Jalr | Jal | (Beq & is_RegEq) | (Bne & ~is_RegEq)) ? 1'b1 : 1'b0;

	always @(*) begin
		Hazard_Stall = 1'b0;
		Flush_IDEX = 1'b0;
		if (EX_WrtBack_addr==RS1_addr & (Beq|Bne)) begin
			Hazard_Stall = 1'b0;
			Flush_IDEX = 1'b0;
		end
		else if (EX_MemRead & ((EX_WrtBack_addr == RS1_addr) | (EX_WrtBack_addr == RS2_addr )) & (EX_WrtBack_addr != 0)) begin // load-use hazard
			Hazard_Stall = 1'b1;
			Flush_IDEX = 1'b1;
		end
		
		else if (Jalr & EX_RegWrite & (EX_WrtBack_addr == RS1_addr) & (EX_WrtBack_addr != 0)) begin // jalr hazard
			Hazard_Stall = 1'b1;
			Flush_IDEX = 1'b1;
		end
		else if ((Bne|Beq) & EX_RegWrite & EX_WrtBack_addr != 0 & 
				(EX_WrtBack_addr == RS1_addr | EX_WrtBack_addr == RS2_addr)) begin // branch hazard
				Hazard_Stall = 1'b1;
				Flush_IDEX = 1'b1;
		end
	end
endmodule

// =========================================
// ==         Pipelined Registers         ==
// =========================================
module IFID (input             clk,
			 input             rst_n,
			 input             Cache_Stall,
			 input             Hazard_Stall,
			 input			   Compression_Stall,
			 input             is_Flush, // flush due to Jump and Branch
			 input      [31:0] nxt_PC,   // next PC from IF stage
			 input      [31:0] nxt_PCplus4,
			 input      [31:0] nxt_Inst, // next Instructions from IF stage
			 input			   nxt_CPCplus2,
			 output reg [31:0] PC,       // PC get into ID stage 
			 output reg [31:0] PCplus4,
			 output reg [31:0] Inst,     // Instructions get into ID stage
			 output reg CPCplus2
	);
	wire is_Stall;

	assign is_Stall = Cache_Stall | Hazard_Stall | Compression_Stall;

	always@ (posedge clk) begin
		CPCplus2 <= 1'b0;
		if (!rst_n) begin
			PC <= 32'b0;
			PCplus4 <= 32'b0;
			Inst <= 32'b000000000000_00000_000_00000_0010011;
		end else if (is_Stall) begin
			PC <= PC;
			PCplus4 <= PCplus4;
			Inst <= Inst;
		end else if (is_Flush) begin
			PC <= nxt_PC;
			PCplus4 <= nxt_PCplus4;
			Inst <= 32'b000000000000_00000_000_00000_0010011; // NOP
		end else begin
			PC <= nxt_PC;
			PCplus4 <= nxt_PCplus4;
			Inst <= nxt_Inst;
			CPCplus2 <= nxt_CPCplus2;
		end
	end
endmodule

module IDEX (input clk,
			 input rst_n,
			 input Cache_Stall,
			 input Compression_Stall,
			 input Hazard_Flush, // flush due to hazards
			 input      [31:0] nxt_RS1_data,
			 input      [31:0] nxt_RS2_data,
			 input      [31:0] nxt_Immediate,
			 input       [4:0] nxt_WrtBack_addr,
			 input      [31:0] nxt_J_StorePC,
			 input             nxt_is_Jump,
			 output reg [31:0] RS1_data,
			 output reg [31:0] RS2_data,    // imm or rs2 according to ALUsrc
			 output reg [31:0] Immediate,
			 output reg  [4:0] WrtBack_addr,     // write back to what register
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
			 input             nxt_ALUSrc, // determine rd == rs2
			 output reg  [4:0] RS1_addr,
			 output reg  [4:0] RS2_addr,
			 output reg        ALUSrc // determine rd == rs2
	);
	always@ (posedge clk) begin
		if (!rst_n) begin
			RS1_data    <= 32'b0;
			RS2_data    <= 32'b0;
			Immediate   <= 32'b0;
			WrtBack_addr<= 5'b0;  
			J_StorePC   <= 32'b0;
			is_Jump     <= 1'b0;
			ALUop       <= 1'b0;
			MemtoReg    <= 1'b0;
			RegWrite    <= 1'b0;
			MemRead     <= 1'b0;
			MemWrite    <= 1'b0;
			RS1_addr    <= 5'b0;
			RS2_addr    <= 5'b0;
			ALUSrc      <= 1'b0;
		end else if (Cache_Stall | Compression_Stall) begin
			RS1_data    <= RS1_data;
			RS2_data    <= RS2_data;
			Immediate   <= Immediate;
			WrtBack_addr     <= WrtBack_addr;
			J_StorePC   <= J_StorePC;
			is_Jump     <= is_Jump;
			ALUop       <= ALUop;
			MemtoReg    <= MemtoReg;
			RegWrite    <= RegWrite;
			MemRead     <= MemRead;
			MemWrite    <= MemWrite;
			RS1_addr    <= RS1_addr;
			RS2_addr    <= RS2_addr;
			ALUSrc      <= ALUSrc;
		end else if (Hazard_Flush) begin
			RS1_data    <= 32'b0;
			RS2_data    <= 32'b0;
			Immediate   <= 32'b0;
			WrtBack_addr<= 5'b0;  
			J_StorePC   <= 32'b0;
			is_Jump     <= 1'b0;
			ALUop       <= 1'b0;
			MemtoReg    <= 1'b0;
			RegWrite    <= 1'b0;
			MemRead     <= 1'b0;
			MemWrite    <= 1'b0;
			RS1_addr    <= 5'b0;
			RS2_addr    <= 5'b0;
			ALUSrc      <= 1'b0;
		end else begin
			RS1_data    <= nxt_RS1_data;
			RS2_data    <= nxt_RS2_data;
			Immediate   <= nxt_Immediate;
			WrtBack_addr<= nxt_WrtBack_addr;
			J_StorePC   <= nxt_J_StorePC;
			is_Jump     <= nxt_is_Jump;
			ALUop       <= nxt_ALUop;
			MemtoReg    <= nxt_MemtoReg;
			RegWrite    <= nxt_RegWrite;
			MemRead     <= nxt_MemRead;
			MemWrite    <= nxt_MemWrite;
			RS1_addr    <= nxt_RS1_addr;
			RS2_addr    <= nxt_RS2_addr;
			ALUSrc      <= nxt_ALUSrc;
		end
	end
endmodule

module EXMEM (input clk,
			  input rst_n,
			  input Cache_Stall,
			  input	Compression_Stall,
			  input      [31:0] nxt_ALU_result,
			  input      [31:0] nxt_MemWrt_data,
			  input       [4:0] nxt_WrtBack_addr,
			  input      [31:0] nxt_J_StorePC,
			  input             nxt_is_Jump,
			  output reg [31:0] ALU_result,
			  output reg [31:0] MemWrt_data,
			  output reg  [4:0] WrtBack_addr,
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
			  // Forwarding is WrtBack_addr
	);
	always@ (posedge clk) begin
		if (!rst_n) begin
			ALU_result  <= 32'b0;
			MemWrt_data <= 32'b0;
			WrtBack_addr<= 5'b0;
			J_StorePC   <= 32'b0;
			is_Jump     <= 1'b0;
			MemtoReg    <= 1'b0;
			RegWrite    <= 1'b0;
			MemRead     <= 1'b0;
			MemWrite    <= 1'b0;
		end else if (Cache_Stall | Compression_Stall) begin
			ALU_result  <= ALU_result;
			MemWrt_data <= MemWrt_data;
			WrtBack_addr<= WrtBack_addr;  
			J_StorePC   <= J_StorePC;
			is_Jump     <= is_Jump;    
			MemtoReg    <= MemtoReg;    
			RegWrite    <= RegWrite;    
			MemRead     <= MemRead;     
			MemWrite    <= MemWrite;       
		end else begin
			ALU_result  <= nxt_ALU_result;
			MemWrt_data <= {nxt_MemWrt_data[7:0], nxt_MemWrt_data[15:8], nxt_MemWrt_data[23:16], nxt_MemWrt_data[31:24]}; // Endian
			WrtBack_addr<= nxt_WrtBack_addr;  
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
			  input Cache_Stall,
			  input	Compression_Stall,
			  input       [31:0] nxt_ALU_result,
			  input       [31:0] nxt_MemRd_data,
			  input        [4:0] nxt_WrtBack_addr,
			  input       [31:0] nxt_J_StorePC,
			  input              nxt_is_Jump,
			  output reg  [31:0] ALU_result,
			  output reg  [31:0] MemRd_data,
			  output reg   [4:0] WrtBack_addr,
			  output reg  [31:0] J_StorePC,
			  output reg         is_Jump,
			  // Control Signals
			  input      nxt_MemtoReg,
			  input      nxt_RegWrite,
			  output reg MemtoReg,
			  output reg RegWrite
			  // Forwarding is WrtBack_addr
	);
	always@ (posedge clk) begin
		if (!rst_n) begin
				ALU_result <= 32'b0;
				MemRd_data <= 32'b0;
				WrtBack_addr<= 5'b0;
				J_StorePC  <= 32'b0;
				is_Jump    <= 1'b0;
				MemtoReg   <= 1'b0;
				RegWrite   <= 1'b0;
		end else if (Cache_Stall | Compression_Stall) begin
				ALU_result <= ALU_result;
				MemRd_data <= MemRd_data;
				WrtBack_addr<= WrtBack_addr;   
				J_StorePC  <= J_StorePC;
				is_Jump    <= is_Jump;   
				MemtoReg   <= MemtoReg;  
				RegWrite   <= RegWrite;    
		end else begin
				ALU_result <= nxt_ALU_result;
				MemRd_data <= {nxt_MemRd_data[7:0], nxt_MemRd_data[15:8], nxt_MemRd_data[23:16], nxt_MemRd_data[31:24]};
				WrtBack_addr    <= nxt_WrtBack_addr;   
				J_StorePC  <= nxt_J_StorePC;
				is_Jump    <= nxt_is_Jump;   
				MemtoReg   <= nxt_MemtoReg;  
				RegWrite   <= nxt_RegWrite;    
		end
	end
endmodule

