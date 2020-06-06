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

//Status : Finish
module MainControl(Inst,Jalr,Jal,BNE,BEQ,MemRead,MemWrite,MemtoReg,ALUSrc,RegWrite,Jtype);
	input [5:0] Inst; //funct3[0] OP[6:2] (INST[12] INST[6:2])
    output reg Jalr;
    output reg Jal;
    output reg BNE;
	output reg BEQ;
    output reg MemRead;
	output reg MemWrite; 
    output reg MemtoReg;
    output reg ALUSrc;
    output reg RegWrite;
	output Jtype;
	assign Jtype = Jalr|Jal;

	always@(*) begin
		case (Inst[4:0])
			5'b01100: begin //R type
                    	Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b1;
                        end
            5'b00000: begin //lw
                        Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'b1; MemWrite = 1'b0; MemtoReg = 1'b1; ALUSrc = 1'b1; RegWrite=1'b1;
                        end
            5'b01000: begin //sw
                        Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'b0; MemWrite = 1'b1; MemtoReg = 1'bx; ALUSrc = 1'b1; RegWrite=1'b0;
                        end
            5'b11000: begin //beq bne
                        Jalr = 1'b0; Jal = 1'b0; BEQ = ~Inst[5]; BNE = Inst[5]; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'bx; ALUSrc = 1'b0; RegWrite=1'b0;
                        end
            5'b11011: begin //jal
                        Jalr = 1'b0; Jal = 1'b1; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'bx; ALUSrc = 1'bx; RegWrite=1'b1;
                        end
            5'b11001: begin //jalr
						Jalr = 1'b1; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'bx; MemWrite = 1'b0; MemtoReg = 1'bx; ALUSrc = 1'bx; RegWrite=1'b1;
                        end  
			default:  begin
				Jalr = 1'b0; Jal = 1'b0; BEQ = 1'b0; BNE = 1'b0; MemRead = 1'b0; MemWrite = 1'b0; MemtoReg = 1'b0; ALUSrc = 1'b0; RegWrite=1'b0;
			end
		endcase
	end
endmodule

//Status : Finish
module ALUControl(Inst,ALUCtrl);
	input [4:0] Inst; // {funct7[5],funct3,OP[4]}
    output reg[3:0] ALUCtrl;
	assign ALUCtrl = Inst[0] ? Inst[4:1] : 4'b0000;
endmodule

//Status : Finish
module ImmGenerator(Inst,Immediate);
	input [31:0] Inst;
    output reg[31:0] Immediate;
    wire [3:0] Typectrl ;
	wire [2:0] Funct3;
    //0000/1101:I 0100:S 1100:B 1111:J
    wire [31:0] I,S,B,J;
	assign Funct3 = Inst[14:12];
    assign Typectrl = {Inst[6:5],Inst[3:2]};
    assign I = {{21{Inst[31]}},Inst[30:20]};
	assign ISHAMT = {27'b0,Inst[24:20]};
    assign S = {{21{Inst[31]}},Inst[30:25],Inst[11:7]};
    assign B = {{20{Inst[31]}},Inst[7],Inst[30:25],Inst[11:8],1'b0};
    assign J = {{12{Inst[31]}},Inst[19:12],Inst[20],Inst[30:21],1'b0};
	always @(*) begin
		case (Typectrl)
			Immediate = 32'b0;
			4'b0000: Immediate = (Funct3==3'b101) ? ISHAMT : I;
			4'b1101: Immediate = I;
			4'b0100: Immediate = S;
			4'b1100: Immediate = B;
			4'b1111: Immediate = J; 
			default: Immediate = 0; 
		endcase
	end
endmodule

module FowardingUnit(RS1_A,RS2_A,WD_A_EXMEM,WD_A_MEMWR,1A,1B,2A,2B);
	input [4:0] RS1_A,RS2_A,WD_A_EXMEM,WD_A_MEMWR;
	output 1A,1B,2A,2B;
	assign 1A = (RS1_A==WD_A_EXMEM) ? 1:0;
	assign 1B = (RS1_A==WD_A_MEMWR) ? 1:0;
	assign 2A = (RS2_A==WD_A_EXMEM) ? 1:0;
	assign 2B = (RS2_A==WD_A_MEMWR) ? 1:0;
endmodule

//Status : Finish
module ALU(X,Y,Ctrl,Result);
	input [31:0] X; //RS1
    input [31:0] Y; //RS2 IMM
    input [3:0] Ctrl; //ALUOp
    output reg[31:0] Result;
	always @(*) begin
		Result = 32'b0;
		case (Ctrl[2:0])
			3'b000: begin
				if(Ctrl[3]): Result = $signed(X)-$signed(Y);
				else:		 Result = $signed(X)+$signed(Y);
			end
			3'b001: Result = X<<1; //SLL
			3'b010: Result = ( (X<Y) ? 1 : 0 ); //SLT
			3'b011: Result = 32'b0; 
			3'b100: Result = X^Y;
			3'b101:  begin
				if(Ctrl[3]): Result = {X[31],X[31:1]}; //SRA
				else:		 Result = {1'b0,X[31:1]}; //SRL
			end 
			3'b110: X|Y;
			3'b111: X&Y;
			default: Result = 32'b0;
		endcase
	end
endmodule

//Status : Finish
module MainRegister(clk,rst_n,WD_D,RS1_A,RS2_A,WD_A,RegWrite,RS1_D,RS2_D);

	input clk,rst_n;
	input RegWrite; //RegWrite=1 if write
    input [4:0] RS1_A,RS2_A,WD_A;
    input [31:0] WD_D;
    output [31:0] RS1_D,RS2_D;
    
    integer i;
    reg [31:0] r32[0:31];
    reg [31:0] nxt_r32[0:31];

    assign RS1_D = r32[RS1_A];
    assign RS2_D = r32[RS2_A];
	always @(*) begin
        nxt_r32[0] = 0; //Const 0;
        for (i=1; i<32; i=i+1)begin
            nxt_r32[i] = r32[i];
        end
        for (i=1; i<32; i=i+1)begin
            nxt_r32[i] = rst_n ? (RegWrite && (WD_A == i)) ? WD_D : r32[i] : 0;
        end
    end    
    
    always @(posedge clk) begin
        r32[0] <= 0;
        for (i=1; i<32; i=i+1)
            r32[i] <= nxt_r32[i];
    end
endmodule
//Status : Finish
module PC(clk,rst_n,RS1_D,RS2_D,nxtr_PC,IMM,Jalr,Jal,BEQ,BNE,cur_PC,nxtr_PC4);
	input clk,rst_n;
	input Jalr,Jal,BEQ,BNE;
	input [31:0] RS1_D,RS2_D,nxtr_PC,IMM;
	output [31:0] cur_PC,nxtr_PC4;

	wire [31:0] nxt_PC,IMMRS1,IMMPC;
	wire zero,JalB;
	PCRegister PCR(.clk(clk),.rst_n(rst_n),.PC(cur_PC),.nxt_PC(nxt_PC));

	assign zero = (RS1_D == RS2_D) ? 1'b1 : 1'b0;
	assign JalB = (zero&BEQ)|(~zero&BNE)|Jal;

	assign IMMRS1 = IMM + RS1_D;
	assign IMMPC = IMM + nxtr_PC;
	assign nxtr_PC4 = nxtr_PC + 4;
	assign cur_PC4 = cur_PC + 4;

	assign nxt_PC = Jalr ? IMMRS1 : 
		   			jalB ? IMMPC : cur_PC4;

endmodule
//Status : Finish
module PCRegister(clk,rst_n,PC,nxt_PC);
	input clk;
    input rst_n;
    input [31:0] nxt_PC;
    output [31:0] PC;

    reg [31:0] r_PC,nxtr_PC;
    
    assign PC = r_PC;
    always@(*) begin
        nxtr_PC = rst_n ? nxt_PC : 0;
    end
    always@(posedge clk) begin
        r_PC <= nxtr_new;
    end
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
		nxtr_PC = rst_n ? PC : 32'b0;
		nxtr_Inst = rst_n ? Inst : 32'b0;
	end
	always@(posedge clk) begin
		PC <= nxtr_PC;
		Inst <= nxtr_Inst;
	end
endmodule
//Status : Finish
module IDEX(clk,rst_n,Jtype,RS1_D,RS2IMM_D,RS1_A,RS2_A,WD_A,ALUOp,MemtoReg,RegWrite,MemRead,MemWrite,ALUsrc,Jctrl;
nxt_Jtype,nxt_RS1_D,nxt_RS2IMM_D,nxt_RS1_A,nxt_RS2_A,nxt_WD_A,nxt_ALUOp,nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_ALUsrc,nxt_Jctrl);
	input clk,rst_n;
	output reg MemtoReg,RegWrite,MemRead,MemWrite,ALUsrc,Jctrl;
	output reg [3:0] ALUOp;
	output reg [4:0] RS1_A,RS2_A,WD_A;
	output reg [31:0] RS1_D,RS2IMM_D,Jtype;
	input nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_ALUsrc,nxt_Jctrl;
	input [3:0] nxt_ALUOp;
	input [4:0] nxt_RS1_A,nxt_RS2_A,nxt_WD_A;
	input [31:0] nxt_RS1_D,nxt_RS2IMM_D,nxt_Jtype;

	reg nxtr_MemtoReg,nxtr_RegWrite,nxtr_MemRead,nxtr_MemWrite,nxtr_ALUsrc,nxtr_Jctrl;
	reg [3:0] nxtr_ALUOp;
	reg [4:0] nxtr_RS1_A,nxtr_RS2_A,nxtr_WD_A;
	reg [31:0] nxtr_RS1_D,nxtr_RS2IMM_D,nxtr_Jtype;

	always@(*) begin
		nxtr_Jtype = rst_n ? nxt_Jtype : 32'b0;
		nxtr_RS1_D = rst_n ? nxt_RS1_D : 32'b0;
		nxtr_RS2IMM_D = rst_n ? nxt_RS2IMM_D : 32'b0;
		nxtr_RS1_A = rst_n ? nxt_RS1_A : 5'b0;
		nxtr_RS2_A = rst_n ? nxt_RS2_A : 5'b0;
		nxtr_WD_A = rst_n ? nxt_WD_A : 5'b0;
		nxtr_ALUOp = rst_n ? nxt_ALUOp : 4'b0;
		nxtr_MemtoReg = rst_n ? nxt_MemtoReg : 1'b0;
		nxtr_RegWrite = rst_n ? nxt_RegWrite : 1'b0;
		nxtr_MemRead = rst_n ? nxt_MemRead : 1'b0;
		nxtr_MemWrite = rst_n ? nxt_MemWrite : 1'b0;
		nxtr_ALUsrc = rst_n ? nxt_ALUsrc : 1'b0;
		nxtr_Jctrl = rst_n ? nxt_Jctrl : 1'b0;
	end
	always@(posedge clk) begin
		{Jtype,RS1_D,RS2IMM_D,RS1_A,RS2_A,WD_A,ALUOp,MemtoReg,RegWrite,MemRead,MemWrite,ALUsrc,Jctrl} <= {nxtr_Jtype,nxtr_RS1_D,nxtr_RS2IMM_D,nxtr_RS1_A,nxtr_RS2_A,nxtr_WD_A,nxtr_ALUOp,nxtr_MemtoReg,nxtr_RegWrite,nxtr_MemRead,nxtr_MemWrite,nxtr_ALUsrc,nxtr_Jctrl};	
	end
endmodule
//Status : Finish
module EXMEM(clk,rst_n,Jtype,ALU,RS2IMM,WD_A,MemtoReg,RegWrite,MemRead,MemWrite,Jctrl,
nxt_Jtype,nxt_ALU,nxt_RS2IMM,nxt_WD_A,nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_Jctrl);
	input clk,rst_n;
	output reg [31:0] Jtype,ALU,RS2IMM;
	output reg [4:0] WD_A;
	output reg MemtoReg,RegWrite,MemRead,MemWrite,Jctrl;
	input [31:0] nxt_Jtype,nxt_ALU,nxt_RS2IMM;
	input [4:0] nxt_WD_A;
	input nxt_MemtoReg,nxt_RegWrite,nxt_MemRead,nxt_MemWrite,nxt_Jctrl;

	reg [31:0] nxtr_Jtype,nxtr_ALU,nxtr_RS2IMM;
	reg [4:0] nxtr_WD_A;
	reg nxtr_MemtoReg,nxtr_RegWrite,nxtr_MemRead,nxtr_MemWrite,nxtr_Jctrl;

	always@(*) begin
		nxtr_Jtype = rst_n ? nxt_Jtype : 32'b0;
		nxtr_ALU = rst_n ? nxt_ALU : 32'b0;
		nxtr_RS2IMM = rst_n ? nxt_RS2IMM : 32'b0;
		nxtr_WD_A = rst_n ? nxt_WD_A : 5'b0;
		nxtr_MemtoReg = rst_n ? nxt_MemtoReg : 1'b0;
		nxtr_RegWrite = rst_n ? nxt_RegWrite : 1'b0;
		nxtr_MemRead = rst_n ? nxt_MemRead : 1'b0;
		nxtr_MemWrite = rst_n ? nxt_MemWrite : 1'b0;
		nxtr_Jctrl = rst_n ? nxt_Jctrl : 1'b0;
	end

	always@(posedge clk) begin
		Jtype <= nxtr_Jtype;
		ALU <= nxtr_ALU;
		RS2IMM <= nxtr_RS2IMM;
		WD_A <= nxtr_WD_A;
		MemtoReg <= nxtr_MemtoReg;
		RegWrite <= nxtr_RegWrite;
		MemRead <= nxtr_MemRead;
		MemWrite <= nxtr_MemWrite;
		Jctrl <= nxtr_Jctrl;
	end

endmodule
//Status : Finish
module MEMWB(clk,rst_n,Jtype,ALU,MEM,WD_A,MemtoReg,RegWrite,Jctrl,nxt_Jtype,nxt_ALU,nxt_MEM,nxt_WD_A,nxt_MemtoReg,nxt_RegWrite,nxt_Jctrl);
	input clk,rst_n;
	output reg [31:0] Jtype,ALU,MEM;
	output reg [4:0] WD_A;
	output reg MemtoReg,RegWrite,Jctrl;
	input [31:0] nxt_Jtype,nxt_ALU,nxt_MEM;
	input [4:0] nxt_WD_A;
	input nxt_MemtoReg,nxt_RegWrite,nxt_Jctrl;

	reg [31:0] nxtr_Jtype,nxtr_ALU,nxtr_MEM;
	reg [4:0] nxtr_WD_A;
	reg nxtr_MemtoReg,nxtr_RegWrite,nxtr_Jctrl;

	always@(*)begin
		nxtr_Jtype = rst_n ? nxt_Jtype : 32'b0;
		nxtr_ALU = rst_n ? nxt_ALU : 32'b0; 
		nxtr_MEM = rst_n ? nxt_MEM : 32'b0;
		nxtr_WD_A = rst_n ? nxt_WD_A : 5'b0;
		nxtr_MemtoReg = rst_n ? nxt_MemtoReg : 1'b0;
		nxtr_RegWrite = rst_n ? nxt_RegWrite : 1'b0;
		nxtr_Jctrl = rst_n ? nxt_Jctrl : 1'b0;
	end

	always@(posedge clk)begin
		Jtype <= nxtr_Jtype;
		ALU <= nxtr_ALU; 
		MEM <= nxtr_MEM;
		WD_A <= nxtr_WD_A;
		MemtoReg <= nxtr_MemtoReg;
		RegWrite <= nxtr_RegWrite;
		Jctrl <= nxtr_Jctrl;
	end

endmodule

