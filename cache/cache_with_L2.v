module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;

    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================
wire [127:0] L2_rdata;
wire         L2_ready;
wire         L2_read, L2_write; 
wire  [27:0] L2_addr;
wire [127:0] L2_wdata;

// module instantiation
L1_cache L1 (.clk(clk), 
             .proc_reset(proc_reset),  
             .proc_read(proc_read),
             .proc_write(proc_write),
             .proc_addr(proc_addr),
             .proc_wdata(proc_wdata),
             .proc_stall(proc_stall),
             .proc_rdata(proc_rdata),
             .L2_rdata(L2_rdata),
             .L2_ready(L2_ready),
             .L2_read(L2_read),
             .L2_write(L2_write),
             .L2_addr(L2_addr),
             .L2_wdata(L2_wdata));

L2_cache L2 (.clk(clk),
             .proc_reset(proc_reset),
             .L2_read(L2_read),
             .L2_write(L2_write),
             .L2_addr(L2_addr),
             .L2_wdata(L2_wdata),
             .L2_rdata(L2_rdata),
             .L2_ready(L2_ready),
             .mem_rdata(mem_rdata),
             .mem_ready(mem_ready),
             .mem_read(mem_read),
             .mem_write(mem_write),
             .mem_addr(mem_addr),
             .mem_wdata(mem_wdata));
endmodule

module L1_cache (
    input             clk,
    input             proc_reset,
    // processor interface
    // addr: 32 bits, data: 32 bits
    input             proc_read, proc_write,
    input      [29:0] proc_addr, // 30 bits due to word-offset
    input      [31:0] proc_wdata,
    output reg        proc_stall,
    output reg [31:0] proc_rdata,
    // L2 interface 
    input      [127:0] L2_rdata,
    input              L2_ready,
    output reg         L2_read, L2_write,
    output reg  [27:0] L2_addr,
    output reg [127:0] L2_wdata
);
// procedure addr
parameter CACHENUMSTART = 4,
          CACHENUMEND   = 2,
          BLOCKIDXSTART = 1,
          BLOCKIDXEND   = 0,
          PROCTAGSTART  = 29,
          PROCTAGEND    = 5; // 25 tags

// L1 Cache (154 bits) = 1 valid + 25 tag + 4 words data (128 bits)
// TAG: 25 bits (30 bits proc_addr - 2 bits block offset - 3 bits index)
parameter BLOCKSIZE = 154,
          CACHENUM  = 8,
          VALIDBIT = 153,
          TAGSTART = 152,
          TAGEND   = 128,
          DATA3START = 127,
          DATA3END   = 96,
          DATA2START = 95,
          DATA2END   = 64,
          DATA1START = 63,
          DATA1END   = 32,
          DATA0START = 31,
          DATA0END   = 0; 

reg [BLOCKSIZE-1:0] cache [0:CACHENUM-1];
reg [BLOCKSIZE-1:0] cache_nxt [0:CACHENUM-1];
reg  dirtyBlock [0:CACHENUM-1];
reg  dirtyBlock_nxt [0:CACHENUM-1];
wire isBlockDirty, isBlockHit;                  

// state name
parameter [1:0] IDLE   = 2'b00,
                CMPTAG = 2'b01,
                RDMEM  = 2'b11,
                WRTMEM = 2'b10;
reg [1:0]       state, state_nxt;

integer i;

// assignments of wires
assign isBlockDirty = dirtyBlock [proc_addr[CACHENUMSTART:CACHENUMEND]];
assign isBlockHit   = (cache [proc_addr[CACHENUMSTART:CACHENUMEND]][TAGSTART:TAGEND] == proc_addr[PROCTAGSTART:PROCTAGEND]) &
                      cache [proc_addr[CACHENUMSTART:CACHENUMEND]][VALIDBIT];

// FSM
always@ (*) begin
    state_nxt = state;
    case (state)
        IDLE  : state_nxt = CMPTAG;
        CMPTAG: begin
            if (isBlockDirty && ~isBlockHit) state_nxt = WRTMEM;
            else if (~isBlockHit) state_nxt = RDMEM; 
        end
        RDMEM  : state_nxt = L2_ready ? CMPTAG: RDMEM; 
        WRTMEM : state_nxt = L2_ready ? RDMEM : WRTMEM;
    endcase
end

// combinational output
always@ (*) begin
    proc_stall = 1'b1;
    proc_rdata = 32'b0;
    L2_read    = 1'b0;
    L2_write   = 1'b0;
    L2_addr    = proc_addr [PROCTAGSTART:CACHENUMEND];
    L2_wdata   = 128'b0;
    for (i = 0; i <= CACHENUM-1; i = i + 1) begin
        cache_nxt [i] = cache [i];
        dirtyBlock_nxt [i] = dirtyBlock [i];
    end

    case (state)
        CMPTAG: begin
            proc_stall = (~isBlockHit & (proc_read ^ proc_write));
            if (proc_read && ~proc_write) begin // read
                case (proc_addr [1:0])
                    2'b00: proc_rdata = cache [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA0START:DATA0END];
                    2'b01: proc_rdata = cache [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA1START:DATA1END];
                    2'b10: proc_rdata = cache [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA2START:DATA2END];
                    2'b11: proc_rdata = cache [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA3START:DATA3END];
                endcase
            end
            else if (proc_write & ~proc_read & isBlockHit) begin // write if the block clean and hit
                dirtyBlock_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]] = 1'b1;
                case (proc_addr [1:0])
                    2'b00: cache_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA0START:DATA0END] = proc_wdata;
                    2'b01: cache_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA1START:DATA1END] = proc_wdata;
                    2'b10: cache_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA2START:DATA2END] = proc_wdata;
                    2'b11: cache_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA3START:DATA3END] = proc_wdata;
                endcase
            end
        end
        RDMEM  : begin
            L2_read  = 1'b1;
            cache_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA3START:DATA0END] = L2_rdata; // set data
            cache_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]][TAGSTART:TAGEND] = proc_addr [PROCTAGSTART:PROCTAGEND]; // set tag
            cache_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]][VALIDBIT] = 1'b1; // set valid
        end
        WRTMEM : begin
            dirtyBlock_nxt [proc_addr [CACHENUMSTART:CACHENUMEND]] = 1'b0;
            L2_write = 1'b1;
            L2_wdata = cache [proc_addr [CACHENUMSTART:CACHENUMEND]][DATA3START:DATA0END];
            L2_addr  = {cache [proc_addr [CACHENUMSTART:CACHENUMEND]][TAGSTART:TAGEND], proc_addr [CACHENUMSTART:CACHENUMEND]};
        end

    endcase
end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state <= IDLE;
        for (i = 0; i <= CACHENUM-1; i = i + 1) begin
            cache [i] <= {BLOCKSIZE{1'b0}};
            dirtyBlock [i] <= 1'b0;
        end
    end
    else begin
        state <= state_nxt;
        for (i = 0; i <= CACHENUM-1; i = i + 1) begin
            cache [i] <= cache_nxt [i];
            dirtyBlock [i] <= dirtyBlock_nxt [i];
        end
    end
end

// TODO: synchronous active low reset
endmodule

module L2_cache (
    input clk,
    input proc_reset,
    // L1 interface
    // addr: 28 bits, data: 128 bits
    input              L2_read, L2_write,
    input       [27:0] L2_addr,
    input      [127:0] L2_wdata,
    output reg [127:0] L2_rdata,
    output reg         L2_ready,
    // memory interface
    input      [127:0] mem_rdata,
    input              mem_ready,
    output reg         mem_read, mem_write,
    output reg  [27:0] mem_addr,
    output reg [127:0] mem_wdata
);

// procedure addr
parameter CACHENUMSTART = 5,
          CACHENUMEND   = 0,
          PROCTAGSTART  = 27,
          PROCTAGEND    = 6; // 22 tags

// L2 Cache (151 bits) = 1 valid + 22 tag + 4 words data (128 bits)
// TAG: 22 bits (28 bits L1_addr - 6 bits index)
parameter BLOCKSIZE = 151,
          CACHENUM  = 64,
          VALIDBIT = 150,
          TAGSTART = 149,
          TAGEND   = 128,
          DATASTART = 127,
          DATAEND   = 0; 

reg [BLOCKSIZE-1:0] cache [0:CACHENUM-1];
reg [BLOCKSIZE-1:0] cache_nxt [0:CACHENUM-1];
reg  dirtyBlock [0:CACHENUM-1];
reg  dirtyBlock_nxt [0:CACHENUM-1];
wire isBlockDirty, isBlockHit;

// state name
parameter [1:0] IDLE   = 2'b00,
                CMPTAG = 2'b01,
                RDMEM  = 2'b11,
                WRTMEM = 2'b10;
reg [1:0]       state, state_nxt;

integer i;

// assignments of wires
assign isBlockDirty = dirtyBlock [L2_addr[CACHENUMSTART:CACHENUMEND]];
assign isBlockHit   = (cache [L2_addr[CACHENUMSTART:CACHENUMEND]][TAGSTART:TAGEND] == L2_addr[PROCTAGSTART:PROCTAGEND]) &
                      cache [L2_addr[CACHENUMSTART:CACHENUMEND]][VALIDBIT];

// FSM
always@ (*) begin
    state_nxt = state;
    case (state)
        IDLE  : state_nxt = CMPTAG;
        CMPTAG: begin
            if (isBlockDirty && ~isBlockHit) state_nxt = WRTMEM;
            else if (~isBlockHit) state_nxt = RDMEM; 
        end
        RDMEM  : state_nxt = mem_ready ? CMPTAG: RDMEM; 
        WRTMEM : state_nxt = mem_ready ? RDMEM : WRTMEM;
    endcase
end

// combinational output
always@ (*) begin
    L2_rdata = 128'b0;
    L2_ready = 1'b0;
    mem_read = 1'b0;
    mem_write = 1'b0;
    mem_addr = L2_addr; // all addr copied
    mem_wdata = 128'b0;
    for (i = 0; i <= CACHENUM-1; i = i + 1) begin
        cache_nxt [i] = cache [i];
        dirtyBlock_nxt [i] = dirtyBlock [i];
    end

    case (state)
        CMPTAG: begin
            if (L2_read && ~L2_write & isBlockHit) begin // read
                L2_rdata = cache [L2_addr [CACHENUMSTART:CACHENUMEND]][DATASTART:DATAEND];
                L2_ready = 1'b1;
            end
            else if (L2_write & ~L2_read & isBlockHit) begin // write if the block hit
                dirtyBlock_nxt [L2_addr [CACHENUMSTART:CACHENUMEND]] = 1'b1;
                cache_nxt [L2_addr [CACHENUMSTART:CACHENUMEND]][DATASTART:DATAEND] = L2_wdata;
                L2_ready = 1'b1;
            end
        end
        RDMEM  : begin
            mem_read  = 1'b1;
            cache_nxt [L2_addr [CACHENUMSTART:CACHENUMEND]][DATASTART:DATAEND] = mem_rdata; // set data
            cache_nxt [L2_addr [CACHENUMSTART:CACHENUMEND]][TAGSTART:TAGEND] = L2_addr [PROCTAGSTART:PROCTAGEND]; // set tag
            cache_nxt [L2_addr [CACHENUMSTART:CACHENUMEND]][VALIDBIT] = 1'b1; // set valid
        end
        WRTMEM : begin
            dirtyBlock_nxt [L2_addr [CACHENUMSTART:CACHENUMEND]] = 1'b0;
            mem_write = 1'b1;
            mem_wdata = cache [L2_addr [CACHENUMSTART:CACHENUMEND]][DATASTART:DATAEND];
            mem_addr  = {cache [L2_addr [CACHENUMSTART:CACHENUMEND]][TAGSTART:TAGEND], L2_addr [CACHENUMSTART:CACHENUMEND]};
        end
    endcase
end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state <= IDLE;
        for (i = 0; i <= CACHENUM-1; i = i + 1) begin
            cache [i] <= {BLOCKSIZE{1'b0}};
            dirtyBlock [i] <= 1'b0;
        end
    end
    else begin
        state <= state_nxt;
        for (i = 0; i <= CACHENUM-1; i = i + 1) begin
            cache [i] <= cache_nxt [i];
            dirtyBlock [i] <= dirtyBlock_nxt [i];
        end
    end
end
endmodule