// 4 blocks * 4 words
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
    input              clk;
    // processor interface
    input              proc_reset;
    input              proc_read, proc_write;
    input       [29:0] proc_addr;
    input       [31:0] proc_wdata;
    output reg         proc_stall;
    output reg  [31:0] proc_rdata;
    // memory interface
    input      [127:0] mem_rdata;
    input              mem_ready;
    output reg         mem_read; 
    output reg         mem_write;
    output reg  [27:0] mem_addr;
    output reg [127:0] mem_wdata;

// procedure addr
parameter ADDRTAGBEG  = 29, // 26 tags
          ADDRTAGEND  = 4, 
          BLOCKIDXBEG = 3,  // 4 blocks (2 bits)
          BLOCKIDXEND = 2,
          WORDIDXBEG  = 1,  // 4 words (2 bits)
          WORDIDXEND  = 0;

// cache structure
// cache (155 bits) = 1 valid + 26 tag + 4 words data (128 bits)
parameter BLOCKSIZE = 155,
          BLOCKNUM = 4,
          BLOCKBIT = 2, // 2 bits to represent blocks
          VALIDBIT = 154,
          TAGBEG = 153,
          TAGEND = 128,
          DATA3BEG = 127,
          DATA3END = 96,
          DATA2BEG = 95,
          DATA2END = 64,
          DATA1BEG = 63,
          DATA1END = 32,
          DATA0BEG = 31,
          DATA0END = 0;

reg [BLOCKSIZE-1:0] cache [0:BLOCKNUM-1];
reg [BLOCKSIZE-1:0] cache_nxt [0:BLOCKNUM-1];
reg                 dirtyBlock [0:BLOCKNUM-1];
reg                 dirtyBlock_nxt [0:BLOCKNUM-1];
wire                isBlockHit;
wire                isBlockDirty;

wire [BLOCKBIT-1:0] block_index;

// state name
parameter [1:0] IDLE   = 2'b00,
                CMPTAG = 2'b01,
                RDMEM  = 2'b11,
                WRTMEM = 2'b10;
reg [1:0]       state, state_nxt;

integer i;

// assignments of wires
assign block_index = proc_addr[BLOCKIDXBEG:BLOCKIDXEND];
assign isBlockDirty = dirtyBlock [block_index];
assign isBlockHit = (cache [block_index][TAGBEG:TAGEND] == proc_addr[ADDRTAGBEG:ADDRTAGEND]) &
                    cache [block_index][VALIDBIT];

//==== combinational circuit ==============================
// FSM
always@ (*) begin
    state_nxt = state;
    case (state)
        IDLE  : state_nxt = CMPTAG;
        CMPTAG: begin
            if (isBlockDirty & ~isBlockHit & (proc_read ^ proc_write)) state_nxt = WRTMEM;
            else if (~isBlockHit & (proc_read ^ proc_write)) state_nxt = RDMEM; 
        end
        RDMEM  : state_nxt = mem_ready ? CMPTAG: RDMEM; 
        WRTMEM : state_nxt = mem_ready ? RDMEM : WRTMEM;
    endcase
end

// output
always@ (*) begin
    proc_stall = 1'b0;
    proc_rdata = 32'b0_0010011000000000000_00000_000_0000;
    mem_read   = 1'b0;
    mem_write  = 1'b0;
    mem_wdata  = 128'b0;
    mem_addr   = proc_addr [ADDRTAGBEG:BLOCKIDXEND];
    for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
        cache_nxt [i] = cache [i];
        dirtyBlock_nxt [i] = dirtyBlock [i];
    end

    case (state) 
        IDLE  : proc_stall = 1'b1;
        CMPTAG: begin
            if (~isBlockHit & (proc_read ^ proc_write)) proc_stall = 1'b1;
            if (proc_read & ~proc_write) begin // read
                case (proc_addr [1:0])
                    2'b00: proc_rdata = cache [block_index][DATA0BEG:DATA0END];
                    2'b01: proc_rdata = cache [block_index][DATA1BEG:DATA1END];
                    2'b10: proc_rdata = cache [block_index][DATA2BEG:DATA2END];
                    2'b11: proc_rdata = cache [block_index][DATA3BEG:DATA3END];
                endcase
            end
            else if (proc_write & ~proc_read & isBlockHit) begin // write if the block clean and hit
                dirtyBlock_nxt [block_index] = 1'b1;
                case (proc_addr [1:0])
                    2'b00: cache_nxt [block_index][DATA0BEG:DATA0END] = proc_wdata;
                    2'b01: cache_nxt [block_index][DATA1BEG:DATA1END] = proc_wdata;
                    2'b10: cache_nxt [block_index][DATA2BEG:DATA2END] = proc_wdata;
                    2'b11: cache_nxt [block_index][DATA3BEG:DATA3END] = proc_wdata;
                endcase
            end
        end
        RDMEM  : begin
            proc_stall = 1'b1;
            mem_read  = 1'b1;
            cache_nxt [block_index][DATA3BEG:DATA0END] = mem_rdata; // set data
            cache_nxt [block_index][TAGBEG:TAGEND] = proc_addr [ADDRTAGBEG:ADDRTAGEND]; // set tag
            cache_nxt [block_index][VALIDBIT] = 1'b1; // set valid
        end
        WRTMEM : begin
            proc_stall = 1'b1;
            dirtyBlock_nxt [block_index] = 1'b0;
            mem_write = 1'b1;
            mem_wdata = cache [block_index][DATA3BEG:DATA0END];
            mem_addr = {cache [block_index][TAGBEG:TAGEND], block_index};
        end
    endcase
end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state <= IDLE;
        for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
            cache [i] <= {BLOCKSIZE{1'b0}};
            dirtyBlock [i] <= 1'b0;
        end
    end
    else begin
        state <= state_nxt;
        for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
            cache [i] <= cache_nxt [i];
            dirtyBlock [i] <= dirtyBlock_nxt [i];
        end
    end
end

endmodule
