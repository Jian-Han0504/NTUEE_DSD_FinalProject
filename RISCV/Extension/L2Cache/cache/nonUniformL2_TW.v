// I cache: Dirrect mapped
// D cache: two-way associated
module cache(
        input             clk,
        input             proc_reset,
        // processor interface
        input             proc_read, 
        input             proc_write,
        input      [29:0] proc_addr, // 30 bits due to word-offset
        input      [31:0] proc_wdata,
        output reg        proc_stall,
        output reg [31:0] proc_rdata,
        // L2 interface
        input              L2_ready,
        input      [127:0] L2_rdata, // 4 words (128 bits)
        output reg         L2_read, 
        output reg         L2_write,
        output reg  [27:0] L2_addr, // 28 bits since read 4 words once
        output reg [127:0] L2_wdata
);
// procedure addr
parameter ADDRTAGBEG  = 29, // 26 tags
          ADDRTAGEND  = 4, 
          BLOCKIDXBEG = 3,  // 4 blocks (2 bits)
          BLOCKIDXEND = 2,
          WORDIDXBEG  = 1,  // 4 words (2 bit)
          WORDIDXEND  = 0;
              
// L1 Cache (155 bits) = 1 valid + 26 tag + 4 words data (128 bits)
// TAG: 25 bits (30 bits proc_addr - 2 bits block offset - 3 bits index)
parameter BLOCKSIZE = 155,
          BLOCKNUM = 4,
          VALIDBIT = 154,
          TAGBEG   = 153,
          TAGEND   = 128,
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
wire                 isBlockHit;
wire                isBlockDirty;

// state name
parameter [1:0] IDLE   = 2'b00,
                CMPTAG = 2'b01,
                RDMEM  = 2'b11,
                WRTMEM = 2'b10;
reg [1:0]       state, state_nxt;

integer i;

// assignments of wires
assign isBlockDirty = dirtyBlock [proc_addr[BLOCKIDXBEG:BLOCKIDXEND]];
assign isBlockHit = (cache[proc_addr[BLOCKIDXBEG:BLOCKIDXEND]][TAGBEG:TAGEND] == proc_addr[ADDRTAGBEG:ADDRTAGEND]) &
                    cache[proc_addr[BLOCKIDXBEG:BLOCKIDXEND]][VALIDBIT];

// FSM
always@ (*) begin
    state_nxt = state;
    case (state)
        IDLE  : state_nxt = CMPTAG;
        CMPTAG: begin
            if (isBlockDirty & ~isBlockHit & (proc_read ^ proc_write)) state_nxt = WRTMEM;
            else if (~isBlockHit & (proc_read ^ proc_write)) state_nxt = RDMEM; 
        end
        RDMEM  : state_nxt = L2_ready ? CMPTAG: RDMEM; 
        WRTMEM : state_nxt = L2_ready ? RDMEM : WRTMEM;
    endcase
end

// combinational output
always@ (*) begin
    proc_stall = 1'b0;
    proc_rdata = 32'b0;
    L2_read    = 1'b0;
    L2_write   = 1'b0;
    L2_addr    = proc_addr [ADDRTAGBEG:BLOCKIDXEND];
    L2_wdata   = 128'b0;
    for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
        cache_nxt [i] = cache [i];
        dirtyBlock_nxt [i] = dirtyBlock [i];
    end
    case (state)
        IDLE: proc_stall = 1'b1;
        CMPTAG: begin
            if (~isBlockHit && (proc_read ^ proc_write)) proc_stall = 1'b1;
            if (proc_read && ~proc_write) begin // read
                case (proc_addr [1:0])
                    2'b00: proc_rdata = cache [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATA0BEG:DATA0END];
                    2'b01: proc_rdata = cache [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATA1BEG:DATA1END];
                    2'b10: proc_rdata = cache [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATA2BEG:DATA2END];
                    2'b11: proc_rdata = cache [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATA3BEG:DATA3END];
                endcase
            end
            else if (proc_write & ~proc_read & isBlockHit) begin // write if the block clean and hit
                dirtyBlock_nxt [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]] = 1'b1;
                case (proc_addr [1:0])
                    2'b00: cache_nxt [proc_addr[BLOCKIDXBEG:BLOCKIDXEND]][DATA0BEG:DATA0END] = proc_wdata;
                    2'b01: cache_nxt [proc_addr[BLOCKIDXBEG:BLOCKIDXEND]][DATA1BEG:DATA1END] = proc_wdata;
                    2'b10: cache_nxt [proc_addr[BLOCKIDXBEG:BLOCKIDXEND]][DATA2BEG:DATA2END] = proc_wdata;
                    2'b11: cache_nxt [proc_addr[BLOCKIDXBEG:BLOCKIDXEND]][DATA3BEG:DATA3END] = proc_wdata;
                endcase
            end
        end
        RDMEM  : begin
            proc_stall = 1'b1;
            L2_read  = 1'b1;
            cache_nxt [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATA3BEG:DATA0END] = L2_rdata; // set data
            cache_nxt [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][TAGBEG:TAGEND] = proc_addr [ADDRTAGBEG:ADDRTAGEND]; // set tag
            cache_nxt [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][VALIDBIT] = 1'b1; // set valid
        end
        WRTMEM : begin
            proc_stall = 1'b1;
            dirtyBlock_nxt [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]] = 1'b0;
            L2_write = 1'b1;
            L2_wdata = cache [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATA3BEG:DATA0END];
            L2_addr  = {cache [proc_addr [BLOCKIDXBEG:BLOCKIDXEND]][TAGBEG:TAGEND], proc_addr [BLOCKIDXBEG:BLOCKIDXEND]};
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

// I cache: 16 blocks * 4 wrods
module L2_Icache (
        input clk,
        input proc_reset,
        // L1 I_interface. addr: 28 bits, data: 128 bits
        input              L2_read, 
        input              L2_write,
        input       [27:0] L2_addr,
        input      [127:0] L2_wdata,
        output reg [127:0] L2_rdata,
        output reg         L2_ready,
        // memory interface. addr: 28 bits, data: 128 bits
        input      [127:0] mem_rdata,
        input              mem_ready,
        output reg         mem_read, 
        output reg         mem_write,
        output reg  [27:0] mem_addr,
        output reg [127:0] mem_wdata
    );

    // L1 -> L2 addr
    parameter ADDRTAGBEG  = 27,
              ADDRTAGEND  = 4, // 24-bit tag
              BLOCKIDXBEG = 3, // 16 bloacks (4 bits)
              BLOCKIDXEND = 0;
              

    // L2 Cache (153 bits) = 1 valid + 24 tag + 4 words data (128 bits)
    parameter BLOCKSIZE = 153,
              BLOCKNUM  = 16,
              VALIDBIT  = 152,
              TAGBEG    = 151,
              TAGEND    = 128,
              DATASTART = 127,
              DATAEND   = 0; 

    reg [BLOCKSIZE-1:0] cache [0:BLOCKNUM-1];
    reg [BLOCKSIZE-1:0] cache_nxt [0:BLOCKNUM-1];
    reg                 dirtyBlock [0:BLOCKNUM-1];
    reg                 dirtyBlock_nxt [0:BLOCKNUM-1];
    wire                isBlockHit;
    wire                isBlockDirty;

    // state name
    parameter [1:0] IDLE   = 2'b00,
                    CMPTAG = 2'b01,
                    RDMEM  = 2'b11,
                    WRTMEM = 2'b10;
    reg [1:0]       state, state_nxt;

    integer i;

    // assignments of wires
    assign isBlockDirty = dirtyBlock[L2_addr[BLOCKIDXBEG:BLOCKIDXEND]];
    assign isBlockHit = (cache[L2_addr[BLOCKIDXBEG:BLOCKIDXEND]][TAGBEG:TAGEND] == L2_addr[ADDRTAGBEG:ADDRTAGEND]) &
                        cache[L2_addr[BLOCKIDXBEG:BLOCKIDXEND]][VALIDBIT];

    // FSM
    always@ (*) begin
        state_nxt = state;
        case (state)
                IDLE  : state_nxt = CMPTAG;
                CMPTAG: begin
                    if (isBlockDirty & ~isBlockHit & (L2_read ^ L2_write)) state_nxt = WRTMEM;
                    else if (~isBlockHit & (L2_read ^ L2_write)) state_nxt = RDMEM; 
                end
                RDMEM  : state_nxt = mem_ready ? CMPTAG: RDMEM; 
                WRTMEM : state_nxt = mem_ready ? RDMEM : WRTMEM;
        endcase
    end

    // combinational output
    always@ (*) begin
        // isBlockHit = 1'b0;
        L2_rdata = 128'b0;
        L2_ready = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_addr = L2_addr; // all addr copied
        mem_wdata = 128'b0;
        for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
            cache_nxt [i] = cache [i];
            dirtyBlock_nxt [i] = dirtyBlock [i];
        end

        case (state)
            CMPTAG: begin
                
                if (L2_read & ~L2_write & isBlockHit) begin // read
                    L2_rdata = cache [L2_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATASTART:DATAEND];
                    L2_ready = 1'b1;
                end
                else if (L2_write & ~L2_read & isBlockHit) begin // write if the block hit
                    dirtyBlock_nxt [L2_addr[BLOCKIDXBEG:BLOCKIDXEND]] = 1'b1;
                    cache_nxt [L2_addr[BLOCKIDXBEG:BLOCKIDXEND]][DATASTART:DATAEND] = L2_wdata;
                    L2_ready = 1'b1;
                end
            end
            RDMEM  : begin
                mem_read  = 1'b1;
                cache_nxt [L2_addr[BLOCKIDXBEG:BLOCKIDXEND]][DATASTART:DATAEND] = mem_rdata; // set data
                cache_nxt [L2_addr[BLOCKIDXBEG:BLOCKIDXEND]][TAGBEG:TAGEND] = L2_addr [ADDRTAGBEG:ADDRTAGEND]; // set tag
                cache_nxt [L2_addr[BLOCKIDXBEG:BLOCKIDXEND]][VALIDBIT] = 1'b1; // set valid
            end
            WRTMEM : begin
                dirtyBlock_nxt [L2_addr [BLOCKIDXBEG:BLOCKIDXEND]] = 1'b0;
                mem_write = 1'b1;
                mem_wdata = cache [L2_addr [BLOCKIDXBEG:BLOCKIDXEND]][DATASTART:DATAEND];
                mem_addr  = {cache [L2_addr [BLOCKIDXBEG:BLOCKIDXEND]][TAGBEG:TAGEND], L2_addr [BLOCKIDXBEG:BLOCKIDXEND]};
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

// D cache: 24 blocks * 4 words each * 2 way
// module L2_Dcache (
//         input clk,
//         input proc_reset,
//         // L1 I_interface. addr: 28 bits, data: 128 bits
//         input              L2_read, 
//         input              L2_write,
//         input       [27:0] L2_addr,
//         input      [127:0] L2_wdata,
//         output reg [127:0] L2_rdata,
//         output reg         L2_ready,
//         // memory interface. addr: 28 bits, data: 128 bits
//         input      [127:0] mem_rdata,
//         input              mem_ready,
//         output reg         mem_read, 
//         output reg         mem_write,
//         output reg  [27:0] mem_addr,
//         output reg [127:0] mem_wdata
//     );

//     // L1 -> L2 addr
//     parameter ADDRTAGBIT  = 23,
//               ADDRTAGBEG  = 27,
//               ADDRTAGEND  = 5, // 23-bit proc tag
//               BLOCKIDXBEG = 4, // 24 bloacks (5 bits)
//               BLOCKIDXEND = 0;
              

//     // L2 Cache (154 bits) = 1 valid + 1 last-used + 24 tag + 4 words data (128 bits)
//     parameter BLOCKSIZE = 154,
//               BLOCKNUM  = 24,
//               BLOCKBIT  = 5,
//               TAGBIT    = 24, // with modulo bit
//               VALIDBIT  = 153,
//               LASTUSED  = 152,
//               TAGBEG    = 151,
//               TAGEND    = 128,
//               DATASTART = 127,
//               DATAEND   = 0; 

//     reg [BLOCKSIZE-1:0] cache0 [0:BLOCKNUM-1];
//     reg [BLOCKSIZE-1:0] cache0_nxt [0:BLOCKNUM-1];
//     reg [BLOCKSIZE-1:0] cache1 [0:BLOCKNUM-1];
//     reg [BLOCKSIZE-1:0] cache1_nxt [0:BLOCKNUM-1];
//     reg                 dirtyBlock0 [0:BLOCKNUM-1];
//     reg                 dirtyBlock0_nxt [0:BLOCKNUM-1];
//     reg                 dirtyBlock1 [0:BLOCKNUM-1];
//     reg                 dirtyBlock1_nxt [0:BLOCKNUM-1];
//     wire                isBlock0Hit;
//     wire                isBlock1Hit;
//     wire                isBlock0Dirty;
//     wire                isBlock1Dirty;

//     // combined logic
//     wire isBlockHit, isBlockDirty;

//     wire   [BLOCKBIT-1:0] block_index;
//     wire                  block_tag;
//     wire [ADDRTAGBIT-1:0] proc_tag;
//     wire     [TAGBIT-1:0] cache_tag;

//     // state name
//     parameter [1:0] IDLE   = 2'b00,
//                     CMPTAG = 2'b01,
//                     RDMEM  = 2'b11,
//                     WRTMEM = 2'b10;
//     reg [1:0]       state, state_nxt;

//     integer i;

//     assign {block_tag, block_index} = L2_addr [BLOCKIDXBEG:BLOCKIDXEND] >= 5'd24 ? 
//                                       {1'b1, L2_addr [BLOCKIDXBEG:BLOCKIDXEND] - 5'd24} :
//                                       {1'b0, L2_addr [BLOCKIDXBEG:BLOCKIDXEND]};
//     assign block_index = L2_addr [BLOCKIDXBEG:BLOCKIDXEND];
//     assign proc_tag  = L2_addr [ADDRTAGBEG:ADDRTAGEND];
//     assign cache_tag = {proc_tag, block_tag};

//     // assignments of wires
//     assign isBlock0Dirty = dirtyBlock0 [block_index];
//     assign isBlock1Dirty = dirtyBlock1 [block_index];
//     assign isBlock0Hit = (cache0 [block_index][TAGBEG:TAGEND] == cache_tag) & cache0 [block_index][VALIDBIT];
//     assign isBlock1Hit = (cache1 [block_index][TAGBEG:TAGEND] == cache_tag) & cache1 [block_index][VALIDBIT];

//     // combined logic
//     assign isBlockHit = isBlock0Hit | isBlock1Hit;

//     // if way0 is last used and dirty, then it is dirty (when missed)
//     // we do not care about hit since it does not matter
//     assign isBlockDirty = ~cache0 [block_index][LASTUSED] ? isBlock0Dirty : isBlock1Dirty;

//     // FSM
//     always@ (*) begin
//         state_nxt = state;
//         case (state)
//                 IDLE  : state_nxt = CMPTAG;
//                 CMPTAG: begin
//                     if (isBlockDirty & ~isBlockHit & (L2_read ^ L2_write)) state_nxt = WRTMEM;
//                     else if (~isBlockHit & (L2_read ^ L2_write)) state_nxt = RDMEM; 
//                 end
//                 RDMEM  : state_nxt = mem_ready ? CMPTAG: RDMEM; 
//                 WRTMEM : state_nxt = mem_ready ? RDMEM : WRTMEM;
//         endcase
//     end

//     // combinational output
//     always@ (*) begin
//         // isBlockHit = 1'b0;
//         L2_rdata = 128'b0;
//         L2_ready = 1'b0;
//         mem_read = 1'b0;
//         mem_write = 1'b0;
//         mem_addr = L2_addr; // all addr copied
//         mem_wdata = 128'b0;
//         for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
//             cache0_nxt [i] = cache0 [i];
//             cache1_nxt [i] = cache1 [i];
//             dirtyBlock0_nxt [i] = dirtyBlock0 [i];
//             dirtyBlock1_nxt [i] = dirtyBlock1 [i];
//         end

//         case (state)
//             CMPTAG: begin
//                 if (L2_read & ~L2_write & isBlock0Hit) begin // read way 0
//                     L2_rdata = cache0 [block_index][DATASTART:DATAEND];
//                     L2_ready = 1'b1;
//                     // update last-used bit
//                     cache0 [block_index][LASTUSED] = 1;
//                     cache1 [block_index][LASTUSED] = 0;
//                 end else if (L2_read & ~L2_write & isBlock1Hit) begin // read way 1
//                     L2_rdata = cache1 [block_index][DATASTART:DATAEND];
//                     L2_ready = 1'b1;
//                     // update last-used bit
//                     cache0 [block_index][LASTUSED] = 0;
//                     cache1 [block_index][LASTUSED] = 1;
//                 end else if (L2_write & ~L2_read & isBlock0Hit) begin // write if the way 0 block hit
//                     dirtyBlock0_nxt [block_index] = 1'b1;
//                     cache0_nxt [block_index][DATASTART:DATAEND] = L2_wdata;
//                     L2_ready = 1'b1;
//                     // update last-used bit
//                     cache0 [block_index][LASTUSED] = 1;
//                     cache1 [block_index][LASTUSED] = 0;
//                 end else if (L2_write & ~L2_read & isBlock1Hit) begin // write if the way 1 block hit
//                     dirtyBlock1_nxt [block_index] = 1'b1;
//                     cache1_nxt [block_index][DATASTART:DATAEND] = L2_wdata;
//                     L2_ready = 1'b1;
//                     // update last-used bit
//                     cache0 [block_index][LASTUSED] = 0;
//                     cache1 [block_index][LASTUSED] = 1;
//                 end
//             end
//             RDMEM  : begin
//                 mem_read  = 1'b1;
//                 if (~cache0 [block_index][LASTUSED] & mem_ready) begin
//                     cache0_nxt [block_index][DATASTART:DATAEND] = mem_rdata; // set data
//                     cache0_nxt [block_index][TAGBEG:TAGEND] = L2_addr [ADDRTAGBEG:ADDRTAGEND]; // set tag
//                     cache0_nxt [block_index][VALIDBIT] = 1'b1; // set valid
//                 end else if (mem_ready) begin
//                     cache1_nxt [block_index][DATASTART:DATAEND] = mem_rdata;
//                     cache1_nxt [block_index][TAGBEG:TAGEND] = L2_addr [ADDRTAGBEG:ADDRTAGEND]; // set tag
//                     cache1_nxt [block_index][VALIDBIT] = 1'b1; // set valid
//                 end
//             end
//             WRTMEM : begin
//                 mem_write = 1'b1;
//                 if (~cache0 [block_index][LASTUSED]) begin
//                     dirtyBlock0_nxt [block_index] = 1'b0;
//                     mem_wdata = cache0 [block_index][DATASTART:DATAEND];
//                     mem_addr  = {cache0 [block_index][TAGBEG:TAGEND], block_index};
//                 end else begin
//                     dirtyBlock1_nxt [L2_addr [BLOCKIDXBEG:BLOCKIDXEND]] = 1'b0;
//                     mem_wdata = cache1 [block_index][DATASTART:DATAEND];
//                     mem_addr  = {cache1 [block_index][TAGBEG:TAGEND], block_index};               
//                 end
//             end
//         endcase
//     end

//     //==== sequential circuit =================================
//     always@( posedge clk ) begin
//         if( proc_reset ) begin
//             state <= IDLE;
//             for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
//                 cache0 [i] <= {BLOCKSIZE{1'b0}};
//                 cache1 [i] <= {BLOCKSIZE{1'b0}};
//                 dirtyBlock0 [i] <= 1'b0;
//                 dirtyBlock1 [i] <= 1'b0;
//             end
//         end
//         else begin
//             state <= state_nxt;
//             for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
//                 cache0 [i] <= cache0_nxt [i];
//                 cache1 [i] <= cache1_nxt [i];
//                 dirtyBlock0 [i] <= dirtyBlock0_nxt [i];
//                 dirtyBlock1 [i] <= dirtyBlock1_nxt [i];
//             end
//         end
//     end
// endmodule

// 24 blocks * 4 words * two-way
module L2_Dcache (
        input clk,
        input proc_reset,
        // L1 I_interface. addr: 28 bits, data: 128 bits
        input              L2_read, 
        input              L2_write,
        input       [27:0] L2_addr,
        input      [127:0] L2_wdata,
        output reg [127:0] L2_rdata,
        output reg         L2_ready,
        // memory interface. addr: 28 bits, data: 128 bits
        input      [127:0] mem_rdata,
        input              mem_ready,
        output reg         mem_read, 
        output reg         mem_write,
        output reg  [27:0] mem_addr,
        output reg [127:0] mem_wdata
    );

    // L1 -> L2 addr 2-way associated
    parameter ADDRTAGBIT  = 23,
              ADDRTAGBEG  = 27,
              ADDRTAGEND  = 5, // 23-bit tag
              BLOCKIDXBEG = 4, // 24 bloacks (5 bits)
              BLOCKIDXEND = 0;
              

    // L2 Cache (154 bits) = 1 valid + 1 last-used + 24 tag + 4 words data (128 bits)
    parameter BLOCKSIZE = 154,
              BLOCKNUM  = 24,
              BLOCKBIT  = 5,
              BLOCKWAY  = 2,
              TAGBIT    = 24,
              VALIDBIT  = 153,
              LASTUSED  = 152,
              TAGBEG    = 151,
              TAGEND    = 128,
              DATASTART = 127,
              DATAEND   = 0; 

    reg [BLOCKSIZE-1:0] cache0 [0:BLOCKNUM-1];
    reg [BLOCKSIZE-1:0] cache0_nxt [0:BLOCKNUM-1];
    reg [BLOCKSIZE-1:0] cache1 [0:BLOCKNUM-1];
    reg [BLOCKSIZE-1:0] cache1_nxt [0:BLOCKNUM-1];
    reg                 dirtyBlock0 [0:BLOCKNUM-1];
    reg                 dirtyBlock0_nxt [0:BLOCKNUM-1];
    reg                 dirtyBlock1 [0:BLOCKNUM-1];
    reg                 dirtyBlock1_nxt [0:BLOCKNUM-1];
    wire                isBlock0Hit;
    wire                isBlock1Hit;
    wire                isBlock0Dirty;
    wire                isBlock1Dirty;

    // combined logic
    wire isBlockHit, isBlockDirty;

    wire   [BLOCKBIT-1:0] block_index;
    wire                  block_tag;
    wire [ADDRTAGBIT-1:0] proc_tag;
    wire     [TAGBIT-1:0] cache_tag;     

    // state name
    parameter [1:0] IDLE   = 2'b00,
                    CMPTAG = 2'b01,
                    RDMEM  = 2'b11,
                    WRTMEM = 2'b10;
    reg [1:0]       state, state_nxt;

    integer i;

    // assignments of wires
    assign {block_tag, block_index} = L2_addr [BLOCKIDXBEG:BLOCKIDXEND] >= 5'd24 ?
                                      {1'b1, L2_addr [BLOCKIDXBEG:BLOCKIDXEND] - 5'd24} :
                                      {1'b0, L2_addr [BLOCKIDXBEG:BLOCKIDXEND]};
    assign proc_tag = L2_addr [ADDRTAGBEG:ADDRTAGEND];
    assign cache_tag = {proc_tag, block_tag};

    assign isBlock0Dirty = dirtyBlock0 [block_index];
    assign isBlock1Dirty = dirtyBlock1 [block_index];
    assign isBlock0Hit = (cache0 [block_index][TAGBEG:TAGEND] == cache_tag) &
                         cache0 [block_index][VALIDBIT];
    assign isBlock1Hit = (cache1 [block_index][TAGBEG:TAGEND] == cache_tag) &
                         cache1 [block_index][VALIDBIT];
    // combined logic
    assign isBlockHit = isBlock0Hit | isBlock1Hit;

    // if way0 is not last used but dirty, then it is dirty (when missed)
    // we do not care about hit since it does not matter
    assign isBlockDirty = ~cache0 [block_index][LASTUSED] ? isBlock0Dirty : isBlock1Dirty;

    // FSM
    always@ (*) begin
        state_nxt = state;
        case (state)
                IDLE  : state_nxt = CMPTAG;
                CMPTAG: begin
                    if ((L2_read ^ L2_write) & isBlockDirty & ~isBlockHit) state_nxt = WRTMEM;
                    else if (~isBlockHit & (L2_read ^ L2_write)) state_nxt = RDMEM;
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
        for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
            cache0_nxt [i] = cache0 [i];
            cache1_nxt [i] = cache1 [i];
            dirtyBlock0_nxt [i] = dirtyBlock0 [i];
            dirtyBlock1_nxt [i] = dirtyBlock1 [i];
        end

        case (state)
            CMPTAG: begin
                if (L2_read & ~L2_write & isBlock0Hit) begin // read way 0
                    L2_rdata = cache0 [block_index][DATASTART:DATAEND];
                    L2_ready = 1'b1;
                    // update last-used bit
                    cache0 [block_index][LASTUSED] = 1;
                    cache1 [block_index][LASTUSED] = 0;
                end else if (L2_read & ~L2_write & isBlock1Hit) begin // read way 1
                    L2_rdata = cache1 [block_index][DATASTART:DATAEND];
                    L2_ready = 1'b1;
                    // update last-used bit
                    cache0 [block_index][LASTUSED] = 0;
                    cache1 [block_index][LASTUSED] = 1;
                end else if (L2_write & ~L2_read & isBlock0Hit) begin // write if the way 0 block hit
                    dirtyBlock0_nxt [block_index] = 1'b1;
                    cache0_nxt [block_index][DATASTART:DATAEND] = L2_wdata;
                    L2_ready = 1'b1;
                    // update last-used bit
                    cache0 [block_index][LASTUSED] = 1;
                    cache1 [block_index][LASTUSED] = 0;
                end else if (L2_write & ~L2_read & isBlock1Hit) begin // write if the way 1 block hit
                    dirtyBlock1_nxt [block_index] = 1'b1;
                    cache1_nxt [block_index][DATASTART:DATAEND] = L2_wdata;
                    L2_ready = 1'b1;
                    // update last-used bit
                    cache0 [block_index][LASTUSED] = 0;
                    cache1 [block_index][LASTUSED] = 1;
                end
            end
            RDMEM  : begin
                mem_read  = 1'b1;
                if (~cache0 [block_index][LASTUSED] & mem_ready) begin
                    cache0_nxt [block_index][DATASTART:DATAEND] = mem_rdata; // set data
                    cache0_nxt [block_index][TAGBEG:TAGEND] = cache_tag; // set tag
                    cache0_nxt [block_index][VALIDBIT] = 1'b1; // set valid
                end else if (mem_ready) begin
                    cache1_nxt [block_index][DATASTART:DATAEND] = mem_rdata;
                    cache1_nxt [block_index][TAGBEG:TAGEND] = cache_tag; // set tag
                    cache1_nxt [block_index][VALIDBIT] = 1'b1; // set valid
                end
            end
            WRTMEM : begin
                mem_write = 1'b1;
                if (~cache0 [block_index][LASTUSED]) begin
                    dirtyBlock0_nxt [block_index] = 1'b0;
                    mem_wdata = cache0 [block_index][DATASTART:DATAEND];
                    mem_addr  = {cache0 [block_index][TAGBEG:TAGEND], block_index};
                end else begin
                    dirtyBlock1_nxt [L2_addr [BLOCKIDXBEG:BLOCKIDXEND]] = 1'b0;
                    mem_wdata = cache1 [block_index][DATASTART:DATAEND];
                    mem_addr  = {cache1 [block_index][TAGBEG:TAGEND], block_index};               
                end
            end
        endcase
    end

    //==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            state <= IDLE;
            for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
                cache0 [i] <= {BLOCKSIZE{1'b0}};
                cache1 [i] <= {BLOCKSIZE{1'b0}};
                dirtyBlock0 [i] <= 1'b0;
                dirtyBlock1 [i] <= 1'b0;
            end
        end
        else begin
            state <= state_nxt;
            for (i = 0; i <= BLOCKNUM-1; i = i + 1) begin
                cache0 [i] <= cache0_nxt [i];
                cache1 [i] <= cache1_nxt [i];
                dirtyBlock0 [i] <= dirtyBlock0_nxt [i];
                dirtyBlock1 [i] <= dirtyBlock1_nxt [i];
            end
        end
    end
endmodule