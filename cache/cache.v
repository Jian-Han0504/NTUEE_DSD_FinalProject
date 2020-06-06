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
reg         proc_stall;
reg [ 31:0] proc_rdata;
reg         mem_read, mem_write;
reg [127:0] mem_wdata;
reg [ 27:0] mem_addr_r;

// cache = 1 valid + 25 tag + 128 data = 154 bits
reg [153:0] cache     [0:7];
reg [153:0] cache_nxt [0:7];
reg [  1:0] state, state_nxt;
reg         dirtyBlock [0:7];
reg         dirtyBlock_nxt [0:7];
reg         isHit;
wire        isBlockDirty;

integer i;

// state name
parameter [1:0] IDLE   = 2'b00,
                CMPTAG = 2'b01,
                RDMEM  = 2'b11,
                WRTMEM = 2'b10;

// cache structure
parameter VALIDBIT = 153,
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

//==== combinational circuit ==============================
// FSM
always@ (*) begin
    state_nxt = state;
    case (state)
        IDLE  : state_nxt = CMPTAG;
        CMPTAG: begin
            if (isBlockDirty && ~isHit) state_nxt = WRTMEM;
            else if (~isHit) state_nxt = RDMEM; 
        end
        RDMEM  : state_nxt = mem_ready ? CMPTAG: RDMEM; 
        WRTMEM : state_nxt = mem_ready ? RDMEM : WRTMEM;
    endcase
end

// output
always@ (*) begin
    isHit      = 1'b0;
    proc_stall = 1'b0;
    proc_rdata = 32'b0;
    mem_read   = 1'b0;
    mem_write  = 1'b0;
    mem_wdata  = 128'b0;
    mem_addr_r = proc_addr [29:2];
    for (i = 0; i <= 7; i = i + 1) begin
        cache_nxt [i] = cache [i];
        dirtyBlock_nxt [i] = dirtyBlock [i];
    end

    case (state) 
        CMPTAG: begin
            isHit = ((proc_addr [29:5] == cache [proc_addr [4:2]][TAGSTART:TAGEND]) && cache [proc_addr [4:2]][VALIDBIT]) ? 
                    1'b1 : 1'b0;
            if (~isHit && (proc_read ^ proc_write)) proc_stall = 1'b1;

            if (proc_read && ~proc_write) begin // read
                case (proc_addr [1:0])
                    2'b00: proc_rdata = cache [proc_addr [4:2]][DATA0START:DATA0END];
                    2'b01: proc_rdata = cache [proc_addr [4:2]][DATA1START:DATA1END];
                    2'b10: proc_rdata = cache [proc_addr [4:2]][DATA2START:DATA2END];
                    2'b11: proc_rdata = cache [proc_addr [4:2]][DATA3START:DATA3END];
                endcase
            end
            else if (proc_write & ~proc_read & isHit) begin // write if the block clean and hit
                dirtyBlock_nxt [proc_addr [4:2]] = 1'b1;
                case (proc_addr [1:0])
                    2'b00: cache_nxt [proc_addr [4:2]][DATA0START:DATA0END] = proc_wdata;
                    2'b01: cache_nxt [proc_addr [4:2]][DATA1START:DATA1END] = proc_wdata;
                    2'b10: cache_nxt [proc_addr [4:2]][DATA2START:DATA2END] = proc_wdata;
                    2'b11: cache_nxt [proc_addr [4:2]][DATA3START:DATA3END] = proc_wdata;
                endcase
            end
        end
        RDMEM  : begin
            proc_stall = 1'b1;
            mem_read  = 1'b1;
            cache_nxt [proc_addr [4:2]][127:  0] = mem_rdata; // set data
            cache_nxt [proc_addr [4:2]][152:128] = mem_addr [27:3]; // set tag
            cache_nxt [proc_addr [4:2]][153]     = 1'b1; // set valid
        end
        WRTMEM : begin
            proc_stall = 1'b1;
            dirtyBlock_nxt [proc_addr [4:2]] = 1'b0;
            mem_write = 1'b1;
            mem_wdata = cache [proc_addr [4:2]][127:0];
            mem_addr_r = {cache [proc_addr [4:2]][TAGSTART:TAGEND], proc_addr [4:2]};
        end
    endcase
end

assign mem_addr = mem_addr_r;
assign isBlockDirty = dirtyBlock [proc_addr [4:2]];

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state <= IDLE;
        for (i = 0; i <= 7; i = i + 1) begin
            cache [i] <= 154'b0;
            dirtyBlock [i] <= 1'b0;
        end
    end
    else begin
        state <= state_nxt;
        for (i = 0; i <= 7; i = i + 1) begin
            cache [i] <= cache_nxt [i];
            dirtyBlock [i] <= dirtyBlock_nxt [i];
        end
    end
end

endmodule
