module cache (
    input           clk,
    input           resetn,

    // Core-facing interface (connects to PicoRV32)
    // core_valid: core is requesting a memory transaction
    // core_ready: cache signals to core that transaction is complete
    // core_wstrb: byte-enable mask (0000 = read, else = write)
    input           core_valid,
    output reg      core_ready,
    input  [31:0]   core_addr,
    input  [31:0]   core_wdata,
    input  [3:0]    core_wstrb,
    output reg [31:0] core_rdata,

    // Memory-facing interface (connects to shared memory)
    // mem_valid: cache is requesting a memory transaction
    // mem_ready: memory signals to cache that transaction is complete
    // mem_wstrb: 0000 = read (fetch), 1111 = write (writeback)
    output reg          mem_valid,
    input               mem_ready,
    output reg [31:0]   mem_addr,
    output reg [31:0]   mem_wdata,
    output reg [3:0]    mem_wstrb,
    input      [31:0]   mem_rdata
);

    // -------------------------------------------------------------------------
    // Cache storage - 4 direct-mapped lines, one word each
    //   valid: is there anything stored here at all?
    //   dirty: has this line been written locally but not sent to memory yet?
    //   tag:   which memory address does this line belong to?
    //   data:  the actual cached word
    // -------------------------------------------------------------------------
    reg [31:0] data  [0:3];
    reg [27:0] tag   [0:3];
    reg        valid [0:3];
    reg        dirty [0:3];

    // -------------------------------------------------------------------------
    // Address breakdown
    // Every 32-bit address splits into three fields:
    //   [31:4] tag   (28 bits) - which memory block is this?
    //   [3:2]  index (2 bits)  - which of the 4 cache lines to look in
    //   [1:0]  byte offset     - ignored, we are word-addressed
    //
    // addr_tag/addr_index: from core_addr, used in IDLE to sample the request
    // latched_tag/latched_index: from latched_addr, stable across all cycles
    // -------------------------------------------------------------------------
    wire [27:0] addr_tag    = core_addr[31:4];
    wire [1:0]  addr_index  = core_addr[3:2];

    wire [27:0] latched_tag   = latched_addr[31:4];
    wire [1:0]  latched_index = latched_addr[3:2];

    // -------------------------------------------------------------------------
    // Hit detection
    // A hit requires both:
    //   1. The slot is occupied (valid bit set)
    //   2. The slot contains our address (tag matches)
    // Uses latched address since we check this after IDLE has captured request
    // -------------------------------------------------------------------------
    wire hit = valid[latched_index] && (tag[latched_index] == latched_tag);

    // -------------------------------------------------------------------------
    // State machine parameters
    // 3 bits needed to represent 5 states (2 bits only gives 4)
    // -------------------------------------------------------------------------
    localparam IDLE      = 3'd0;  // waiting for core request
    localparam COMPARE   = 3'd1;  // check tag and valid bit
    localparam WRITEBACK = 3'd2;  // flush dirty line to memory before eviction
    localparam FETCH     = 3'd3;  // load requested line from memory
    localparam DONE      = 3'd4;  // serve core, return to IDLE

    reg [2:0] state;

    // -------------------------------------------------------------------------
    // Latched request registers
    // PicoRV32 holds its output signals stable only until mem_ready goes high.
    // We may spend many cycles in WRITEBACK and FETCH before we are done.
    // So we capture the core's request in IDLE and hold it ourselves.
    // -------------------------------------------------------------------------
    reg [31:0] latched_addr;
    reg [31:0] latched_wdata;
    reg [3:0]  latched_wstrb;

    // -------------------------------------------------------------------------
    // Reset + state machine - MUST be one always block
    // Having two separate always blocks drive the same registers (state, valid,
    // dirty) creates multiple drivers which is undefined in Verilog.
    // Pattern: if (!resetn) handle reset, else run state machine.
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk) begin
        if (!resetn) begin
            // Clear valid and dirty bits - cache starts empty.
            // tag and data don't need clearing - irrelevant when valid=0.
            for (i = 0; i < 4; i = i + 1) begin
                valid[i] <= 0;
                dirty[i] <= 0;
            end
            state      <= IDLE;
            core_ready <= 0;
            mem_valid  <= 0;
        end else begin
            case (state)

                // IDLE: do nothing until core makes a request.
                // When core_valid goes high, latch the request and move to COMPARE.
                // We latch here because core_addr/wdata/wstrb may change next cycle.
                IDLE: begin
                    core_ready <= 0;
                    mem_valid  <= 0;
                    // Guard with !core_ready: when DONE asserts core_ready=1 and
                    // transitions to IDLE in the same cycle, core_valid is still
                    // high (testbench hasn't deasserted yet). Without this guard
                    // we'd immediately re-latch the same request and corrupt state.
                    // core_ready=1 at this moment means we just came from DONE --
                    // skip this cycle and wait for core_valid to be re-asserted.
                    if (core_valid && !core_ready) begin
                        latched_addr  <= core_addr;
                        latched_wdata <= core_wdata;
                        latched_wstrb <= core_wstrb;
                        state         <= COMPARE;
                    end
                end

                // COMPARE: check whether the requested address is in the cache.
                // Hit:        go directly to DONE (serve core next cycle)
                // Clean miss: go to FETCH (slot is empty or stale, safe to overwrite)
                // Dirty miss: go to WRITEBACK first (must flush before overwriting)
                COMPARE: begin
                    if (hit) begin
                        state <= DONE;
                    end else begin
                        if (dirty[latched_index]) begin
                            state <= WRITEBACK;
                        end else begin
                            state <= FETCH;
                        end
                    end
                end

                // WRITEBACK: the slot we need is occupied by a dirty line.
                // We must send that dirty line to memory before we can use the slot.
                // The writeback address is reconstructed from the stored tag and index
                // (not latched_addr - that's the new address we want, not the old one).
                // Hold mem_valid high until mem_ready confirms the write completed.
                WRITEBACK: begin
                    mem_valid <= 1;
                    mem_addr  <= {tag[latched_index], latched_index, 2'b00};
                    mem_wdata <= data[latched_index];
                    mem_wstrb <= 4'b1111;  // full word write
                    if (mem_ready) begin
                        mem_valid <= 0;
                        state     <= FETCH;
                    end
                end

                // FETCH: load the requested line from memory into the cache slot.
                // mem_wstrb = 0000 signals a read to memory.
                // When mem_ready goes high, store the returned data, update the tag,
                // mark the line valid and clean, then proceed to DONE.
                FETCH: begin
                    mem_valid <= 1;
                    mem_addr  <= {latched_tag, latched_index, 2'b00};
                    mem_wstrb <= 4'b0000;  // read
                    if (mem_ready) begin
                        mem_valid            <= 0;
                        data[latched_index]  <= mem_rdata;
                        tag[latched_index]   <= latched_tag;
                        valid[latched_index] <= 1;
                        dirty[latched_index] <= 0;
                        state                <= DONE;
                    end
                end

                // DONE: the cache line is valid and current. Serve the core.
                // Read:  put the cached data on core_rdata
                // Write: update the cache line byte-by-byte using wstrb mask,
                //        set dirty bit (write-back policy: don't write to memory yet)
                // Either way, assert core_ready for one cycle then return to IDLE.
                DONE: begin
                    core_ready <= 1;
                    if (latched_wstrb == 4'b0000) begin
                        // Read
                        core_rdata <= data[latched_index];
                    end else begin
                        // Write - apply byte enables individually
                        if (latched_wstrb[0]) data[latched_index][ 7: 0] <= latched_wdata[ 7: 0];
                        if (latched_wstrb[1]) data[latched_index][15: 8] <= latched_wdata[15: 8];
                        if (latched_wstrb[2]) data[latched_index][23:16] <= latched_wdata[23:16];
                        if (latched_wstrb[3]) data[latched_index][31:24] <= latched_wdata[31:24];
                        dirty[latched_index] <= 1;
                    end
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule