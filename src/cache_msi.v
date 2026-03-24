// =============================================================================
// cache_msi.v - Direct-mapped write-back cache with MSI snooping coherence
//
// Extends cache.v with:
//   - 2-bit MSI coherence state per cache line (I=0, S=1, M=2)
//   - Bus arbiter interface (request/grant)
//   - Snooping bus interface (drive when owner, read always)
//   - Transient states (IS, IM, SM, MI) for in-flight transactions
//
// Cache parameters:
//   - 4 direct-mapped lines, one 32-bit word per line
//   - Write-back, write-allocate
//   - Address: [31:4] tag, [3:2] index, [1:0] byte offset (ignored)
// =============================================================================

module cache_msi (
    input clk,
    input resetn,

    // Core ID - tells this controller which grant bit to watch
    // and which core ID to put on the bus
    input [1:0] core_id,

    // -------------------------------------------------------------------------
    // Core-facing interface (same as cache.v)
    // -------------------------------------------------------------------------
    input           core_valid,
    output reg      core_ready,
    input  [31:0]   core_addr,
    input  [31:0]   core_wdata,
    input  [3:0]    core_wstrb,
    output reg [31:0] core_rdata,

    // -------------------------------------------------------------------------
    // Memory-facing interface (same as cache.v)
    // Cache still talks to memory for FETCH and WRITEBACK,
    // but only after winning bus arbitration
    // -------------------------------------------------------------------------
    output reg        mem_valid,
    input             mem_ready,
    output reg [31:0] mem_addr,
    output reg [31:0] mem_wdata,
    output reg [3:0]  mem_wstrb,
    input      [31:0] mem_rdata,

    // -------------------------------------------------------------------------
    // Arbiter interface
    // bus_req: raise to request bus access
    // bus_grant: arbiter asserts when this core owns the bus
    // -------------------------------------------------------------------------
    output reg bus_req,
    input      bus_grant,

    // -------------------------------------------------------------------------
    // Bus outputs - driven by this controller when bus_grant is high
    // All other controllers snoop these signals
    // -------------------------------------------------------------------------
    output reg        bus_req_valid,  // a transaction is on the bus
    output reg [1:0]  bus_req_type,   // BusRd=0, BusRdX=1, BusUpgr=2
    output reg [31:0] bus_req_addr,   // address of transaction
    output reg [1:0]  bus_req_core,   // which core issued this (= core_id)

    // -------------------------------------------------------------------------
    // Bus inputs - snooped by this controller always, even when not bus owner
    // -------------------------------------------------------------------------
    input        snoop_valid,    // a transaction is currently on the bus
    input [1:0]  snoop_type,     // transaction type
    input [31:0] snoop_addr,     // transaction address
    input [1:0]  snoop_core,     // which core issued the transaction

    // Memory response - driven by memory after all snoop reactions complete
    input [31:0] bus_resp_data,
    input        bus_resp_valid,

    // -------------------------------------------------------------------------
    // Snoop busy - asserted by this controller while it is reacting to a
    // bus transaction (e.g. writing back an M-state line before releasing it)
    // Memory waits until snoop_busy is clear before responding
    // -------------------------------------------------------------------------
    output reg snoop_busy
);

    // -------------------------------------------------------------------------
    // Bus transaction type encoding
    // -------------------------------------------------------------------------
    localparam BUS_RD   = 2'd0;  // read request, want S copy
    localparam BUS_RDX  = 2'd1;  // read-exclusive, want M copy
    localparam BUS_UPGR = 2'd2;  // upgrade S→M, already have data

    // -------------------------------------------------------------------------
    // MSI coherence state encoding (per cache line)
    // -------------------------------------------------------------------------
    localparam MSI_I = 2'd0;  // Invalid  - no valid copy
    localparam MSI_S = 2'd1;  // Shared   - read-only copy, memory up to date
    localparam MSI_M = 2'd2;  // Modified - exclusive copy, memory may be stale

    // -------------------------------------------------------------------------
    // Cache controller FSM states
    //
    // Stable states (waiting for something to do):
    //   IDLE     - waiting for core request
    //   COMPARE  - checking tag/valid/MSI state
    //   DONE     - serving core, returning to IDLE
    //
    // Cache miss states (same as cache.v):
    //   WRITEBACK - flushing dirty line to memory before eviction
    //   FETCH     - loading new line from memory (now via bus)
    //
    // Transient coherence states (waiting for bus transaction to complete):
    //   IS  - issued BusRd from I, waiting for memory to respond with S copy
    //   IM  - issued BusRdX from I, waiting for exclusive M copy
    //   SM  - issued BusUpgr from S, waiting for others to invalidate
    //   MI  - in M, saw BusRd/BusRdX from another core, writing back to memory
    // -------------------------------------------------------------------------
    localparam IDLE      = 4'd0;
    localparam COMPARE   = 4'd1;
    localparam WRITEBACK = 4'd2;
    localparam FETCH     = 4'd3;
    localparam DONE      = 4'd4;
    localparam IS        = 4'd5;
    localparam IM        = 4'd6;
    localparam SM        = 4'd7;
    localparam MI        = 4'd8;

    reg [3:0] state;

    // -------------------------------------------------------------------------
    // Cache storage - 4 direct-mapped lines
    // Now includes msi_state per line in addition to cache.v fields
    // -------------------------------------------------------------------------
    reg [31:0] data      [0:3];
    reg [27:0] tag       [0:3];
    reg        valid     [0:3];
    reg        dirty     [0:3];
    reg [1:0]  msi_state [0:3];  // MSI_I, MSI_S, or MSI_M per line

    // -------------------------------------------------------------------------
    // Address decomposition
    // latched_* used throughout state machine (stable across cycles)
    // core_* only used in IDLE when sampling incoming request
    // -------------------------------------------------------------------------
    wire [27:0] latched_tag   = latched_addr[31:4];
    wire [1:0]  latched_index = latched_addr[3:2];
    wire [27:0] snoop_tag     = snoop_addr[31:4];
    wire [1:0]  snoop_index   = snoop_addr[3:2];

    // -------------------------------------------------------------------------
    // Hit detection
    // In MSI, a hit also requires the line to not be in I state
    // -------------------------------------------------------------------------
    wire hit = valid[latched_index]
            && (tag[latched_index] == latched_tag)
            && (msi_state[latched_index] != MSI_I);

    // Write hit: have the line AND have write permission (M state)
    wire write_hit = hit && (msi_state[latched_index] == MSI_M);

    // Read hit: have the line in S or M (both allow reads)
    wire read_hit = hit;

    // -------------------------------------------------------------------------
    // Latched request registers
    // -------------------------------------------------------------------------
    reg [31:0] latched_addr;
    reg [31:0] latched_wdata;
    reg [3:0]  latched_wstrb;

    // -------------------------------------------------------------------------
    // Snoop match: does this snoop transaction target an address we hold?
    // Only react if: transaction is valid, address matches a line we hold,
    // and it wasn't issued by us
    // -------------------------------------------------------------------------
    wire snoop_match = snoop_valid
                    && (snoop_core != core_id)
                    && valid[snoop_index]
                    && (tag[snoop_index] == snoop_tag)
                    && (msi_state[snoop_index] != MSI_I);

    // -------------------------------------------------------------------------
    // Reset + state machine (merged into one always block - see cache.v notes)
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk) begin
        if (!resetn) begin
            for (i = 0; i < 4; i = i + 1) begin
                valid[i]     <= 0;
                dirty[i]     <= 0;
                msi_state[i] <= MSI_I;
            end
            state        <= IDLE;
            core_ready   <= 0;
            mem_valid    <= 0;
            bus_req      <= 0;
            bus_req_valid <= 0;
            snoop_busy   <= 0;
        end else begin
            // State machine goes here in next step
        end
    end

endmodule