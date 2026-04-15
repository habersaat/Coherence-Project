module cache_mesi (
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
    // Cache still talks to memory for WRITEBACK and MI state writebacks,
    // but only after winning bus arbitration for new fetches
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
    output wire snoop_busy,

    // -------------------------------------------------------------------------
    // MESI additions
    // snoop_shared: asserted when we hold an S or E copy of the BusRd-snooped
    //   address. Top-level ORs all four to produce bus_resp_excl for requester.
    // bus_resp_excl: asserted by top-level when no cache drove snoop_shared,
    //   meaning this requester is the sole reader and should receive E not S.
    // -------------------------------------------------------------------------
    output wire snoop_shared,
    input       bus_resp_excl,

    // silent_upgrade: pulses high for one cycle when a write hits an E-state
    // line in COMPARE and silently upgrades to M without any bus transaction.
    // that would have been BusUpgr under MSI but was eliminated by E state.
    output wire silent_upgrade
);

    // -------------------------------------------------------------------------
    // Bus transaction type encoding
    // -------------------------------------------------------------------------
    localparam BUS_RD   = 2'd0;  // read request, want S copy
    localparam BUS_RDX  = 2'd1;  // read-exclusive, want M copy
    localparam BUS_UPGR = 2'd2;  // upgrade S->M, already have data

    // -------------------------------------------------------------------------
    // MESI coherence state encoding (per cache line)
    // -------------------------------------------------------------------------
    localparam MSI_I = 2'd0;  // Invalid   - no valid copy
    localparam MSI_S = 2'd1;  // Shared    - read-only, others may also have copies
    localparam MSI_M = 2'd2;  // Modified  - exclusive dirty copy, memory stale
    localparam MSI_E = 2'd3;  // Exclusive - sole clean copy, silent upgrade to M on write

    // -------------------------------------------------------------------------
    // Cache controller FSM states
    //
    // Stable states:
    //   IDLE      - waiting for core request
    //   COMPARE   - checking tag/valid/MSI state
    //   DONE      - serving core, returning to IDLE
    //
    // Cache miss states:
    //   WRITEBACK - flushing dirty line to memory before eviction
    //   FETCH     - unused in MSI (replaced by IS/IM), kept for completeness
    //
    // Transient coherence states:
    //   IS  - issued BusRd from I, waiting for memory to respond with S copy
    //   IM  - issued BusRdX from I, waiting for exclusive M copy
    //   SM  - issued BusUpgr from S, waiting for others to invalidate
    //   MI  - in M, saw BusRd/BusRdX from another core, writing back to memory
    // -------------------------------------------------------------------------
    localparam IDLE      = 4'd0;
    localparam COMPARE   = 4'd1;
    localparam WRITEBACK = 4'd2;
    localparam FETCH     = 4'd3;  // unused in MSI design - IS/IM replace it
    localparam DONE      = 4'd4;
    localparam IS        = 4'd5;
    localparam IM        = 4'd6;
    localparam SM        = 4'd7;
    localparam MI        = 4'd8;

    reg [3:0] state;

    // -------------------------------------------------------------------------
    // Cache storage - 4 direct-mapped lines
    // -------------------------------------------------------------------------
    reg [31:0] data      [0:3];
    reg [27:0] tag       [0:3];
    reg        valid     [0:3];
    reg        dirty     [0:3];
    reg [1:0]  msi_state [0:3];  // MSI_I, MSI_S, or MSI_M per line

    // Saves which FSM state to return to after MI writeback completes.
    // MI can interrupt IDLE, COMPARE, WRITEBACK, IS, IM, or SM.
    reg [3:0] return_state;

    // Captured snoop signals at the moment we enter MI.
    // We cannot use the live snoop_type/snoop_index wires inside MI because
    // the requesting core may release the bus before our writeback finishes,
    // collapsing snoop_type to 0 (BUS_RD). We would then incorrectly downgrade
    // to S instead of invalidating to I on a BusRdX reaction.
    // Capturing at MI entry freezes the correct values for the duration.
    reg [1:0] mi_snoop_type;
    reg [1:0] mi_snoop_index;

    // -------------------------------------------------------------------------
    // Address decomposition
    // latched_* used throughout state machine (stable across cycles)
    // snoop_* decoded directly from bus input signals
    // -------------------------------------------------------------------------
    wire [27:0] latched_tag   = latched_addr[31:4];
    wire [1:0]  latched_index = latched_addr[3:2];
    wire [27:0] snoop_tag     = snoop_addr[31:4];
    wire [1:0]  snoop_index   = snoop_addr[3:2];

    // -------------------------------------------------------------------------
    // Hit detection
    // In MSI, a hit also requires the line to not be in I state.
    // Uses latched address since we check this after IDLE has captured request.
    // -------------------------------------------------------------------------
    wire hit = valid[latched_index]
            && (tag[latched_index] == latched_tag)
            && (msi_state[latched_index] != MSI_I);

    // Write hit: have the line AND have write permission (M state, or E for silent upgrade)
    wire write_hit = hit && (msi_state[latched_index] == MSI_M
                          || msi_state[latched_index] == MSI_E);

    // Read hit: S or M both permit reads
    wire read_hit = hit;

    // -------------------------------------------------------------------------
    // Latched request registers
    // PicoRV32 holds outputs stable only until mem_ready - capture in IDLE
    // -------------------------------------------------------------------------
    reg [31:0] latched_addr;
    reg [31:0] latched_wdata;
    reg [3:0]  latched_wstrb;

    // -------------------------------------------------------------------------
    // Snoop match: does this bus transaction target an address we hold?
    // Only react if: valid transaction, from another core, line in S or M
    // -------------------------------------------------------------------------
    wire snoop_match = snoop_valid
                    && (snoop_core != core_id)
                    && valid[snoop_index]
                    && (tag[snoop_index] == snoop_tag)
                    && (msi_state[snoop_index] != MSI_I);

    // snoop_busy: combinational so memory sees it the same cycle we detect the
    // M-state match. A registered snoop_busy arrives one cycle too late - the
    // two-stage memory model would fire bus_resp_valid with stale data before
    // the writeback completes.
    // High in two cases:
    //   1. We detected an M-state match this cycle (will enter MI next cycle)
    //   2. We are already in MI doing the writeback
    assign snoop_busy = (state == MI) ||
                        (snoop_match
                         && msi_state[snoop_index] == MSI_M
                         && (snoop_type == BUS_RD || snoop_type == BUS_RDX));

    // snoop_shared: driven high when we have an S or E copy of the snooped address
    // on a BusRd. Top-level ORs across all four caches to tell the requester
    // whether anyone already has a copy (requester gets S) or not (requester gets E).
    assign snoop_shared = snoop_valid
                       && (snoop_type == BUS_RD)
                       && (snoop_core != core_id)
                       && valid[snoop_index]
                       && (tag[snoop_index] == snoop_tag)
                       && (msi_state[snoop_index] == MSI_S
                        || msi_state[snoop_index] == MSI_E);

    // silent_upgrade: high for exactly one cycle when COMPARE detects a write
    // to an E-state line. write_hit includes E (see above), and the FSM
    // immediately moves to DONE, so this is a single-cycle combinational pulse.
    // Under MSI this is always 0 (no E state). Under MESI it counts the bus
    // transactions saved vs MSI.
    assign silent_upgrade = (state == COMPARE)
                          && (latched_wstrb != 4'b0000)
                          && write_hit
                          && (msi_state[latched_index] == MSI_E);

    // -------------------------------------------------------------------------
    // Reset + state machine (one always block - see cache.v for why)
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk) begin
        if (!resetn) begin
            for (i = 0; i < 4; i = i + 1) begin
                valid[i]     <= 0;
                dirty[i]     <= 0;
                msi_state[i] <= MSI_I;
            end
            state          <= IDLE;
            core_ready     <= 0;
            mem_valid      <= 0;
            bus_req        <= 0;
            bus_req_valid  <= 0;
            return_state   <= IDLE;
            mi_snoop_type  <= 0;
            mi_snoop_index <= 0;
        end else begin

            // -----------------------------------------------------------------
            // S-state invalidation: handled outside the case statement so it
            // fires in every state without duplicating code.
            //
            // If another core issues BusRdX or BusUpgr targeting a line we
            // hold in S, we just drop it to I. No writeback needed - memory
            // is already up to date for S lines.
            //
            // M-state reactions (BusRd/BusRdX on our M line) DO require a
            // writeback, so those are handled inside each case state where
            // we can also capture mi_snoop_* and set return_state before MI.
            // -----------------------------------------------------------------
            // S-state invalidation: on BusRdX or BusUpgr, drop our shared copy to I.
            // Memory is up to date for S lines - no writeback needed.
            if (snoop_match
                    && msi_state[snoop_index] == MSI_S
                    && (snoop_type == BUS_RDX || snoop_type == BUS_UPGR)) begin
                msi_state[snoop_index] <= MSI_I;
            end

            // E-state reactions (MESI): memory is up to date, no writeback needed.
            //   BusRd  -> E->S: another core is now reading; we downgrade to shared.
            //             (snoop_shared fires combinatorially, telling requester -> S)
            //   BusRdX -> E->I: another core wants exclusive; we fully vacate.
            //   BusUpgr is architecturally impossible on an E line (no one else
            //   has S to upgrade from) but handled defensively as E->I.
            if (snoop_match && msi_state[snoop_index] == MSI_E) begin
                if (snoop_type == BUS_RD)
                    msi_state[snoop_index] <= MSI_S;
                else
                    msi_state[snoop_index] <= MSI_I;
            end

            case (state)

                // -----------------------------------------------------------------
                // IDLE: wait for a core request.
                // Clear bus outputs so we are not driving anything.
                // Still must check for M-state snoop reactions even at rest.
                // -----------------------------------------------------------------
                IDLE: begin
                    core_ready    <= 0;
                    mem_valid     <= 0;
                    bus_req       <= 0;
                    bus_req_valid <= 0;

                    if (snoop_match
                            && msi_state[snoop_index] == MSI_M
                            && (snoop_type == BUS_RD || snoop_type == BUS_RDX)) begin
                        // We hold a dirty line another core needs. Must write it back.
                        // Capture snoop signals before bus can change mid-writeback.
                        mi_snoop_type  <= snoop_type;
                        mi_snoop_index <= snoop_index;
                        return_state   <= IDLE;
                        state          <= MI;
                    end else if (core_valid && !core_ready) begin
                        // See cache.v: guard with !core_ready to prevent re-latching
                        // on the cycle DONE transitions back to IDLE with core_ready=1.
                        latched_addr  <= core_addr;
                        latched_wdata <= core_wdata;
                        latched_wstrb <= core_wstrb;
                        state         <= COMPARE;
                    end
                end

                // -----------------------------------------------------------------
                // COMPARE: determine what to do with the latched request.
                //
                // MSI adds more cases than cache.v:
                //   Read  hit  (S or M)     -> DONE (no bus traffic)
                //   Write hit  (M only)     -> DONE (no bus traffic)
                //   Write to S line         -> BusUpgr -> SM (have data, need permission)
                //   Read  miss, dirty evict -> WRITEBACK first
                //   Read  miss, clean       -> BusRd  -> IS
                //   Write miss, dirty evict -> WRITEBACK first
                //   Write miss, clean       -> BusRdX -> IM
                //
                // For bus requests: assert bus_req and stay in COMPARE until grant
                // arrives. Only then drive the bus and transition. This keeps IS/IM/SM
                // clean - they only wait for the memory response, not the grant.
                // -----------------------------------------------------------------
                COMPARE: begin
                    if (snoop_match
                            && msi_state[snoop_index] == MSI_M
                            && (snoop_type == BUS_RD || snoop_type == BUS_RDX)) begin
                        mi_snoop_type  <= snoop_type;
                        mi_snoop_index <= snoop_index;
                        return_state   <= COMPARE;
                        state          <= MI;
                    end else if (latched_wstrb == 4'b0000) begin
                        // --- READ ---
                        if (read_hit) begin
                            // S or M both allow reads - no bus traffic needed.
                            state <= DONE;
                        end else if (valid[latched_index]
                                    && dirty[latched_index]
                                    && tag[latched_index] != latched_tag) begin
                            // Slot occupied by a dirty line for a different address.
                            // Must evict before we can fetch.
                            state <= WRITEBACK;
                        end else begin
                            // Clean miss - request a shared copy via BusRd.
                            bus_req <= 1;
                            if (bus_grant) begin
                                bus_req_valid <= 1;
                                bus_req_type  <= BUS_RD;
                                bus_req_addr  <= latched_addr;
                                bus_req_core  <= core_id;
                                state         <= IS;
                            end
                            // If !bus_grant, stay in COMPARE with bus_req=1
                            // until the arbiter gives us the bus.
                        end
                    end else begin
                        // --- WRITE ---
                        if (write_hit) begin
                            // Already in M - we have exclusive access, no bus needed.
                            state <= DONE;
                        end else if (hit && msi_state[latched_index] == MSI_S) begin
                            // We have the data in S but need write permission.
                            // BusUpgr tells other cores to invalidate - no fetch needed.
                            bus_req <= 1;
                            if (bus_grant) begin
                                bus_req_valid <= 1;
                                bus_req_type  <= BUS_UPGR;
                                bus_req_addr  <= latched_addr;
                                bus_req_core  <= core_id;
                                state         <= SM;
                            end
                        end else if (valid[latched_index]
                                    && dirty[latched_index]
                                    && tag[latched_index] != latched_tag) begin
                            // Write miss with dirty eviction needed.
                            state <= WRITEBACK;
                        end else begin
                            // Clean write miss - request exclusive copy via BusRdX.
                            bus_req <= 1;
                            if (bus_grant) begin
                                bus_req_valid <= 1;
                                bus_req_type  <= BUS_RDX;
                                bus_req_addr  <= latched_addr;
                                bus_req_core  <= core_id;
                                state         <= IM;
                            end
                        end
                    end
                end

                // -----------------------------------------------------------------
                // WRITEBACK: evict a dirty resident line before fetching the new one.
                //
                // Identical to cache.v except: after mem_ready, instead of going
                // straight to FETCH, we request the bus and go to IS or IM.
                // IS for reads, IM for writes - determined by latched_wstrb.
                // -----------------------------------------------------------------
                WRITEBACK: begin
                    if (snoop_match
                            && msi_state[snoop_index] == MSI_M
                            && (snoop_type == BUS_RD || snoop_type == BUS_RDX)) begin
                        mi_snoop_type  <= snoop_type;
                        mi_snoop_index <= snoop_index;
                        return_state   <= WRITEBACK;
                        state          <= MI;
                    end else begin
                        // Reconstruct the evicted line's address from its stored tag,
                        // not from latched_addr (that's the NEW address we want).
                        mem_valid <= 1;
                        mem_addr  <= {tag[latched_index], latched_index, 2'b00};
                        mem_wdata <= data[latched_index];
                        mem_wstrb <= 4'b1111;
                        if (mem_ready) begin
                            mem_valid                <= 0;
                            dirty[latched_index]     <= 0;
                            msi_state[latched_index] <= MSI_I; // line is evicted
                            // Eviction done. Now win the bus and fetch the new line.
                            bus_req <= 1;
                            if (bus_grant) begin
                                bus_req_valid <= 1;
                                bus_req_addr  <= latched_addr;
                                bus_req_core  <= core_id;
                                // Read miss after eviction  -> shared copy (BusRd -> IS)
                                // Write miss after eviction -> exclusive copy (BusRdX -> IM)
                                if (latched_wstrb == 4'b0000) begin
                                    bus_req_type <= BUS_RD;
                                    state        <= IS;
                                end else begin
                                    bus_req_type <= BUS_RDX;
                                    state        <= IM;
                                end
                            end
                        end
                    end
                end

                // -----------------------------------------------------------------
                // IS: issued BusRd from I, waiting for memory to supply a shared copy.
                //
                // The bus transaction is already on the wire. We just wait for
                // bus_resp_valid. When it arrives, store the data and enter S state.
                // Continue snooping while we wait - another line we hold might be M.
                // -----------------------------------------------------------------
                IS: begin
                    if (snoop_match
                            && msi_state[snoop_index] == MSI_M
                            && (snoop_type == BUS_RD || snoop_type == BUS_RDX)) begin
                        mi_snoop_type  <= snoop_type;
                        mi_snoop_index <= snoop_index;
                        return_state   <= IS;
                        state          <= MI;
                    end else if (bus_resp_valid) begin
                        // Memory responded. Grant E if no other cache has a copy
                        // (bus_resp_excl asserted), S otherwise.
                        data[latched_index]      <= bus_resp_data;
                        tag[latched_index]       <= latched_tag;
                        valid[latched_index]     <= 1;
                        dirty[latched_index]     <= 0;
                        msi_state[latched_index] <= bus_resp_excl ? MSI_E : MSI_S;
                        bus_req                  <= 0;
                        bus_req_valid            <= 0;
                        state                    <= DONE;
                    end
                end

                // -----------------------------------------------------------------
                // IM: issued BusRdX from I, waiting for exclusive copy from memory.
                //
                // Same as IS but we arrive in M state. All other caches will have
                // invalidated their copies in response to our BusRdX.
                // -----------------------------------------------------------------
                IM: begin
                    if (snoop_match
                            && msi_state[snoop_index] == MSI_M
                            && (snoop_type == BUS_RD || snoop_type == BUS_RDX)) begin
                        mi_snoop_type  <= snoop_type;
                        mi_snoop_index <= snoop_index;
                        return_state   <= IM;
                        state          <= MI;
                    end else if (bus_resp_valid) begin
                        data[latched_index]      <= bus_resp_data;
                        tag[latched_index]       <= latched_tag;
                        valid[latched_index]     <= 1;
                        dirty[latched_index]     <= 0;
                        msi_state[latched_index] <= MSI_M;  // exclusive writable copy
                        bus_req                  <= 0;
                        bus_req_valid            <= 0;
                        state                    <= DONE;
                    end
                end

                // -----------------------------------------------------------------
                // SM: issued BusUpgr from S, waiting for other caches to invalidate.
                //
                // Unlike IS/IM, we already have the data - no fetch from memory.
                // We just need acknowledgment that others have dropped their S copies.
                // bus_resp_valid serves as that acknowledgment.
                // -----------------------------------------------------------------
                SM: begin
                    if (snoop_match
                            && msi_state[snoop_index] == MSI_M
                            && (snoop_type == BUS_RD || snoop_type == BUS_RDX)) begin
                        mi_snoop_type  <= snoop_type;
                        mi_snoop_index <= snoop_index;
                        return_state   <= SM;
                        state          <= MI;
                    end else if (bus_resp_valid) begin
                        // Others have invalidated. We now have exclusive access.
                        // Data is unchanged - we had it in S already.
                        msi_state[latched_index] <= MSI_M;
                        bus_req                  <= 0;
                        bus_req_valid            <= 0;
                        state                    <= DONE;
                    end
                end

                // -----------------------------------------------------------------
                // MI: snoop reaction for an M-state line another core needs.
                //
                // We hold the only up-to-date copy. We must write it back before
                // the requesting core can use it. snoop_busy tells memory to stall
                // its response to the other core until we finish.
                //
                // Uses mi_snoop_type and mi_snoop_index (captured at MI entry) rather
                // than the live snoop_* wires. The requesting core may release the bus
                // before our writeback completes, which would collapse the live wires
                // to zero (BUS_RD) and cause an incorrect S downgrade on a BusRdX.
                //
                // After writeback:
                //   BusRd from other  -> downgrade to S (we keep a shared copy)
                //   BusRdX from other -> invalidate to I (they get exclusive)
                //
                // return_state tells us where to resume after writeback.
                // -----------------------------------------------------------------
                MI: begin
                    mem_valid  <= 1;
                    mem_addr   <= {tag[mi_snoop_index], mi_snoop_index, 2'b00};
                    mem_wdata  <= data[mi_snoop_index];
                    mem_wstrb  <= 4'b1111;
                    if (mem_ready) begin
                        mem_valid  <= 0;
                        dirty[mi_snoop_index] <= 0;
                        // BusRd:  requester wants shared copy -> we downgrade to S
                        // BusRdX: requester wants exclusive   -> we must fully vacate
                        msi_state[mi_snoop_index] <= (mi_snoop_type == BUS_RD) ? MSI_S : MSI_I;
                        state <= return_state;
                    end
                end

                // -----------------------------------------------------------------
                // DONE: serve the core for one cycle then return to IDLE.
                //
                // MSI state was already set in IS/IM/SM before we arrived here.
                // For write hits (M->M), msi_state was already MSI_M from before.
                // We set dirty=1 on writes (write-back policy).
                // -----------------------------------------------------------------
                DONE: begin
                    core_ready <= 1;
                    if (latched_wstrb == 4'b0000) begin
                        // Read - return the cached word
                        core_rdata <= data[latched_index];
                    end else begin
                        // Write - apply byte enables, mark line dirty, set M state.
                        // Explicitly setting MSI_M handles E->M silent upgrade and
                        // is idempotent when already in M (from IM or SM path).
                        if (latched_wstrb[0]) data[latched_index][ 7: 0] <= latched_wdata[ 7: 0];
                        if (latched_wstrb[1]) data[latched_index][15: 8] <= latched_wdata[15: 8];
                        if (latched_wstrb[2]) data[latched_index][23:16] <= latched_wdata[23:16];
                        if (latched_wstrb[3]) data[latched_index][31:24] <= latched_wdata[31:24];
                        dirty[latched_index]     <= 1;
                        msi_state[latched_index] <= MSI_M;
                    end
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule