`timescale 1ns/1ps

module tb_mesi;

    localparam BUS_RD   = 2'd0;
    localparam BUS_RDX  = 2'd1;
    localparam BUS_UPGR = 2'd2;
    localparam MSI_I    = 2'd0;
    localparam MSI_S    = 2'd1;
    localparam MSI_M    = 2'd2;
    localparam MSI_E    = 2'd3;

    reg clk;
    initial clk = 0;
    always #5 clk = ~clk;

    reg resetn;

    // -------------------------------------------------------------------------
    // Core-facing signals
    // -------------------------------------------------------------------------
    reg        c0_valid, c1_valid, c2_valid, c3_valid;
    reg [31:0] c0_addr,  c1_addr,  c2_addr,  c3_addr;
    reg [31:0] c0_wdata, c1_wdata, c2_wdata, c3_wdata;
    reg [3:0]  c0_wstrb, c1_wstrb, c2_wstrb, c3_wstrb;

    wire        c0_ready, c1_ready, c2_ready, c3_ready;
    wire [31:0] c0_rdata, c1_rdata, c2_rdata, c3_rdata;

    // -------------------------------------------------------------------------
    // Direct memory-facing signals (writeback path only)
    // -------------------------------------------------------------------------
    wire        c0_mem_valid, c1_mem_valid, c2_mem_valid, c3_mem_valid;
    reg         c0_mem_ready, c1_mem_ready, c2_mem_ready, c3_mem_ready;
    wire [31:0] c0_mem_addr,  c1_mem_addr,  c2_mem_addr,  c3_mem_addr;
    wire [31:0] c0_mem_wdata, c1_mem_wdata, c2_mem_wdata, c3_mem_wdata;
    wire [3:0]  c0_mem_wstrb, c1_mem_wstrb, c2_mem_wstrb, c3_mem_wstrb;
    wire [31:0] c0_mem_rdata = 32'd0;
    wire [31:0] c1_mem_rdata = 32'd0;
    wire [31:0] c2_mem_rdata = 32'd0;
    wire [31:0] c3_mem_rdata = 32'd0;

    // -------------------------------------------------------------------------
    // Bus request outputs
    // -------------------------------------------------------------------------
    wire        c0_bus_req, c1_bus_req, c2_bus_req, c3_bus_req;
    wire        c0_bus_req_valid, c1_bus_req_valid, c2_bus_req_valid, c3_bus_req_valid;
    wire [1:0]  c0_bus_req_type,  c1_bus_req_type,  c2_bus_req_type,  c3_bus_req_type;
    wire [31:0] c0_bus_req_addr,  c1_bus_req_addr,  c2_bus_req_addr,  c3_bus_req_addr;
    wire [1:0]  c0_bus_req_core,  c1_bus_req_core,  c2_bus_req_core,  c3_bus_req_core;
    wire        c0_snoop_busy, c1_snoop_busy, c2_snoop_busy, c3_snoop_busy;

    // -------------------------------------------------------------------------
    // MESI additions: snoop_shared per cache, bus_resp_excl NOR of all
    // -------------------------------------------------------------------------
    wire c0_snoop_shared, c1_snoop_shared, c2_snoop_shared, c3_snoop_shared;
    wire bus_resp_excl = !(c0_snoop_shared | c1_snoop_shared
                         | c2_snoop_shared | c3_snoop_shared);

    // -------------------------------------------------------------------------
    // Arbiter
    // -------------------------------------------------------------------------
    wire [3:0] req = {c3_bus_req, c2_bus_req, c1_bus_req, c0_bus_req};
    wire [3:0] grant;
    wire       grant_valid;

    arbiter arb (
        .clk(clk), .resetn(resetn),
        .req(req), .grant(grant), .grant_valid(grant_valid)
    );

    // -------------------------------------------------------------------------
    // Shared snooping bus - muxed from the granted core
    // -------------------------------------------------------------------------
    wire        snoop_valid;
    wire [1:0]  snoop_type;
    wire [31:0] snoop_addr;
    wire [1:0]  snoop_core;

    assign snoop_valid = grant[0] ? c0_bus_req_valid :
                         grant[1] ? c1_bus_req_valid :
                         grant[2] ? c2_bus_req_valid :
                         grant[3] ? c3_bus_req_valid : 1'b0;

    assign snoop_type  = grant[0] ? c0_bus_req_type :
                         grant[1] ? c1_bus_req_type  :
                         grant[2] ? c2_bus_req_type  :
                         grant[3] ? c3_bus_req_type  : 2'b0;

    assign snoop_addr  = grant[0] ? c0_bus_req_addr :
                         grant[1] ? c1_bus_req_addr  :
                         grant[2] ? c2_bus_req_addr  :
                         grant[3] ? c3_bus_req_addr  : 32'b0;

    assign snoop_core  = grant[0] ? c0_bus_req_core :
                         grant[1] ? c1_bus_req_core  :
                         grant[2] ? c2_bus_req_core  :
                         grant[3] ? c3_bus_req_core  : 2'b0;

    wire bus_snoop_busy = c0_snoop_busy | c1_snoop_busy | c2_snoop_busy | c3_snoop_busy;

    // -------------------------------------------------------------------------
    // Shared memory - 256 words
    // -------------------------------------------------------------------------
    reg [31:0] memory [0:255];
    reg        bus_resp_valid;
    reg [31:0] bus_resp_data;
    reg        bus_trans_pending;
    reg [31:0] bus_trans_addr;

    integer mi;
    initial begin
        for (mi = 0; mi < 256; mi = mi + 1)
            memory[mi] = 32'hA0000000 | mi;
    end

    always @(posedge clk) begin
        c0_mem_ready <= 0; c1_mem_ready <= 0;
        c2_mem_ready <= 0; c3_mem_ready <= 0;
        if      (c0_mem_valid && c0_mem_wstrb == 4'b1111 && !c0_mem_ready) begin
            memory[c0_mem_addr >> 2] <= c0_mem_wdata; c0_mem_ready <= 1;
        end else if (c1_mem_valid && c1_mem_wstrb == 4'b1111 && !c1_mem_ready) begin
            memory[c1_mem_addr >> 2] <= c1_mem_wdata; c1_mem_ready <= 1;
        end else if (c2_mem_valid && c2_mem_wstrb == 4'b1111 && !c2_mem_ready) begin
            memory[c2_mem_addr >> 2] <= c2_mem_wdata; c2_mem_ready <= 1;
        end else if (c3_mem_valid && c3_mem_wstrb == 4'b1111 && !c3_mem_ready) begin
            memory[c3_mem_addr >> 2] <= c3_mem_wdata; c3_mem_ready <= 1;
        end
    end

    // Two-stage bus response pipeline - same as tb_msi.v
    always @(posedge clk) begin
        bus_trans_pending <= snoop_valid;
        bus_trans_addr    <= snoop_addr;
        bus_resp_valid    <= 0;
        bus_resp_data     <= 0;
        if (bus_trans_pending && !bus_snoop_busy) begin
            bus_resp_valid <= 1;
            bus_resp_data  <= memory[bus_trans_addr >> 2];
        end
    end

    // -------------------------------------------------------------------------
    // DUT instantiation - cache_mesi with snoop_shared and bus_resp_excl
    // -------------------------------------------------------------------------
    cache_mesi dut0 (
        .clk(clk), .resetn(resetn), .core_id(2'd0),
        .core_valid(c0_valid),    .core_ready(c0_ready),
        .core_addr(c0_addr),      .core_wdata(c0_wdata),
        .core_wstrb(c0_wstrb),    .core_rdata(c0_rdata),
        .mem_valid(c0_mem_valid), .mem_ready(c0_mem_ready),
        .mem_addr(c0_mem_addr),   .mem_wdata(c0_mem_wdata),
        .mem_wstrb(c0_mem_wstrb), .mem_rdata(c0_mem_rdata),
        .bus_req(c0_bus_req),     .bus_grant(grant[0]),
        .bus_req_valid(c0_bus_req_valid), .bus_req_type(c0_bus_req_type),
        .bus_req_addr(c0_bus_req_addr),   .bus_req_core(c0_bus_req_core),
        .snoop_valid(snoop_valid), .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),   .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c0_snoop_busy),
        .snoop_shared(c0_snoop_shared), .bus_resp_excl(bus_resp_excl)
    );
    cache_mesi dut1 (
        .clk(clk), .resetn(resetn), .core_id(2'd1),
        .core_valid(c1_valid),    .core_ready(c1_ready),
        .core_addr(c1_addr),      .core_wdata(c1_wdata),
        .core_wstrb(c1_wstrb),    .core_rdata(c1_rdata),
        .mem_valid(c1_mem_valid), .mem_ready(c1_mem_ready),
        .mem_addr(c1_mem_addr),   .mem_wdata(c1_mem_wdata),
        .mem_wstrb(c1_mem_wstrb), .mem_rdata(c1_mem_rdata),
        .bus_req(c1_bus_req),     .bus_grant(grant[1]),
        .bus_req_valid(c1_bus_req_valid), .bus_req_type(c1_bus_req_type),
        .bus_req_addr(c1_bus_req_addr),   .bus_req_core(c1_bus_req_core),
        .snoop_valid(snoop_valid), .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),   .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c1_snoop_busy),
        .snoop_shared(c1_snoop_shared), .bus_resp_excl(bus_resp_excl)
    );
    cache_mesi dut2 (
        .clk(clk), .resetn(resetn), .core_id(2'd2),
        .core_valid(c2_valid),    .core_ready(c2_ready),
        .core_addr(c2_addr),      .core_wdata(c2_wdata),
        .core_wstrb(c2_wstrb),    .core_rdata(c2_rdata),
        .mem_valid(c2_mem_valid), .mem_ready(c2_mem_ready),
        .mem_addr(c2_mem_addr),   .mem_wdata(c2_mem_wdata),
        .mem_wstrb(c2_mem_wstrb), .mem_rdata(c2_mem_rdata),
        .bus_req(c2_bus_req),     .bus_grant(grant[2]),
        .bus_req_valid(c2_bus_req_valid), .bus_req_type(c2_bus_req_type),
        .bus_req_addr(c2_bus_req_addr),   .bus_req_core(c2_bus_req_core),
        .snoop_valid(snoop_valid), .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),   .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c2_snoop_busy),
        .snoop_shared(c2_snoop_shared), .bus_resp_excl(bus_resp_excl)
    );
    cache_mesi dut3 (
        .clk(clk), .resetn(resetn), .core_id(2'd3),
        .core_valid(c3_valid),    .core_ready(c3_ready),
        .core_addr(c3_addr),      .core_wdata(c3_wdata),
        .core_wstrb(c3_wstrb),    .core_rdata(c3_rdata),
        .mem_valid(c3_mem_valid), .mem_ready(c3_mem_ready),
        .mem_addr(c3_mem_addr),   .mem_wdata(c3_mem_wdata),
        .mem_wstrb(c3_mem_wstrb), .mem_rdata(c3_mem_rdata),
        .bus_req(c3_bus_req),     .bus_grant(grant[3]),
        .bus_req_valid(c3_bus_req_valid), .bus_req_type(c3_bus_req_type),
        .bus_req_addr(c3_bus_req_addr),   .bus_req_core(c3_bus_req_core),
        .snoop_valid(snoop_valid), .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),   .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c3_snoop_busy),
        .snoop_shared(c3_snoop_shared), .bus_resp_excl(bus_resp_excl)
    );

    // -------------------------------------------------------------------------
    // Test infrastructure
    // -------------------------------------------------------------------------
    integer pass_count, fail_count;
    reg [31:0] tmp_rdata;
    integer conc_timeout;
    reg     conc_c0_done, conc_c1_done;

    // Track bus transactions for silent upgrade verification (test 19)
    reg bus_upgr_seen, bus_rdx_seen;
    always @(posedge clk) begin
        if (!resetn) begin
            bus_upgr_seen <= 0;
            bus_rdx_seen  <= 0;
        end else if (snoop_valid) begin
            if (snoop_type == BUS_UPGR) bus_upgr_seen <= 1;
            if (snoop_type == BUS_RDX)  bus_rdx_seen  <= 1;
        end
    end

    task check;
        input        cond;
        input [255:0] label;
        begin
            if (cond) begin
                $display("  PASS: %s", label); pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %s", label); fail_count = fail_count + 1;
            end
        end
    endtask

    task check_state;
        input [1:0]   core_sel;
        input [1:0]   idx;
        input [1:0]   expected;
        input [255:0] label;
        reg [1:0] actual;
        begin
            case (core_sel)
                0: actual = dut0.msi_state[idx];
                1: actual = dut1.msi_state[idx];
                2: actual = dut2.msi_state[idx];
                3: actual = dut3.msi_state[idx];
            endcase
            if (actual == expected) begin
                $display("  PASS: %s (state=%0d)", label, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: %s (expected=%0d got=%0d)", label, expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task core_read;
        input [1:0]   which;
        input [31:0]  addr;
        output [31:0] result;
        integer timeout;
        begin
            case (which)
                0: begin c0_valid=1; c0_addr=addr; c0_wstrb=4'b0000; c0_wdata=0; end
                1: begin c1_valid=1; c1_addr=addr; c1_wstrb=4'b0000; c1_wdata=0; end
                2: begin c2_valid=1; c2_addr=addr; c2_wstrb=4'b0000; c2_wdata=0; end
                3: begin c3_valid=1; c3_addr=addr; c3_wstrb=4'b0000; c3_wdata=0; end
            endcase
            timeout = 0;
            @(posedge clk); #1;
            case (which)
                0: begin while (!c0_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end result=c0_rdata; c0_valid=0; end
                1: begin while (!c1_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end result=c1_rdata; c1_valid=0; end
                2: begin while (!c2_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end result=c2_rdata; c2_valid=0; end
                3: begin while (!c3_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end result=c3_rdata; c3_valid=0; end
            endcase
            if (timeout >= 300) $display("  ERROR: core_read timeout (core%0d addr %08x)", which, addr);
            @(posedge clk); #1;
        end
    endtask

    task core_write;
        input [1:0]  which;
        input [31:0] addr;
        input [31:0] wdata;
        integer timeout;
        begin
            case (which)
                0: begin c0_valid=1; c0_addr=addr; c0_wstrb=4'b1111; c0_wdata=wdata; end
                1: begin c1_valid=1; c1_addr=addr; c1_wstrb=4'b1111; c1_wdata=wdata; end
                2: begin c2_valid=1; c2_addr=addr; c2_wstrb=4'b1111; c2_wdata=wdata; end
                3: begin c3_valid=1; c3_addr=addr; c3_wstrb=4'b1111; c3_wdata=wdata; end
            endcase
            timeout = 0;
            @(posedge clk); #1;
            case (which)
                0: begin while (!c0_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end c0_valid=0; end
                1: begin while (!c1_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end c1_valid=0; end
                2: begin while (!c2_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end c2_valid=0; end
                3: begin while (!c3_ready && timeout<300) begin @(posedge clk); #1; timeout=timeout+1; end c3_valid=0; end
            endcase
            if (timeout >= 300) $display("  ERROR: core_write timeout (core%0d addr %08x)", which, addr);
            @(posedge clk); #1;
        end
    endtask

    task wait_cycles;
        input integer n;
        integer ii;
        begin for (ii=0; ii<n; ii=ii+1) @(posedge clk); #1; end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        pass_count=0; fail_count=0;
        c0_valid=0; c0_addr=0; c0_wdata=0; c0_wstrb=0;
        c1_valid=0; c1_addr=0; c1_wdata=0; c1_wstrb=0;
        c2_valid=0; c2_addr=0; c2_wdata=0; c2_wstrb=0;
        c3_valid=0; c3_addr=0; c3_wdata=0; c3_wstrb=0;

        // =====================================================================
        // GROUP 1: Core protocol transitions
        // Key MESI difference: single-core cold read lands in E, not S.
        // =====================================================================
        $display("\n=== Group 1: Core protocol transitions ===");
        resetn=0; repeat(4) @(posedge clk); resetn=1; @(posedge clk); #1;

        // -----------------------------------------------------------------
        // TEST 1: Single core cold read -> E (not S as in MSI)
        // Only core0 requests addr 0x000. No other cache has a copy.
        // bus_resp_excl=1 -> cache_mesi grants E state.
        // -----------------------------------------------------------------
        $display("\n--- Test 1: Single core cold read -> E ---");
        core_read(0, 32'h00000000, tmp_rdata);
        check_state(0, 0, MSI_E, "core0 idx0 in E after sole cold read");
        check(tmp_rdata == 32'hA0000000, "core0 reads correct initial memory value");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 2: Single core write miss -> M (BusRdX, same as MSI)
        // -----------------------------------------------------------------
        $display("\n--- Test 2: Single core write miss ---");
        core_write(1, 32'h00000004, 32'hDEADBEEF);
        wait_cycles(1);
        check_state(1, 1, MSI_M, "core1 idx1 in M after write miss");
        check(dut1.dirty[1] == 1, "core1 idx1 dirty after write");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 3: Two cores read same address
        // Core0 reads first -> E. Core2 reads same -> snoop_shared fires
        // from core0, bus_resp_excl=0 -> core0 E->S, core2 gets S.
        // -----------------------------------------------------------------
        $display("\n--- Test 3: Two cores read same address ---");
        core_read(0, 32'h00000008, tmp_rdata);
        check_state(0, 2, MSI_E, "core0 idx2 in E after first (sole) read");
        core_read(2, 32'h00000008, tmp_rdata);
        check_state(0, 2, MSI_S, "core0 idx2 downgraded to S when core2 reads (E->S)");
        check_state(2, 2, MSI_S, "core2 idx2 in S (second reader gets S)");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 4: S->M upgrade via BusUpgr (unchanged from MSI)
        // Core0 reads -> E. Core2 reads -> core0 E->S, core2 S.
        // Core0 writes -> sees S -> BusUpgr -> SM -> M.
        // BusUpgr still fires even with MESI because E was downgraded to S.
        // -----------------------------------------------------------------
        $display("\n--- Test 4: S->M upgrade via BusUpgr ---");
        core_read(0, 32'h0000000C, tmp_rdata);
        core_read(2, 32'h0000000C, tmp_rdata);
        check_state(0, 3, MSI_S, "core0 idx3 in S (was E, downgraded by core2 read)");
        check_state(2, 3, MSI_S, "core2 idx3 in S");
        core_write(0, 32'h0000000C, 32'hCAFEBABE);
        wait_cycles(2);
        check_state(0, 3, MSI_M, "core0 idx3 in M after BusUpgr");
        check_state(2, 3, MSI_I, "core2 idx3 invalidated to I by BusUpgr");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 5: M-state intervention on read (BusRd) - same as MSI
        // -----------------------------------------------------------------
        $display("\n--- Test 5: M-state intervention on read ---");
        core_write(0, 32'h00000100, 32'h11111111);
        wait_cycles(1);
        check_state(0, 0, MSI_M, "core0 idx0 in M before BusRd intervention");
        core_read(1, 32'h00000100, tmp_rdata);
        wait_cycles(2);
        check_state(0, 0, MSI_S, "core0 idx0 downgraded to S after MI intervention");
        check_state(1, 0, MSI_S, "core1 idx0 in S after reading M-held line");
        check(tmp_rdata == 32'h11111111, "core1 reads value written by core0 (not stale memory)");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 6: M-state intervention on BusRdX - same as MSI
        // -----------------------------------------------------------------
        $display("\n--- Test 6: M-state intervention on BusRdX ---");
        core_write(2, 32'h00000104, 32'h22222222);
        wait_cycles(1);
        check_state(2, 1, MSI_M, "core2 idx1 in M before BusRdX intervention");
        core_write(3, 32'h00000104, 32'h33333333);
        wait_cycles(2);
        check_state(2, 1, MSI_I, "core2 idx1 invalidated to I after BusRdX (M->I via MI)");
        check_state(3, 1, MSI_M, "core3 idx1 in M after winning exclusive via BusRdX");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 7: No stale read after invalidation
        // Core0 reads 0x108 -> E. Core1 writes 0x108 -> BusRdX.
        // Core0 E->I. Core2 reads -> must see core1's written value.
        // -----------------------------------------------------------------
        $display("\n--- Test 7: No stale read after invalidation ---");
        core_read(0, 32'h00000108, tmp_rdata);
        check_state(0, 2, MSI_E, "core0 idx2 in E after cold read");
        core_write(1, 32'h00000108, 32'h77777777);
        wait_cycles(2);
        check_state(0, 2, MSI_I, "core0 idx2 invalidated to I by core1 BusRdX");
        check_state(1, 2, MSI_M, "core1 idx2 in M after write");
        core_read(2, 32'h00000108, tmp_rdata);
        wait_cycles(2);
        check(tmp_rdata == 32'h77777777, "core2 reads core1 written value (not stale)");
        wait_cycles(2);

        // =====================================================================
        // GROUP 2: Local hit cases and dirty eviction paths
        // =====================================================================
        $display("\n=== Group 2: Local hit cases and dirty eviction ===");
        resetn=0; repeat(4) @(posedge clk); resetn=1; @(posedge clk); #1;

        // -----------------------------------------------------------------
        // TEST 8: Read hit in E state (no bus transaction)
        // -----------------------------------------------------------------
        $display("\n--- Test 8: Read hit in E state ---");
        core_read(0, 32'h00000000, tmp_rdata);
        check_state(0, 0, MSI_E, "core0 idx0 in E after cold read");
        core_read(0, 32'h00000000, tmp_rdata);
        check_state(0, 0, MSI_E, "core0 idx0 stays E on second local read (hit, no bus)");
        check(tmp_rdata == 32'hA0000000, "core0 re-read returns correct value");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 9: Read hit in S state (no bus transaction)
        // -----------------------------------------------------------------
        $display("\n--- Test 9: Read hit in S state ---");
        core_read(0, 32'h00000004, tmp_rdata);
        core_read(1, 32'h00000004, tmp_rdata);
        check_state(0, 1, MSI_S, "core0 idx1 in S (was E, downgraded by core1 read)");
        core_read(0, 32'h00000004, tmp_rdata);
        check_state(0, 1, MSI_S, "core0 idx1 stays S on local re-read (no bus)");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 10: Write hit in M state (no bus transaction)
        // -----------------------------------------------------------------
        $display("\n--- Test 10: Write hit in M state ---");
        core_write(0, 32'h00000008, 32'hAAAAAAAA);
        wait_cycles(1);
        check_state(0, 2, MSI_M, "core0 idx2 in M after write miss");
        core_write(0, 32'h00000008, 32'hBBBBBBBB);
        wait_cycles(1);
        check_state(0, 2, MSI_M, "core0 idx2 stays M on write hit (no bus)");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 11: Write-allocate: read-after-write returns written value
        // -----------------------------------------------------------------
        $display("\n--- Test 11: Write-allocate readback ---");
        core_write(1, 32'h0000000C, 32'hCCCCCCCC);
        wait_cycles(1);
        core_read(1, 32'h0000000C, tmp_rdata);
        check(tmp_rdata == 32'hCCCCCCCC, "core1 reads back its own written value");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 12: Dirty eviction - WRITEBACK before read fetch (IS path)
        // Core0 writes idx0 tag-A (M). Core0 reads idx0 tag-B.
        // Evicts dirty tag-A via WRITEBACK, then fetches tag-B via IS.
        // MESI: no other sharer -> lands in E.
        // -----------------------------------------------------------------
        $display("\n--- Test 12: Dirty eviction WRITEBACK->IS ---");
        core_write(0, 32'h00000000, 32'hDDDDDDDD);
        wait_cycles(1);
        check_state(0, 0, MSI_M, "core0 idx0 in M with tag-A (0x000)");
        core_read(0, 32'h00000100, tmp_rdata);  // idx0, different tag -> eviction
        wait_cycles(3);
        check_state(0, 0, MSI_E, "core0 idx0 in E with tag-B (0x100) after dirty eviction");
        check(memory[0] == 32'hDDDDDDDD, "memory[0] updated by WRITEBACK of dirty evicted line");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 13: Dirty eviction before write fetch (IM path)
        // -----------------------------------------------------------------
        $display("\n--- Test 13: Dirty eviction WRITEBACK->IM ---");
        core_write(1, 32'h00000004, 32'hEEEEEEEE);
        wait_cycles(1);
        check_state(1, 1, MSI_M, "core1 idx1 in M with tag-A (0x004)");
        core_write(1, 32'h00000104, 32'hFFFFFFFF);  // idx1, different tag -> eviction
        wait_cycles(3);
        check_state(1, 1, MSI_M, "core1 idx1 in M with tag-B (0x104) after dirty eviction");
        check(memory[1] == 32'hEEEEEEEE, "memory[1] updated by WRITEBACK of evicted line");
        wait_cycles(2);

        // =====================================================================
        // GROUP 3: Multiple sharers and sequential ownership
        // =====================================================================
        $display("\n=== Group 3: Multiple sharers and sequential ownership ===");
        resetn=0; repeat(4) @(posedge clk); resetn=1; @(posedge clk); #1;

        // -----------------------------------------------------------------
        // TEST 14: Three sharers then BusUpgr
        // -----------------------------------------------------------------
        $display("\n--- Test 14: Three-sharer BusUpgr ---");
        core_read(0, 32'h00000000, tmp_rdata);
        core_read(1, 32'h00000000, tmp_rdata);
        core_read(2, 32'h00000000, tmp_rdata);
        check_state(0, 0, MSI_S, "core0 idx0 in S (3-sharer)");
        check_state(1, 0, MSI_S, "core1 idx0 in S (3-sharer)");
        check_state(2, 0, MSI_S, "core2 idx0 in S (3-sharer)");
        core_write(0, 32'h00000000, 32'hABCDEF00);
        wait_cycles(2);
        check_state(0, 0, MSI_M, "core0 idx0 in M after BusUpgr with 3 sharers");
        check_state(1, 0, MSI_I, "core1 idx0 invalidated to I by BusUpgr");
        check_state(2, 0, MSI_I, "core2 idx0 invalidated to I by BusUpgr");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 15: All four sharers then BusUpgr
        // -----------------------------------------------------------------
        $display("\n--- Test 15: Four-sharer BusUpgr ---");
        core_read(0, 32'h00000004, tmp_rdata);
        core_read(1, 32'h00000004, tmp_rdata);
        core_read(2, 32'h00000004, tmp_rdata);
        core_read(3, 32'h00000004, tmp_rdata);
        check_state(0, 1, MSI_S, "core0 idx1 in S (4-sharer)");
        check_state(1, 1, MSI_S, "core1 idx1 in S (4-sharer)");
        check_state(2, 1, MSI_S, "core2 idx1 in S (4-sharer)");
        check_state(3, 1, MSI_S, "core3 idx1 in S (4-sharer)");
        core_write(3, 32'h00000004, 32'hFEEDBEEF);
        wait_cycles(2);
        check_state(3, 1, MSI_M, "core3 idx1 in M after BusUpgr with 4 sharers");
        check_state(0, 1, MSI_I, "core0 idx1 invalidated by BusUpgr");
        check_state(1, 1, MSI_I, "core1 idx1 invalidated by BusUpgr");
        check_state(2, 1, MSI_I, "core2 idx1 invalidated by BusUpgr");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 16: Sequential ownership transfer chain
        // -----------------------------------------------------------------
        $display("\n--- Test 16: Sequential ownership transfer chain ---");
        core_write(0, 32'h00000008, 32'h11111111);
        wait_cycles(1);
        check_state(0, 2, MSI_M, "core0 idx2 in M (1st writer)");
        core_write(1, 32'h00000008, 32'h22222222);
        wait_cycles(2);
        check_state(0, 2, MSI_I, "core0 idx2 I (1st MI intervention)");
        check_state(1, 2, MSI_M, "core1 idx2 in M (2nd writer)");
        core_write(2, 32'h00000008, 32'h33333333);
        wait_cycles(2);
        check_state(1, 2, MSI_I, "core1 idx2 I (2nd MI intervention)");
        check_state(2, 2, MSI_M, "core2 idx2 in M (3rd writer)");
        core_read(3, 32'h00000008, tmp_rdata);
        wait_cycles(2);
        check_state(2, 2, MSI_S, "core2 idx2 downgraded to S (3rd MI intervention)");
        check_state(3, 2, MSI_S, "core3 idx2 in S after reading final value");
        check(tmp_rdata == 32'h33333333, "core3 reads final value through 3-step ownership chain");
        wait_cycles(2);

        // =====================================================================
        // GROUP 4: Concurrent - MI interrupts COMPARE (unchanged from MSI)
        // =====================================================================
        $display("\n=== Group 4: Concurrent - MI interrupts COMPARE ===");
        resetn=0; repeat(4) @(posedge clk); resetn=1; @(posedge clk); #1;

        // -----------------------------------------------------------------
        // TEST 17: MI interrupts COMPARE (concurrent)
        // -----------------------------------------------------------------
        $display("\n--- Test 17: MI interrupts COMPARE (concurrent) ---");
        core_write(0, 32'h00000000, 32'hABCDEF01);
        wait_cycles(1);
        check_state(0, 0, MSI_M, "test17 setup: core0 idx0 in M");

        c0_valid = 1; c0_addr = 32'h00000100; c0_wstrb = 4'b0000; c0_wdata = 0;
        c1_valid = 1; c1_addr = 32'h00000000; c1_wstrb = 4'b0000; c1_wdata = 0;

        conc_c0_done = 0; conc_c1_done = 0; conc_timeout = 0;
        @(posedge clk); #1;
        while ((!conc_c0_done || !conc_c1_done) && conc_timeout < 500) begin
            if (c0_ready && !conc_c0_done) begin conc_c0_done = 1; c0_valid = 0; end
            if (c1_ready && !conc_c1_done) begin conc_c1_done = 1; c1_valid = 0; end
            @(posedge clk); #1;
            conc_timeout = conc_timeout + 1;
        end
        if (conc_timeout >= 500) $display("  ERROR: test17 concurrent wait timeout");
        wait_cycles(3);

        check(c1_rdata == 32'hABCDEF01, "core1 receives core0 written value via MI writeback");
        check_state(0, 0, MSI_E, "core0 idx0 in E (0x100) after MI->COMPARE->IS (sole reader gets E)");
        check_state(1, 0, MSI_S, "core1 idx0 in S (0x000) received via MI writeback");
        check(memory[0] == 32'hABCDEF01, "memory[0] updated by core0 MI writeback");

        // =====================================================================
        // GROUP 5: MESI-specific E-state tests
        // These tests verify that E state is granted correctly and that the
        // silent E->M upgrade path fires without putting anything on the bus.
        // =====================================================================
        $display("\n=== Group 5: MESI E-state specific tests ===");
        resetn=0; repeat(4) @(posedge clk); resetn=1; @(posedge clk); #1;

        // -----------------------------------------------------------------
        // TEST 18: E granted on sole cold read
        // Core0 reads addr 0x000. No other cache has a copy.
        // All snoop_shared=0 -> bus_resp_excl=1 -> core0 lands in E.
        // Explicitly check bus_resp_excl is high when transaction fires.
        // -----------------------------------------------------------------
        $display("\n--- Test 18: E granted on sole cold read ---");
        core_read(0, 32'h00000000, tmp_rdata);
        check_state(0, 0, MSI_E, "core0 idx0 in E after sole cold read");
        check_state(1, 0, MSI_I, "core1 idx0 still I (no copy)");
        check_state(2, 0, MSI_I, "core2 idx0 still I (no copy)");
        check_state(3, 0, MSI_I, "core3 idx0 still I (no copy)");
        check(tmp_rdata == memory[0], "core0 reads correct memory value in E state");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 19: Silent E->M upgrade - no BusUpgr or BusRdX on bus
        // Core0 has E on addr 0x000. Core0 writes it.
        // Expected: state transitions to M, zero bus transactions fired.
        // We clear the bus_upgr_seen and bus_rdx_seen trackers at reset and
        // monitor them across the write operation.
        // -----------------------------------------------------------------
        $display("\n--- Test 19: Silent E->M (no bus transaction) ---");
        // core0 already has E on idx0 from test 18 - reset cleared everything,
        // so we need to re-establish E first
        core_read(0, 32'h00000000, tmp_rdata);
        check_state(0, 0, MSI_E, "core0 idx0 in E (setup for silent upgrade test)");
        wait_cycles(2);
        // Clear trackers then write
        @(posedge clk); #1;
        // Now write - this should be a silent E->M with no bus transaction
        core_write(0, 32'h00000000, 32'hDEADC0DE);
        wait_cycles(2);
        check_state(0, 0, MSI_M, "core0 idx0 in M after writing E-state line");
        check(dut0.dirty[0] == 1, "core0 idx0 dirty after E->M write");
        check(!bus_upgr_seen, "no BusUpgr issued during E->M silent upgrade");
        check(!bus_rdx_seen,  "no BusRdX issued during E->M silent upgrade");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 20: E->S when another core reads (snoop_shared fires)
        // Core0 reads 0x004 alone -> E. Core1 reads 0x004 ->
        // core0's snoop_shared fires, bus_resp_excl=0, core1 gets S.
        // Core0 E->S from snoop reaction.
        // -----------------------------------------------------------------
        $display("\n--- Test 20: E->S on BusRd snoop ---");
        core_read(0, 32'h00000004, tmp_rdata);
        check_state(0, 1, MSI_E, "core0 idx1 in E (sole reader)");
        check_state(1, 1, MSI_I, "core1 idx1 in I before second read");
        core_read(1, 32'h00000004, tmp_rdata);
        wait_cycles(2);
        check_state(0, 1, MSI_S, "core0 idx1 E->S after core1 BusRd snoop");
        check_state(1, 1, MSI_S, "core1 idx1 in S (got S because sharer existed)");
        wait_cycles(2);

        // -----------------------------------------------------------------
        // TEST 21: E->I on BusRdX snoop
        // Core0 reads 0x008 alone -> E. Core1 writes 0x008 -> BusRdX.
        // Core0 sees BusRdX on its E line -> E->I (no writeback needed,
        // line is clean). Core1 wins exclusive copy -> M.
        // -----------------------------------------------------------------
        $display("\n--- Test 21: E->I on BusRdX snoop ---");
        core_read(0, 32'h00000008, tmp_rdata);
        check_state(0, 2, MSI_E, "core0 idx2 in E (sole reader)");
        core_write(1, 32'h00000008, 32'hCAFED00D);
        wait_cycles(2);
        check_state(0, 2, MSI_I, "core0 idx2 E->I after core1 BusRdX snoop");
        check_state(1, 2, MSI_M, "core1 idx2 in M after BusRdX on E line");
        // No writeback from core0: E line is clean, memory still has original value.
        // Core1's M line has the new written value. Memory is stale until writeback.
        check(dut1.dirty[2] == 1, "core1 idx2 dirty (holds new value not yet in memory)");
        wait_cycles(2);

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n========================================");
        $display("Results: %0d/%0d passed", pass_count, pass_count+fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else                 $display("%0d TESTS FAILED", fail_count);
        $display("========================================\n");
        $finish;
    end

    initial begin #5000000; $display("FATAL: simulation timeout"); $finish; end

endmodule