`timescale 1ns/1ps

module tb_cache;

    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    reg clk;
    reg resetn;

    initial clk = 0;
    always #5 clk = ~clk;  // 100MHz clock, 10ns period

    // -------------------------------------------------------------------------
    // Cache DUT ports
    // -------------------------------------------------------------------------
    reg         core_valid;
    wire        core_ready;
    reg  [31:0] core_addr;
    reg  [31:0] core_wdata;
    reg  [3:0]  core_wstrb;
    wire [31:0] core_rdata;

    wire        mem_valid;
    reg         mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    reg  [31:0] mem_rdata;

    // -------------------------------------------------------------------------
    // Instantiate cache
    // -------------------------------------------------------------------------
    cache dut (
        .clk        (clk),
        .resetn     (resetn),
        .core_valid (core_valid),
        .core_ready (core_ready),
        .core_addr  (core_addr),
        .core_wdata (core_wdata),
        .core_wstrb (core_wstrb),
        .core_rdata (core_rdata),
        .mem_valid  (mem_valid),
        .mem_ready  (mem_ready),
        .mem_addr   (mem_addr),
        .mem_wdata  (mem_wdata),
        .mem_wstrb  (mem_wstrb),
        .mem_rdata  (mem_rdata)
    );

    // -------------------------------------------------------------------------
    // Simple memory model
    // 256 words (1KB). Responds to cache requests in 1 cycle.
    // mem_ready goes high the cycle after mem_valid, mimicking a real memory.
    // -------------------------------------------------------------------------
    reg [31:0] memory [0:255];
    integer    mem_idx;

    // Pre-load memory with known values so we can verify fetches
    initial begin
        for (mem_idx = 0; mem_idx < 256; mem_idx = mem_idx + 1)
            memory[mem_idx] = 32'hA000_0000 + mem_idx;  // e.g. 0xA0000005 at index 5
    end

    always @(posedge clk) begin
        mem_ready <= 0;
        if (mem_valid && !mem_ready) begin
            mem_ready <= 1;
            if (mem_wstrb == 4'b0000) begin
                // Read (fetch)
                mem_rdata <= memory[mem_addr >> 2];
                $display("  [MEM] READ  addr=0x%08h data=0x%08h", mem_addr, memory[mem_addr >> 2]);
            end else begin
                // Write (writeback)
                if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
                if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
                if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
                if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
                $display("  [MEM] WRITE addr=0x%08h data=0x%08h wstrb=%b", mem_addr, mem_wdata, mem_wstrb);
            end
        end
    end

    // -------------------------------------------------------------------------
    // Test infrastructure
    // -------------------------------------------------------------------------
    integer pass_count;
    integer fail_count;

    // Issue a read request to the cache and wait for core_ready.
    // Returns the data on core_rdata.
    task cache_read;
        input [31:0] addr;
        output [31:0] rdata;
        begin
            @(negedge clk);
            core_valid <= 1;
            core_addr  <= addr;
            core_wdata <= 32'b0;
            core_wstrb <= 4'b0000;  // 0000 = read
            @(posedge clk);
            while (!core_ready) @(posedge clk);
            rdata = core_rdata;
            @(negedge clk);
            core_valid <= 0;
            @(posedge clk);
        end
    endtask

    // Issue a write request to the cache and wait for core_ready.
    task cache_write;
        input [31:0] addr;
        input [31:0] wdata;
        input [3:0]  wstrb;
        begin
            @(negedge clk);
            core_valid <= 1;
            core_addr  <= addr;
            core_wdata <= wdata;
            core_wstrb <= wstrb;
            @(posedge clk);
            while (!core_ready) @(posedge clk);
            @(negedge clk);
            core_valid <= 0;
            @(posedge clk);
        end
    endtask

    // Check an expected value and print pass/fail
    task check;
        input [31:0] got;
        input [31:0] expected;
        input [127:0] test_name;
        begin
            if (got === expected) begin
                $display("  [PASS] %s: got 0x%08h", test_name, got);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] %s: got 0x%08h, expected 0x%08h", test_name, got, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    reg [31:0] rdata;

    initial begin
        // Waveform dump - view with gtkwave tb_cache.vcd
        $dumpfile("tb_cache.vcd");
        $dumpvars(0, tb_cache);

        pass_count = 0;
        fail_count = 0;

        // Reset
        resetn     = 0;
        core_valid = 0;
        core_addr  = 0;
        core_wdata = 0;
        core_wstrb = 0;
        mem_ready  = 0;
        repeat(4) @(posedge clk);
        resetn = 1;
        repeat(2) @(posedge clk);

        // =====================================================================
        // Test 1: Read miss (cold cache)
        // Address 0x00 has never been accessed. Cache is empty.
        // Expected: cache goes IDLE→COMPARE→FETCH→DONE, fetches from memory.
        // Memory at word 0 was initialized to 0xA0000000.
        // =====================================================================
        $display("\n--- Test 1: Read miss (cold cache) ---");
        $display("  Accessing 0x00000000 for the first time");
        cache_read(32'h0000_0000, rdata);
        check(rdata, 32'hA000_0000, "cold read 0x00");

        // =====================================================================
        // Test 2: Read hit
        // Address 0x00 is now cached. Second access should hit immediately.
        // Expected: cache goes IDLE→COMPARE→DONE, no memory access.
        // =====================================================================
        $display("\n--- Test 2: Read hit ---");
        $display("  Accessing 0x00000000 again (should hit, no memory access)");
        cache_read(32'h0000_0000, rdata);
        check(rdata, 32'hA000_0000, "read hit 0x00");

        // =====================================================================
        // Test 3: Write hit
        // Address 0x00 is cached. Write to it - should update cache and set
        // dirty bit. No memory write yet (write-back policy).
        // =====================================================================
        $display("\n--- Test 3: Write hit ---");
        $display("  Writing 0xDEADBEEF to 0x00000000 (should hit, no memory write)");
        cache_write(32'h0000_0000, 32'hDEAD_BEEF, 4'b1111);

        // =====================================================================
        // Test 4: Read-back after write
        // Immediately read 0x00 back - should get 0xDEADBEEF from cache,
        // not 0xA0000000 from memory.
        // =====================================================================
        $display("\n--- Test 4: Read-back after write ---");
        cache_read(32'h0000_0000, rdata);
        check(rdata, 32'hDEAD_BEEF, "read-back after write");

        // =====================================================================
        // Test 5: Clean eviction
        // Address 0x10 maps to the same index as 0x00 (both have addr[3:2]=0).
        // Cache line 0 currently holds 0x00 (dirty from Test 3).
        // Wait - this is actually a dirty eviction. Let's first do a clean one.
        // Cache a new address at index 1 (0x04), then evict it cleanly with 0x14.
        // =====================================================================
        $display("\n--- Test 5: Clean eviction ---");
        $display("  First cache 0x00000004 (index 1, clean)");
        cache_read(32'h0000_0004, rdata);
        check(rdata, 32'hA000_0001, "cold read 0x04");

        $display("  Now access 0x00000014 (also index 1) - clean eviction of 0x04");
        cache_read(32'h0000_0014, rdata);
        // Memory at word 5 (0x14 >> 2 = 5) = 0xA0000005
        check(rdata, 32'hA000_0005, "clean eviction fetch 0x14");

        // =====================================================================
        // Test 6: Dirty eviction
        // Line 0 still holds 0x00 with dirty data (0xDEADBEEF from Test 3).
        // Access 0x10 which maps to same index 0.
        // Expected: cache writes back 0xDEADBEEF to addr 0x00, then fetches 0x10.
        // =====================================================================
        $display("\n--- Test 6: Dirty eviction ---");
        $display("  Accessing 0x00000010 (index 0) - should writeback 0x00 first");
        $display("  Expect to see [MEM] WRITE of 0xDEADBEEF then [MEM] READ of 0x10");
        cache_read(32'h0000_0010, rdata);
        // Memory at word 4 (0x10 >> 2 = 4) = 0xA0000004
        check(rdata, 32'hA000_0004, "dirty eviction fetch 0x10");
        // Verify writeback happened - memory[0] should now be 0xDEADBEEF
        check(memory[0], 32'hDEAD_BEEF, "writeback to memory[0]");

        // =====================================================================
        // Test 7: Write miss (write-allocate)
        // Write to 0x20 which is not cached.
        // Expected: fetch 0x20 from memory first, then apply the write.
        // After the write, reading back should return the written value.
        // =====================================================================
        $display("\n--- Test 7: Write miss (write-allocate) ---");
        $display("  Writing 0x12345678 to 0x00000020 (not cached)");
        $display("  Expect [MEM] READ of 0x20 (fetch), then no memory write yet");
        cache_write(32'h0000_0020, 32'h1234_5678, 4'b1111);
        cache_read(32'h0000_0020, rdata);
        check(rdata, 32'h1234_5678, "write-allocate read-back");

        // =====================================================================
        // Test 8: Byte-enable writes
        // Write only the low byte of a cached word using wstrb = 4'b0001.
        // The other three bytes should be unchanged.
        // =====================================================================
        $display("\n--- Test 8: Byte-enable (partial write) ---");
        // First cache 0x30
        cache_read(32'h0000_0030, rdata);
        $display("  Cached 0x30, value = 0x%08h", rdata);
        // Write only the lowest byte with 0xAB
        $display("  Writing 0x000000AB to 0x30 with wstrb=0001 (low byte only)");
        cache_write(32'h0000_0030, 32'h0000_00AB, 4'b0001);
        cache_read(32'h0000_0030, rdata);
        // Original value was 0xA000000C (word index 0x30>>2 = 12 = 0xC)
        // Low byte replaced with 0xAB: expect 0xA00000AB
        check(rdata, 32'hA000_00AB, "byte-enable low byte");

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n=================================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        $display("=================================================\n");

        if (fail_count == 0)
            $display("ALL TESTS PASSED!\n");
        else
            $display("SOME TESTS FAILED - check output above\n");

        $finish;
    end

    // Timeout watchdog - if simulation runs over 10000 cycles something is hung
    initial begin
        #100000;
        $display("[TIMEOUT] Simulation exceeded 100000ns - possible hang in state machine");
        $finish;
    end

endmodule