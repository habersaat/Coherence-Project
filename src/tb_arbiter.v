// =============================================================================
// tb_arbiter.v - Directed testbench for round-robin arbiter
//
// Resets between each test to get clean priority state (priority=0).
//
// Tests covered:
//   1. Single requester         - only core 2 requests, should get grant
//   2. Simultaneous requests    - cores 1,2,3 all request, verify round-robin
//   3. All four cores           - all request, verify full round-robin cycle
//   4. Late arrival             - core joins mid-sequence
// =============================================================================

`timescale 1ns/1ps

module tb_arbiter;

    reg        clk;
    reg        resetn;
    reg  [3:0] req;
    wire [3:0] grant;
    wire       grant_valid;

    initial clk = 0;
    always #5 clk = ~clk;

    arbiter dut (
        .clk         (clk),
        .resetn      (resetn),
        .req         (req),
        .grant       (grant),
        .grant_valid (grant_valid)
    );

    integer pass_count;
    integer fail_count;

    task check_grant;
        input [3:0] expected_grant;
        input [63:0] test_name;
        begin
            if (grant === expected_grant) begin
                $display("  [PASS] %s: grant=0b%04b", test_name, grant);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] %s: got 0b%04b, expected 0b%04b",
                         test_name, grant, expected_grant);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task wait_for_grant;
        integer timeout;
        begin
            timeout = 0;
            while (!grant_valid && timeout < 20) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 20)
                $display("  [TIMEOUT] waited too long for grant");
        end
    endtask

    task release_bus;
        begin
            req = req & ~grant;
            @(posedge clk);
            @(posedge clk);
        end
    endtask

    // Full reset - clears grant and priority back to 0
    task do_reset;
        begin
            req    = 4'b0000;
            resetn = 0;
            repeat(4) @(posedge clk);
            resetn = 1;
            repeat(2) @(posedge clk);
        end
    endtask

    initial begin
        $dumpfile("tb_arbiter.vcd");
        $dumpvars(0, tb_arbiter);

        pass_count = 0;
        fail_count = 0;

        // =====================================================================
        // Test 1: Single requester - core 2 only
        // Priority=0 after reset. Core 2 is only requester.
        // Expected: core 2 gets grant
        // =====================================================================
        $display("\n--- Test 1: Single requester (core 2) ---");
        do_reset;
        req = 4'b0100;
        wait_for_grant;
        @(posedge clk);
        check_grant(4'b0100, "single req core 2");
        release_bus;

        // =====================================================================
        // Test 2: Simultaneous requests from cores 1, 2, 3
        // Priority=0 after reset. Core 0 not requesting.
        // Expected order: core 1, core 2, core 3
        // =====================================================================
        $display("\n--- Test 2: Simultaneous requests (cores 1,2,3) ---");
        do_reset;
        req = 4'b1110;
        wait_for_grant;
        @(posedge clk);
        check_grant(4'b0010, "simultaneous: core 1 first");
        release_bus;

        wait_for_grant;
        @(posedge clk);
        check_grant(4'b0100, "simultaneous: core 2 second");
        release_bus;

        wait_for_grant;
        @(posedge clk);
        check_grant(4'b1000, "simultaneous: core 3 third");
        release_bus;

        // =====================================================================
        // Test 3: All four cores request simultaneously
        // Priority=0 after reset.
        // Expected order: 0, 1, 2, 3
        // =====================================================================
        $display("\n--- Test 3: All four cores request ---");
        do_reset;
        req = 4'b1111;
        wait_for_grant;
        @(posedge clk);
        check_grant(4'b0001, "all four: core 0 first");
        release_bus;

        wait_for_grant;
        @(posedge clk);
        check_grant(4'b0010, "all four: core 1 second");
        release_bus;

        wait_for_grant;
        @(posedge clk);
        check_grant(4'b0100, "all four: core 2 third");
        release_bus;

        wait_for_grant;
        @(posedge clk);
        check_grant(4'b1000, "all four: core 3 fourth");
        release_bus;

        // =====================================================================
        // Test 4: Late arrival
        // Priority=0 after reset. Core 0 requests alone, then core 3 joins.
        // Expected: core 0 wins first, then core 3
        // =====================================================================
        $display("\n--- Test 4: Late arrival ---");
        do_reset;
        req = 4'b0001;
        wait_for_grant;
        @(posedge clk);
        check_grant(4'b0001, "late arrival: core 0 first");
        req = req | 4'b1000;  // core 3 joins while core 0 has bus
        release_bus;

        wait_for_grant;
        @(posedge clk);
        check_grant(4'b1000, "late arrival: core 3 next");
        release_bus;

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

    initial begin
        #10000;
        $display("[TIMEOUT] simulation hung");
        $finish;
    end

endmodule