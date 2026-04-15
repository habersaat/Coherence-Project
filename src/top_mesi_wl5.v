`timescale 1ns/1ps

module top_mesi_wl5 (
    input         CLK100MHZ,
    input         CPU_RESETN,  // active-low reset (tied to board reset button)
    input         BTNC,        // center button: cycle performance counter display
    output [4:0]  LED,         // one-hot counter select: 0=BusRd 1=BusRdX 2=BusUpgr 3=Interv 4=Inval
    output        CA, CB, CC, CD, CE, CF, CG,
    output [7:0]  AN
);

    // -------------------------------------------------------------------------
    // Bus transaction type encoding (matches cache_mesi.v localparams)
    // -------------------------------------------------------------------------
    localparam BUS_RD   = 2'd0;
    localparam BUS_RDX  = 2'd1;
    localparam BUS_UPGR = 2'd2;

    // -------------------------------------------------------------------------
    // PicoRV32 memory interface signals - one set per core.
    // PicoRV32's mem_* ports connect directly to cache_mesi's core_* ports.
    // mem_instr is unused by the cache (no instruction/data distinction).
    // -------------------------------------------------------------------------
    wire        c0_cpu_valid, c1_cpu_valid, c2_cpu_valid, c3_cpu_valid;
    wire        c0_cpu_instr, c1_cpu_instr, c2_cpu_instr, c3_cpu_instr;
    wire        c0_cpu_ready, c1_cpu_ready, c2_cpu_ready, c3_cpu_ready;
    wire [31:0] c0_cpu_addr,  c1_cpu_addr,  c2_cpu_addr,  c3_cpu_addr;
    wire [31:0] c0_cpu_wdata, c1_cpu_wdata, c2_cpu_wdata, c3_cpu_wdata;
    wire [3:0]  c0_cpu_wstrb, c1_cpu_wstrb, c2_cpu_wstrb, c3_cpu_wstrb;
    wire [31:0] c0_cpu_rdata, c1_cpu_rdata, c2_cpu_rdata, c3_cpu_rdata;

    // -------------------------------------------------------------------------
    // Direct memory-facing signals (per core, writeback path only).
    // cache_mesi only issues wstrb=1111 writes on this interface (WRITEBACK
    // and MI states). All read data comes back through bus_resp_data instead.
    // mem_rdata is tied to 0 - caches never read on this path in MSI.
    // -------------------------------------------------------------------------
    wire        c0_mem_valid, c1_mem_valid, c2_mem_valid, c3_mem_valid;
    reg         c0_mem_ready, c1_mem_ready, c2_mem_ready, c3_mem_ready;
    wire [31:0] c0_mem_addr,  c1_mem_addr,  c2_mem_addr,  c3_mem_addr;
    wire [31:0] c0_mem_wdata, c1_mem_wdata, c2_mem_wdata, c3_mem_wdata;
    wire [3:0]  c0_mem_wstrb, c1_mem_wstrb, c2_mem_wstrb, c3_mem_wstrb;

    // -------------------------------------------------------------------------
    // Bus request outputs from each cache controller
    // -------------------------------------------------------------------------
    wire        c0_bus_req,   c1_bus_req,   c2_bus_req,   c3_bus_req;
    wire        c0_bus_req_valid, c1_bus_req_valid;
    wire        c2_bus_req_valid, c3_bus_req_valid;
    wire [1:0]  c0_bus_req_type,  c1_bus_req_type;
    wire [1:0]  c2_bus_req_type,  c3_bus_req_type;
    wire [31:0] c0_bus_req_addr,  c1_bus_req_addr;
    wire [31:0] c2_bus_req_addr,  c3_bus_req_addr;
    wire [1:0]  c0_bus_req_core,  c1_bus_req_core;
    wire [1:0]  c2_bus_req_core,  c3_bus_req_core;
    wire        c0_snoop_busy, c1_snoop_busy, c2_snoop_busy, c3_snoop_busy;

    // -------------------------------------------------------------------------
    // Round-robin bus arbiter
    // -------------------------------------------------------------------------
    wire [3:0] req = {c3_bus_req, c2_bus_req, c1_bus_req, c0_bus_req};
    wire [3:0] grant;
    wire       grant_valid;

    arbiter arb (
        .clk     (CLK100MHZ),
        .resetn  (CPU_RESETN),
        .req     (req),
        .grant   (grant),
        .grant_valid (grant_valid)
    );

    // -------------------------------------------------------------------------
    // Shared snooping bus.
    // Muxed from the currently granted core. When no core has the bus,
    // all signals default to 0 (snoop_valid=0, no reactions fire).
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

    // OR of all snoop_busy outputs. snoop_busy is combinational in cache_mesi
    // so this wire goes high the same cycle an M-state match is detected,
    // giving memory the correct one-cycle warning before it would respond.
    wire bus_snoop_busy = c0_snoop_busy | c1_snoop_busy |
                          c2_snoop_busy | c3_snoop_busy;

    // -------------------------------------------------------------------------
    // Shared memory - 4KB (1024 x 32-bit words)
    //
    // Vivado can infer this as BRAM even with the initial block: the
    // synthesis tool extracts the initial values as BRAM init parameters.
    // This avoids the LUT explosion that affects much larger memories.
    //
    // Program: tight shared-counter increment loop, same as top_fpga.v.
    // All four cores boot from address 0 and run this same code.
    //
    //   Disassembly:
    //   0x00: ff010113   addi  sp, sp, -16
    //   0x04: 00812623   sw    ra, 12(sp)
    //   0x08: 01010413   addi  s0, sp, 16
    //   0x0C: 10002783   lw    a5, 256(zero)   <- read shared (0x100)
    //   0x10: 00178713   addi  a4, a5, 1
    //   0x14: 10e02023   sw    a4, 256(zero)   <- write shared (0x100)
    //   0x18: ff5ff06f   jal   zero, -12       <- loop back to lw
    //
    // -------------------------------------------------------------------------
    reg [31:0] memory [0:1023];
    initial begin
        // Workload 5: Stencil - Neighbor Update
        // Each core reads its own element and right neighbor's element
        // (circular), adds them, writes back. Cross-core sharing (BusRd
        // on neighbor's M line) + write invalidation (BusRdX on own line).
        // ID assignment header (0x000-0x020):
        memory[ 0] = 32'h20000393;  // addi t2, zero, 0x200
        memory[ 1] = 32'h0003a403;  // lw   s0, 0(t2)          [ID retry]
        memory[ 2] = 32'h00140e13;  // addi t3, s0, 1
        memory[ 3] = 32'h01c3a023;  // sw   t3, 0(t2)
        memory[ 4] = 32'h0003ae83;  // lw   t4, 0(t2)
        memory[ 5] = 32'hffce98e3;  // bne  t4, t3, -16
        memory[ 6] = 32'h00441493;  // slli s1, s0, 4
        memory[ 7] = 32'h30000393;  // addi t2, zero, 0x300
        memory[ 8] = 32'h007484b3;  // add  s1, s1, t2
        // Stencil loop (0x024):
        memory[ 9] = 32'h0004a283;  // lw   t0, 0(s1)          [loop]
        memory[10] = 32'h00140e13;  // addi t3, s0, 1
        memory[11] = 32'h00400e93;  // addi t4, zero, 4
        memory[12] = 32'h01de1463;  // bne  t3, t4, +8
        memory[13] = 32'h00000e13;  // addi t3, zero, 0        (wrap)
        memory[14] = 32'h004e1e93;  // slli t4, t3, 4
        memory[15] = 32'h30000f13;  // addi t5, zero, 0x300
        memory[16] = 32'h01ee8eb3;  // add  t4, t4, t5
        memory[17] = 32'h000ea303;  // lw   t1, 0(t4)
        memory[18] = 32'h006282b3;  // add  t0, t0, t1
        memory[19] = 32'h0054a023;  // sw   t0, 0(s1)
        memory[20] = 32'hfd5ff06f;  // jal  zero, -44
        // Shared array: 16 words at 0x300 (word index 192-207), values 1..16
        memory[192] = 32'd1;  memory[193] = 32'd2;  memory[194] = 32'd3;
        memory[195] = 32'd4;  memory[196] = 32'd5;  memory[197] = 32'd6;
        memory[198] = 32'd7;  memory[199] = 32'd8;  memory[200] = 32'd9;
        memory[201] = 32'd10; memory[202] = 32'd11; memory[203] = 32'd12;
        memory[204] = 32'd13; memory[205] = 32'd14; memory[206] = 32'd15;
        memory[207] = 32'd16;
    end

    reg        bus_resp_valid;
    reg [31:0] bus_resp_data;
    reg        bus_trans_pending;
    reg [31:0] bus_trans_addr;

    // Writeback path: accept dirty-line writes from any core.
    // Priority-encoded c0>c1>c2>c3. Guard on !c*_mem_ready prevents
    // double-write: cache holds mem_valid for one extra cycle after mem_ready.
    always @(posedge CLK100MHZ) begin
        c0_mem_ready <= 0; c1_mem_ready <= 0;
        c2_mem_ready <= 0; c3_mem_ready <= 0;
        if (c0_mem_valid && c0_mem_wstrb == 4'b1111 && !c0_mem_ready) begin
            memory[c0_mem_addr >> 2] <= c0_mem_wdata;
            c0_mem_ready <= 1;
        end else if (c1_mem_valid && c1_mem_wstrb == 4'b1111 && !c1_mem_ready) begin
            memory[c1_mem_addr >> 2] <= c1_mem_wdata;
            c1_mem_ready <= 1;
        end else if (c2_mem_valid && c2_mem_wstrb == 4'b1111 && !c2_mem_ready) begin
            memory[c2_mem_addr >> 2] <= c2_mem_wdata;
            c2_mem_ready <= 1;
        end else if (c3_mem_valid && c3_mem_wstrb == 4'b1111 && !c3_mem_ready) begin
            memory[c3_mem_addr >> 2] <= c3_mem_wdata;
            c3_mem_ready <= 1;
        end
    end

    // Bus read response: two-stage pipeline.
    // Stage 1 (this cycle): latch that a bus transaction appeared.
    // Stage 2 (next cycle): respond only if no snoop reactions are in flight.
    //
    // The one-cycle gap is essential: snoop_busy is combinational in cache_mesi
    // and goes high in the same cycle the transaction appears. If memory
    // responded in cycle N (same cycle as transaction), it would fire before
    // the M-state owner's writeback, serving stale data. The latch gives the
    // M-state owner one full clock edge to assert snoop_busy, after which
    // memory only fires once bus_snoop_busy has cleared (writeback complete).
    //
    // BUS_UPGR is included so SM state receives its ack pulse to complete.
    // The cache SM state ignores bus_resp_data for upgrades.
    always @(posedge CLK100MHZ) begin
        bus_trans_pending <= snoop_valid;
        bus_trans_addr    <= snoop_addr;
        bus_resp_valid    <= 0;
        bus_resp_data     <= 0;
        i`timescale 1ns/1ps

module top_mesi_wl4 (
    input         CLK100MHZ,
    input         CPU_RESETN,  // active-low reset (tied to board reset button)
    input         BTNC,        // center button: cycle performance counter display
    output [4:0]  LED,         // one-hot counter select: 0=BusRd 1=BusRdX 2=BusUpgr 3=Interv 4=Inval
    output        CA, CB, CC, CD, CE, CF, CG,
    output [7:0]  AN
);

    // -------------------------------------------------------------------------
    // Bus transaction type encoding (matches cache_mesi.v localparams)
    // -------------------------------------------------------------------------
    localparam BUS_RD   = 2'd0;
    localparam BUS_RDX  = 2'd1;
    localparam BUS_UPGR = 2'd2;

    // -------------------------------------------------------------------------
    // PicoRV32 memory interface signals - one set per core.
    // PicoRV32's mem_* ports connect directly to cache_mesi's core_* ports.
    // mem_instr is unused by the cache (no instruction/data distinction).
    // -------------------------------------------------------------------------
    wire        c0_cpu_valid, c1_cpu_valid, c2_cpu_valid, c3_cpu_valid;
    wire        c0_cpu_instr, c1_cpu_instr, c2_cpu_instr, c3_cpu_instr;
    wire        c0_cpu_ready, c1_cpu_ready, c2_cpu_ready, c3_cpu_ready;
    wire [31:0] c0_cpu_addr,  c1_cpu_addr,  c2_cpu_addr,  c3_cpu_addr;
    wire [31:0] c0_cpu_wdata, c1_cpu_wdata, c2_cpu_wdata, c3_cpu_wdata;
    wire [3:0]  c0_cpu_wstrb, c1_cpu_wstrb, c2_cpu_wstrb, c3_cpu_wstrb;
    wire [31:0] c0_cpu_rdata, c1_cpu_rdata, c2_cpu_rdata, c3_cpu_rdata;

    // -------------------------------------------------------------------------
    // Direct memory-facing signals (per core, writeback path only).
    // cache_mesi only issues wstrb=1111 writes on this interface (WRITEBACK
    // and MI states). All read data comes back through bus_resp_data instead.
    // mem_rdata is tied to 0 - caches never read on this path in MSI.
    // -------------------------------------------------------------------------
    wire        c0_mem_valid, c1_mem_valid, c2_mem_valid, c3_mem_valid;
    reg         c0_mem_ready, c1_mem_ready, c2_mem_ready, c3_mem_ready;
    wire [31:0] c0_mem_addr,  c1_mem_addr,  c2_mem_addr,  c3_mem_addr;
    wire [31:0] c0_mem_wdata, c1_mem_wdata, c2_mem_wdata, c3_mem_wdata;
    wire [3:0]  c0_mem_wstrb, c1_mem_wstrb, c2_mem_wstrb, c3_mem_wstrb;

    // -------------------------------------------------------------------------
    // Bus request outputs from each cache controller
f (bus_trans_pending && !bus_snoop_busy) begin
            bus_resp_valid <= 1;
            bus_resp_data  <= memory[bus_trans_addr >> 2];
        end
    end

    // -------------------------------------------------------------------------
    // cache_mesi instantiations - one per core
    // -------------------------------------------------------------------------
    // MESI additions: snoop_shared from each cache ORed together.
    // bus_resp_excl: no other cache has an S/E copy -> requester gets E state.
    // -------------------------------------------------------------------------
    wire c0_snoop_shared, c1_snoop_shared, c2_snoop_shared, c3_snoop_shared;
    wire bus_resp_excl = !(c0_snoop_shared | c1_snoop_shared
                         | c2_snoop_shared | c3_snoop_shared);

    cache_mesi dut0 (
        .clk(CLK100MHZ), .resetn(CPU_RESETN), .core_id(2'd0),
        .core_valid(c0_cpu_valid), .core_ready(c0_cpu_ready),
        .core_addr(c0_cpu_addr),   .core_wdata(c0_cpu_wdata),
        .core_wstrb(c0_cpu_wstrb), .core_rdata(c0_cpu_rdata),
        .mem_valid(c0_mem_valid),  .mem_ready(c0_mem_ready),
        .mem_addr(c0_mem_addr),    .mem_wdata(c0_mem_wdata),
        .mem_wstrb(c0_mem_wstrb),  .mem_rdata(32'd0),
        .bus_req(c0_bus_req),      .bus_grant(grant[0]),
        .bus_req_valid(c0_bus_req_valid), .bus_req_type(c0_bus_req_type),
        .bus_req_addr(c0_bus_req_addr),   .bus_req_core(c0_bus_req_core),
        .snoop_valid(snoop_valid),  .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),    .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c0_snoop_busy), .snoop_shared(c0_snoop_shared),
        .bus_resp_excl(bus_resp_excl)
    );
    cache_mesi dut1 (
        .clk(CLK100MHZ), .resetn(CPU_RESETN), .core_id(2'd1),
        .core_valid(c1_cpu_valid), .core_ready(c1_cpu_ready),
        .core_addr(c1_cpu_addr),   .core_wdata(c1_cpu_wdata),
        .core_wstrb(c1_cpu_wstrb), .core_rdata(c1_cpu_rdata),
        .mem_valid(c1_mem_valid),  .mem_ready(c1_mem_ready),
        .mem_addr(c1_mem_addr),    .mem_wdata(c1_mem_wdata),
        .mem_wstrb(c1_mem_wstrb),  .mem_rdata(32'd0),
        .bus_req(c1_bus_req),      .bus_grant(grant[1]),
        .bus_req_valid(c1_bus_req_valid), .bus_req_type(c1_bus_req_type),
        .bus_req_addr(c1_bus_req_addr),   .bus_req_core(c1_bus_req_core),
        .snoop_valid(snoop_valid),  .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),    .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c1_snoop_busy), .snoop_shared(c1_snoop_shared),
        .bus_resp_excl(bus_resp_excl)
    );
    cache_mesi dut2 (
        .clk(CLK100MHZ), .resetn(CPU_RESETN), .core_id(2'd2),
        .core_valid(c2_cpu_valid), .core_ready(c2_cpu_ready),
        .core_addr(c2_cpu_addr),   .core_wdata(c2_cpu_wdata),
        .core_wstrb(c2_cpu_wstrb), .core_rdata(c2_cpu_rdata),
        .mem_valid(c2_mem_valid),  .mem_ready(c2_mem_ready),
        .mem_addr(c2_mem_addr),    .mem_wdata(c2_mem_wdata),
        .mem_wstrb(c2_mem_wstrb),  .mem_rdata(32'd0),
        .bus_req(c2_bus_req),      .bus_grant(grant[2]),
        .bus_req_valid(c2_bus_req_valid), .bus_req_type(c2_bus_req_type),
        .bus_req_addr(c2_bus_req_addr),   .bus_req_core(c2_bus_req_core),
        .snoop_valid(snoop_valid),  .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),    .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c2_snoop_busy), .snoop_shared(c2_snoop_shared),
        .bus_resp_excl(bus_resp_excl)
    );
    cache_mesi dut3 (
        .clk(CLK100MHZ), .resetn(CPU_RESETN), .core_id(2'd3),
        .core_valid(c3_cpu_valid), .core_ready(c3_cpu_ready),
        .core_addr(c3_cpu_addr),   .core_wdata(c3_cpu_wdata),
        .core_wstrb(c3_cpu_wstrb), .core_rdata(c3_cpu_rdata),
        .mem_valid(c3_mem_valid),  .mem_ready(c3_mem_ready),
        .mem_addr(c3_mem_addr),    .mem_wdata(c3_mem_wdata),
        .mem_wstrb(c3_mem_wstrb),  .mem_rdata(32'd0),
        .bus_req(c3_bus_req),      .bus_grant(grant[3]),
        .bus_req_valid(c3_bus_req_valid), .bus_req_type(c3_bus_req_type),
        .bus_req_addr(c3_bus_req_addr),   .bus_req_core(c3_bus_req_core),
        .snoop_valid(snoop_valid),  .snoop_type(snoop_type),
        .snoop_addr(snoop_addr),    .snoop_core(snoop_core),
        .bus_resp_data(bus_resp_data), .bus_resp_valid(bus_resp_valid),
        .snoop_busy(c3_snoop_busy), .snoop_shared(c3_snoop_shared),
        .bus_resp_excl(bus_resp_excl)
    );

    // -------------------------------------------------------------------------
    // PicoRV32 instantiations - one per core
    //
    // All cores start at address 0 (PROGADDR_RESET) running the same program.
    // STACKADDR=0xFFC: all cores share the stack area. This is intentional -
    // the four cores immediately fight over 0xFF8 on startup (sw ra, 12(sp)),
    // generating coherence traffic from the first instruction executed.
    // MUL/DIV disabled: not needed for the counter loop, saves ~1000 LUTs/core.
    // -------------------------------------------------------------------------
    picorv32 #(
        .STACKADDR    (32'h0000_0FFC),
        .PROGADDR_RESET(32'h0000_0000),
        .ENABLE_MUL   (0),
        .ENABLE_DIV   (0)
    ) cpu0 (
        .clk       (CLK100MHZ),    .resetn    (CPU_RESETN),
        .trap      (),
        .mem_valid (c0_cpu_valid), .mem_instr (c0_cpu_instr),
        .mem_ready (c0_cpu_ready), .mem_addr  (c0_cpu_addr),
        .mem_wdata (c0_cpu_wdata), .mem_wstrb (c0_cpu_wstrb),
        .mem_rdata (c0_cpu_rdata),
        .mem_la_read(), .mem_la_write(), .mem_la_addr(), .mem_la_wdata(), .mem_la_wstrb(),
        .pcpi_valid(), .pcpi_insn(), .pcpi_rs1(), .pcpi_rs2(),
        .pcpi_wr(1'b0), .pcpi_rd(32'b0), .pcpi_wait(1'b0), .pcpi_ready(1'b0),
        .irq(32'b0), .eoi()
    );
    picorv32 #(
        .STACKADDR    (32'h0000_0FFC),
        .PROGADDR_RESET(32'h0000_0000),
        .ENABLE_MUL   (0),
        .ENABLE_DIV   (0)
    ) cpu1 (
        .clk       (CLK100MHZ),    .resetn    (CPU_RESETN),
        .trap      (),
        .mem_valid (c1_cpu_valid), .mem_instr (c1_cpu_instr),
        .mem_ready (c1_cpu_ready), .mem_addr  (c1_cpu_addr),
        .mem_wdata (c1_cpu_wdata), .mem_wstrb (c1_cpu_wstrb),
        .mem_rdata (c1_cpu_rdata),
        .mem_la_read(), .mem_la_write(), .mem_la_addr(), .mem_la_wdata(), .mem_la_wstrb(),
        .pcpi_valid(), .pcpi_insn(), .pcpi_rs1(), .pcpi_rs2(),
        .pcpi_wr(1'b0), .pcpi_rd(32'b0), .pcpi_wait(1'b0), .pcpi_ready(1'b0),
        .irq(32'b0), .eoi()
    );
    picorv32 #(
        .STACKADDR    (32'h0000_0FFC),
        .PROGADDR_RESET(32'h0000_0000),
        .ENABLE_MUL   (0),
        .ENABLE_DIV   (0)
    ) cpu2 (
        .clk       (CLK100MHZ),    .resetn    (CPU_RESETN),
        .trap      (),
        .mem_valid (c2_cpu_valid), .mem_instr (c2_cpu_instr),
        .mem_ready (c2_cpu_ready), .mem_addr  (c2_cpu_addr),
        .mem_wdata (c2_cpu_wdata), .mem_wstrb (c2_cpu_wstrb),
        .mem_rdata (c2_cpu_rdata),
        .mem_la_read(), .mem_la_write(), .mem_la_addr(), .mem_la_wdata(), .mem_la_wstrb(),
        .pcpi_valid(), .pcpi_insn(), .pcpi_rs1(), .pcpi_rs2(),
        .pcpi_wr(1'b0), .pcpi_rd(32'b0), .pcpi_wait(1'b0), .pcpi_ready(1'b0),
        .irq(32'b0), .eoi()
    );
    picorv32 #(
        .STACKADDR    (32'h0000_0FFC),
        .PROGADDR_RESET(32'h0000_0000),
        .ENABLE_MUL   (0),
        .ENABLE_DIV   (0)
    ) cpu3 (
        .clk       (CLK100MHZ),    .resetn    (CPU_RESETN),
        .trap      (),
        .mem_valid (c3_cpu_valid), .mem_instr (c3_cpu_instr),
        .mem_ready (c3_cpu_ready), .mem_addr  (c3_cpu_addr),
        .mem_wdata (c3_cpu_wdata), .mem_wstrb (c3_cpu_wstrb),
        .mem_rdata (c3_cpu_rdata),
        .mem_la_read(), .mem_la_write(), .mem_la_addr(), .mem_la_wdata(), .mem_la_wstrb(),
        .pcpi_valid(), .pcpi_insn(), .pcpi_rs1(), .pcpi_rs2(),
        .pcpi_wr(1'b0), .pcpi_rd(32'b0), .pcpi_wait(1'b0), .pcpi_ready(1'b0),
        .irq(32'b0), .eoi()
    );

    // -------------------------------------------------------------------------
    // Performance counters
    //
    // Count once per bus transaction using the rising edge of snoop_valid.
    // snoop_valid stays high for the entire duration of a transaction (from
    // when the cache drives bus_req_valid until bus_resp_valid is received).
    // Using the rising edge avoids counting the same transaction multiple times.
    //
    // Counter definitions:
    //   cnt_busrd:  BusRd transactions (I->S, read misses needing shared copy)
    //   cnt_busrdx: BusRdX transactions (I->M, write misses needing exclusive)
    //   cnt_busupgr:BusUpgr transactions (S->M, upgrades with data already held)
    //   cnt_interv: Transactions where an M-state owner had to write back first.
    //               Detected by bus_snoop_busy being high when the transaction
    //               starts (combinational snoop_busy from cache_mesi).
    //   cnt_inval:  Transactions that caused S->I invalidations (BusRdX and
    //               BusUpgr both force other S-state holders to invalidate).
    //               Counts the number of invalidating transactions, not the
    //               number of individual cache lines invalidated.
    // -------------------------------------------------------------------------
    reg snoop_valid_prev;
    always @(posedge CLK100MHZ) snoop_valid_prev <= snoop_valid;
    wire snoop_rising = snoop_valid && !snoop_valid_prev;

    reg [31:0] cnt_busrd, cnt_busrdx, cnt_busupgr, cnt_interv, cnt_inval;
    always @(posedge CLK100MHZ) begin
        if (!CPU_RESETN) begin
            cnt_busrd   <= 0;
            cnt_busrdx  <= 0;
            cnt_busupgr <= 0;
            cnt_interv  <= 0;
            cnt_inval   <= 0;
        end else if (snoop_rising) begin
            // Count by transaction type
            if (snoop_type == BUS_RD)   cnt_busrd   <= cnt_busrd   + 1;
            if (snoop_type == BUS_RDX)  cnt_busrdx  <= cnt_busrdx  + 1;
            if (snoop_type == BUS_UPGR) cnt_busupgr <= cnt_busupgr + 1;
            // Intervention: an M-state owner is asserting snoop_busy right now,
            // meaning memory will stall until the writeback completes
            if (bus_snoop_busy) cnt_interv <= cnt_interv + 1;
            // Invalidation: BusRdX and BusUpgr both force S->I on other caches
            if (snoop_type == BUS_RDX || snoop_type == BUS_UPGR)
                cnt_inval <= cnt_inval + 1;
        end
    end

    // -------------------------------------------------------------------------
    // Button debounce - 20-bit counter gives ~10ms debounce window at 100MHz
    // -------------------------------------------------------------------------
    reg [19:0] btn_debounce;
    reg        btn_stable, btn_prev;

    always @(posedge CLK100MHZ) begin
        if (!CPU_RESETN) begin
            btn_debounce <= 0;
            btn_stable   <= 0;
            btn_prev     <= 0;
        end else begin
            if (BTNC == btn_stable)
                btn_debounce <= 0;
            else begin
                btn_debounce <= btn_debounce + 1;
                if (&btn_debounce) btn_stable <= BTNC;
            end
            btn_prev <= btn_stable;
        end
    end

    // -------------------------------------------------------------------------
    // Long press / short press detection
    //
    // Hold timer counts how long BTNC has been continuously held.
    // Long press threshold: 2 seconds = 200,000,000 cycles.
    //   - Long press (while still held): starts a new 10-second capture run.
    //     Separated from short press so holding never accidentally cycles display.
    // Short press: button released before long press threshold.
    //   - Cycles the display through the 5 counters in LIVE or FROZEN state.
    // -------------------------------------------------------------------------
    reg [27:0] hold_timer;  // 2^28 = 268M > 200M, fits 2-second hold

    always @(posedge CLK100MHZ) begin
        if (!CPU_RESETN)
            hold_timer <= 0;
        else if (btn_stable && hold_timer < 28'd199_999_999)
            hold_timer <= hold_timer + 1;
        else if (!btn_stable)
            hold_timer <= 0;
    end

    wire long_press  = (hold_timer == 28'd199_999_999);             // fires once after 2s hold
    wire btn_release = !btn_stable && btn_prev;                     // falling edge
    wire short_press = btn_release && (hold_timer < 28'd199_999_999); // released before 2s

    // -------------------------------------------------------------------------
    // Capture mode FSM
    //
    // LIVE:      normal operation. Counters run, display shows live scaled values.
    //            Short press cycles display. Long press starts a capture run.
    // CAPTURING: 10-second timed window. Dedicated capture counters accumulate
    //            only events in this window (reset at capture start). Display
    //            shows the capture counters climbing so you can watch it run.
    //            Button ignored during capture.
    // FROZEN:    Capture complete. Display shows the frozen 10-second results.
    //            Short press cycles through all 5 frozen values for recording.
    //            Long press starts a new capture run.
    //
    // Capture duration: 10 seconds = 1,000,000,000 cycles (fits in 30 bits).
    // Using dedicated capture counters means the result is exactly the events
    // in the 10-second window regardless of what happened before - no need to
    // manually reset before each run.
    // -------------------------------------------------------------------------
    localparam CAP_LIVE      = 2'd0;
    localparam CAP_CAPTURING = 2'd1;
    localparam CAP_FROZEN    = 2'd2;

    reg [1:0]  cap_state;
    reg [29:0] cap_timer;  // counts up to 1,000,000,000

    // Dedicated capture counters - only increment during CAP_CAPTURING
    reg [31:0] cap_busrd, cap_busrdx, cap_busupgr, cap_interv, cap_inval;

    // Frozen results - latched simultaneously at end of capture window
    reg [31:0] frz_busrd, frz_busrdx, frz_busupgr, frz_interv, frz_inval;

    always @(posedge CLK100MHZ) begin
        if (!CPU_RESETN) begin
            cap_state   <= CAP_LIVE;
            cap_timer   <= 0;
            cap_busrd   <= 0; cap_busrdx  <= 0; cap_busupgr <= 0;
            cap_interv  <= 0; cap_inval   <= 0;
            frz_busrd   <= 0; frz_busrdx  <= 0; frz_busupgr <= 0;
            frz_interv  <= 0; frz_inval   <= 0;
        end else begin
            case (cap_state)
                CAP_LIVE: begin
                    if (long_press) begin
                        // Reset capture counters and start 10-second window
                        cap_timer   <= 0;
                        cap_busrd   <= 0; cap_busrdx  <= 0; cap_busupgr <= 0;
                        cap_interv  <= 0; cap_inval   <= 0;
                        cap_state   <= CAP_CAPTURING;
                    end
                end
                CAP_CAPTURING: begin
                    // Accumulate events during the capture window
                    if (snoop_rising) begin
                        if (snoop_type == BUS_RD)   cap_busrd   <= cap_busrd   + 1;
                        if (snoop_type == BUS_RDX)  cap_busrdx  <= cap_busrdx  + 1;
                        if (snoop_type == BUS_UPGR) cap_busupgr <= cap_busupgr + 1;
                        if (bus_snoop_busy)          cap_interv  <= cap_interv  + 1;
                        if (snoop_type == BUS_RDX || snoop_type == BUS_UPGR)
                                                     cap_inval   <= cap_inval   + 1;
                    end
                    // Count cycles, freeze at exactly 10 seconds
                    cap_timer <= cap_timer + 1;
                    if (cap_timer == 30'd999_999_999) begin
                        frz_busrd   <= cap_busrd;
                        frz_busrdx  <= cap_busrdx;
                        frz_busupgr <= cap_busupgr;
                        frz_interv  <= cap_interv;
                        frz_inval   <= cap_inval;
                        cap_state   <= CAP_FROZEN;
                    end
                end
                CAP_FROZEN: begin
                    // Long press starts a new capture run
                    if (long_press) begin
                        cap_timer   <= 0;
                        cap_busrd   <= 0; cap_busrdx  <= 0; cap_busupgr <= 0;
                        cap_interv  <= 0; cap_inval   <= 0;
                        cap_state   <= CAP_CAPTURING;
                    end
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Counter select - cycles on short press in LIVE and FROZEN states
    // -------------------------------------------------------------------------
    reg [2:0] cnt_sel;
    always @(posedge CLK100MHZ) begin
        if (!CPU_RESETN)
            cnt_sel <= 0;
        else if (short_press && cap_state != CAP_CAPTURING)
            cnt_sel <= (cnt_sel == 3'd4) ? 3'd0 : cnt_sel + 1;
    end

    // -------------------------------------------------------------------------
    // Display mux
    // LIVE:      show live always-running counters (scaled /1024, live view)
    // CAPTURING: show capture counters climbing during the run
    // FROZEN:    show frozen 10-second results
    // -------------------------------------------------------------------------
    reg [31:0] cnt_display;
    always @(*) begin
        case (cap_state)
            CAP_LIVE: begin
                case (cnt_sel)
                    3'd0: cnt_display = cnt_busrd;
                    3'd1: cnt_display = cnt_busrdx;
                    3'd2: cnt_display = cnt_busupgr;
                    3'd3: cnt_display = cnt_interv;
                    3'd4: cnt_display = cnt_inval;
                    default: cnt_display = 32'd0;
                endcase
            end
            CAP_CAPTURING: begin
                case (cnt_sel)
                    3'd0: cnt_display = cap_busrd;
                    3'd1: cnt_display = cap_busrdx;
                    3'd2: cnt_display = cap_busupgr;
                    3'd3: cnt_display = cap_interv;
                    3'd4: cnt_display = cap_inval;
                    default: cnt_display = 32'd0;
                endcase
            end
            CAP_FROZEN: begin
                case (cnt_sel)
                    3'd0: cnt_display = frz_busrd;
                    3'd1: cnt_display = frz_busrdx;
                    3'd2: cnt_display = frz_busupgr;
                    3'd3: cnt_display = frz_interv;
                    3'd4: cnt_display = frz_inval;
                    default: cnt_display = 32'd0;
                endcase
            end
            default: cnt_display = 32'd0;
        endcase
    end

    // -------------------------------------------------------------------------
    // LED outputs - indicate both which counter is shown and capture state
    //
    // LIVE:      selected LED solid on
    // CAPTURING: all 5 LEDs blink together (2Hz) - indicates run in progress
    // FROZEN:    selected LED blinks slowly (2Hz) - indicates frozen results
    //
    // Blink source: cap_timer[25] in CAPTURING (bit 25 of a 30-bit 100MHz
    // counter toggles at ~3Hz), or a free-running blink counter in FROZEN.
    // -------------------------------------------------------------------------
    reg [25:0] blink_counter;
    always @(posedge CLK100MHZ) blink_counter <= blink_counter + 1;
    wire blink = blink_counter[25];  // ~3Hz blink

    assign LED[0] = (cap_state == CAP_LIVE)   ? (cnt_sel == 3'd0) :
                    (cap_state == CAP_CAPTURING) ? blink :
                    /* FROZEN */                  (cnt_sel == 3'd0) && blink;
    assign LED[1] = (cap_state == CAP_LIVE)   ? (cnt_sel == 3'd1) :
                    (cap_state == CAP_CAPTURING) ? blink :
                    /* FROZEN */                  (cnt_sel == 3'd1) && blink;
    assign LED[2] = (cap_state == CAP_LIVE)   ? (cnt_sel == 3'd2) :
                    (cap_state == CAP_CAPTURING) ? blink :
                    /* FROZEN */                  (cnt_sel == 3'd2) && blink;
    assign LED[3] = (cap_state == CAP_LIVE)   ? (cnt_sel == 3'd3) :
                    (cap_state == CAP_CAPTURING) ? blink :
                    /* FROZEN */                  (cnt_sel == 3'd3) && blink;
    assign LED[4] = (cap_state == CAP_LIVE)   ? (cnt_sel == 3'd4) :
                    (cap_state == CAP_CAPTURING) ? blink :
                    /* FROZEN */                  (cnt_sel == 3'd4) && blink;

    // -------------------------------------------------------------------------
    // Binary to BCD conversion - Double Dabble algorithm
    //
    // Feeds cnt_display >> 10 (divide by 1024, approx /1000) into the BCD
    // converter. The display shows "kilo-events" - the ones digit represents
    // ~1000 actual bus transactions. The display stays live and readable:
    // digits advance at a human-readable pace while clearly showing relative
    // rates between counters after a 30-second run.
    // Note in the paper: displayed value = actual count / 1024.
    //
    // Double Dabble: iterate 32 times (one per input bit).
    //   1. For each 4-bit BCD column >= 5, add 3 (pre-corrects for overflow).
    //   2. Shift entire register left 1, bringing in the next input bit.
    // Purely combinational - Vivado unrolls loops into a LUT tree.
    // -------------------------------------------------------------------------
    wire [31:0] cnt_scaled = cnt_display >> 10;  // divide by 1024 (~x1000)

    reg [39:0] bcd;  // 10 BCD digits x 4 bits each
    integer    db_i, db_j;
    always @(*) begin
        bcd = 40'd0;
        for (db_i = 31; db_i >= 0; db_i = db_i - 1) begin
            for (db_j = 0; db_j <= 9; db_j = db_j + 1) begin
                if (bcd[db_j*4 +: 4] >= 4'd5)
                    bcd[db_j*4 +: 4] = bcd[db_j*4 +: 4] + 4'd3;
            end
            bcd = {bcd[38:0], cnt_scaled[db_i]};
        end
    end

    // Extract the 8 display digits from the BCD result.
    // Digit 0 is the ones place (rightmost), digit 7 is ten-millions place.
    wire [3:0] bcd_digit [0:7];
    assign bcd_digit[0] = bcd[3:0];
    assign bcd_digit[1] = bcd[7:4];
    assign bcd_digit[2] = bcd[11:8];
    assign bcd_digit[3] = bcd[15:12];
    assign bcd_digit[4] = bcd[19:16];
    assign bcd_digit[5] = bcd[23:20];
    assign bcd_digit[6] = bcd[27:24];
    assign bcd_digit[7] = bcd[31:28];

    // -------------------------------------------------------------------------
    // 7-segment display - all 8 digits, decimal
    //
    // Refresh counter: 18 bits at 100MHz gives ~2.6ms full cycle.
    // Bits [17:15] select one of 8 digits (AN[7]=leftmost, AN[0]=rightmost).
    // AN is active-low: pull the selected digit low, all others high.
    //
    // Only digits 0-7 (decimal) appear so the seg_reg case only needs 0-9.
    // -------------------------------------------------------------------------
    reg [17:0] refresh;
    always @(posedge CLK100MHZ) refresh <= refresh + 1;

    reg [3:0] digit;
    reg [7:0] AN_reg;
    always @(*) begin
        case (refresh[17:15])
            3'd0: begin AN_reg = 8'b11111110; digit = bcd_digit[0]; end
            3'd1: begin AN_reg = 8'b11111101; digit = bcd_digit[1]; end
            3'd2: begin AN_reg = 8'b11111011; digit = bcd_digit[2]; end
            3'd3: begin AN_reg = 8'b11110111; digit = bcd_digit[3]; end
            3'd4: begin AN_reg = 8'b11101111; digit = bcd_digit[4]; end
            3'd5: begin AN_reg = 8'b11011111; digit = bcd_digit[5]; end
            3'd6: begin AN_reg = 8'b10111111; digit = bcd_digit[6]; end
            3'd7: begin AN_reg = 8'b01111111; digit = bcd_digit[7]; end
        endcase
    end
    assign AN = AN_reg;

    // 7-segment decoder (Nexys A7 is active-low)
    // Segment order: {CA, CB, CC, CD, CE, CF, CG}
    // Only digits 0-9 are needed - BCD values are always valid decimal digits.
    reg [6:0] seg_reg;
    always @(*) begin
        case (digit)
            4'd0:    seg_reg = 7'b0000001;
            4'd1:    seg_reg = 7'b1001111;
            4'd2:    seg_reg = 7'b0010010;
            4'd3:    seg_reg = 7'b0000110;
            4'd4:    seg_reg = 7'b1001100;
            4'd5:    seg_reg = 7'b0100100;
            4'd6:    seg_reg = 7'b0100000;
            4'd7:    seg_reg = 7'b0001111;
            4'd8:    seg_reg = 7'b0000000;
            4'd9:    seg_reg = 7'b0000100;
            default: seg_reg = 7'b1111111; // blank for any invalid BCD value
        endcase
    end
    assign {CA, CB, CC, CD, CE, CF, CG} = seg_reg;

endmodule