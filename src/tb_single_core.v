`timescale 1ns/1ps

module tb_single_core;

    reg clk;
    reg resetn;

    wire        mem_valid;
    wire        mem_instr;
    reg         mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    reg  [31:0] mem_rdata;

    picorv32 #(
        .STACKADDR(32'h0000_2000),
        .PROGADDR_RESET(32'h0000_0000),
        .ENABLE_MUL(1),
        .ENABLE_DIV(1)
    ) cpu (
        .clk       (clk),
        .resetn    (resetn),
        .mem_valid (mem_valid),
        .mem_instr (mem_instr),
        .mem_ready (mem_ready),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_wstrb (mem_wstrb),
        .mem_rdata (mem_rdata)
    );

    reg [31:0] memory [0:32767];

    integer i;
    initial begin
        for (i = 0; i < 32768; i = i + 1)
            memory[i] = 32'h0;
        $readmemh("test_words.hex", memory);
    end

    always @(posedge clk) begin
        mem_ready <= 0;
        if (mem_valid && !mem_ready) begin
            if (mem_addr < 32'h0001_0000) begin
                mem_ready <= 1;
                mem_rdata <= memory[mem_addr >> 2];
                if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
                if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
                if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
                if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
            end
        end
    end

    // Print counter value every 1000 cycles so we can see progress
    always @(posedge clk) begin
        if (mem_valid && mem_ready && mem_wstrb != 0 && mem_addr == 32'h1024)
            $display("shared = %0d", mem_wdata);
    end

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        resetn = 0;
        repeat(100) @(posedge clk);
        resetn = 1;
    end

    initial begin
        $dumpfile("tb_single_core.vcd");
        $dumpvars(0, tb_single_core);
        repeat(100000) @(posedge clk);
        $finish;
    end

endmodule