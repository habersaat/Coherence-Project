`timescale 1ns/1ps

module top (
    input CLK100MHZ,
    input CPU_RESETN,
    output CA, CB, CC, CD, CE, CF, CG,
    output [7:0] AN
);

    // PicoRV32 signals
    wire        mem_valid;
    wire        mem_instr;
    reg         mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    reg  [31:0] mem_rdata;

    // Instantiate PicoRV32
    picorv32 #(
        .STACKADDR(32'h0000_0FFC),
        .PROGADDR_RESET(32'h0000_0000),
        .ENABLE_MUL(1),
        .ENABLE_DIV(1)
    ) cpu (
        .clk       (CLK100MHZ),
        .resetn    (CPU_RESETN),
        .trap      (),
        .mem_valid (mem_valid),
        .mem_instr (mem_instr),
        .mem_ready (mem_ready),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_wstrb (mem_wstrb),
        .mem_rdata (mem_rdata),
        .mem_la_read  (),
        .mem_la_write (),
        .mem_la_addr  (),
        .mem_la_wdata (),
        .mem_la_wstrb (),
        .pcpi_valid (),
        .pcpi_insn  (),
        .pcpi_rs1   (),
        .pcpi_rs2   (),
        .pcpi_wr    (1'b0),
        .pcpi_rd    (32'b0),
        .pcpi_wait  (1'b0),
        .pcpi_ready (1'b0),
        .irq        (32'b0),
        .eoi        ()
    );

    // 4KB memory - distributed RAM supports async reads for display
    reg [31:0] memory [0:1023];
    initial begin
        memory[0] = 32'hff010113;
        memory[1] = 32'h00812623;
        memory[2] = 32'h01010413;
        memory[3] = 32'h10002783;
        memory[4] = 32'h00178713;
        memory[5] = 32'h10e02023;
        memory[6] = 32'hff5ff06f;
    end

    // Memory response
    always @(posedge CLK100MHZ) begin
        mem_ready <= 0;
        if (mem_valid && !mem_ready) begin
            if (mem_addr < 32'h0000_1000) begin
                mem_ready <= 1;
                mem_rdata <= memory[mem_addr >> 2];
                if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
                if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
                if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
                if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
            end
        end
    end

    // Count CPU writes to 0x100
    reg [31:0] write_count;
    always @(posedge CLK100MHZ) begin
        if (!CPU_RESETN)
            write_count <= 0;
        else if (mem_valid && mem_ready && mem_wstrb != 0 && mem_addr == 32'h100)
            write_count <= write_count + 1;
    end

    // Display bits [26:11] - updates roughly every 200ms at ~10M writes/sec
    wire [15:0] num = write_count[26:11];

    // Refresh counter for multiplexing
    reg [16:0] refresh;
    always @(posedge CLK100MHZ) refresh <= refresh + 1;

    reg [3:0] digit;
    reg [7:0] AN_reg;
    always @(*) begin
        case (refresh[16:15])
            2'b00: begin AN_reg = 8'b11111110; digit = num[3:0];   end
            2'b01: begin AN_reg = 8'b11111101; digit = num[7:4];   end
            2'b10: begin AN_reg = 8'b11111011; digit = num[11:8];  end
            2'b11: begin AN_reg = 8'b11110111; digit = num[15:12]; end
        endcase
    end
    assign AN = AN_reg;

    reg [6:0] seg_reg;
    always @(*) begin
        case (digit)
            4'd0: seg_reg = 7'b0000001;
            4'd1: seg_reg = 7'b1001111;
            4'd2: seg_reg = 7'b0010010;
            4'd3: seg_reg = 7'b0000110;
            4'd4: seg_reg = 7'b1001100;
            4'd5: seg_reg = 7'b0100100;
            4'd6: seg_reg = 7'b0100000;
            4'd7: seg_reg = 7'b0001111;
            4'd8: seg_reg = 7'b0000000;
            4'd9: seg_reg = 7'b0000100;
            4'ha: seg_reg = 7'b0001000;
            4'hb: seg_reg = 7'b1100000;
            4'hc: seg_reg = 7'b0110001;
            4'hd: seg_reg = 7'b1000010;
            4'he: seg_reg = 7'b0110000;
            4'hf: seg_reg = 7'b0111000;
            default: seg_reg = 7'b1111111;
        endcase
    end
    assign {CA, CB, CC, CD, CE, CF, CG} = seg_reg;

endmodule