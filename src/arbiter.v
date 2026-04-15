module arbiter (
    input            clk,
    input            resetn,
    input      [3:0] req,
    output reg [3:0] grant,      // one-hot
    output           grant_valid // is anyone currently granted?
);

    assign grant_valid = |grant;

    // priority is which core gets checked first next arbitration round
    reg [1:0] priority;

    always @(posedge clk) begin
        if (!resetn) begin
            grant    <= 4'b0000;
            priority <= 2'd0;
        end else begin
            // Release grant when the granted core drops its req bit
            if (grant_valid && !(req & grant)) begin
                grant <= 4'b0000;
            end
            // Scan all four cores starting from priority, wrapping around.
            // Rotate priority to (winner+1) after each grant so no core starves.
            else if (!grant_valid) begin
                if (req[(priority+0) % 4]) begin
                    grant    <= 4'b0001 << ((priority+0) % 4);
                    priority <= (priority+1) % 4;
                end
                else if (req[(priority+1) % 4]) begin
                    grant    <= 4'b0001 << ((priority+1) % 4);
                    priority <= (priority+2) % 4;
                end
                else if (req[(priority+2) % 4]) begin
                    grant    <= 4'b0001 << ((priority+2) % 4);
                    priority <= (priority+3) % 4;
                end
                else if (req[(priority+3) % 4]) begin
                    grant    <= 4'b0001 << ((priority+3) % 4);
                    priority <= (priority+4) % 4;
                end
            end
        end
    end

endmodule