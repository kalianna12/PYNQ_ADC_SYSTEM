module uart_tx #(
    parameter integer CLKS_PER_BIT = 1085
) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       start,
    output reg        tx,
    output reg        busy,
    output reg        done
);

    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_START = 3'd1;
    localparam [2:0] S_DATA  = 3'd2;
    localparam [2:0] S_STOP  = 3'd3;

    reg [2:0] state = S_IDLE;
    reg [15:0] clk_count = 16'd0;
    reg [2:0] bit_index = 3'd0;
    reg [7:0] tx_shift = 8'd0;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            clk_count <= 16'd0;
            bit_index <= 3'd0;
            tx_shift <= 8'd0;
            tx <= 1'b1;
            busy <= 1'b0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
            S_IDLE: begin
                tx <= 1'b1;
                busy <= 1'b0;
                clk_count <= 16'd0;
                bit_index <= 3'd0;
                if (start) begin
                    tx_shift <= data;
                    busy <= 1'b1;
                    state <= S_START;
                end
            end

            S_START: begin
                tx <= 1'b0;
                busy <= 1'b1;
                if (clk_count == (CLKS_PER_BIT - 1)) begin
                    clk_count <= 16'd0;
                    state <= S_DATA;
                end else begin
                    clk_count <= clk_count + 16'd1;
                end
            end

            S_DATA: begin
                tx <= tx_shift[bit_index];
                busy <= 1'b1;
                if (clk_count == (CLKS_PER_BIT - 1)) begin
                    clk_count <= 16'd0;
                    if (bit_index == 3'd7) begin
                        bit_index <= 3'd0;
                        state <= S_STOP;
                    end else begin
                        bit_index <= bit_index + 3'd1;
                    end
                end else begin
                    clk_count <= clk_count + 16'd1;
                end
            end

            S_STOP: begin
                tx <= 1'b1;
                busy <= 1'b1;
                if (clk_count == (CLKS_PER_BIT - 1)) begin
                    clk_count <= 16'd0;
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end else begin
                    clk_count <= clk_count + 16'd1;
                end
            end

            default: begin
                state <= S_IDLE;
            end
            endcase
        end
    end

endmodule
