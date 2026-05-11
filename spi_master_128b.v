module spi_master_128b #(
    parameter integer CLK_DIV_HALF = 625
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [1023:0] tx_frame,
    input  wire        miso,
    output reg         mosi,
    output reg         sclk,
    output reg         cs_n,
    output reg  [1023:0] rx_frame,
    output reg         busy,
    output reg         done
);

    reg [15:0] div_count = 16'd0;
    reg [7:0] byte_index = 8'd0;
    reg [2:0] bit_index = 3'd7;

    always @(posedge clk) begin
        if (rst) begin
            mosi <= 1'b0;
            sclk <= 1'b0;
            cs_n <= 1'b1;
            rx_frame <= 1024'd0;
            busy <= 1'b0;
            done <= 1'b0;
            div_count <= 16'd0;
            byte_index <= 8'd0;
            bit_index <= 3'd7;
        end else begin
            done <= 1'b0;

            if (!busy) begin
                sclk <= 1'b0;
                cs_n <= 1'b1;
                div_count <= 16'd0;
                byte_index <= 8'd0;
                bit_index <= 3'd7;

                if (start) begin
                    busy <= 1'b1;
                    cs_n <= 1'b0;
                    rx_frame <= 1024'd0;
                    mosi <= tx_frame[0*8 + 7];
                end
            end else begin
                if (div_count == (CLK_DIV_HALF - 1)) begin
                    div_count <= 16'd0;

                    if (sclk == 1'b0) begin
                        sclk <= 1'b1;
                        rx_frame[byte_index*8 + bit_index] <= miso;
                    end else begin
                        sclk <= 1'b0;

                        if (bit_index == 3'd0) begin
                            if (byte_index == 8'd127) begin
                                busy <= 1'b0;
                                cs_n <= 1'b1;
                                mosi <= 1'b0;
                                done <= 1'b1;
                            end else begin
                                byte_index <= byte_index + 8'd1;
                                bit_index <= 3'd7;
                                mosi <= tx_frame[(byte_index + 8'd1)*8 + 7];
                            end
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            mosi <= tx_frame[byte_index*8 + (bit_index - 3'd1)];
                        end
                    end
                end else begin
                    div_count <= div_count + 16'd1;
                end
            end
        end
    end

endmodule
