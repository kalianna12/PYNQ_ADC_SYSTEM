module ad9226_ch_a_capture #(
    parameter integer CLK_HZ = 125000000,
    parameter integer ADC_SAMPLE_HZ = 100000,
    parameter integer SAMPLE_COUNT = 256,
    parameter integer SAMPLE_DELAY_CLKS = 2
) (
    input  wire clk,
    input  wire rst,

    input  wire start,
    output reg  busy,
    output reg  done,

    output reg  adc_a_clk,
    input  wire [11:0] adc_a_data,
    input  wire adc_a_ora,

    output reg  [15:0] sample_index,
    output reg  [11:0] sample_raw,
    output reg  signed [15:0] sample_mv,
    output reg  sample_valid,

    output reg  signed [15:0] min_mv,
    output reg  signed [15:0] max_mv,
    output reg  signed [31:0] sum_mv,
    output reg  [11:0] min_raw,
    output reg  [11:0] max_raw,
    output reg  [31:0] sum_raw,
    output reg  overrange_seen
);

    localparam integer ADC_HALF_PERIOD_CLKS = CLK_HZ / (ADC_SAMPLE_HZ * 2);
    localparam integer HALF_DIV =
        (ADC_HALF_PERIOD_CLKS < 1) ? 1 : ADC_HALF_PERIOD_CLKS;

    reg [31:0] div_cnt = 32'd0;
    reg [15:0] captured_count = 16'd0;
    reg [7:0]  sample_delay_cnt = 8'd0;
    reg        sample_pending = 1'b0;

    wire [11:0] raw_inverted = adc_a_data ^ 12'hFFF;
    wire [31:0] scaled_mv = raw_inverted * 32'd10000;
    wire [13:0] mv_unipolar = scaled_mv / 32'd4095;
    wire signed [15:0] converted_mv = $signed({1'b0, mv_unipolar}) - 16'sd5000;

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            adc_a_clk <= 1'b0;
            sample_index <= 16'd0;
            sample_raw <= 12'd0;
            sample_mv <= 16'sd0;
            sample_valid <= 1'b0;
            min_mv <= 16'sd0;
            max_mv <= 16'sd0;
            sum_mv <= 32'sd0;
            min_raw <= 12'd0;
            max_raw <= 12'd0;
            sum_raw <= 32'd0;
            overrange_seen <= 1'b0;
            div_cnt <= 32'd0;
            captured_count <= 16'd0;
            sample_delay_cnt <= 8'd0;
            sample_pending <= 1'b0;
        end else begin
            done <= 1'b0;
            sample_valid <= 1'b0;

            if (!busy) begin
                adc_a_clk <= 1'b0;
                div_cnt <= 32'd0;
                sample_pending <= 1'b0;
                sample_delay_cnt <= 8'd0;

                if (start) begin
                    busy <= 1'b1;
                    captured_count <= 16'd0;
                    sample_index <= 16'd0;
                    sample_raw <= 12'd0;
                    sample_mv <= 16'sd0;
                    min_mv <= 16'sd0;
                    max_mv <= 16'sd0;
                    sum_mv <= 32'sd0;
                    min_raw <= 12'd0;
                    max_raw <= 12'd0;
                    sum_raw <= 32'd0;
                    overrange_seen <= 1'b0;
                end
            end else begin
                if (div_cnt >= (HALF_DIV - 1)) begin
                    div_cnt <= 32'd0;
                    adc_a_clk <= ~adc_a_clk;

                    if (!adc_a_clk) begin
                        sample_pending <= 1'b1;
                        sample_delay_cnt <= SAMPLE_DELAY_CLKS;
                    end
                end else begin
                    div_cnt <= div_cnt + 32'd1;
                end

                if (sample_pending) begin
                    if (sample_delay_cnt != 8'd0) begin
                        sample_delay_cnt <= sample_delay_cnt - 8'd1;
                    end else begin
                        sample_pending <= 1'b0;
                        sample_valid <= 1'b1;
                        sample_index <= captured_count;
                        sample_raw <= adc_a_data;
                        sample_mv <= converted_mv;
                        sum_mv <= sum_mv + {{16{converted_mv[15]}}, converted_mv};
                        sum_raw <= sum_raw + {20'd0, adc_a_data};
                        overrange_seen <= overrange_seen | adc_a_ora;

                        if (captured_count == 16'd0) begin
                            min_mv <= converted_mv;
                            max_mv <= converted_mv;
                            min_raw <= adc_a_data;
                            max_raw <= adc_a_data;
                        end else begin
                            if (converted_mv < min_mv)
                                min_mv <= converted_mv;
                            if (converted_mv > max_mv)
                                max_mv <= converted_mv;
                            if (adc_a_data < min_raw)
                                min_raw <= adc_a_data;
                            if (adc_a_data > max_raw)
                                max_raw <= adc_a_data;
                        end

                        if (captured_count >= (SAMPLE_COUNT - 1)) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            adc_a_clk <= 1'b0;
                        end else begin
                            captured_count <= captured_count + 16'd1;
                        end
                    end
                end
            end
        end
    end

endmodule
