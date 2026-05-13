`timescale 1ns / 1ps

module ad9226_capture_dual #(
    parameter integer ADC_CLK_HALF_PERIOD_CLKS = 625,
    parameter integer SAMPLE_DELAY_CLKS        = 40,
    parameter integer SAMPLE_COUNT             = 4096
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,

    output reg         busy,
    output reg         done,

    output wire        adc_a_clk,
    output wire        adc_b_clk,
    input  wire [11:0] adc_a_data,
    input  wire [11:0] adc_b_data,
    input  wire        adc_a_ora,
    input  wire        adc_b_orb,

    output reg  [15:0] sample_index,
    output reg  [11:0] sample_a_raw,
    output reg  [11:0] sample_b_raw,
    output reg         sample_valid,

    output reg  [11:0] min_a_raw,
    output reg  [11:0] max_a_raw,
    output reg  [11:0] vpp_a_raw,
    output reg  [31:0] sum_a_raw,

    output reg  [11:0] min_b_raw,
    output reg  [11:0] max_b_raw,
    output reg  [11:0] vpp_b_raw,
    output reg  [31:0] sum_b_raw,

    output reg         adc_a_ora_sync,
    output reg         adc_b_orb_sync,
    output reg         overrange_seen
);

    localparam integer ADC_DIV_CNT_WIDTH =
        (ADC_CLK_HALF_PERIOD_CLKS <= 2)    ? 1 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 4)    ? 2 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 8)    ? 3 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 16)   ? 4 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 32)   ? 5 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 64)   ? 6 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 128)  ? 7 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 256)  ? 8 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 512)  ? 9 :
        (ADC_CLK_HALF_PERIOD_CLKS <= 1024) ? 10 : 16;

    (* mark_debug = "true" *) reg adc_a_clk_reg = 1'b0;
    (* mark_debug = "true" *) reg adc_b_clk_reg = 1'b0;
    (* mark_debug = "true" *) reg [ADC_DIV_CNT_WIDTH-1:0] adc_clk_div_cnt = {ADC_DIV_CNT_WIDTH{1'b0}};
    (* mark_debug = "true" *) reg sample_delay_active = 1'b0;
    (* mark_debug = "true" *) reg [7:0] sample_delay_cnt = 8'd0;

    reg adc_a_ora_meta = 1'b0;
    reg adc_b_orb_meta = 1'b0;

    wire [11:0] min_a_next = (adc_a_data < min_a_raw) ? adc_a_data : min_a_raw;
    wire [11:0] max_a_next = (adc_a_data > max_a_raw) ? adc_a_data : max_a_raw;
    wire [11:0] min_b_next = (adc_b_data < min_b_raw) ? adc_b_data : min_b_raw;
    wire [11:0] max_b_next = (adc_b_data > max_b_raw) ? adc_b_data : max_b_raw;

    assign adc_a_clk = adc_a_clk_reg;
    assign adc_b_clk = adc_b_clk_reg;

    always @(posedge clk) begin
        if (rst) begin
            adc_a_clk_reg <= 1'b0;
            adc_b_clk_reg <= 1'b0;
            adc_clk_div_cnt <= {ADC_DIV_CNT_WIDTH{1'b0}};
            sample_delay_active <= 1'b0;
            sample_delay_cnt <= 8'd0;
            busy <= 1'b0;
            done <= 1'b0;
            sample_index <= 16'd0;
            sample_a_raw <= 12'd0;
            sample_b_raw <= 12'd0;
            sample_valid <= 1'b0;
            min_a_raw <= 12'hfff;
            max_a_raw <= 12'd0;
            vpp_a_raw <= 12'd0;
            sum_a_raw <= 32'd0;
            min_b_raw <= 12'hfff;
            max_b_raw <= 12'd0;
            vpp_b_raw <= 12'd0;
            sum_b_raw <= 32'd0;
            adc_a_ora_meta <= 1'b0;
            adc_a_ora_sync <= 1'b0;
            adc_b_orb_meta <= 1'b0;
            adc_b_orb_sync <= 1'b0;
            overrange_seen <= 1'b0;
        end else begin
            done <= 1'b0;
            sample_valid <= 1'b0;

            adc_a_ora_meta <= adc_a_ora;
            adc_a_ora_sync <= adc_a_ora_meta;
            adc_b_orb_meta <= adc_b_orb;
            adc_b_orb_sync <= adc_b_orb_meta;

            if (start && !busy) begin
                busy <= 1'b1;
                adc_a_clk_reg <= 1'b0;
                adc_b_clk_reg <= 1'b0;
                adc_clk_div_cnt <= {ADC_DIV_CNT_WIDTH{1'b0}};
                sample_delay_active <= 1'b0;
                sample_delay_cnt <= 8'd0;
                sample_index <= 16'd0;
                sample_a_raw <= 12'd0;
                sample_b_raw <= 12'd0;
                min_a_raw <= 12'hfff;
                max_a_raw <= 12'd0;
                vpp_a_raw <= 12'd0;
                sum_a_raw <= 32'd0;
                min_b_raw <= 12'hfff;
                max_b_raw <= 12'd0;
                vpp_b_raw <= 12'd0;
                sum_b_raw <= 32'd0;
                overrange_seen <= 1'b0;
            end else if (busy) begin
                if (adc_clk_div_cnt == ADC_CLK_HALF_PERIOD_CLKS - 1) begin
                    adc_clk_div_cnt <= {ADC_DIV_CNT_WIDTH{1'b0}};
                    adc_a_clk_reg <= ~adc_a_clk_reg;
                    adc_b_clk_reg <= ~adc_b_clk_reg;

                    if (!sample_delay_active && adc_a_clk_reg == 1'b0) begin
                        sample_delay_active <= 1'b1;
                        sample_delay_cnt <= SAMPLE_DELAY_CLKS;
                    end
                end else begin
                    adc_clk_div_cnt <= adc_clk_div_cnt + 1'b1;
                end

                if (sample_delay_active) begin
                    if (sample_delay_cnt == 8'd0) begin
                        sample_delay_active <= 1'b0;
                        sample_a_raw <= adc_a_data;
                        sample_b_raw <= adc_b_data;
                        sample_valid <= 1'b1;
                        sum_a_raw <= sum_a_raw + {20'd0, adc_a_data};
                        sum_b_raw <= sum_b_raw + {20'd0, adc_b_data};
                        overrange_seen <= overrange_seen | adc_a_ora_sync | adc_b_orb_sync;

                        if (sample_index == 16'd0) begin
                            min_a_raw <= adc_a_data;
                            max_a_raw <= adc_a_data;
                            vpp_a_raw <= 12'd0;
                            min_b_raw <= adc_b_data;
                            max_b_raw <= adc_b_data;
                            vpp_b_raw <= 12'd0;
                        end else begin
                            min_a_raw <= min_a_next;
                            max_a_raw <= max_a_next;
                            vpp_a_raw <= max_a_next - min_a_next;
                            min_b_raw <= min_b_next;
                            max_b_raw <= max_b_next;
                            vpp_b_raw <= max_b_next - min_b_next;
                        end

                        if (sample_index >= SAMPLE_COUNT - 1) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                        end else begin
                            sample_index <= sample_index + 1'b1;
                        end
                    end else begin
                        sample_delay_cnt <= sample_delay_cnt - 1'b1;
                    end
                end
            end
        end
    end

endmodule
