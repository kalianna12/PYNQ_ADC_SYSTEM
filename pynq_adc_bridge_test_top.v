module pynq_adc_bridge_test_top #(
    parameter integer CLKS_PER_BIT = 1085,
    parameter integer DDS_SPI_CLK_DIV_HALF = 625,
    parameter integer DDS_SPI_PERIOD_CLKS = 2500000
) (
    input  wire clk_125m,

    input  wire uart_rx,
    output wire uart_tx,

    // Debug LEDs on PYNQADC board.
    // led0: heartbeat
    // led1: blinks when a 128-byte SPI transaction to PYNQDDS completes
    // led2: reserved / currently off
    // led3: blinks when UART receives a byte or a valid DDS->ADC frame is received
    output wire led0,
    output wire led1,
    output wire led2,
    output wire led3,

    // Reserved SPI1 to ESP32-P4. Idle in this ADC<->DDS link test.
    output wire p4_spi_mosi,
    input  wire p4_spi_miso,
    output wire p4_spi_sclk,
    output wire p4_spi_cs_n,

    // SPI2 to PYNQDDS through PmodB P7-P10.
    output wire dds_spi_mosi,
    input  wire dds_spi_miso,
    output wire dds_spi_sclk,
    output wire dds_spi_cs_n
);

    // Keep ESP32-P4 SPI pins safe while this test only checks ADC<->DDS link.
    assign p4_spi_mosi = 1'b0;
    assign p4_spi_sclk = 1'b0;
    assign p4_spi_cs_n = 1'b1;
    wire unused_p4_miso = p4_spi_miso;

    reg [15:0] rst_count = 16'hFFFF;
    wire rst = (rst_count != 16'd0);

    always @(posedge clk_125m) begin
        if (rst_count != 16'd0) begin
            rst_count <= rst_count - 16'd1;
        end
    end

    // ============================================================
    // Debug LEDs
    // ============================================================
    reg [26:0] heartbeat_cnt = 27'd0;
    reg [23:0] spi_ok_blink_cnt = 24'd0;
    reg [23:0] rx_blink_cnt = 24'd0;

    always @(posedge clk_125m) begin
        if (rst) begin
            heartbeat_cnt <= 27'd0;
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 27'd1;
        end
    end

    assign led0 = heartbeat_cnt[26];
    assign led1 = (spi_ok_blink_cnt != 24'd0);
    assign led2 = 1'b0;
    assign led3 = (rx_blink_cnt != 24'd0);

    // ============================================================
    // UART line input from PC. Type text + Enter, then send to DDS.
    // ============================================================
    wire [7:0] uart_rx_data;
    wire uart_rx_valid;

    uart_rx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_rx (
        .clk(clk_125m),
        .rst(rst),
        .rx(uart_rx),
        .data(uart_rx_data),
        .valid(uart_rx_valid)
    );

    reg [831:0] line_buf = 832'd0;
    reg [31:0]  line_len = 32'd0;
    reg [831:0] adc_text = 832'd0;
    reg [31:0]  adc_text_len = 32'd0;
    reg [31:0]  adc_seq = 32'd0;

    always @(posedge clk_125m) begin
        if (rst) begin
            line_buf <= 832'd0;
            line_len <= 32'd0;
            adc_text <= 832'd0;
            adc_text_len <= 32'd0;
            adc_seq <= 32'd0;
        end else if (uart_rx_valid) begin
            if (uart_rx_data == 8'h0D || uart_rx_data == 8'h0A) begin
                if (line_len != 32'd0) begin
                    adc_text <= line_buf;
                    adc_text_len <= line_len;
                    adc_seq <= (adc_seq == 32'hFFFFFFFF) ? 32'd1 : (adc_seq + 32'd1);
                    line_buf <= 832'd0;
                    line_len <= 32'd0;
                end
            end else if (line_len < 32'd104) begin
                line_buf[line_len*8 +: 8] <= uart_rx_data;
                line_len <= line_len + 32'd1;
            end
        end
    end

    // ADC -> DDS uses text frame 0xE1. DDS -> ADC uses text frame 0xE2.
    wire [1023:0] dds_spi_tx_frame;
    text_frame_builder_param #(
        .FRAME_TYPE(8'hE1)
    ) u_dds_text_frame_builder (
        .seq(adc_seq),
        .text_len(adc_text_len),
        .text_bytes(adc_text),
        .frame(dds_spi_tx_frame)
    );

    reg [31:0] dds_spi_period_count = 32'd0;
    reg dds_spi_start = 1'b0;
    wire dds_spi_busy;
    wire dds_spi_done;
    wire [1023:0] dds_spi_rx_frame;

    always @(posedge clk_125m) begin
        if (rst) begin
            dds_spi_period_count <= 32'd0;
            dds_spi_start <= 1'b0;
        end else begin
            dds_spi_start <= 1'b0;
            if (dds_spi_period_count >= (DDS_SPI_PERIOD_CLKS - 1)) begin
                dds_spi_period_count <= 32'd0;
                if (!dds_spi_busy) begin
                    dds_spi_start <= 1'b1;
                end
            end else begin
                dds_spi_period_count <= dds_spi_period_count + 32'd1;
            end
        end
    end

    spi_master_128b #(
        .CLK_DIV_HALF(DDS_SPI_CLK_DIV_HALF)
    ) u_dds_spi_master (
        .clk(clk_125m),
        .rst(rst),
        .start(dds_spi_start),
        .tx_frame(dds_spi_tx_frame),
        .miso(dds_spi_miso),
        .mosi(dds_spi_mosi),
        .sclk(dds_spi_sclk),
        .cs_n(dds_spi_cs_n),
        .rx_frame(dds_spi_rx_frame),
        .busy(dds_spi_busy),
        .done(dds_spi_done)
    );

    wire dds_frame_valid;
    wire [31:0] dds_seq;
    wire [31:0] dds_text_len;
    wire [831:0] dds_text_bytes;

    text_frame_parser_param #(
        .EXPECT_TYPE(8'hE2)
    ) u_dds_text_frame_parser (
        .clk(clk_125m),
        .rst(rst),
        .parse_en(dds_spi_done),
        .frame(dds_spi_rx_frame),
        .valid(dds_frame_valid),
        .seq(dds_seq),
        .text_len(dds_text_len),
        .text_bytes(dds_text_bytes)
    );

    // LED debug pulses. At 125 MHz, 12_500_000 cycles is about 100 ms.
    localparam [23:0] LED_BLINK_TICKS = 24'd12_500_000;

    wire any_rx_event = uart_rx_valid || (dds_frame_valid && dds_seq != 32'd0);

    always @(posedge clk_125m) begin
        if (rst) begin
            spi_ok_blink_cnt <= 24'd0;
            rx_blink_cnt <= 24'd0;
        end else begin
            if (dds_spi_done) begin
                spi_ok_blink_cnt <= LED_BLINK_TICKS;
            end else if (spi_ok_blink_cnt != 24'd0) begin
                spi_ok_blink_cnt <= spi_ok_blink_cnt - 24'd1;
            end

            if (any_rx_event) begin
                rx_blink_cnt <= LED_BLINK_TICKS;
            end else if (rx_blink_cnt != 24'd0) begin
                rx_blink_cnt <= rx_blink_cnt - 24'd1;
            end
        end
    end

    reg [31:0] last_dds_seq = 32'd0;
    reg [831:0] print_text = 832'd0;
    reg [31:0] print_text_len = 32'd0;
    reg print_pending = 1'b0;

    always @(posedge clk_125m) begin
        if (rst) begin
            last_dds_seq <= 32'd0;
            print_text <= 832'd0;
            print_text_len <= 32'd0;
            print_pending <= 1'b0;
        end else if (dds_frame_valid && dds_seq != 32'd0 && dds_seq != last_dds_seq) begin
            last_dds_seq <= dds_seq;
            print_text <= dds_text_bytes;
            print_text_len <= dds_text_len;
            print_pending <= 1'b1;
        end else if (print_start_msg) begin
            print_pending <= 1'b0;
        end
    end

    // ============================================================
    // UART printer
    // ============================================================
    reg [7:0] uart_tx_data = 8'd0;
    reg uart_tx_start = 1'b0;
    wire uart_tx_busy;
    wire uart_tx_done;

    uart_tx #(
        .CLKS_PER_BIT(CLKS_PER_BIT)
    ) u_uart_tx (
        .clk(clk_125m),
        .rst(rst),
        .data(uart_tx_data),
        .start(uart_tx_start),
        .tx(uart_tx),
        .busy(uart_tx_busy),
        .done(uart_tx_done)
    );

    localparam [1:0] PRINT_READY = 2'd0;
    localparam [1:0] PRINT_MSG   = 2'd1;

    reg print_active = 1'b1;
    reg [1:0] print_mode = PRINT_READY;
    reg [7:0] print_index = 8'd0;
    reg print_start_msg = 1'b0;

    function [7:0] ready_char;
        input [7:0] idx;
        begin
            case (idx)
            8'd0: ready_char = "P";  8'd1: ready_char = "Y";  8'd2: ready_char = "N";  8'd3: ready_char = "Q";
            8'd4: ready_char = "A";  8'd5: ready_char = "D";  8'd6: ready_char = "C";  8'd7: ready_char = " ";
            8'd8: ready_char = "D";  8'd9: ready_char = "D";  8'd10: ready_char = "S"; 8'd11: ready_char = " ";
            8'd12: ready_char = "S"; 8'd13: ready_char = "P"; 8'd14: ready_char = "I"; 8'd15: ready_char = " ";
            8'd16: ready_char = "R"; 8'd17: ready_char = "E"; 8'd18: ready_char = "A"; 8'd19: ready_char = "D"; 8'd20: ready_char = "Y";
            8'd21: ready_char = 8'h0D; 8'd22: ready_char = 8'h0A;
            default: ready_char = 8'h00;
            endcase
        end
    endfunction

    function [7:0] prefix_char;
        input [7:0] idx;
        begin
            case (idx)
            8'd0: prefix_char = "R"; 8'd1: prefix_char = "X"; 8'd2: prefix_char = "_"; 8'd3: prefix_char = "F";
            8'd4: prefix_char = "R"; 8'd5: prefix_char = "O"; 8'd6: prefix_char = "M"; 8'd7: prefix_char = "_";
            8'd8: prefix_char = "D"; 8'd9: prefix_char = "D"; 8'd10: prefix_char = "S"; 8'd11: prefix_char = ":"; 8'd12: prefix_char = " ";
            default: prefix_char = 8'h00;
            endcase
        end
    endfunction

    always @(posedge clk_125m) begin
        if (rst) begin
            uart_tx_start <= 1'b0;
            uart_tx_data <= 8'd0;
            print_active <= 1'b1;
            print_mode <= PRINT_READY;
            print_index <= 8'd0;
            print_start_msg <= 1'b0;
        end else begin
            uart_tx_start <= 1'b0;
            print_start_msg <= 1'b0;

            if (!print_active && print_pending) begin
                print_active <= 1'b1;
                print_mode <= PRINT_MSG;
                print_index <= 8'd0;
                print_start_msg <= 1'b1;
            end else if (print_active && !uart_tx_busy && !uart_tx_start) begin
                if (print_mode == PRINT_READY) begin
                    if (print_index < 8'd23) begin
                        uart_tx_data <= ready_char(print_index);
                        uart_tx_start <= 1'b1;
                        print_index <= print_index + 8'd1;
                    end else begin
                        print_active <= 1'b0;
                    end
                end else begin
                    if (print_index < 8'd13) begin
                        uart_tx_data <= prefix_char(print_index);
                        uart_tx_start <= 1'b1;
                        print_index <= print_index + 8'd1;
                    end else if (print_index < (8'd13 + print_text_len[7:0])) begin
                        uart_tx_data <= print_text[(print_index - 8'd13)*8 +: 8];
                        uart_tx_start <= 1'b1;
                        print_index <= print_index + 8'd1;
                    end else if (print_index == (8'd13 + print_text_len[7:0])) begin
                        uart_tx_data <= 8'h0D;
                        uart_tx_start <= 1'b1;
                        print_index <= print_index + 8'd1;
                    end else if (print_index == (8'd14 + print_text_len[7:0])) begin
                        uart_tx_data <= 8'h0A;
                        uart_tx_start <= 1'b1;
                        print_index <= print_index + 8'd1;
                    end else begin
                        print_active <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
