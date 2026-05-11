module pynq_adc_system_top #(
    parameter integer SPI_A_CLK_DIV_HALF  = 625,       // 125MHz/(2*625)=100kHz
    parameter integer SPI_A_PERIOD_CLKS   = 12500000,   // 100ms poll period
    parameter integer SPI_B_CLK_DIV_HALF  = 2500,       // 125MHz/(2*2500)=25kHz
    parameter integer SETTLE_CLKS         = 12500000,   // 100ms settle
    parameter integer DDS_ACK_TIMEOUT_CLKS = 37500000   // 300ms timeout
) (
    input  wire clk_125m,

    // Debug LEDs
    // led0: heartbeat
    // led1: SPI-A transaction toggle (to ESP32-P4)
    // led2: SPI-B transaction toggle (to PYNQDDS)
    // led3: error pulse
    output wire led0,
    output wire led1,
    output wire led2,
    output wire led3,

    // SPI-A to ESP32-P4 (Master, PmodB P1-P4)
    output wire p4_spi_mosi,
    input  wire p4_spi_miso,
    output wire p4_spi_sclk,
    output wire p4_spi_cs_n,

    // SPI-B to PYNQDDS (Master, PmodB P7-P10)
    output wire dds_spi_mosi,
    input  wire dds_spi_miso,
    output wire dds_spi_sclk,
    output wire dds_spi_cs_n
);

    // ============================================================
    // Reset
    // ============================================================
    reg [15:0] rst_count = 16'hFFFF;
    wire rst = (rst_count != 16'd0);

    always @(posedge clk_125m) begin
        if (rst_count != 16'd0)
            rst_count <= rst_count - 16'd1;
    end

    // ============================================================
    // State machine
    // ============================================================
    localparam [3:0] ST_IDLE            = 4'd0;
    localparam [3:0] ST_START_SCAN      = 4'd1;
    localparam [3:0] ST_SEND_DDS_FREQ   = 4'd2;
    localparam [3:0] ST_WAIT_DDS_ACK    = 4'd3;
    localparam [3:0] ST_WAIT_SETTLE     = 4'd4;
    localparam [3:0] ST_ADC_CAPTURE     = 4'd5;
    localparam [3:0] ST_SEND_RESULT     = 4'd6;
    localparam [3:0] ST_NEXT_FREQ       = 4'd7;
    localparam [3:0] ST_DONE            = 4'd8;
    localparam [3:0] ST_STOP            = 4'd9;
    localparam [3:0] ST_ERROR           = 4'd10;

    localparam [31:0] MODE_SWEEP  = 32'd0;
    localparam [31:0] MODE_SINGLE = 32'd1;

    localparam [31:0] STATE_IDLE     = 32'd0;
    localparam [31:0] STATE_SCANNING = 32'd1;
    localparam [31:0] STATE_DONE     = 32'd2;
    localparam [31:0] STATE_ERROR    = 32'd3;

    localparam [31:0] CMD_START          = 32'd1;
    localparam [31:0] CMD_STOP           = 32'd2;
    localparam [31:0] CMD_SET_MODE       = 32'd3;
    localparam [31:0] CMD_SET_START_FREQ = 32'd4;
    localparam [31:0] CMD_SET_STOP_FREQ  = 32'd5;
    localparam [31:0] CMD_SET_STEP_FREQ  = 32'd6;
    localparam [31:0] CMD_SET_SINGLE_FREQ= 32'd7;
    localparam [31:0] CMD_CLEAR_TABLE    = 32'd8;

    localparam [31:0] DDS_CMD_NOP        = 32'd0;
    localparam [31:0] DDS_CMD_SET_FREQ   = 32'd1;
    localparam [31:0] DDS_CMD_SET_SINGLE = 32'd4;
    localparam [31:0] DDS_CMD_STOP       = 32'd3;

    reg [3:0]  state = ST_IDLE;
    reg [31:0] settle_cnt = 32'd0;

    // Config from ESP
    reg [31:0] config_mode        = MODE_SWEEP;
    reg [31:0] config_start_freq  = 32'd100;
    reg [31:0] config_stop_freq   = 32'd100000;
    reg [31:0] config_step_freq   = 32'd10000;
    reg [31:0] config_single_freq = 32'd1000;

    // Sweep state
    reg [31:0] current_freq_hz  = 32'd0;
    reg [31:0] point_index      = 32'd0;
    reg [31:0] total_points     = 32'd0;
    reg        cmd_start_flag   = 1'b0;
    reg        cmd_stop_flag    = 1'b0;

    wire [31:0] calc_total_points =
        (config_mode == MODE_SWEEP && config_step_freq != 32'd0) ?
        ((config_stop_freq - config_start_freq) / config_step_freq + 32'd1) :
        32'd1;

    // Status registers (for 0x10 frame)
    reg [31:0] stat_state         = STATE_IDLE;
    reg [31:0] stat_mode          = MODE_SWEEP;
    reg [31:0] stat_filter_type   = 32'd1;  // Low-pass
    reg [31:0] stat_link_ok       = 32'd1;
    reg [31:0] stat_progress      = 32'd0;
    reg [31:0] stat_start_freq    = 32'd0;
    reg [31:0] stat_stop_freq     = 32'd0;
    reg [31:0] stat_step_freq     = 32'd0;
    reg [31:0] stat_single_freq   = 32'd0;
    reg [31:0] stat_current_freq  = 32'd0;
    reg [31:0] stat_point_index   = 32'd0;
    reg [31:0] stat_total_points  = 32'd0;
    reg [31:0] stat_vin_mv        = 32'd0;
    reg [31:0] stat_vout_mv       = 32'd0;
    reg [31:0] stat_gain_x1000    = 32'd0;
    reg [31:0] stat_theory_gain   = 32'd0;
    reg [31:0] stat_error_x10     = 32'd0;
    reg [31:0] stat_phase_deg_x10 = 32'd0;
    reg [31:0] stat_cutoff_freq   = 32'd1590;

    // ESP command parser state
    reg [31:0] esp_cmd      = 32'd0;
    reg [31:0] esp_cmd_seq  = 32'd0;
    reg [31:0] esp_arg0     = 32'd0;
    reg [31:0] esp_last_seq = 32'hFFFFFFFF;
    reg        esp_cmd_valid = 1'b0;

    // SPI-B control
    reg        spi_b_start = 1'b0;
    reg        spi_b_request_in_progress = 1'b0;
    reg [31:0] spi_b_timeout = 32'd0;
    wire       spi_b_done;
    wire       spi_b_busy;
    wire [1023:0] spi_b_rx_frame;

    wire [31:0] dds_expected_cmd =
        (config_mode == MODE_SINGLE) ? DDS_CMD_SET_SINGLE : DDS_CMD_SET_FREQ;
    wire [31:0] d2_ack_seq = {
        spi_b_rx_frame[7*8 +: 8], spi_b_rx_frame[6*8 +: 8],
        spi_b_rx_frame[5*8 +: 8], spi_b_rx_frame[4*8 +: 8]};
    wire [31:0] d2_ack_cmd = {
        spi_b_rx_frame[11*8 +: 8], spi_b_rx_frame[10*8 +: 8],
        spi_b_rx_frame[9*8 +: 8],  spi_b_rx_frame[8*8 +: 8]};
    wire [31:0] d2_ack_freq = {
        spi_b_rx_frame[15*8 +: 8], spi_b_rx_frame[14*8 +: 8],
        spi_b_rx_frame[13*8 +: 8], spi_b_rx_frame[12*8 +: 8]};
    wire [31:0] d2_ack_flags = {
        spi_b_rx_frame[19*8 +: 8], spi_b_rx_frame[18*8 +: 8],
        spi_b_rx_frame[17*8 +: 8], spi_b_rx_frame[16*8 +: 8]};

    reg [7:0] d2_chk;
    integer d2_i;
    always @* begin
        d2_chk = 8'd0;
        for (d2_i = 0; d2_i < 116; d2_i = d2_i + 1)
            d2_chk = d2_chk ^ spi_b_rx_frame[d2_i*8 +: 8];
    end

    wire dds_ack_valid =
        spi_b_rx_frame[0*8 +: 8]   == 8'hA5 &&
        spi_b_rx_frame[1*8 +: 8]   == 8'h5A &&
        spi_b_rx_frame[2*8 +: 8]   == 8'hD2 &&
        spi_b_rx_frame[3*8 +: 8]   == 8'd112 &&
        spi_b_rx_frame[116*8 +: 8] == d2_chk &&
        d2_ack_seq                 == point_index &&
        d2_ack_cmd                 == dds_expected_cmd &&
        d2_ack_freq                == current_freq_hz &&
        d2_ack_flags[1:0]          == 2'b11;

    // ============================================================
    // Fake measurement computation (combinational)
    // RC low-pass model: fc=1590Hz
    // ============================================================
    wire [63:0] ratio_m10;    // (freq * 10000) / 1590
    wire [31:0] fake_theory;  // theory gain x1000
    wire [31:0] fake_vout;    // mV
    wire [31:0] fake_gain;    // gain x1000
    wire [31:0] fake_error;   // error x10
    wire [31:0] fake_phase;   // phase deg x10

    assign ratio_m10 = (current_freq_hz * 64'd10000) / 64'd1590;

    assign fake_theory =
        (ratio_m10 < 64'd5000)  ? (32'd1000 - (ratio_m10 * ratio_m10 / 64'd50000)) :
        (ratio_m10 < 64'd20000) ? (32'd900 - ((ratio_m10 - 64'd5000) * 64'd400 / 64'd15000)) :
        (32'd5000000 / ratio_m10);

    assign fake_vout  = (32'd1000 * fake_theory / 32'd1000) + ({27'd0, point_index[4:0]} % 32'd7);
    assign fake_gain  = (fake_vout * 32'd1000) / (32'd995 + {27'd0, point_index[4:0]});
    assign fake_error = (fake_gain > fake_theory) ?
        ((fake_gain - fake_theory) * 32'd100 / fake_theory) :
        ((fake_theory - fake_gain) * 32'd100 / fake_theory);
    assign fake_phase = (ratio_m10 * 32'd900) / (32'd1000 + ratio_m10);

    integer ii;

    // ============================================================
    // Main state machine
    // ============================================================
    always @(posedge clk_125m) begin
        if (rst) begin
            state <= ST_IDLE;
            settle_cnt <= 32'd0;
            config_mode <= MODE_SWEEP;
            config_start_freq <= 32'd100;
            config_stop_freq <= 32'd100000;
            config_step_freq <= 32'd10000;
            config_single_freq <= 32'd1000;
            current_freq_hz <= 32'd0;
            point_index <= 32'd0;
            total_points <= 32'd0;
            cmd_start_flag <= 1'b0;
            cmd_stop_flag <= 1'b0;
            stat_state <= STATE_IDLE;
            stat_mode <= MODE_SWEEP;
            stat_progress <= 32'd0;
            stat_start_freq <= 32'd0;
            stat_stop_freq <= 32'd0;
            stat_step_freq <= 32'd0;
            stat_single_freq <= 32'd0;
            stat_current_freq <= 32'd0;
            stat_point_index <= 32'd0;
            stat_total_points <= 32'd0;
            stat_vin_mv <= 32'd0;
            stat_vout_mv <= 32'd0;
            stat_gain_x1000 <= 32'd0;
            stat_theory_gain <= 32'd0;
            stat_error_x10 <= 32'd0;
            stat_phase_deg_x10 <= 32'd0;
            stat_filter_type <= 32'd1;
            stat_cutoff_freq <= 32'd1590;
            stat_link_ok <= 32'd1;
            spi_b_timeout <= 32'd0;
        end else begin
            // Process ESP commands generated by the SPI-A parser.
            // Do not drive esp_cmd_valid here; the parser owns that pulse.
            if (esp_cmd_valid) begin
                case (esp_cmd)
                    CMD_START: begin
                        cmd_start_flag <= 1'b1;
                        cmd_stop_flag <= 1'b0;
                    end
                    CMD_STOP: begin
                        cmd_stop_flag <= 1'b1;
                        cmd_start_flag <= 1'b0;
                    end
                    CMD_SET_MODE:        config_mode <= esp_arg0;
                    CMD_SET_START_FREQ:  config_start_freq <= esp_arg0;
                    CMD_SET_STOP_FREQ:   config_stop_freq <= esp_arg0;
                    CMD_SET_STEP_FREQ:   config_step_freq <= esp_arg0;
                    CMD_SET_SINGLE_FREQ: config_single_freq <= esp_arg0;
                    default: ;
                endcase
            end

            case (state)
                ST_IDLE: begin
                    stat_state <= STATE_IDLE;
                    stat_link_ok <= 32'd1;

                    if (cmd_start_flag) begin
                        cmd_start_flag <= 1'b0;
                        state <= ST_START_SCAN;
                    end
                end

                ST_START_SCAN: begin
                    stat_mode <= config_mode;
                    stat_start_freq <= config_start_freq;
                    stat_stop_freq <= config_stop_freq;
                    stat_step_freq <= config_step_freq;
                    stat_single_freq <= config_single_freq;
                    stat_state <= STATE_SCANNING;
                    stat_point_index <= 32'd1;
                    stat_total_points <= calc_total_points;

                    if (config_mode == MODE_SWEEP) begin
                        current_freq_hz <= config_start_freq;
                    end else begin
                        current_freq_hz <= config_single_freq;
                    end
                    total_points <= calc_total_points;
                    point_index <= 32'd1;
                    spi_b_timeout <= 32'd0;
                    state <= ST_SEND_DDS_FREQ;
                end

                ST_SEND_DDS_FREQ: begin
                    // SPI-B trigger handled in separate block: send 0xD1 SET_FREQ.
                    if (spi_b_done) begin
                        spi_b_timeout <= 32'd0;
                        state <= ST_WAIT_DDS_ACK;
                    end
                end

                ST_WAIT_DDS_ACK: begin
                    if (spi_b_done) begin
                        if (dds_ack_valid) begin
                            state <= ST_WAIT_SETTLE;
                            settle_cnt <= 32'd0;
                            spi_b_timeout <= 32'd0;
                            stat_link_ok <= 32'd1;
                        end else if (spi_b_timeout >= DDS_ACK_TIMEOUT_CLKS) begin
                            stat_state <= STATE_ERROR;
                            stat_link_ok <= 32'd0;
                            state <= ST_ERROR;
                        end
                    end else if (spi_b_timeout >= DDS_ACK_TIMEOUT_CLKS) begin
                        stat_state <= STATE_ERROR;
                        stat_link_ok <= 32'd0;
                        state <= ST_ERROR;
                    end else begin
                        spi_b_timeout <= spi_b_timeout + 32'd1;
                    end
                end

                ST_WAIT_SETTLE: begin
                    if (settle_cnt >= SETTLE_CLKS)
                        state <= ST_ADC_CAPTURE;
                    else
                        settle_cnt <= settle_cnt + 32'd1;
                end

                ST_ADC_CAPTURE: begin
                    // Fake measurement - compute RC low-pass model
                    // ratio_m10 = (freq_hz * 10000) / fc, where fc=1590
                    // Gain approximation (gain_x1000)
                    stat_current_freq <= current_freq_hz;
                    stat_point_index <= point_index;

                    // Compute fake data
                    stat_vin_mv <= 32'd995 + {27'd0, point_index[4:0]};
                    stat_vout_mv <= fake_vout;
                    stat_gain_x1000 <= fake_gain;
                    stat_theory_gain <= fake_theory;
                    stat_error_x10 <= fake_error;
                    stat_phase_deg_x10 <= fake_phase;
                    stat_filter_type <= 32'd1;
                    stat_cutoff_freq <= 32'd1590;

                    stat_progress <= (point_index * 32'd1000) / total_points;

                    state <= ST_SEND_RESULT;
                end

                ST_SEND_RESULT: begin
                    // Status updated, SPI-A sends automatically
                    state <= ST_NEXT_FREQ;
                end

                ST_NEXT_FREQ: begin
                    if (config_mode == MODE_SINGLE || point_index >= total_points) begin
                        state <= ST_DONE;
                    end else begin
                        current_freq_hz <= current_freq_hz + config_step_freq;
                        point_index <= point_index + 32'd1;
                        state <= ST_SEND_DDS_FREQ;
                    end
                end

                ST_DONE: begin
                    stat_state <= STATE_DONE;
                    stat_progress <= 32'd1000;
                    stat_cutoff_freq <= 32'd1590;
                    stat_filter_type <= 32'd1;
                    // Stay in DONE until new START
                    if (cmd_start_flag) begin
                        cmd_start_flag <= 1'b0;
                        state <= ST_START_SCAN;
                    end
                end

                ST_STOP: begin
                    stat_state <= STATE_IDLE;
                    cmd_stop_flag <= 1'b0;
                    spi_b_timeout <= 32'd0;
                    state <= ST_IDLE;
                end

                ST_ERROR: begin
                    stat_state <= STATE_ERROR;
                    stat_link_ok <= 32'd0;

                    if (cmd_start_flag) begin
                        cmd_start_flag <= 1'b0;
                        stat_link_ok <= 32'd1;
                        state <= ST_START_SCAN;
                    end
                end

                default: state <= ST_IDLE;
            endcase

            // Global STOP check
            if (cmd_stop_flag && state != ST_IDLE && state != ST_DONE && state != ST_STOP) begin
                state <= ST_STOP;
            end
        end
    end

    // ============================================================
    // SPI-A to ESP32-P4 (periodic, 100ms)
    // ============================================================
    reg [31:0] spi_a_period_cnt = 32'd0;
    reg        spi_a_start = 1'b0;
    wire       spi_a_busy;
    wire       spi_a_done;
    wire [1023:0] spi_a_rx_frame;

    always @(posedge clk_125m) begin
        if (rst) begin
            spi_a_period_cnt <= 32'd0;
            spi_a_start <= 1'b0;
        end else begin
            spi_a_start <= 1'b0;
            if (spi_a_period_cnt >= (SPI_A_PERIOD_CLKS - 1)) begin
                spi_a_period_cnt <= 32'd0;
                if (!spi_a_busy) begin
                    spi_a_start <= 1'b1;
                end
            end else begin
                spi_a_period_cnt <= spi_a_period_cnt + 32'd1;
            end
        end
    end

    // 0x10 Status frame builder (combinational)
    reg [1023:0] frame_0x10;
    reg [7:0] f10_chk;
    integer f10_i;

    always @* begin
        frame_0x10 = 1024'd0;
        frame_0x10[0*8 +: 8]  = 8'hA5;
        frame_0x10[1*8 +: 8]  = 8'h5A;
        frame_0x10[2*8 +: 8]  = 8'h10;
        frame_0x10[3*8 +: 8]  = 8'd112;
        frame_0x10[4*8 +: 8]  = stat_state[7:0];         frame_0x10[5*8 +: 8]  = stat_state[15:8];         frame_0x10[6*8 +: 8]  = stat_state[23:16];         frame_0x10[7*8 +: 8]  = stat_state[31:24];
        frame_0x10[8*8 +: 8]  = stat_mode[7:0];          frame_0x10[9*8 +: 8]  = stat_mode[15:8];          frame_0x10[10*8 +: 8] = stat_mode[23:16];          frame_0x10[11*8 +: 8] = stat_mode[31:24];
        frame_0x10[12*8 +: 8] = stat_filter_type[7:0];   frame_0x10[13*8 +: 8] = stat_filter_type[15:8];   frame_0x10[14*8 +: 8] = stat_filter_type[23:16];   frame_0x10[15*8 +: 8] = stat_filter_type[31:24];
        frame_0x10[16*8 +: 8] = stat_link_ok[7:0];       frame_0x10[17*8 +: 8] = stat_link_ok[15:8];       frame_0x10[18*8 +: 8] = stat_link_ok[23:16];       frame_0x10[19*8 +: 8] = stat_link_ok[31:24];
        frame_0x10[20*8 +: 8] = stat_progress[7:0];      frame_0x10[21*8 +: 8] = stat_progress[15:8];      frame_0x10[22*8 +: 8] = stat_progress[23:16];      frame_0x10[23*8 +: 8] = stat_progress[31:24];
        frame_0x10[24*8 +: 8] = stat_start_freq[7:0];    frame_0x10[25*8 +: 8] = stat_start_freq[15:8];    frame_0x10[26*8 +: 8] = stat_start_freq[23:16];    frame_0x10[27*8 +: 8] = stat_start_freq[31:24];
        frame_0x10[28*8 +: 8] = stat_stop_freq[7:0];     frame_0x10[29*8 +: 8] = stat_stop_freq[15:8];     frame_0x10[30*8 +: 8] = stat_stop_freq[23:16];     frame_0x10[31*8 +: 8] = stat_stop_freq[31:24];
        frame_0x10[32*8 +: 8] = stat_step_freq[7:0];     frame_0x10[33*8 +: 8] = stat_step_freq[15:8];     frame_0x10[34*8 +: 8] = stat_step_freq[23:16];     frame_0x10[35*8 +: 8] = stat_step_freq[31:24];
        frame_0x10[36*8 +: 8] = stat_single_freq[7:0];   frame_0x10[37*8 +: 8] = stat_single_freq[15:8];   frame_0x10[38*8 +: 8] = stat_single_freq[23:16];   frame_0x10[39*8 +: 8] = stat_single_freq[31:24];
        frame_0x10[40*8 +: 8] = stat_current_freq[7:0];  frame_0x10[41*8 +: 8] = stat_current_freq[15:8];  frame_0x10[42*8 +: 8] = stat_current_freq[23:16];  frame_0x10[43*8 +: 8] = stat_current_freq[31:24];
        frame_0x10[44*8 +: 8] = stat_point_index[7:0];   frame_0x10[45*8 +: 8] = stat_point_index[15:8];   frame_0x10[46*8 +: 8] = stat_point_index[23:16];   frame_0x10[47*8 +: 8] = stat_point_index[31:24];
        frame_0x10[48*8 +: 8] = stat_total_points[7:0];  frame_0x10[49*8 +: 8] = stat_total_points[15:8];  frame_0x10[50*8 +: 8] = stat_total_points[23:16];  frame_0x10[51*8 +: 8] = stat_total_points[31:24];
        frame_0x10[52*8 +: 8] = stat_vin_mv[7:0];        frame_0x10[53*8 +: 8] = stat_vin_mv[15:8];        frame_0x10[54*8 +: 8] = stat_vin_mv[23:16];        frame_0x10[55*8 +: 8] = stat_vin_mv[31:24];
        frame_0x10[56*8 +: 8] = stat_vout_mv[7:0];       frame_0x10[57*8 +: 8] = stat_vout_mv[15:8];       frame_0x10[58*8 +: 8] = stat_vout_mv[23:16];       frame_0x10[59*8 +: 8] = stat_vout_mv[31:24];
        frame_0x10[60*8 +: 8] = stat_gain_x1000[7:0];    frame_0x10[61*8 +: 8] = stat_gain_x1000[15:8];    frame_0x10[62*8 +: 8] = stat_gain_x1000[23:16];    frame_0x10[63*8 +: 8] = stat_gain_x1000[31:24];
        frame_0x10[64*8 +: 8] = stat_theory_gain[7:0];   frame_0x10[65*8 +: 8] = stat_theory_gain[15:8];   frame_0x10[66*8 +: 8] = stat_theory_gain[23:16];   frame_0x10[67*8 +: 8] = stat_theory_gain[31:24];
        frame_0x10[68*8 +: 8] = stat_error_x10[7:0];     frame_0x10[69*8 +: 8] = stat_error_x10[15:8];     frame_0x10[70*8 +: 8] = stat_error_x10[23:16];     frame_0x10[71*8 +: 8] = stat_error_x10[31:24];
        frame_0x10[72*8 +: 8] = stat_phase_deg_x10[7:0]; frame_0x10[73*8 +: 8] = stat_phase_deg_x10[15:8]; frame_0x10[74*8 +: 8] = stat_phase_deg_x10[23:16]; frame_0x10[75*8 +: 8] = stat_phase_deg_x10[31:24];
        frame_0x10[76*8 +: 8] = stat_cutoff_freq[7:0];   frame_0x10[77*8 +: 8] = stat_cutoff_freq[15:8];   frame_0x10[78*8 +: 8] = stat_cutoff_freq[23:16];   frame_0x10[79*8 +: 8] = stat_cutoff_freq[31:24];

        f10_chk = 8'd0;
        for (f10_i = 0; f10_i < 116; f10_i = f10_i + 1)
            f10_chk = f10_chk ^ frame_0x10[f10_i*8 +: 8];
        frame_0x10[116*8 +: 8] = f10_chk;
    end

    spi_master_128b #(.CLK_DIV_HALF(SPI_A_CLK_DIV_HALF)) u_spi_a (
        .clk(clk_125m), .rst(rst),
        .start(spi_a_start), .tx_frame(frame_0x10),
        .miso(p4_spi_miso), .mosi(p4_spi_mosi),
        .sclk(p4_spi_sclk), .cs_n(p4_spi_cs_n),
        .rx_frame(spi_a_rx_frame), .busy(spi_a_busy), .done(spi_a_done)
    );

    // 0x80 Command parser (from ESP32-P4)
    reg [7:0] p80_chk;
    integer p80_i;

    always @(posedge clk_125m) begin
        if (rst) begin
            esp_cmd <= 32'd0;
            esp_cmd_seq <= 32'd0;
            esp_arg0 <= 32'd0;
            esp_last_seq <= 32'hFFFFFFFF;
            esp_cmd_valid <= 1'b0;
        end else begin
            esp_cmd_valid <= 1'b0;

            if (spi_a_done) begin
                // Parse 0x80 frame (payload = 16 bytes)
                p80_chk = 8'd0;
                for (p80_i = 0; p80_i < 20; p80_i = p80_i + 1)
                    p80_chk = p80_chk ^ spi_a_rx_frame[p80_i*8 +: 8];

                if (spi_a_rx_frame[0*8 +: 8] == 8'hA5 &&
                    spi_a_rx_frame[1*8 +: 8] == 8'h5A &&
                    spi_a_rx_frame[2*8 +: 8] == 8'h80 &&
                    spi_a_rx_frame[3*8 +: 8] == 8'd16 &&
                    spi_a_rx_frame[20*8 +: 8] == p80_chk) begin

                    // seq (LE)
                    esp_cmd_seq <= {
                        spi_a_rx_frame[7*8 +: 8], spi_a_rx_frame[6*8 +: 8],
                        spi_a_rx_frame[5*8 +: 8], spi_a_rx_frame[4*8 +: 8]};

                    // cmd (LE)
                    esp_cmd <= {
                        spi_a_rx_frame[11*8 +: 8], spi_a_rx_frame[10*8 +: 8],
                        spi_a_rx_frame[9*8 +: 8],  spi_a_rx_frame[8*8 +: 8]};

                    // arg0 (LE)
                    esp_arg0 <= {
                        spi_a_rx_frame[15*8 +: 8], spi_a_rx_frame[14*8 +: 8],
                        spi_a_rx_frame[13*8 +: 8], spi_a_rx_frame[12*8 +: 8]};

                    // Use frame bytes directly for dedup (NBAs not yet taken effect)
                    if ({spi_a_rx_frame[7*8+:8], spi_a_rx_frame[6*8+:8], spi_a_rx_frame[5*8+:8], spi_a_rx_frame[4*8+:8]} != esp_last_seq &&
                        {spi_a_rx_frame[11*8+:8], spi_a_rx_frame[10*8+:8], spi_a_rx_frame[9*8+:8], spi_a_rx_frame[8*8+:8]} != 32'd0) begin
                        esp_last_seq <= {spi_a_rx_frame[7*8+:8], spi_a_rx_frame[6*8+:8], spi_a_rx_frame[5*8+:8], spi_a_rx_frame[4*8+:8]};
                        esp_cmd_valid <= 1'b1;
                    end
                end
            end
        end
    end

    // ============================================================
    // SPI-B to PYNQDDS (on-demand, triggered by state machine)
    // ============================================================
    wire [1023:0] spi_b_tx_frame;

    // SPI-B trigger control
    always @(posedge clk_125m) begin
        if (rst) begin
            spi_b_start <= 1'b0;
            spi_b_request_in_progress <= 1'b0;
        end else begin
            spi_b_start <= 1'b0;
            if ((state == ST_SEND_DDS_FREQ || state == ST_WAIT_DDS_ACK) &&
                !spi_b_request_in_progress && !spi_b_busy) begin
                spi_b_start <= 1'b1;
                spi_b_request_in_progress <= 1'b1;
            end
            if (spi_b_done) begin
                spi_b_request_in_progress <= 1'b0;
            end
        end
    end

    // 0xD1 DDS command frame builder (combinational)
    reg [1023:0] frame_d1;
    reg [7:0] d1_chk;
    integer d1_i;

    always @* begin
        frame_d1 = 1024'd0;
        frame_d1[0*8 +: 8] = 8'hA5;
        frame_d1[1*8 +: 8] = 8'h5A;
        frame_d1[2*8 +: 8] = 8'hD1;
        frame_d1[3*8 +: 8] = 8'd112;

        // seq = point_index
        frame_d1[4*8 +: 8]  = point_index[7:0];
        frame_d1[5*8 +: 8]  = point_index[15:8];
        frame_d1[6*8 +: 8]  = point_index[23:16];
        frame_d1[7*8 +: 8]  = point_index[31:24];

        // cmd = SET_FREQ or SET_SINGLE
        if (config_mode == MODE_SINGLE) begin
            frame_d1[8*8 +: 8]  = DDS_CMD_SET_SINGLE[7:0];
            frame_d1[9*8 +: 8]  = DDS_CMD_SET_SINGLE[15:8];
            frame_d1[10*8 +: 8] = DDS_CMD_SET_SINGLE[23:16];
            frame_d1[11*8 +: 8] = DDS_CMD_SET_SINGLE[31:24];
        end else begin
            frame_d1[8*8 +: 8]  = DDS_CMD_SET_FREQ[7:0];
            frame_d1[9*8 +: 8]  = DDS_CMD_SET_FREQ[15:8];
            frame_d1[10*8 +: 8] = DDS_CMD_SET_FREQ[23:16];
            frame_d1[11*8 +: 8] = DDS_CMD_SET_FREQ[31:24];
        end

        // freq_hz (LE)
        frame_d1[12*8 +: 8] = current_freq_hz[7:0];
        frame_d1[13*8 +: 8] = current_freq_hz[15:8];
        frame_d1[14*8 +: 8] = current_freq_hz[23:16];
        frame_d1[15*8 +: 8] = current_freq_hz[31:24];

        // amplitude = 1000 (LE)
        frame_d1[16*8 +: 8] = 8'hE8;  // 1000
        frame_d1[17*8 +: 8] = 8'h03;
        frame_d1[18*8 +: 8] = 8'h00;
        frame_d1[19*8 +: 8] = 8'h00;

        // rest stays 0 (phase=0, start=0, stop=0, step=0, mode=0, flags=0)

        // checksum
        d1_chk = 8'd0;
        for (d1_i = 0; d1_i < 116; d1_i = d1_i + 1)
            d1_chk = d1_chk ^ frame_d1[d1_i*8 +: 8];
        frame_d1[116*8 +: 8] = d1_chk;
    end

    // 0xD1 NOP/POLL frame. The returned MISO payload carries the previous
    // transaction's 0xD2 ACK from PYNQDDS.
    reg [1023:0] frame_d1_nop;
    reg [7:0] d1_nop_chk;
    integer d1_nop_i;

    always @* begin
        frame_d1_nop = 1024'd0;
        frame_d1_nop[0*8 +: 8] = 8'hA5;
        frame_d1_nop[1*8 +: 8] = 8'h5A;
        frame_d1_nop[2*8 +: 8] = 8'hD1;
        frame_d1_nop[3*8 +: 8] = 8'd112;

        // Keep the seq equal to the point being acknowledged; cmd=NOP.
        frame_d1_nop[4*8 +: 8] = point_index[7:0];
        frame_d1_nop[5*8 +: 8] = point_index[15:8];
        frame_d1_nop[6*8 +: 8] = point_index[23:16];
        frame_d1_nop[7*8 +: 8] = point_index[31:24];

        d1_nop_chk = 8'd0;
        for (d1_nop_i = 0; d1_nop_i < 116; d1_nop_i = d1_nop_i + 1)
            d1_nop_chk = d1_nop_chk ^ frame_d1_nop[d1_nop_i*8 +: 8];
        frame_d1_nop[116*8 +: 8] = d1_nop_chk;
    end

    assign spi_b_tx_frame = (state == ST_WAIT_DDS_ACK) ? frame_d1_nop : frame_d1;

    spi_master_128b #(.CLK_DIV_HALF(SPI_B_CLK_DIV_HALF)) u_spi_b (
        .clk(clk_125m), .rst(rst),
        .start(spi_b_start), .tx_frame(spi_b_tx_frame),
        .miso(dds_spi_miso), .mosi(dds_spi_mosi),
        .sclk(dds_spi_sclk), .cs_n(dds_spi_cs_n),
        .rx_frame(spi_b_rx_frame), .busy(spi_b_busy), .done(spi_b_done)
    );

    // ============================================================
    // LEDs
    // ============================================================
    localparam [23:0] LED_BLINK_TICKS = 24'd12_500_000;

    reg [26:0] heartbeat_cnt = 27'd0;
    reg        spi_a_toggle = 1'b0;
    reg        spi_b_toggle = 1'b0;
    reg [23:0] err_blink_cnt = 24'd0;
    reg        any_err = 1'b0;

    always @(posedge clk_125m) begin
        if (rst) begin
            heartbeat_cnt <= 27'd0;
            spi_a_toggle <= 1'b0;
            spi_b_toggle <= 1'b0;
            err_blink_cnt <= 24'd0;
            any_err <= 1'b0;
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 27'd1;

            if (spi_a_done)
                spi_a_toggle <= ~spi_a_toggle;

            if (spi_b_done)
                spi_b_toggle <= ~spi_b_toggle;

            // Error detection
            any_err <= 1'b0;
            if (state == ST_WAIT_DDS_ACK && spi_b_timeout >= DDS_ACK_TIMEOUT_CLKS)
                any_err <= 1'b1;

            if (any_err)
                err_blink_cnt <= LED_BLINK_TICKS;
            else if (err_blink_cnt != 24'd0)
                err_blink_cnt <= err_blink_cnt - 24'd1;
        end
    end

    assign led0 = heartbeat_cnt[26];
    assign led1 = spi_a_toggle;
    assign led2 = spi_b_toggle;
    assign led3 = (err_blink_cnt != 24'd0);

endmodule
