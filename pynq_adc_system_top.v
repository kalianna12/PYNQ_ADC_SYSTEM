
module pynq_adc_system_top #(
    parameter integer SPI_A_CLK_DIV_HALF  = 7,        // 125MHz/(2*63)=992kHz
    parameter integer SPI_A_PERIOD_CLKS   = 12500000,    // 2ms poll period at 125MHz; stable systems may use 125000 for 1ms
    parameter integer SPI_B_CLK_DIV_HALF  = 2500,       // 125MHz/(2*2500)=25kHz
    parameter integer SETTLE_CLKS         = 1250000,    // 10ms settle
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
    output wire dds_spi_cs_n,

    // AD9226 A/B dual-channel capture
    output wire adc_a_clk,
    input  wire [11:0] adc_a_data,
    input  wire adc_a_ora,
    output wire adc_b_clk,
    input  wire [11:0] adc_b_data,
    input  wire adc_b_orb
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
    localparam [4:0] ST_IDLE              = 5'd0;
    localparam [4:0] ST_START_SCAN        = 5'd1;
    localparam [4:0] ST_SEND_DDS_FREQ     = 5'd2;
    localparam [4:0] ST_WAIT_DDS_ACK      = 5'd3;
    localparam [4:0] ST_WAIT_SETTLE       = 5'd4;
    localparam [4:0] ST_ADC_CAPTURE       = 5'd5;
    localparam [4:0] ST_SEND_RESULT       = 5'd6;
    localparam [4:0] ST_NEXT_FREQ         = 5'd7;
    localparam [4:0] ST_DONE              = 5'd8;
    localparam [4:0] ST_STOP              = 5'd9;
    localparam [4:0] ST_ERROR             = 5'd10;
    localparam [4:0] ST_ADC_TEST_SET_DDS  = 5'd11;
    localparam [4:0] ST_ADC_TEST_WAIT_ACK = 5'd12;
    localparam [4:0] ST_ADC_TEST_SETTLE   = 5'd13;
    localparam [4:0] ST_ADC_TEST_CAPTURE  = 5'd14;
    localparam [4:0] ST_ADC_TEST_ANALYZE  = 5'd15;
    localparam [4:0] ST_ADC_TEST_PREP_RESULT = 5'd16;
    localparam [4:0] ST_ADC_TEST_CHK_RESULT  = 5'd17;
    localparam [4:0] ST_ADC_TEST_SEND_RESULT = 5'd18;
    localparam [4:0] ST_ADC_TEST_PREP_WAVE_CHUNK = 5'd19;
    localparam [4:0] ST_ADC_TEST_CHK_WAVE = 5'd20;
    localparam [4:0] ST_ADC_TEST_SEND_WAVE= 5'd21;
    localparam [4:0] ST_ADC_TEST_DONE     = 5'd22;
    localparam [4:0] ST_ADC_TEST_ERROR    = 5'd23;
    localparam [1:0] DDS_MAX_ACK_RETRY  = 2'd3;

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
    localparam [31:0] CMD_ADV_CAPTURE    = 32'd20;
    localparam [31:0] CMD_ADV_RECONSTRUCT= 32'd21;
    localparam [31:0] CMD_ADV_SEND_TO_DDS= 32'd22;
    localparam [31:0] CMD_ADC_TEST_START = 32'd30;
    localparam [31:0] CMD_ADC_TEST_STOP  = 32'd31;

    localparam [31:0] DDS_CMD_NOP        = 32'd0;
    localparam [31:0] DDS_CMD_SET_FREQ   = 32'd1;
    localparam [31:0] DDS_CMD_SET_SINGLE = 32'd4;
    localparam [31:0] DDS_CMD_STOP       = 32'd3;

    // Formal baseline path uses DDS -> AD9226 A/B capture -> 0x13 raw-Vpp result.
    // Legacy ADC/raw-wave/fake paths are kept for reference but unreachable by default.
    localparam ENABLE_LEGACY_ADC_TEST = 1'b0;
    localparam ENABLE_RAW_WAVE_TX     = 1'b0;
    localparam ENABLE_FAKE_MEASURE    = 1'b0;

    reg [4:0]  state = ST_IDLE;
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
    reg        cmd_clear_flag   = 1'b0;
    reg        cmd_adc_test_start_flag = 1'b0;
    reg        cmd_adc_test_stop_flag  = 1'b0;
    reg        cmd_adv_capture_flag = 1'b0;
    reg        cmd_adv_reconstruct_flag = 1'b0;
    reg        cmd_adv_send_to_dds_flag = 1'b0;

    wire        calc_sweep_valid = (config_mode == MODE_SWEEP &&
                                    config_step_freq != 32'd0 &&
                                    config_stop_freq >= config_start_freq);
    wire [31:0] calc_sweep_span = config_stop_freq - config_start_freq;
    wire [31:0] calc_safe_step = (config_step_freq == 32'd0) ? 32'd1 : config_step_freq;
    wire [31:0] calc_sweep_base_points =
        calc_sweep_valid ? (calc_sweep_span / calc_safe_step + 32'd1) : 32'd1;
    wire        calc_sweep_has_remainder =
        calc_sweep_valid && ((calc_sweep_span % calc_safe_step) != 32'd0);
    wire [31:0] calc_total_points =
        calc_sweep_valid ?
        (calc_sweep_base_points + (calc_sweep_has_remainder ? 32'd1 : 32'd0)) :
        32'd1;
    wire [32:0] next_freq_sum = {1'b0, current_freq_hz} + {1'b0, config_step_freq};

    // Status registers (for 0x10 frame)
    reg [31:0] stat_state         = STATE_IDLE;
    reg [31:0] stat_mode          = MODE_SWEEP;
    reg [31:0] stat_filter_type   = 32'd0;  // Unknown in raw-Vpp baseline
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
    reg [31:0] stat_cutoff_freq   = 32'd0;

    // ESP command parser state
    reg [31:0] esp_cmd      = 32'd0;
    reg [31:0] esp_cmd_seq  = 32'd0;
    reg [31:0] esp_arg0     = 32'd0;
    reg [31:0] esp_last_seq = 32'hFFFFFFFF;
    reg        esp_cmd_valid = 1'b0;

    // SPI-A signals are declared before the main FSM because the FSM advances
    // ADC waveform chunks after completed SPI-A transfers.
    reg        spi_a_tx_was_wave = 1'b0;
    reg        spi_a_tx_was_adv = 1'b0;
    reg        spi_a_request_in_progress = 1'b0;
    wire       spi_a_busy;
    wire       spi_a_done;
    wire [1023:0] spi_a_rx_frame;

    // SPI-B control
    reg        spi_b_start = 1'b0;
    reg        spi_b_request_in_progress = 1'b0;
    reg [31:0] spi_b_timeout = 32'd0;
    reg [1:0]  dds_ack_retry_count = 2'd0;
    reg        stop_frame_sent = 1'b0;
    wire       spi_b_done;
    wire       spi_b_busy;
    wire [1023:0] spi_b_rx_frame;

    localparam [31:0] ADC_TEST_DDS_FREQ_HZ = 32'd1000;
    localparam [31:0] ADC_TEST_DDS_SEQ     = 32'hADC00001;

    wire adc_test_dds_state = ENABLE_LEGACY_ADC_TEST &&
        (state == ST_ADC_TEST_SET_DDS ||
         state == ST_ADC_TEST_WAIT_ACK ||
         state == ST_ADC_TEST_SETTLE ||
         state == ST_ADC_TEST_CAPTURE ||
         state == ST_ADC_TEST_SEND_WAVE ||
         state == ST_ADC_TEST_DONE);

    wire [31:0] dds_expected_cmd =
        adc_test_dds_state ? DDS_CMD_SET_FREQ :
        ((config_mode == MODE_SINGLE) ? DDS_CMD_SET_SINGLE : DDS_CMD_SET_FREQ);
    wire [31:0] dds_expected_seq =
        adc_test_dds_state ? ADC_TEST_DDS_SEQ : point_index;
    wire [31:0] dds_expected_freq =
        adc_test_dds_state ? ADC_TEST_DDS_FREQ_HZ : current_freq_hz;
    wire [31:0] dds_tx_cmd =
        (state == ST_STOP) ? DDS_CMD_STOP : dds_expected_cmd;
    wire [31:0] dds_tx_seq =
        (state == ST_STOP) ? (point_index | 32'h80000000) : dds_expected_seq;
    wire [31:0] dds_tx_freq =
        (state == ST_STOP) ? 32'd0 : dds_expected_freq;
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
        d2_ack_seq                 == dds_expected_seq &&
        d2_ack_cmd                 == dds_expected_cmd &&
        d2_ack_freq                == dds_expected_freq &&
        d2_ack_flags[1:0]          == 2'b11;

    // ============================================================
    // Disabled fake measurement placeholders
    // ============================================================
    wire [63:0] ratio_m10    = 64'd0;
    wire [31:0] fake_theory  = 32'd0;
    wire [31:0] fake_vout    = 32'd0;
    wire [31:0] fake_gain    = 32'd0;
    wire [31:0] fake_error   = 32'd0;
    wire [31:0] fake_phase   = 32'd0;

    integer ii;

    // ============================================================
    // AD9226 A/B dual-channel capture block
    // ============================================================
    localparam [31:0] ADC_A_SAMPLE_HZ = 32'd100000;
    localparam integer ADC_A_SAMPLE_COUNT = 4096;
    localparam [31:0] ADC_A_SAMPLE_COUNT_U32 = 32'd4096;
    localparam integer ADC_A_CLK_HALF_PERIOD_CLKS = 625;
    localparam integer ADC_A_SAMPLE_DELAY_CLKS = 40;
    localparam integer ADC_WAVE_SAMPLES_PER_CHUNK = 30;
    localparam [31:0] ADC_WAVE_CHUNK_COUNT =
        (ADC_A_SAMPLE_COUNT + ADC_WAVE_SAMPLES_PER_CHUNK - 1) / ADC_WAVE_SAMPLES_PER_CHUNK;
    localparam [31:0] ADC_WAVE_FLAGS_OVERRANGE = 32'h00000001;
    localparam [31:0] ADC_WAVE_FLAGS_DONE      = 32'h00000002;
    localparam [31:0] ADC_WAVE_FLAGS_VALID     = 32'h00000004;
    localparam [31:0] ADC_WAVE_FLAGS_RAW       = 32'h00000008;
    localparam [31:0] ADC_WAVE_FLAGS_CLIP      = 32'h00000010;
    localparam [31:0] ADC_WAVE_FLAGS_FREQ_VALID= 32'h00000020;
    localparam [31:0] ADC_WAVE_FLAGS_DFT_VALID = 32'h00000040;
    localparam [31:0] ADC_WAVE_FLAGS_ADC_RATE_CLAMPED = 32'h00000100;
    localparam [31:0] ADC_WAVE_FLAGS_PHASE_VALID      = 32'h00000200;
    localparam [31:0] ADC_WAVE_FLAGS_CORDIC_ORDER_UNCERTAIN = 32'h00000400;
    localparam [31:0] ADC_WAVE_CHUNK_GAP_CLKS  = 32'd0;
    localparam        ADC_WAVE_RAW_DEBUG       = 1'b1;
    localparam [31:0] ADC_DISPLAY_SAMPLE_COUNT_U32 = 32'd256;
    localparam [31:0] ADC_DISPLAY_DECIMATION = 32'd1;
    localparam [31:0] ADC_INPUT_CLK_HZ = 32'd125000000;
    localparam [15:0] ADC_HALF_PERIOD_MIN_CLKS = 16'd7;
    localparam [15:0] ADC_SYNC_SAMPLE_COUNT_CFG = 16'd4096;
    localparam [15:0] ADV_SAMPLE_COUNT_CFG = 16'd1024;
    localparam [31:0] ADV_SAMPLE_RATE_HZ = 32'd100000;
    localparam [2:0] ADV_IDLE = 3'd0;
    localparam [2:0] ADV_CAPTURE_START = 3'd1;
    localparam [2:0] ADV_CAPTURE_WAIT = 3'd2;
    localparam [2:0] ADV_RECON_START = 3'd3;
    localparam [2:0] ADV_RECON_WAIT = 3'd4;
    localparam [2:0] ADV_SEND_WAVE = 3'd5;
    localparam [2:0] ADV_SEND_STATUS = 3'd6;
    localparam [31:0] ADV_ERR_NONE = 32'd0;
    localparam [31:0] ADV_ERR_BUSY = 32'd1;
    localparam [31:0] ADV_ERR_NO_CAPTURE = 32'd2;
    localparam [31:0] ADV_ERR_NOT_IMPLEMENTED = 32'd3;

    reg [31:0] sync_samples_per_cycle_m = 32'd256;
    reg [11:0] sync_phase_step_cfg = 12'd16;
    reg [15:0] adc_half_period_clks_cfg = ADC_A_CLK_HALF_PERIOD_CLKS;
    reg [7:0]  adc_sample_delay_clks_cfg = ADC_A_SAMPLE_DELAY_CLKS;
    reg        adc_rate_clamped_cfg = 1'b0;
    reg [63:0] adc_cfg_denom = 64'd0;
    reg [63:0] adc_half_period_calc64 = 64'd0;

    always @* begin
        if (current_freq_hz < 32'd300) begin
            sync_samples_per_cycle_m = 32'd1024;
            sync_phase_step_cfg = 12'd4;
        end else if (current_freq_hz < 32'd1000) begin
            sync_samples_per_cycle_m = 32'd512;
            sync_phase_step_cfg = 12'd8;
        end else if (current_freq_hz < 32'd5000) begin
            sync_samples_per_cycle_m = 32'd256;
            sync_phase_step_cfg = 12'd16;
        end else if (current_freq_hz < 32'd20000) begin
            sync_samples_per_cycle_m = 32'd128;
            sync_phase_step_cfg = 12'd32;
        end else if (current_freq_hz < 32'd100000) begin
            sync_samples_per_cycle_m = 32'd64;
            sync_phase_step_cfg = 12'd64;
        end else if (current_freq_hz < 32'd250000) begin
            sync_samples_per_cycle_m = 32'd32;
            sync_phase_step_cfg = 12'd128;
        end else if (current_freq_hz < 32'd500000) begin
            sync_samples_per_cycle_m = 32'd16;
            sync_phase_step_cfg = 12'd256;
        end else begin
            sync_samples_per_cycle_m = 32'd8;
            sync_phase_step_cfg = 12'd512;
        end

        adc_cfg_denom =
            {32'd0, current_freq_hz} *
            {32'd0, sync_samples_per_cycle_m} *
            64'd2;

        if (adc_cfg_denom == 64'd0)
            adc_half_period_calc64 = ADC_A_CLK_HALF_PERIOD_CLKS;
        else
            adc_half_period_calc64 =
                ({32'd0, ADC_INPUT_CLK_HZ} + (adc_cfg_denom >> 1)) /
                adc_cfg_denom;

        adc_rate_clamped_cfg = 1'b0;
        if (adc_half_period_calc64 < ADC_HALF_PERIOD_MIN_CLKS) begin
            adc_half_period_clks_cfg = ADC_HALF_PERIOD_MIN_CLKS;
            adc_rate_clamped_cfg = 1'b1;
        end else if (adc_half_period_calc64 > 64'd65535) begin
            adc_half_period_clks_cfg = 16'hffff;
            adc_rate_clamped_cfg = 1'b1;
        end else begin
            adc_half_period_clks_cfg = adc_half_period_calc64[15:0];
        end

        if (adc_half_period_clks_cfg > 16'd80)
            adc_sample_delay_clks_cfg = ADC_A_SAMPLE_DELAY_CLKS;
        else if (adc_half_period_clks_cfg > 16'd4)
            adc_sample_delay_clks_cfg = adc_half_period_clks_cfg[8:1];
        else
            adc_sample_delay_clks_cfg = 8'd1;
    end

    reg adc_capture_start = 1'b0;
    reg adc_capture_started = 1'b0;
    reg adc_capture_done_latched = 1'b0;
    wire adc_capture_busy;
    wire adc_capture_done;
    wire [15:0] adc_sample_index;
    wire [11:0] adc_a_sample_raw;
    wire [11:0] adc_b_sample_raw;
    wire signed [15:0] adc_a_sample_mv = 16'sd0;
    wire adc_sample_valid;
    wire signed [15:0] adc_a_min_mv = 16'sd0;
    wire signed [15:0] adc_a_max_mv = 16'sd0;
    wire signed [31:0] adc_a_sum_mv = 32'sd0;
    wire [11:0] adc_a_min_raw;
    wire [11:0] adc_a_max_raw;
    wire [11:0] adc_a_vpp_raw;
    wire [31:0] adc_a_sum_raw;
    wire [11:0] adc_b_min_raw;
    wire [11:0] adc_b_max_raw;
    wire [11:0] adc_b_vpp_raw;
    wire [31:0] adc_b_sum_raw;
    wire adc_a_ora_sync;
    wire adc_b_orb_sync;
    wire adc_overrange_seen;
    wire adc_sync_busy;
    wire adc_sync_done;
    wire [31:0] adc_sync_amp_a_code;
    wire [31:0] adc_sync_amp_b_code;
    wire signed [31:0] adc_sync_phase_deg_x10;
    wire signed [31:0] adc_sync_phase_a_raw;
    wire signed [31:0] adc_sync_phase_b_raw;
    wire signed [31:0] adc_sync_phase_diff_raw;
    wire [31:0] adc_sync_debug_flags;

    reg [15:0] adc_sample_ram [0:ADC_A_SAMPLE_COUNT-1];
    reg [31:0] adc_wave_seq = 32'd0;
    reg [31:0] adc_wave_chunk_index = 32'd0;
    reg        adc_wave_send_active = 1'b0;
    reg signed [31:0] adc_wave_min_mv = 32'sd0;
    reg signed [31:0] adc_wave_max_mv = 32'sd0;
    reg signed [31:0] adc_wave_sum_mv = 32'sd0;
    reg signed [31:0] adc_wave_mean_mv = 32'sd0;
    reg signed [31:0] adc_wave_vpp_mv = 32'sd0;
    reg        adc_wave_overrange = 1'b0;
    reg        adc_result_send_active = 1'b0;
    reg [31:0] adc_result_flags = 32'd0;
    reg [31:0] adc_result_raw_rms = 32'd0;
    reg [31:0] adc_result_measured_freq_hz = 32'd0;
    reg [31:0] adc_result_amp_peak_raw = 32'd0;
    reg [31:0] adc_result_amp_rms_raw = 32'd0;
    reg signed [31:0] adc_result_phase_deg_x10 = 32'sd0;
    reg [31:0] adc_result_zero_cross_count = 32'd0;
    wire [31:0] adc_raw_mid = (adc_wave_min_mv + adc_wave_max_mv) >> 1;
    wire [31:0] adc_raw_vpp_u32 = {16'd0, adc_a_max_raw} - {16'd0, adc_a_min_raw};
    reg [1023:0] frame_0x12_reg = 1024'd0;
    reg [1023:0] frame_0x13_reg = 1024'd0;
    reg [7:0] adc_wave_prep_index = 8'd0;
    reg [31:0] adc_wave_prep_sample_index = 32'd0;
    reg [15:0] adc_wave_sample_for_frame = 16'd0;
    wire [31:0] adc_wave_flags_now =
        ADC_WAVE_FLAGS_VALID |
        ((adc_wave_chunk_index >= (ADC_WAVE_CHUNK_COUNT - 1)) ? ADC_WAVE_FLAGS_DONE : 32'd0) |
        (adc_wave_overrange ? ADC_WAVE_FLAGS_OVERRANGE : 32'd0) |
        (ADC_WAVE_RAW_DEBUG ? ADC_WAVE_FLAGS_RAW : 32'd0);

    reg [2:0] adv_state = ADV_IDLE;
    reg       adv_capture_active = 1'b0;
    reg       adv_capture_start = 1'b0;
    reg       adv_capture_clear = 1'b0;
    reg       adv_capture_finish = 1'b0;
    reg       adv_reconstruct_start = 1'b0;
    reg       adv_tx_pending = 1'b0;
    reg [1023:0] adv_tx_frame = 1024'd0;
    reg [31:0] adv_seq = 32'd0;
    reg [31:0] adv_error_code = ADV_ERR_NONE;
    reg [31:0] adv_wave_type = 32'd0;
    reg [31:0] adv_wave_chunk_index = 32'd0;
    reg [31:0] adv_capture_done_seen = 32'd0;
    reg [31:0] adv_recon_done_seen = 32'd0;

    wire adv_main_idle = (state == ST_IDLE || state == ST_DONE);
    wire adc_capture_start_any = adc_capture_start | adv_capture_start;
    wire [15:0] adc_sample_count_cfg_runtime =
        adv_capture_active ? ADV_SAMPLE_COUNT_CFG : ADC_SYNC_SAMPLE_COUNT_CFG;
    wire [15:0] adc_half_period_clks_runtime =
        adv_capture_active ? ADC_A_CLK_HALF_PERIOD_CLKS : adc_half_period_clks_cfg;
    wire [7:0] adc_sample_delay_clks_runtime =
        adv_capture_active ? ADC_A_SAMPLE_DELAY_CLKS : adc_sample_delay_clks_cfg;

    wire [31:0] adv_frame_chunk_count;
    wire [1023:0] adv_wave_frame;
    wire adv_core_capture_done;
    wire adv_core_recon_done;
    wire adv_core_busy;
    wire [31:0] adv_core_status_flags;
    wire signed [31:0] adv_y_min;
    wire signed [31:0] adv_y_max;
    wire signed [31:0] adv_y_mean;
    wire signed [31:0] adv_y_vpp;
    wire signed [31:0] adv_x_min;
    wire signed [31:0] adv_x_max;
    wire signed [31:0] adv_x_mean;
    wire signed [31:0] adv_x_vpp;
    wire [31:0] adv_capture_done_count;
    wire [31:0] adv_recon_done_count;
    wire [31:0] adv_fft_overflow_count;
    wire [31:0] adv_ifft_overflow_count;
    wire [31:0] adv_tlast_missing_count;
    wire [31:0] adv_tlast_unexpected_count;
    reg [1023:0] frame_0x14_status;
    reg [7:0] f14_chk;
    integer f14_i;

    ad9226_capture_dual #(
        .ADC_CLK_HALF_PERIOD_CLKS(ADC_A_CLK_HALF_PERIOD_CLKS),
        .SAMPLE_DELAY_CLKS(ADC_A_SAMPLE_DELAY_CLKS),
        .SAMPLE_COUNT(ADC_A_SAMPLE_COUNT)
    ) u_ad9226_capture_dual (
        .clk(clk_125m),
        .rst(rst),
        .start(adc_capture_start_any),
        .adc_half_period_clks_cfg(adc_half_period_clks_runtime),
        .sample_delay_clks_cfg(adc_sample_delay_clks_runtime),
        .sample_count_cfg(adc_sample_count_cfg_runtime),
        .busy(adc_capture_busy),
        .done(adc_capture_done),
        .adc_a_clk(adc_a_clk),
        .adc_b_clk(adc_b_clk),
        .adc_a_data(adc_a_data),
        .adc_b_data(adc_b_data),
        .adc_a_ora(adc_a_ora),
        .adc_b_orb(adc_b_orb),
        .sample_index(adc_sample_index),
        .sample_a_raw(adc_a_sample_raw),
        .sample_b_raw(adc_b_sample_raw),
        .sample_valid(adc_sample_valid),
        .min_a_raw(adc_a_min_raw),
        .max_a_raw(adc_a_max_raw),
        .vpp_a_raw(adc_a_vpp_raw),
        .sum_a_raw(adc_a_sum_raw),
        .min_b_raw(adc_b_min_raw),
        .max_b_raw(adc_b_max_raw),
        .vpp_b_raw(adc_b_vpp_raw),
        .sum_b_raw(adc_b_sum_raw),
        .adc_a_ora_sync(adc_a_ora_sync),
        .adc_b_orb_sync(adc_b_orb_sync),
        .overrange_seen(adc_overrange_seen)
    );

    sync_detector #(
        .DEFAULT_SAMPLE_COUNT(ADC_A_SAMPLE_COUNT),
        .CORDIC_INPUT_SHIFT(9),
        .CORDIC_XY_SWAP(1'b0),
        .CORDIC_OUT_SWAP(1'b0),
        .PHASE_SIGN_INVERT(1'b0)
    ) u_sync_detector (
        .clk(clk_125m),
        .rst(rst),
        .start(adc_capture_start_any),
        .sample_valid(adc_sample_valid),
        .sample_a_raw(adc_a_sample_raw),
        .sample_b_raw(adc_b_sample_raw),
        .phase_step_cfg(sync_phase_step_cfg),
        .sample_count_cfg(adc_sample_count_cfg_runtime),
        .busy(adc_sync_busy),
        .done(adc_sync_done),
        .amp_a_code(adc_sync_amp_a_code),
        .amp_b_code(adc_sync_amp_b_code),
        .phase_deg_x10(adc_sync_phase_deg_x10),
        .phase_a_raw(adc_sync_phase_a_raw),
        .phase_b_raw(adc_sync_phase_b_raw),
        .phase_diff_raw(adc_sync_phase_diff_raw),
        .debug_flags(adc_sync_debug_flags)
    );

    adv_h0_bypass_core #(
        .FFT_N(1024),
        .SAMPLE_RATE_HZ(ADV_SAMPLE_RATE_HZ),
        .SAMPLES_PER_CHUNK(30)
    ) u_adv_h0_bypass_core (
        .clk(clk_125m),
        .rst(rst),
        .capture_clear(adv_capture_clear),
        .capture_enable(adv_capture_active),
        .capture_finish(adv_capture_finish),
        .sample_valid(adc_sample_valid),
        .sample_index(adc_sample_index),
        .sample_b_raw(adc_b_sample_raw),
        .reconstruct_start(adv_reconstruct_start),
        .frame_seq(adv_seq),
        .frame_wave_type(adv_wave_type),
        .frame_chunk_index(adv_wave_chunk_index),
        .frame_chunk_count(adv_frame_chunk_count),
        .wave_frame(adv_wave_frame),
        .capture_done(adv_core_capture_done),
        .recon_done(adv_core_recon_done),
        .busy(adv_core_busy),
        .status_flags(adv_core_status_flags),
        .y_min(adv_y_min),
        .y_max(adv_y_max),
        .y_mean(adv_y_mean),
        .y_vpp(adv_y_vpp),
        .x_min(adv_x_min),
        .x_max(adv_x_max),
        .x_mean(adv_x_mean),
        .x_vpp(adv_x_vpp),
        .capture_done_count(adv_capture_done_count),
        .recon_done_count(adv_recon_done_count),
        .fft_overflow_count(adv_fft_overflow_count),
        .ifft_overflow_count(adv_ifft_overflow_count),
        .tlast_missing_count(adv_tlast_missing_count),
        .tlast_unexpected_count(adv_tlast_unexpected_count)
    );

    always @(posedge clk_125m) begin
        if (rst) begin
            for (ii = 0; ii < ADC_A_SAMPLE_COUNT; ii = ii + 1)
                adc_sample_ram[ii] <= 16'd0;
        end else if (adc_sample_valid && adc_sample_index < ADC_A_SAMPLE_COUNT) begin
            if (ADC_WAVE_RAW_DEBUG)
                adc_sample_ram[adc_sample_index] <= {4'd0, adc_a_sample_raw};
            else
                adc_sample_ram[adc_sample_index] <= adc_a_sample_mv;
        end
    end

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
            cmd_clear_flag <= 1'b0;
            cmd_adc_test_start_flag <= 1'b0;
            cmd_adc_test_stop_flag <= 1'b0;
            cmd_adv_capture_flag <= 1'b0;
            cmd_adv_reconstruct_flag <= 1'b0;
            cmd_adv_send_to_dds_flag <= 1'b0;
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
            stat_filter_type <= 32'd0;
            stat_cutoff_freq <= 32'd0;
            stat_link_ok <= 32'd1;
            spi_b_timeout <= 32'd0;
            dds_ack_retry_count <= 2'd0;
            stop_frame_sent <= 1'b0;
            adc_capture_start <= 1'b0;
            adc_capture_started <= 1'b0;
            adc_capture_done_latched <= 1'b0;
            adc_wave_seq <= 32'd0;
            adc_wave_chunk_index <= 32'd0;
            adc_wave_send_active <= 1'b0;
            adc_wave_min_mv <= 32'sd0;
            adc_wave_max_mv <= 32'sd0;
            adc_wave_sum_mv <= 32'sd0;
            adc_wave_mean_mv <= 32'sd0;
            adc_wave_vpp_mv <= 32'sd0;
            adc_wave_overrange <= 1'b0;
            adc_result_send_active <= 1'b0;
            adc_result_flags <= 32'd0;
            adc_result_raw_rms <= 32'd0;
            adc_result_measured_freq_hz <= 32'd0;
            adc_result_amp_peak_raw <= 32'd0;
            adc_result_amp_rms_raw <= 32'd0;
            adc_result_phase_deg_x10 <= 32'sd0;
            adc_result_zero_cross_count <= 32'd0;
            frame_0x12_reg <= 1024'd0;
            frame_0x13_reg <= 1024'd0;
            adc_wave_prep_index <= 8'd0;
            adc_wave_prep_sample_index <= 32'd0;
            adv_state <= ADV_IDLE;
            adv_capture_active <= 1'b0;
            adv_capture_start <= 1'b0;
            adv_capture_clear <= 1'b0;
            adv_capture_finish <= 1'b0;
            adv_reconstruct_start <= 1'b0;
            adv_tx_pending <= 1'b0;
            adv_tx_frame <= 1024'd0;
            adv_seq <= 32'd0;
            adv_error_code <= ADV_ERR_NONE;
            adv_wave_type <= 32'd0;
            adv_wave_chunk_index <= 32'd0;
            adv_capture_done_seen <= 32'd0;
            adv_recon_done_seen <= 32'd0;
        end else begin
            adc_capture_start <= 1'b0;
            adv_capture_start <= 1'b0;
            adv_capture_clear <= 1'b0;
            adv_capture_finish <= 1'b0;
            adv_reconstruct_start <= 1'b0;

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
                    CMD_CLEAR_TABLE: begin
                        cmd_clear_flag <= 1'b1;
                        cmd_start_flag <= 1'b0;
                    end
                    CMD_ADV_CAPTURE: begin
                        cmd_adv_capture_flag <= 1'b1;
                    end
                    CMD_ADV_RECONSTRUCT: begin
                        cmd_adv_reconstruct_flag <= 1'b1;
                    end
                    CMD_ADV_SEND_TO_DDS: begin
                        cmd_adv_send_to_dds_flag <= 1'b1;
                    end
                    CMD_ADC_TEST_START: begin
                        if (ENABLE_LEGACY_ADC_TEST) begin
                            cmd_adc_test_start_flag <= 1'b1;
                            cmd_adc_test_stop_flag <= 1'b0;
                            cmd_start_flag <= 1'b0;
                        end
                    end
                    CMD_ADC_TEST_STOP: begin
                        if (ENABLE_LEGACY_ADC_TEST) begin
                            cmd_adc_test_stop_flag <= 1'b1;
                            cmd_adc_test_start_flag <= 1'b0;
                        end
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
                    end else if (ENABLE_LEGACY_ADC_TEST && cmd_adc_test_start_flag) begin
                        cmd_adc_test_start_flag <= 1'b0;
                        current_freq_hz <= ADC_TEST_DDS_FREQ_HZ;
                        point_index <= 32'd1;
                        total_points <= 32'd1;
                        stat_state <= STATE_SCANNING;
                        stat_mode <= MODE_SINGLE;
                        stat_current_freq <= ADC_TEST_DDS_FREQ_HZ;
                        stat_point_index <= 32'd0;
                        stat_total_points <= ADC_A_SAMPLE_COUNT;
                        stat_progress <= 32'd0;
                        stat_link_ok <= 32'd1;
                        spi_b_timeout <= 32'd0;
                        dds_ack_retry_count <= 2'd0;
                        adc_wave_send_active <= 1'b0;
                        adc_wave_chunk_index <= 32'd0;
                        state <= ST_ADC_TEST_SET_DDS;
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
                    dds_ack_retry_count <= 2'd0;
                    stop_frame_sent <= 1'b0;
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
                            dds_ack_retry_count <= 2'd0;
                            stat_link_ok <= 32'd1;
                        end else if (dds_ack_retry_count < DDS_MAX_ACK_RETRY) begin
                            dds_ack_retry_count <= dds_ack_retry_count + 2'd1;
                            spi_b_timeout <= 32'd0;
                            state <= ST_SEND_DDS_FREQ;
                        end else begin
                            stat_state <= STATE_ERROR;
                            stat_link_ok <= 32'd0;
                            state <= ST_ERROR;
                        end
                    end else if (spi_b_timeout >= DDS_ACK_TIMEOUT_CLKS) begin
                        if (dds_ack_retry_count < DDS_MAX_ACK_RETRY) begin
                            dds_ack_retry_count <= dds_ack_retry_count + 2'd1;
                            spi_b_timeout <= 32'd0;
                            state <= ST_SEND_DDS_FREQ;
                        end else begin
                            stat_state <= STATE_ERROR;
                            stat_link_ok <= 32'd0;
                            state <= ST_ERROR;
                        end
                    end else begin
                        spi_b_timeout <= spi_b_timeout + 32'd1;
                    end
                end

                ST_WAIT_SETTLE: begin
                    if (settle_cnt >= SETTLE_CLKS) begin
                        adc_capture_started <= 1'b0;
                        state <= ST_ADC_CAPTURE;
                    end else begin
                        settle_cnt <= settle_cnt + 32'd1;
                    end
                end

                ST_ADC_CAPTURE: begin
                    if (!adc_capture_started) begin
                        adc_capture_start <= 1'b1;
                        adc_capture_started <= 1'b1;
                        adc_capture_done_latched <= 1'b0;
                    end else begin
                        if (adc_capture_done)
                            adc_capture_done_latched <= 1'b1;

                        if ((adc_capture_done || adc_capture_done_latched) && adc_sync_done) begin
                            stat_current_freq <= current_freq_hz;
                            stat_point_index <= point_index;
                            stat_vin_mv <= {20'd0, adc_a_vpp_raw};
                            stat_vout_mv <= {20'd0, adc_b_vpp_raw};
                            stat_gain_x1000 <= 32'd0;
                            stat_theory_gain <= 32'd0;
                            stat_error_x10 <= 32'd0;
                            stat_phase_deg_x10 <= adc_sync_phase_deg_x10;
                            stat_filter_type <= 32'd0;
                            stat_cutoff_freq <= 32'd0;
                            stat_progress <= (point_index * 32'd1000) / total_points;
                            adc_wave_seq <= adc_wave_seq + 32'd1;
                            adc_result_measured_freq_hz <= current_freq_hz;
                            // DFT/CORDIC magnitude is relative for now; raw Vpp below
                            // remains the calibrated ESP32 display path.
                            adc_result_amp_peak_raw <= adc_sync_amp_a_code;
                            adc_result_amp_rms_raw <= adc_sync_amp_b_code;
                            adc_result_phase_deg_x10 <= adc_sync_phase_deg_x10;
                            adc_result_flags <= ADC_WAVE_FLAGS_VALID |
                                                ADC_WAVE_FLAGS_DONE |
                                                ADC_WAVE_FLAGS_FREQ_VALID |
                                                ADC_WAVE_FLAGS_DFT_VALID |
                                                ADC_WAVE_FLAGS_PHASE_VALID |
                                                (adc_rate_clamped_cfg ? ADC_WAVE_FLAGS_ADC_RATE_CLAMPED : 32'd0) |
                                                (adc_overrange_seen ? ADC_WAVE_FLAGS_OVERRANGE : 32'd0) |
                                                ((adc_a_min_raw == 12'd0 || adc_a_max_raw == 12'd4095 ||
                                                  adc_b_min_raw == 12'd0 || adc_b_max_raw == 12'd4095) ? ADC_WAVE_FLAGS_CLIP : 32'd0) |
                                                {8'd0, adc_sync_debug_flags[7:0], 16'd0};
                            adc_result_send_active <= 1'b1;
                            adc_capture_started <= 1'b0;
                            adc_capture_done_latched <= 1'b0;
                            state <= ST_SEND_RESULT;
                        end
                    end
                end

                ST_SEND_RESULT: begin
                    if (!adc_result_send_active)
                        state <= ST_NEXT_FREQ;
                end

                ST_NEXT_FREQ: begin
                    if (config_mode == MODE_SINGLE ||
                        point_index >= total_points ||
                        current_freq_hz >= config_stop_freq) begin
                        state <= ST_DONE;
                    end else begin
                        if (next_freq_sum >= {1'b0, config_stop_freq})
                            current_freq_hz <= config_stop_freq;
                        else
                            current_freq_hz <= next_freq_sum[31:0];
                        point_index <= point_index + 32'd1;
                        dds_ack_retry_count <= 2'd0;
                        state <= ST_SEND_DDS_FREQ;
                    end
                end

                ST_DONE: begin
                    stat_state <= STATE_DONE;
                    stat_progress <= 32'd1000;
                    stat_cutoff_freq <= 32'd0;
                    stat_filter_type <= 32'd0;
                    // Stay in DONE until new START
                    if (cmd_start_flag) begin
                        cmd_start_flag <= 1'b0;
                        state <= ST_START_SCAN;
                    end else if (ENABLE_LEGACY_ADC_TEST && cmd_adc_test_start_flag) begin
                        cmd_adc_test_start_flag <= 1'b0;
                        current_freq_hz <= ADC_TEST_DDS_FREQ_HZ;
                        point_index <= 32'd1;
                        total_points <= 32'd1;
                        stat_state <= STATE_SCANNING;
                        stat_mode <= MODE_SINGLE;
                        stat_current_freq <= ADC_TEST_DDS_FREQ_HZ;
                        stat_point_index <= 32'd0;
                        stat_total_points <= ADC_A_SAMPLE_COUNT;
                        stat_progress <= 32'd0;
                        stat_link_ok <= 32'd1;
                        spi_b_timeout <= 32'd0;
                        dds_ack_retry_count <= 2'd0;
                        adc_wave_send_active <= 1'b0;
                        adc_wave_chunk_index <= 32'd0;
                        state <= ST_ADC_TEST_SET_DDS;
                    end
                end

                ST_STOP: begin
                    stat_state <= STATE_IDLE;
                    stat_link_ok <= 32'd1;
                    stat_progress <= 32'd0;
                    stat_current_freq <= 32'd0;
                    stat_point_index <= 32'd0;
                    stat_total_points <= 32'd0;
                    stat_vin_mv <= 32'd0;
                    stat_vout_mv <= 32'd0;
                    stat_gain_x1000 <= 32'd0;
                    stat_theory_gain <= 32'd0;
                    stat_error_x10 <= 32'd0;
                    stat_phase_deg_x10 <= 32'd0;
                    cmd_stop_flag <= 1'b0;
                    cmd_start_flag <= 1'b0;
                    spi_b_timeout <= 32'd0;
                    dds_ack_retry_count <= 2'd0;

                    if (!stop_frame_sent && !spi_b_busy && !spi_b_request_in_progress) begin
                        stop_frame_sent <= 1'b1;
                    end else if (stop_frame_sent && spi_b_done) begin
                        stop_frame_sent <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                ST_ERROR: begin
                    stat_state <= STATE_ERROR;
                    stat_link_ok <= 32'd0;

                    if (cmd_start_flag) begin
                        cmd_start_flag <= 1'b0;
                        stat_link_ok <= 32'd1;
                        dds_ack_retry_count <= 2'd0;
                        state <= ST_START_SCAN;
                    end
                end

                ST_ADC_TEST_SET_DDS: begin
                    stat_state <= STATE_SCANNING;
                    stat_current_freq <= ADC_TEST_DDS_FREQ_HZ;
                    if (spi_b_done) begin
                        spi_b_timeout <= 32'd0;
                        state <= ST_ADC_TEST_WAIT_ACK;
                    end
                end

                ST_ADC_TEST_WAIT_ACK: begin
                    if (spi_b_done) begin
                        if (dds_ack_valid) begin
                            settle_cnt <= 32'd0;
                            spi_b_timeout <= 32'd0;
                            dds_ack_retry_count <= 2'd0;
                            stat_link_ok <= 32'd1;
                            state <= ST_ADC_TEST_SETTLE;
                        end else if (dds_ack_retry_count < DDS_MAX_ACK_RETRY) begin
                            dds_ack_retry_count <= dds_ack_retry_count + 2'd1;
                            spi_b_timeout <= 32'd0;
                            state <= ST_ADC_TEST_SET_DDS;
                        end else begin
                            stat_state <= STATE_ERROR;
                            stat_link_ok <= 32'd0;
                            state <= ST_ADC_TEST_ERROR;
                        end
                    end else if (spi_b_timeout >= DDS_ACK_TIMEOUT_CLKS) begin
                        if (dds_ack_retry_count < DDS_MAX_ACK_RETRY) begin
                            dds_ack_retry_count <= dds_ack_retry_count + 2'd1;
                            spi_b_timeout <= 32'd0;
                            state <= ST_ADC_TEST_SET_DDS;
                        end else begin
                            stat_state <= STATE_ERROR;
                            stat_link_ok <= 32'd0;
                            state <= ST_ADC_TEST_ERROR;
                        end
                    end else begin
                        spi_b_timeout <= spi_b_timeout + 32'd1;
                    end
                end

                ST_ADC_TEST_SETTLE: begin
                    if (settle_cnt >= DDS_ACK_TIMEOUT_CLKS) begin
                        adc_capture_start <= 1'b1;
                        adc_capture_started <= 1'b1;
                        stat_progress <= 32'd250;
                        state <= ST_ADC_TEST_CAPTURE;
                    end else begin
                        settle_cnt <= settle_cnt + 32'd1;
                    end
                end

                ST_ADC_TEST_CAPTURE: begin
                    if (adc_capture_done) begin
                        adc_wave_seq <= adc_wave_seq + 32'd1;
                        adc_wave_chunk_index <= 32'd0;
                        adc_wave_send_active <= 1'b0;
                        if (ADC_WAVE_RAW_DEBUG) begin
                            adc_wave_min_mv <= {4'd0, adc_a_min_raw};
                            adc_wave_max_mv <= {4'd0, adc_a_max_raw};
                            adc_wave_sum_mv <= adc_a_sum_raw;
                            adc_wave_mean_mv <= adc_a_sum_raw / ADC_A_SAMPLE_COUNT;
                            adc_wave_vpp_mv <= {20'd0, adc_a_vpp_raw};
                            stat_vin_mv <= {20'd0, adc_a_vpp_raw};
                            stat_vout_mv <= {20'd0, adc_b_vpp_raw};
                        end else begin
                            adc_wave_min_mv <= adc_a_min_mv;
                            adc_wave_max_mv <= adc_a_max_mv;
                            adc_wave_sum_mv <= adc_a_sum_mv;
                            adc_wave_mean_mv <= adc_a_sum_mv / ADC_A_SAMPLE_COUNT;
                            adc_wave_vpp_mv <= {{16{adc_a_max_mv[15]}}, adc_a_max_mv} -
                                               {{16{adc_a_min_mv[15]}}, adc_a_min_mv};
                            stat_vin_mv <= adc_a_sum_mv / ADC_A_SAMPLE_COUNT;
                            stat_vout_mv <= {{16{adc_a_max_mv[15]}}, adc_a_max_mv} -
                                            {{16{adc_a_min_mv[15]}}, adc_a_min_mv};
                        end
                        adc_wave_overrange <= adc_overrange_seen;
                        adc_result_flags <= ADC_WAVE_FLAGS_VALID |
                                            ADC_WAVE_FLAGS_DONE |
                                            ADC_WAVE_FLAGS_RAW |
                                            ADC_WAVE_FLAGS_FREQ_VALID |
                                            ADC_WAVE_FLAGS_DFT_VALID |
                                            (adc_overrange_seen ? ADC_WAVE_FLAGS_OVERRANGE : 32'd0) |
                                            ((adc_a_min_raw == 12'd0 || adc_a_max_raw == 12'd4095) ? ADC_WAVE_FLAGS_CLIP : 32'd0);
                        adc_result_raw_rms <= adc_a_sum_raw / ADC_A_SAMPLE_COUNT;
                        adc_result_measured_freq_hz <= ADC_TEST_DDS_FREQ_HZ;
                        adc_result_amp_peak_raw <= {20'd0, adc_a_vpp_raw};
                        adc_result_amp_rms_raw <= {20'd0, adc_b_vpp_raw};
                        adc_result_phase_deg_x10 <= 32'sd0;
                        adc_result_zero_cross_count <= 32'd3;
                        stat_gain_x1000 <= 32'd0;
                        stat_phase_deg_x10 <= 32'd0;
                        stat_progress <= 32'd700;
                        adc_capture_started <= 1'b0;
                        state <= ST_ADC_TEST_ANALYZE;
                    end
                end

                ST_ADC_TEST_ANALYZE: begin
                    stat_state <= STATE_SCANNING;
                    stat_progress <= 32'd800;
                    state <= ST_ADC_TEST_PREP_RESULT;
                end

                ST_ADC_TEST_PREP_RESULT: begin
                    stat_state <= STATE_SCANNING;
                    stat_progress <= 32'd850;
                    frame_0x13_reg <= 1024'd0;
                    frame_0x13_reg[0*8 +: 8] <= 8'hA5;
                    frame_0x13_reg[1*8 +: 8] <= 8'h5A;
                    frame_0x13_reg[2*8 +: 8] <= 8'h13;
                    frame_0x13_reg[3*8 +: 8] <= 8'd112;
                    frame_0x13_reg[4*8 +: 8] <= adc_wave_seq[7:0];
                    frame_0x13_reg[5*8 +: 8] <= adc_wave_seq[15:8];
                    frame_0x13_reg[6*8 +: 8] <= adc_wave_seq[23:16];
                    frame_0x13_reg[7*8 +: 8] <= adc_wave_seq[31:24];
                    frame_0x13_reg[8*8 +: 8] <= ADC_A_SAMPLE_HZ[7:0];
                    frame_0x13_reg[9*8 +: 8] <= ADC_A_SAMPLE_HZ[15:8];
                    frame_0x13_reg[10*8 +: 8] <= ADC_A_SAMPLE_HZ[23:16];
                    frame_0x13_reg[11*8 +: 8] <= ADC_A_SAMPLE_HZ[31:24];
                    frame_0x13_reg[12*8 +: 8] <= ADC_TEST_DDS_FREQ_HZ[7:0];
                    frame_0x13_reg[13*8 +: 8] <= ADC_TEST_DDS_FREQ_HZ[15:8];
                    frame_0x13_reg[14*8 +: 8] <= ADC_TEST_DDS_FREQ_HZ[23:16];
                    frame_0x13_reg[15*8 +: 8] <= ADC_TEST_DDS_FREQ_HZ[31:24];
                    frame_0x13_reg[16*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[7:0];
                    frame_0x13_reg[17*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[15:8];
                    frame_0x13_reg[18*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[23:16];
                    frame_0x13_reg[19*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[31:24];
                    frame_0x13_reg[20*8 +: 8] <= ADC_DISPLAY_SAMPLE_COUNT_U32[7:0];
                    frame_0x13_reg[21*8 +: 8] <= ADC_DISPLAY_SAMPLE_COUNT_U32[15:8];
                    frame_0x13_reg[22*8 +: 8] <= ADC_DISPLAY_SAMPLE_COUNT_U32[23:16];
                    frame_0x13_reg[23*8 +: 8] <= ADC_DISPLAY_SAMPLE_COUNT_U32[31:24];
                    frame_0x13_reg[24*8 +: 8] <= adc_result_measured_freq_hz[7:0];
                    frame_0x13_reg[25*8 +: 8] <= adc_result_measured_freq_hz[15:8];
                    frame_0x13_reg[26*8 +: 8] <= adc_result_measured_freq_hz[23:16];
                    frame_0x13_reg[27*8 +: 8] <= adc_result_measured_freq_hz[31:24];
                    frame_0x13_reg[28*8 +: 8] <= adc_wave_min_mv[7:0];
                    frame_0x13_reg[29*8 +: 8] <= adc_wave_min_mv[15:8];
                    frame_0x13_reg[30*8 +: 8] <= adc_wave_min_mv[23:16];
                    frame_0x13_reg[31*8 +: 8] <= adc_wave_min_mv[31:24];
                    frame_0x13_reg[32*8 +: 8] <= adc_wave_max_mv[7:0];
                    frame_0x13_reg[33*8 +: 8] <= adc_wave_max_mv[15:8];
                    frame_0x13_reg[34*8 +: 8] <= adc_wave_max_mv[23:16];
                    frame_0x13_reg[35*8 +: 8] <= adc_wave_max_mv[31:24];
                    frame_0x13_reg[36*8 +: 8] <= adc_wave_mean_mv[7:0];
                    frame_0x13_reg[37*8 +: 8] <= adc_wave_mean_mv[15:8];
                    frame_0x13_reg[38*8 +: 8] <= adc_wave_mean_mv[23:16];
                    frame_0x13_reg[39*8 +: 8] <= adc_wave_mean_mv[31:24];
                    frame_0x13_reg[40*8 +: 8] <= adc_wave_vpp_mv[7:0];
                    frame_0x13_reg[41*8 +: 8] <= adc_wave_vpp_mv[15:8];
                    frame_0x13_reg[42*8 +: 8] <= adc_wave_vpp_mv[23:16];
                    frame_0x13_reg[43*8 +: 8] <= adc_wave_vpp_mv[31:24];
                    frame_0x13_reg[44*8 +: 8] <= adc_result_raw_rms[7:0];
                    frame_0x13_reg[45*8 +: 8] <= adc_result_raw_rms[15:8];
                    frame_0x13_reg[46*8 +: 8] <= adc_result_raw_rms[23:16];
                    frame_0x13_reg[47*8 +: 8] <= adc_result_raw_rms[31:24];
                    frame_0x13_reg[48*8 +: 8] <= adc_result_amp_peak_raw[7:0];
                    frame_0x13_reg[49*8 +: 8] <= adc_result_amp_peak_raw[15:8];
                    frame_0x13_reg[50*8 +: 8] <= adc_result_amp_peak_raw[23:16];
                    frame_0x13_reg[51*8 +: 8] <= adc_result_amp_peak_raw[31:24];
                    frame_0x13_reg[52*8 +: 8] <= adc_result_amp_rms_raw[7:0];
                    frame_0x13_reg[53*8 +: 8] <= adc_result_amp_rms_raw[15:8];
                    frame_0x13_reg[54*8 +: 8] <= adc_result_amp_rms_raw[23:16];
                    frame_0x13_reg[55*8 +: 8] <= adc_result_amp_rms_raw[31:24];
                    frame_0x13_reg[56*8 +: 8] <= adc_result_phase_deg_x10[7:0];
                    frame_0x13_reg[57*8 +: 8] <= adc_result_phase_deg_x10[15:8];
                    frame_0x13_reg[58*8 +: 8] <= adc_result_phase_deg_x10[23:16];
                    frame_0x13_reg[59*8 +: 8] <= adc_result_phase_deg_x10[31:24];
                    frame_0x13_reg[60*8 +: 8] <= adc_result_flags[7:0];
                    frame_0x13_reg[61*8 +: 8] <= adc_result_flags[15:8];
                    frame_0x13_reg[62*8 +: 8] <= adc_result_flags[23:16];
                    frame_0x13_reg[63*8 +: 8] <= adc_result_flags[31:24];
                    frame_0x13_reg[64*8 +: 8] <= ADC_DISPLAY_DECIMATION[7:0];
                    frame_0x13_reg[65*8 +: 8] <= ADC_DISPLAY_DECIMATION[15:8];
                    frame_0x13_reg[66*8 +: 8] <= ADC_DISPLAY_DECIMATION[23:16];
                    frame_0x13_reg[67*8 +: 8] <= ADC_DISPLAY_DECIMATION[31:24];
                    frame_0x13_reg[68*8 +: 8] <= adc_result_zero_cross_count[7:0];
                    frame_0x13_reg[69*8 +: 8] <= adc_result_zero_cross_count[15:8];
                    frame_0x13_reg[70*8 +: 8] <= adc_result_zero_cross_count[23:16];
                    frame_0x13_reg[71*8 +: 8] <= adc_result_zero_cross_count[31:24];
                    state <= ST_ADC_TEST_CHK_RESULT;
                end

                ST_ADC_TEST_CHK_RESULT: begin
                    frame_0x13_reg[116*8 +: 8] <=
                        frame_0x13_reg[0*8+:8]  ^ frame_0x13_reg[1*8+:8]  ^ frame_0x13_reg[2*8+:8]  ^ frame_0x13_reg[3*8+:8]  ^
                        frame_0x13_reg[4*8+:8]  ^ frame_0x13_reg[5*8+:8]  ^ frame_0x13_reg[6*8+:8]  ^ frame_0x13_reg[7*8+:8]  ^
                        frame_0x13_reg[8*8+:8]  ^ frame_0x13_reg[9*8+:8]  ^ frame_0x13_reg[10*8+:8] ^ frame_0x13_reg[11*8+:8] ^
                        frame_0x13_reg[12*8+:8] ^ frame_0x13_reg[13*8+:8] ^ frame_0x13_reg[14*8+:8] ^ frame_0x13_reg[15*8+:8] ^
                        frame_0x13_reg[16*8+:8] ^ frame_0x13_reg[17*8+:8] ^ frame_0x13_reg[18*8+:8] ^ frame_0x13_reg[19*8+:8] ^
                        frame_0x13_reg[20*8+:8] ^ frame_0x13_reg[21*8+:8] ^ frame_0x13_reg[22*8+:8] ^ frame_0x13_reg[23*8+:8] ^
                        frame_0x13_reg[24*8+:8] ^ frame_0x13_reg[25*8+:8] ^ frame_0x13_reg[26*8+:8] ^ frame_0x13_reg[27*8+:8] ^
                        frame_0x13_reg[28*8+:8] ^ frame_0x13_reg[29*8+:8] ^ frame_0x13_reg[30*8+:8] ^ frame_0x13_reg[31*8+:8] ^
                        frame_0x13_reg[32*8+:8] ^ frame_0x13_reg[33*8+:8] ^ frame_0x13_reg[34*8+:8] ^ frame_0x13_reg[35*8+:8] ^
                        frame_0x13_reg[36*8+:8] ^ frame_0x13_reg[37*8+:8] ^ frame_0x13_reg[38*8+:8] ^ frame_0x13_reg[39*8+:8] ^
                        frame_0x13_reg[40*8+:8] ^ frame_0x13_reg[41*8+:8] ^ frame_0x13_reg[42*8+:8] ^ frame_0x13_reg[43*8+:8] ^
                        frame_0x13_reg[44*8+:8] ^ frame_0x13_reg[45*8+:8] ^ frame_0x13_reg[46*8+:8] ^ frame_0x13_reg[47*8+:8] ^
                        frame_0x13_reg[48*8+:8] ^ frame_0x13_reg[49*8+:8] ^ frame_0x13_reg[50*8+:8] ^ frame_0x13_reg[51*8+:8] ^
                        frame_0x13_reg[52*8+:8] ^ frame_0x13_reg[53*8+:8] ^ frame_0x13_reg[54*8+:8] ^ frame_0x13_reg[55*8+:8] ^
                        frame_0x13_reg[56*8+:8] ^ frame_0x13_reg[57*8+:8] ^ frame_0x13_reg[58*8+:8] ^ frame_0x13_reg[59*8+:8] ^
                        frame_0x13_reg[60*8+:8] ^ frame_0x13_reg[61*8+:8] ^ frame_0x13_reg[62*8+:8] ^ frame_0x13_reg[63*8+:8] ^
                        frame_0x13_reg[64*8+:8] ^ frame_0x13_reg[65*8+:8] ^ frame_0x13_reg[66*8+:8] ^ frame_0x13_reg[67*8+:8] ^
                        frame_0x13_reg[68*8+:8] ^ frame_0x13_reg[69*8+:8] ^ frame_0x13_reg[70*8+:8] ^ frame_0x13_reg[71*8+:8] ^
                        frame_0x13_reg[72*8+:8] ^ frame_0x13_reg[73*8+:8] ^ frame_0x13_reg[74*8+:8] ^ frame_0x13_reg[75*8+:8] ^
                        frame_0x13_reg[76*8+:8] ^ frame_0x13_reg[77*8+:8] ^ frame_0x13_reg[78*8+:8] ^ frame_0x13_reg[79*8+:8] ^
                        frame_0x13_reg[80*8+:8] ^ frame_0x13_reg[81*8+:8] ^ frame_0x13_reg[82*8+:8] ^ frame_0x13_reg[83*8+:8] ^
                        frame_0x13_reg[84*8+:8] ^ frame_0x13_reg[85*8+:8] ^ frame_0x13_reg[86*8+:8] ^ frame_0x13_reg[87*8+:8] ^
                        frame_0x13_reg[88*8+:8] ^ frame_0x13_reg[89*8+:8] ^ frame_0x13_reg[90*8+:8] ^ frame_0x13_reg[91*8+:8] ^
                        frame_0x13_reg[92*8+:8] ^ frame_0x13_reg[93*8+:8] ^ frame_0x13_reg[94*8+:8] ^ frame_0x13_reg[95*8+:8] ^
                        frame_0x13_reg[96*8+:8] ^ frame_0x13_reg[97*8+:8] ^ frame_0x13_reg[98*8+:8] ^ frame_0x13_reg[99*8+:8] ^
                        frame_0x13_reg[100*8+:8]^ frame_0x13_reg[101*8+:8]^ frame_0x13_reg[102*8+:8]^ frame_0x13_reg[103*8+:8]^
                        frame_0x13_reg[104*8+:8]^ frame_0x13_reg[105*8+:8]^ frame_0x13_reg[106*8+:8]^ frame_0x13_reg[107*8+:8]^
                        frame_0x13_reg[108*8+:8]^ frame_0x13_reg[109*8+:8]^ frame_0x13_reg[110*8+:8]^ frame_0x13_reg[111*8+:8]^
                        frame_0x13_reg[112*8+:8]^ frame_0x13_reg[113*8+:8]^ frame_0x13_reg[114*8+:8]^ frame_0x13_reg[115*8+:8];
                    adc_result_send_active <= 1'b1;
                    state <= ST_ADC_TEST_SEND_RESULT;
                end

                ST_ADC_TEST_SEND_RESULT: begin
                    stat_state <= STATE_SCANNING;
                    stat_progress <= 32'd875;
                    if (!adc_result_send_active) begin
                        stat_progress <= 32'd1000;
                        state <= ST_ADC_TEST_DONE;
                    end
                end

                ST_ADC_TEST_PREP_WAVE_CHUNK: begin
                    stat_state <= STATE_SCANNING;
                    stat_progress <= 32'd900;
                    if (adc_wave_prep_index == 8'd0) begin
                        frame_0x12_reg <= 1024'd0;
                        frame_0x12_reg[0*8 +: 8] <= 8'hA5;  frame_0x12_reg[1*8 +: 8] <= 8'h5A;
                        frame_0x12_reg[2*8 +: 8] <= 8'h12;   frame_0x12_reg[3*8 +: 8] <= 8'd112;
                        frame_0x12_reg[4*8 +: 8] <= adc_wave_seq[7:0];      frame_0x12_reg[5*8 +: 8] <= adc_wave_seq[15:8];
                        frame_0x12_reg[6*8 +: 8] <= adc_wave_seq[23:16];    frame_0x12_reg[7*8 +: 8] <= adc_wave_seq[31:24];
                        frame_0x12_reg[8*8 +: 8] <= adc_wave_chunk_index[7:0];   frame_0x12_reg[9*8 +: 8] <= adc_wave_chunk_index[15:8];
                        frame_0x12_reg[10*8 +: 8] <= adc_wave_chunk_index[23:16]; frame_0x12_reg[11*8 +: 8] <= adc_wave_chunk_index[31:24];
                        frame_0x12_reg[12*8 +: 8] <= ADC_WAVE_CHUNK_COUNT[7:0];   frame_0x12_reg[13*8 +: 8] <= ADC_WAVE_CHUNK_COUNT[15:8];
                        frame_0x12_reg[14*8 +: 8] <= ADC_WAVE_CHUNK_COUNT[23:16]; frame_0x12_reg[15*8 +: 8] <= ADC_WAVE_CHUNK_COUNT[31:24];
                        frame_0x12_reg[16*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[7:0]; frame_0x12_reg[17*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[15:8];
                        frame_0x12_reg[18*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[23:16];frame_0x12_reg[19*8 +: 8] <= ADC_A_SAMPLE_COUNT_U32[31:24];
                        frame_0x12_reg[20*8 +: 8] <= (adc_wave_chunk_index * ADC_WAVE_SAMPLES_PER_CHUNK) & 8'hFF;
                        frame_0x12_reg[21*8 +: 8] <= ((adc_wave_chunk_index * ADC_WAVE_SAMPLES_PER_CHUNK) >> 8) & 8'hFF;
                        frame_0x12_reg[22*8 +: 8] <= ((adc_wave_chunk_index * ADC_WAVE_SAMPLES_PER_CHUNK) >> 16) & 8'hFF;
                        frame_0x12_reg[23*8 +: 8] <= ((adc_wave_chunk_index * ADC_WAVE_SAMPLES_PER_CHUNK) >> 24) & 8'hFF;
                        frame_0x12_reg[24*8 +: 8] <= adc_wave_flags_now[7:0];   frame_0x12_reg[25*8 +: 8] <= adc_wave_flags_now[15:8];
                        frame_0x12_reg[26*8 +: 8] <= adc_wave_flags_now[23:16]; frame_0x12_reg[27*8 +: 8] <= adc_wave_flags_now[31:24];
                    end else begin
                        adc_wave_prep_sample_index = adc_wave_chunk_index * ADC_WAVE_SAMPLES_PER_CHUNK +
                            (adc_wave_prep_index - 8'd1);

                        if (adc_wave_prep_sample_index < ADC_A_SAMPLE_COUNT)
                            adc_wave_sample_for_frame = adc_sample_ram[adc_wave_prep_sample_index];
                        else
                            adc_wave_sample_for_frame = 16'd0;

                        frame_0x12_reg[(32 + (adc_wave_prep_index-8'd1)*2)*8 +: 8] <= adc_wave_sample_for_frame[7:0];
                        frame_0x12_reg[(33 + (adc_wave_prep_index-8'd1)*2)*8 +: 8] <= adc_wave_sample_for_frame[15:8];
                    end
                    if (adc_wave_prep_index >= ADC_WAVE_SAMPLES_PER_CHUNK[7:0]) begin
                        adc_wave_prep_index <= 8'd0;
                        state <= ST_ADC_TEST_CHK_WAVE;
                    end else begin
                        adc_wave_prep_index <= adc_wave_prep_index + 8'd1;
                    end
                end

                ST_ADC_TEST_CHK_WAVE: begin
                    frame_0x12_reg[116*8 +: 8] <=
                        frame_0x12_reg[0*8+:8]  ^ frame_0x12_reg[1*8+:8]  ^ frame_0x12_reg[2*8+:8]  ^ frame_0x12_reg[3*8+:8]  ^
                        frame_0x12_reg[4*8+:8]  ^ frame_0x12_reg[5*8+:8]  ^ frame_0x12_reg[6*8+:8]  ^ frame_0x12_reg[7*8+:8]  ^
                        frame_0x12_reg[8*8+:8]  ^ frame_0x12_reg[9*8+:8]  ^ frame_0x12_reg[10*8+:8] ^ frame_0x12_reg[11*8+:8] ^
                        frame_0x12_reg[12*8+:8] ^ frame_0x12_reg[13*8+:8] ^ frame_0x12_reg[14*8+:8] ^ frame_0x12_reg[15*8+:8] ^
                        frame_0x12_reg[16*8+:8] ^ frame_0x12_reg[17*8+:8] ^ frame_0x12_reg[18*8+:8] ^ frame_0x12_reg[19*8+:8] ^
                        frame_0x12_reg[20*8+:8] ^ frame_0x12_reg[21*8+:8] ^ frame_0x12_reg[22*8+:8] ^ frame_0x12_reg[23*8+:8] ^
                        frame_0x12_reg[24*8+:8] ^ frame_0x12_reg[25*8+:8] ^ frame_0x12_reg[26*8+:8] ^ frame_0x12_reg[27*8+:8] ^
                        frame_0x12_reg[28*8+:8] ^ frame_0x12_reg[29*8+:8] ^ frame_0x12_reg[30*8+:8] ^ frame_0x12_reg[31*8+:8] ^
                        frame_0x12_reg[32*8+:8] ^ frame_0x12_reg[33*8+:8] ^ frame_0x12_reg[34*8+:8] ^ frame_0x12_reg[35*8+:8] ^
                        frame_0x12_reg[36*8+:8] ^ frame_0x12_reg[37*8+:8] ^ frame_0x12_reg[38*8+:8] ^ frame_0x12_reg[39*8+:8] ^
                        frame_0x12_reg[40*8+:8] ^ frame_0x12_reg[41*8+:8] ^ frame_0x12_reg[42*8+:8] ^ frame_0x12_reg[43*8+:8] ^
                        frame_0x12_reg[44*8+:8] ^ frame_0x12_reg[45*8+:8] ^ frame_0x12_reg[46*8+:8] ^ frame_0x12_reg[47*8+:8] ^
                        frame_0x12_reg[48*8+:8] ^ frame_0x12_reg[49*8+:8] ^ frame_0x12_reg[50*8+:8] ^ frame_0x12_reg[51*8+:8] ^
                        frame_0x12_reg[52*8+:8] ^ frame_0x12_reg[53*8+:8] ^ frame_0x12_reg[54*8+:8] ^ frame_0x12_reg[55*8+:8] ^
                        frame_0x12_reg[56*8+:8] ^ frame_0x12_reg[57*8+:8] ^ frame_0x12_reg[58*8+:8] ^ frame_0x12_reg[59*8+:8] ^
                        frame_0x12_reg[60*8+:8] ^ frame_0x12_reg[61*8+:8] ^ frame_0x12_reg[62*8+:8] ^ frame_0x12_reg[63*8+:8] ^
                        frame_0x12_reg[64*8+:8] ^ frame_0x12_reg[65*8+:8] ^ frame_0x12_reg[66*8+:8] ^ frame_0x12_reg[67*8+:8] ^
                        frame_0x12_reg[68*8+:8] ^ frame_0x12_reg[69*8+:8] ^ frame_0x12_reg[70*8+:8] ^ frame_0x12_reg[71*8+:8] ^
                        frame_0x12_reg[72*8+:8] ^ frame_0x12_reg[73*8+:8] ^ frame_0x12_reg[74*8+:8] ^ frame_0x12_reg[75*8+:8] ^
                        frame_0x12_reg[76*8+:8] ^ frame_0x12_reg[77*8+:8] ^ frame_0x12_reg[78*8+:8] ^ frame_0x12_reg[79*8+:8] ^
                        frame_0x12_reg[80*8+:8] ^ frame_0x12_reg[81*8+:8] ^ frame_0x12_reg[82*8+:8] ^ frame_0x12_reg[83*8+:8] ^
                        frame_0x12_reg[84*8+:8] ^ frame_0x12_reg[85*8+:8] ^ frame_0x12_reg[86*8+:8] ^ frame_0x12_reg[87*8+:8] ^
                        frame_0x12_reg[88*8+:8] ^ frame_0x12_reg[89*8+:8] ^ frame_0x12_reg[90*8+:8] ^ frame_0x12_reg[91*8+:8] ^
                        frame_0x12_reg[92*8+:8] ^ frame_0x12_reg[93*8+:8] ^ frame_0x12_reg[94*8+:8] ^ frame_0x12_reg[95*8+:8] ^
                        frame_0x12_reg[96*8+:8] ^ frame_0x12_reg[97*8+:8] ^ frame_0x12_reg[98*8+:8] ^ frame_0x12_reg[99*8+:8] ^
                        frame_0x12_reg[100*8+:8]^ frame_0x12_reg[101*8+:8]^ frame_0x12_reg[102*8+:8]^ frame_0x12_reg[103*8+:8]^
                        frame_0x12_reg[104*8+:8]^ frame_0x12_reg[105*8+:8]^ frame_0x12_reg[106*8+:8]^ frame_0x12_reg[107*8+:8]^
                        frame_0x12_reg[108*8+:8]^ frame_0x12_reg[109*8+:8]^ frame_0x12_reg[110*8+:8]^ frame_0x12_reg[111*8+:8]^
                        frame_0x12_reg[112*8+:8]^ frame_0x12_reg[113*8+:8]^ frame_0x12_reg[114*8+:8]^ frame_0x12_reg[115*8+:8];
                    state <= ST_ADC_TEST_SEND_WAVE;
                end

                ST_ADC_TEST_SEND_WAVE: begin
                    stat_state <= STATE_SCANNING;
                    stat_progress <= 32'd900;
                    if (!adc_wave_send_active) begin
                        stat_progress <= 32'd1000;
                        state <= ST_ADC_TEST_DONE;
                    end
                end

                ST_ADC_TEST_DONE: begin
                    stat_state <= STATE_DONE;
                    if (cmd_adc_test_start_flag) begin
                        cmd_adc_test_start_flag <= 1'b0;
                        adc_wave_send_active <= 1'b0;
                        adc_wave_chunk_index <= 32'd0;
                        state <= ST_ADC_TEST_SET_DDS;
                    end
                end

                ST_ADC_TEST_ERROR: begin
                    stat_state <= STATE_ERROR;
                    stat_link_ok <= 32'd0;
                    if (cmd_adc_test_start_flag) begin
                        cmd_adc_test_start_flag <= 1'b0;
                        stat_link_ok <= 32'd1;
                        dds_ack_retry_count <= 2'd0;
                        state <= ST_ADC_TEST_SET_DDS;
                    end
                end

                default: state <= ST_IDLE;
            endcase

            // Global STOP check. Let ST_STOP send one DDS_CMD_STOP frame before
            // returning to idle.
            if (cmd_stop_flag && state != ST_STOP) begin
                stop_frame_sent <= 1'b0;
                state <= ST_STOP;
            end

            if (ENABLE_LEGACY_ADC_TEST && cmd_adc_test_stop_flag) begin
                cmd_adc_test_stop_flag <= 1'b0;
                adc_wave_send_active <= 1'b0;
                adc_wave_chunk_index <= 32'd0;
                state <= ST_STOP;
            end else if (!ENABLE_LEGACY_ADC_TEST && cmd_adc_test_stop_flag) begin
                cmd_adc_test_stop_flag <= 1'b0;
            end

            if (spi_a_done && adc_result_send_active)
                adc_result_send_active <= 1'b0;

            if (ENABLE_RAW_WAVE_TX && spi_a_done && spi_a_tx_was_wave && adc_wave_send_active) begin
                if (adc_wave_chunk_index >= (ADC_WAVE_CHUNK_COUNT - 1)) begin
                    adc_wave_chunk_index <= 32'd0;
                    adc_wave_send_active <= 1'b0;
                end else begin
                    adc_wave_chunk_index <= adc_wave_chunk_index + 32'd1;
                    state <= ST_ADC_TEST_PREP_WAVE_CHUNK;
                end
            end

            if (spi_a_done && spi_a_tx_was_adv && adv_tx_pending) begin
                adv_tx_pending <= 1'b0;
                if (adv_state == ADV_SEND_WAVE) begin
                    if (adv_wave_chunk_index >= (adv_frame_chunk_count - 1)) begin
                        adv_wave_chunk_index <= 32'd0;
                        adv_state <= ADV_IDLE;
                    end else begin
                        adv_wave_chunk_index <= adv_wave_chunk_index + 32'd1;
                    end
                end else if (adv_state == ADV_SEND_STATUS) begin
                    adv_state <= ADV_IDLE;
                end
            end

            case (adv_state)
                ADV_IDLE: begin
                    if (cmd_adv_capture_flag) begin
                        cmd_adv_capture_flag <= 1'b0;
                        adv_error_code <= ADV_ERR_NONE;
                        if (adv_main_idle && !adc_capture_busy && !adv_core_busy) begin
                            adv_capture_clear <= 1'b1;
                            adv_capture_start <= 1'b1;
                            adv_capture_active <= 1'b1;
                            adv_capture_done_seen <= 32'd0;
                            adv_recon_done_seen <= 32'd0;
                            adv_state <= ADV_CAPTURE_WAIT;
                        end else begin
                            adv_error_code <= ADV_ERR_BUSY;
                            adv_state <= ADV_SEND_STATUS;
                        end
                    end else if (cmd_adv_reconstruct_flag) begin
                        cmd_adv_reconstruct_flag <= 1'b0;
                        adv_error_code <= ADV_ERR_NONE;
                        if (!adv_core_capture_done) begin
                            adv_error_code <= ADV_ERR_NO_CAPTURE;
                            adv_state <= ADV_SEND_STATUS;
                        end else if (adv_core_busy) begin
                            adv_error_code <= ADV_ERR_BUSY;
                            adv_state <= ADV_SEND_STATUS;
                        end else begin
                            adv_reconstruct_start <= 1'b1;
                            adv_recon_done_seen <= 32'd0;
                            adv_state <= ADV_RECON_WAIT;
                        end
                    end else if (cmd_adv_send_to_dds_flag) begin
                        cmd_adv_send_to_dds_flag <= 1'b0;
                        adv_error_code <= ADV_ERR_NOT_IMPLEMENTED;
                        adv_state <= ADV_SEND_STATUS;
                    end
                end

                ADV_CAPTURE_WAIT: begin
                    if (adc_capture_done) begin
                        adv_capture_active <= 1'b0;
                        adv_capture_finish <= 1'b1;
                        adv_capture_done_seen <= 32'd1;
                        adv_wave_type <= 32'd0;
                        adv_wave_chunk_index <= 32'd0;
                        adv_error_code <= ADV_ERR_NONE;
                        adv_state <= ADV_SEND_WAVE;
                    end
                end

                ADV_RECON_WAIT: begin
                    if (adv_core_recon_done) begin
                        adv_recon_done_seen <= 32'd1;
                        adv_wave_type <= 32'd1;
                        adv_wave_chunk_index <= 32'd0;
                        adv_error_code <= ADV_ERR_NONE;
                        adv_state <= ADV_SEND_WAVE;
                    end
                end

                ADV_SEND_WAVE: begin
                    if (!adv_tx_pending) begin
                        adv_seq <= adv_seq + 32'd1;
                        adv_tx_frame <= adv_wave_frame;
                        adv_tx_pending <= 1'b1;
                    end
                end

                ADV_SEND_STATUS: begin
                    if (!adv_tx_pending) begin
                        adv_seq <= adv_seq + 32'd1;
                        adv_tx_frame <= frame_0x14_status;
                        adv_tx_pending <= 1'b1;
                    end
                end

                default: adv_state <= ADV_IDLE;
            endcase

            // CLEAR resets the local measurement/status cache and can recover
            // from Error without touching ESP32-P4's saved H(f) model.
            if (cmd_clear_flag) begin
                cmd_clear_flag <= 1'b0;
                cmd_stop_flag <= 1'b0;
                cmd_start_flag <= 1'b0;
                state <= ST_IDLE;
                settle_cnt <= 32'd0;
                current_freq_hz <= 32'd0;
                point_index <= 32'd0;
                total_points <= 32'd0;
                stat_state <= STATE_IDLE;
                stat_progress <= 32'd0;
                stat_current_freq <= 32'd0;
                stat_point_index <= 32'd0;
                stat_total_points <= 32'd0;
                stat_vin_mv <= 32'd0;
                stat_vout_mv <= 32'd0;
                stat_gain_x1000 <= 32'd0;
                stat_theory_gain <= 32'd0;
                stat_error_x10 <= 32'd0;
                stat_phase_deg_x10 <= 32'd0;
                stat_link_ok <= 32'd1;
                spi_b_timeout <= 32'd0;
                dds_ack_retry_count <= 2'd0;
                stop_frame_sent <= 1'b0;
                adc_wave_send_active <= 1'b0;
                adc_wave_chunk_index <= 32'd0;
                adc_wave_overrange <= 1'b0;
                adc_capture_done_latched <= 1'b0;
                cmd_adv_capture_flag <= 1'b0;
                cmd_adv_reconstruct_flag <= 1'b0;
                cmd_adv_send_to_dds_flag <= 1'b0;
                adv_state <= ADV_IDLE;
                adv_capture_active <= 1'b0;
                adv_capture_clear <= 1'b1;
                adv_tx_pending <= 1'b0;
                adv_wave_chunk_index <= 32'd0;
                adv_capture_done_seen <= 32'd0;
                adv_recon_done_seen <= 32'd0;
                adv_error_code <= ADV_ERR_NONE;
            end
        end
    end

    // 0x14 Advanced status frame:
    // byte 4 seq, 8 adv_state, 12 error_code, 16 flags,
    // byte 20 sample_rate, 24 total_samples, 28 capture_done_count,
    // byte 32 recon_done_count, 36 fwd overflow count, 40 inv overflow count,
    // byte 44 tlast_missing_count, 48 tlast_unexpected_count.
    always @* begin
        frame_0x14_status = 1024'd0;
        frame_0x14_status[0*8 +: 8] = 8'hA5;
        frame_0x14_status[1*8 +: 8] = 8'h5A;
        frame_0x14_status[2*8 +: 8] = 8'h14;
        frame_0x14_status[3*8 +: 8] = 8'd112;

        frame_0x14_status[4*8 +: 8] = adv_seq[7:0];
        frame_0x14_status[5*8 +: 8] = adv_seq[15:8];
        frame_0x14_status[6*8 +: 8] = adv_seq[23:16];
        frame_0x14_status[7*8 +: 8] = adv_seq[31:24];

        frame_0x14_status[8*8 +: 8] = {5'd0, adv_state};
        frame_0x14_status[12*8 +: 8] = adv_error_code[7:0];
        frame_0x14_status[13*8 +: 8] = adv_error_code[15:8];
        frame_0x14_status[14*8 +: 8] = adv_error_code[23:16];
        frame_0x14_status[15*8 +: 8] = adv_error_code[31:24];

        frame_0x14_status[16*8 +: 8] = adv_core_status_flags[7:0];
        frame_0x14_status[17*8 +: 8] = adv_core_status_flags[15:8];
        frame_0x14_status[18*8 +: 8] = adv_core_status_flags[23:16];
        frame_0x14_status[19*8 +: 8] = adv_core_status_flags[31:24];

        frame_0x14_status[20*8 +: 8] = ADV_SAMPLE_RATE_HZ[7:0];
        frame_0x14_status[21*8 +: 8] = ADV_SAMPLE_RATE_HZ[15:8];
        frame_0x14_status[22*8 +: 8] = ADV_SAMPLE_RATE_HZ[23:16];
        frame_0x14_status[23*8 +: 8] = ADV_SAMPLE_RATE_HZ[31:24];
        frame_0x14_status[24*8 +: 8] = ADV_SAMPLE_COUNT_CFG[7:0];
        frame_0x14_status[25*8 +: 8] = ADV_SAMPLE_COUNT_CFG[15:8];

        frame_0x14_status[28*8 +: 8] = adv_capture_done_count[7:0];
        frame_0x14_status[29*8 +: 8] = adv_capture_done_count[15:8];
        frame_0x14_status[30*8 +: 8] = adv_capture_done_count[23:16];
        frame_0x14_status[31*8 +: 8] = adv_capture_done_count[31:24];
        frame_0x14_status[32*8 +: 8] = adv_recon_done_count[7:0];
        frame_0x14_status[33*8 +: 8] = adv_recon_done_count[15:8];
        frame_0x14_status[34*8 +: 8] = adv_recon_done_count[23:16];
        frame_0x14_status[35*8 +: 8] = adv_recon_done_count[31:24];
        frame_0x14_status[36*8 +: 8] = adv_fft_overflow_count[7:0];
        frame_0x14_status[37*8 +: 8] = adv_fft_overflow_count[15:8];
        frame_0x14_status[38*8 +: 8] = adv_fft_overflow_count[23:16];
        frame_0x14_status[39*8 +: 8] = adv_fft_overflow_count[31:24];
        frame_0x14_status[40*8 +: 8] = adv_ifft_overflow_count[7:0];
        frame_0x14_status[41*8 +: 8] = adv_ifft_overflow_count[15:8];
        frame_0x14_status[42*8 +: 8] = adv_ifft_overflow_count[23:16];
        frame_0x14_status[43*8 +: 8] = adv_ifft_overflow_count[31:24];
        frame_0x14_status[44*8 +: 8] = adv_tlast_missing_count[7:0];
        frame_0x14_status[45*8 +: 8] = adv_tlast_missing_count[15:8];
        frame_0x14_status[46*8 +: 8] = adv_tlast_missing_count[23:16];
        frame_0x14_status[47*8 +: 8] = adv_tlast_missing_count[31:24];
        frame_0x14_status[48*8 +: 8] = adv_tlast_unexpected_count[7:0];
        frame_0x14_status[49*8 +: 8] = adv_tlast_unexpected_count[15:8];
        frame_0x14_status[50*8 +: 8] = adv_tlast_unexpected_count[23:16];
        frame_0x14_status[51*8 +: 8] = adv_tlast_unexpected_count[31:24];

        f14_chk = 8'd0;
        for (f14_i = 0; f14_i < 116; f14_i = f14_i + 1)
            f14_chk = f14_chk ^ frame_0x14_status[f14_i*8 +: 8];
        frame_0x14_status[116*8 +: 8] = f14_chk;
    end

    // ============================================================
    // SPI-A to ESP32-P4 (periodic, about 2ms)
    // ============================================================
    reg [31:0] spi_a_period_cnt = 32'd0;
    reg [31:0] adc_wave_chunk_gap_cnt = 32'd0;
    reg        spi_a_start = 1'b0;
    wire       adc_result_tx_pending = adc_result_send_active;
    wire       adv_tx_ready_pending = adv_tx_pending;
    wire       adc_wave_tx_pending = ENABLE_RAW_WAVE_TX && adc_wave_send_active &&
                                     (state == ST_ADC_TEST_SEND_WAVE);
    wire [1023:0] spi_a_tx_frame;

    always @(posedge clk_125m) begin
        if (rst) begin
            spi_a_period_cnt <= 32'd0;
            adc_wave_chunk_gap_cnt <= ADC_WAVE_CHUNK_GAP_CLKS;
            spi_a_start <= 1'b0;
            spi_a_tx_was_wave <= 1'b0;
            spi_a_tx_was_adv <= 1'b0;
            spi_a_request_in_progress <= 1'b0;
        end else begin
            spi_a_start <= 1'b0;

            if (spi_a_done)
                spi_a_request_in_progress <= 1'b0;

            if (!adc_wave_tx_pending) begin
                adc_wave_chunk_gap_cnt <= ADC_WAVE_CHUNK_GAP_CLKS;
            end else if (adc_wave_chunk_gap_cnt < ADC_WAVE_CHUNK_GAP_CLKS) begin
                adc_wave_chunk_gap_cnt <= adc_wave_chunk_gap_cnt + 32'd1;
            end

            if (!spi_a_request_in_progress && !spi_a_busy) begin
                if (adc_result_tx_pending) begin
                    spi_a_start <= 1'b1;
                    spi_a_tx_was_wave <= 1'b0;
                    spi_a_tx_was_adv <= 1'b0;
                    spi_a_request_in_progress <= 1'b1;
                    spi_a_period_cnt <= 32'd0;
                end else if (adv_tx_ready_pending) begin
                    spi_a_start <= 1'b1;
                    spi_a_tx_was_wave <= 1'b0;
                    spi_a_tx_was_adv <= 1'b1;
                    spi_a_request_in_progress <= 1'b1;
                    spi_a_period_cnt <= 32'd0;
                end else if (adc_wave_tx_pending && adc_wave_chunk_gap_cnt >= ADC_WAVE_CHUNK_GAP_CLKS) begin
                    spi_a_start <= 1'b1;
                    spi_a_tx_was_wave <= 1'b1;
                    spi_a_tx_was_adv <= 1'b0;
                    spi_a_request_in_progress <= 1'b1;
                    adc_wave_chunk_gap_cnt <= 32'd0;
                    spi_a_period_cnt <= 32'd0;
                end else if (!adc_wave_tx_pending && spi_a_period_cnt >= (SPI_A_PERIOD_CLKS - 1)) begin
                    spi_a_start <= 1'b1;
                    spi_a_tx_was_wave <= 1'b0;
                    spi_a_tx_was_adv <= 1'b0;
                    spi_a_request_in_progress <= 1'b1;
                    spi_a_period_cnt <= 32'd0;
                end else if (!adc_wave_tx_pending) begin
                    spi_a_period_cnt <= spi_a_period_cnt + 32'd1;
                end
            end else if (!adc_wave_tx_pending && spi_a_period_cnt < (SPI_A_PERIOD_CLKS - 1)) begin
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

    // 0x12 frame_0x12_reg and 0x13 frame_0x13_reg are built in
    // ST_ADC_TEST_PREP_WAVE_CHUNK / ST_ADC_TEST_PREP_RESULT.

    // 0x13 point result frame, 128-byte SPI transaction:
    // byte 0=A5, 1=5A, 2=13, 3=112
    // byte 4  seq u32 LE
    // byte 8  point_index u32 LE, 1-based in this PYNQADC implementation
    // byte 12 total_points u32 LE
    // byte 16 freq_req_hz u32 LE
    // byte 20 freq_actual_hz u32 LE
    // byte 24 amp_a_code u32 LE, sync_detector CORDIC magnitude for channel A
    // byte 28 amp_b_code u32 LE, sync_detector CORDIC magnitude for channel B
    // byte 32 raw_a_vpp_code u32 LE
    // byte 36 raw_b_vpp_code u32 LE
    // byte 40 gain_x1000 i32 LE, current baseline 0; ESP32 computes gain
    // byte 44 phase_deg_x10 i32 LE, sync_detector phase difference B-A
    // byte 48 flags u32 LE. Existing low bits are preserved:
    // bit0 overrange, bit1 done, bit2 valid, bit3 raw, bit4 clip,
    // bit5 freq_valid, bit6 dft_valid. Added bits: bit8 adc_rate_clamped,
    // bit9 phase_valid, bit10 cordic_order_uncertain. Bits 16..23 carry
    // sync_detector debug_flags[7:0].
    // byte 52..115 reserved 0, byte 116 checksum xor(frame[0..115]), 117..127 padding 0
    reg [1023:0] frame_0x13_live;
    reg [7:0] f13_chk;
    integer f13_i;

    always @* begin
        frame_0x13_live = 1024'd0;
        frame_0x13_live[0*8 +: 8] = 8'hA5;
        frame_0x13_live[1*8 +: 8] = 8'h5A;
        frame_0x13_live[2*8 +: 8] = 8'h13;
        frame_0x13_live[3*8 +: 8] = 8'd112;

        frame_0x13_live[4*8 +: 8] = adc_wave_seq[7:0];
        frame_0x13_live[5*8 +: 8] = adc_wave_seq[15:8];
        frame_0x13_live[6*8 +: 8] = adc_wave_seq[23:16];
        frame_0x13_live[7*8 +: 8] = adc_wave_seq[31:24];

        frame_0x13_live[8*8 +: 8] = point_index[7:0];
        frame_0x13_live[9*8 +: 8] = point_index[15:8];
        frame_0x13_live[10*8 +: 8] = point_index[23:16];
        frame_0x13_live[11*8 +: 8] = point_index[31:24];

        frame_0x13_live[12*8 +: 8] = total_points[7:0];
        frame_0x13_live[13*8 +: 8] = total_points[15:8];
        frame_0x13_live[14*8 +: 8] = total_points[23:16];
        frame_0x13_live[15*8 +: 8] = total_points[31:24];

        frame_0x13_live[16*8 +: 8] = current_freq_hz[7:0];
        frame_0x13_live[17*8 +: 8] = current_freq_hz[15:8];
        frame_0x13_live[18*8 +: 8] = current_freq_hz[23:16];
        frame_0x13_live[19*8 +: 8] = current_freq_hz[31:24];

        frame_0x13_live[20*8 +: 8] = adc_result_measured_freq_hz[7:0];
        frame_0x13_live[21*8 +: 8] = adc_result_measured_freq_hz[15:8];
        frame_0x13_live[22*8 +: 8] = adc_result_measured_freq_hz[23:16];
        frame_0x13_live[23*8 +: 8] = adc_result_measured_freq_hz[31:24];

        frame_0x13_live[24*8 +: 8] = adc_result_amp_peak_raw[7:0];
        frame_0x13_live[25*8 +: 8] = adc_result_amp_peak_raw[15:8];
        frame_0x13_live[26*8 +: 8] = adc_result_amp_peak_raw[23:16];
        frame_0x13_live[27*8 +: 8] = adc_result_amp_peak_raw[31:24];

        frame_0x13_live[28*8 +: 8] = adc_result_amp_rms_raw[7:0];
        frame_0x13_live[29*8 +: 8] = adc_result_amp_rms_raw[15:8];
        frame_0x13_live[30*8 +: 8] = adc_result_amp_rms_raw[23:16];
        frame_0x13_live[31*8 +: 8] = adc_result_amp_rms_raw[31:24];

        frame_0x13_live[32*8 +: 8] = adc_a_vpp_raw[7:0];
        frame_0x13_live[33*8 +: 8] = adc_a_vpp_raw[11:8];
        frame_0x13_live[36*8 +: 8] = adc_b_vpp_raw[7:0];
        frame_0x13_live[37*8 +: 8] = adc_b_vpp_raw[11:8];

        frame_0x13_live[40*8 +: 8] = 8'd0;
        frame_0x13_live[41*8 +: 8] = 8'd0;
        frame_0x13_live[42*8 +: 8] = 8'd0;
        frame_0x13_live[43*8 +: 8] = 8'd0;

        frame_0x13_live[44*8 +: 8] = adc_result_phase_deg_x10[7:0];
        frame_0x13_live[45*8 +: 8] = adc_result_phase_deg_x10[15:8];
        frame_0x13_live[46*8 +: 8] = adc_result_phase_deg_x10[23:16];
        frame_0x13_live[47*8 +: 8] = adc_result_phase_deg_x10[31:24];

        frame_0x13_live[48*8 +: 8] = adc_result_flags[7:0];
        frame_0x13_live[49*8 +: 8] = adc_result_flags[15:8];
        frame_0x13_live[50*8 +: 8] = adc_result_flags[23:16];
        frame_0x13_live[51*8 +: 8] = adc_result_flags[31:24];

        f13_chk = 8'd0;
        for (f13_i = 0; f13_i < 116; f13_i = f13_i + 1)
            f13_chk = f13_chk ^ frame_0x13_live[f13_i*8 +: 8];
        frame_0x13_live[116*8 +: 8] = f13_chk;
    end

    assign spi_a_tx_frame = adc_result_tx_pending ? frame_0x13_live :
                            (adv_tx_ready_pending ? adv_tx_frame :
                            (adc_wave_tx_pending ? frame_0x12_reg : frame_0x10));

    spi_master_128b #(.CLK_DIV_HALF(SPI_A_CLK_DIV_HALF)) u_spi_a (
        .clk(clk_125m), .rst(rst),
        .start(spi_a_start), .tx_frame(spi_a_tx_frame),
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
            if ((state == ST_SEND_DDS_FREQ ||
                 state == ST_WAIT_DDS_ACK ||
                 (ENABLE_LEGACY_ADC_TEST && state == ST_ADC_TEST_SET_DDS) ||
                 (ENABLE_LEGACY_ADC_TEST && state == ST_ADC_TEST_WAIT_ACK) ||
                 state == ST_STOP) &&
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

        // seq/cmd. STOP uses a high-bit seq so PYNQDDS does not de-dup it
        // against the last SET_FREQ point.
        frame_d1[4*8 +: 8]  = dds_tx_seq[7:0];
        frame_d1[5*8 +: 8]  = dds_tx_seq[15:8];
        frame_d1[6*8 +: 8]  = dds_tx_seq[23:16];
        frame_d1[7*8 +: 8]  = dds_tx_seq[31:24];

        frame_d1[8*8 +: 8]  = dds_tx_cmd[7:0];
        frame_d1[9*8 +: 8]  = dds_tx_cmd[15:8];
        frame_d1[10*8 +: 8] = dds_tx_cmd[23:16];
        frame_d1[11*8 +: 8] = dds_tx_cmd[31:24];

        // freq_hz (LE)
        frame_d1[12*8 +: 8] = dds_tx_freq[7:0];
        frame_d1[13*8 +: 8] = dds_tx_freq[15:8];
        frame_d1[14*8 +: 8] = dds_tx_freq[23:16];
        frame_d1[15*8 +: 8] = dds_tx_freq[31:24];

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

        // Keep the seq equal to the command being acknowledged; cmd=NOP.
        frame_d1_nop[4*8 +: 8] = dds_expected_seq[7:0];
        frame_d1_nop[5*8 +: 8] = dds_expected_seq[15:8];
        frame_d1_nop[6*8 +: 8] = dds_expected_seq[23:16];
        frame_d1_nop[7*8 +: 8] = dds_expected_seq[31:24];

        d1_nop_chk = 8'd0;
        for (d1_nop_i = 0; d1_nop_i < 116; d1_nop_i = d1_nop_i + 1)
            d1_nop_chk = d1_nop_chk ^ frame_d1_nop[d1_nop_i*8 +: 8];
        frame_d1_nop[116*8 +: 8] = d1_nop_chk;
    end

    assign spi_b_tx_frame =
        (state == ST_WAIT_DDS_ACK ||
         (ENABLE_LEGACY_ADC_TEST && state == ST_ADC_TEST_WAIT_ACK)) ?
        frame_d1_nop : frame_d1;

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
