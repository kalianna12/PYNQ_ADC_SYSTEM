`timescale 1ns / 1ps

// Advanced-stage debug core:
// capture ADC CHB y(t) -> xfft_1024_fwd -> H0 bypass X[k]=Y[k]
// -> xfft_1024_inv -> recon RAM. It also packs 0x15 waveform chunks
// for the existing 128-byte SPI-A transport.
module adv_h0_bypass_core #(
    parameter integer FFT_N = 1024,
    parameter [31:0] SAMPLE_RATE_HZ = 32'd100000,
    parameter integer SAMPLES_PER_CHUNK = 30
) (
    input  wire clk,
    input  wire rst,

    input  wire capture_clear,
    input  wire capture_enable,
    input  wire capture_finish,
    input  wire sample_valid,
    input  wire [15:0] sample_index,
    input  wire [11:0] sample_b_raw,

    input  wire reconstruct_start,

    input  wire [31:0] frame_seq,
    input  wire [31:0] frame_wave_type,   // 0=output y(t), 1=reconstructed x(t)
    input  wire [31:0] frame_chunk_index,
    output wire [31:0] frame_chunk_count,
    output reg  [1023:0] wave_frame,

    output reg  capture_done,
    output reg  recon_done,
    output reg  busy,
    output reg  [31:0] status_flags,
    output reg  signed [31:0] y_min,
    output reg  signed [31:0] y_max,
    output reg  signed [31:0] y_mean,
    output reg  signed [31:0] y_vpp,
    output reg  signed [31:0] x_min,
    output reg  signed [31:0] x_max,
    output reg  signed [31:0] x_mean,
    output reg  signed [31:0] x_vpp,
    output reg  [31:0] capture_done_count,
    output reg  [31:0] recon_done_count,
    output reg  [31:0] fft_overflow_count,
    output reg  [31:0] ifft_overflow_count,
    output reg  [31:0] tlast_missing_count,
    output reg  [31:0] tlast_unexpected_count
);

    localparam [15:0] FFT_CONFIG_FWD = 16'h0557;
    localparam [15:0] FFT_CONFIG_INV = 16'h0556;
    localparam [31:0] WAVE_FLAGS_DONE  = 32'h00000002;
    localparam [31:0] WAVE_FLAGS_VALID = 32'h00000004;
    localparam [31:0] WAVE_FLAGS_RAW   = 32'h00000008;
    localparam [31:0] WAVE_FLAGS_H0    = 32'h00001000;

    assign frame_chunk_count = (FFT_N + SAMPLES_PER_CHUNK - 1) / SAMPLES_PER_CHUNK;

    (* ram_style = "distributed" *) reg signed [15:0] capture_ram [0:FFT_N-1];
    (* ram_style = "distributed" *) reg signed [15:0] recon_ram [0:FFT_N-1];
    (* ram_style = "block" *) reg [31:0] spectrum_ram [0:FFT_N-1];

    wire signed [12:0] sample_b_centered = $signed({1'b0, sample_b_raw}) - 13'sd2048;
    wire signed [15:0] sample_b_q15 = {sample_b_centered, 3'b000};
    wire signed [31:0] sample_b_q31 = {{16{sample_b_q15[15]}}, sample_b_q15};

    reg [15:0] cap_count;
    reg signed [47:0] y_sum_acc;
    reg signed [47:0] x_sum_acc;

    always @(posedge clk) begin
        if (rst) begin
            capture_done <= 1'b0;
            cap_count <= 16'd0;
            y_sum_acc <= 48'sd0;
            y_min <= 32'sd0;
            y_max <= 32'sd0;
            y_mean <= 32'sd0;
            y_vpp <= 32'sd0;
            capture_done_count <= 32'd0;
        end else begin
            if (capture_clear) begin
                capture_done <= 1'b0;
                cap_count <= 16'd0;
                y_sum_acc <= 48'sd0;
                y_min <= 32'sd0;
                y_max <= 32'sd0;
                y_mean <= 32'sd0;
                y_vpp <= 32'sd0;
            end

            if (capture_enable && sample_valid && sample_index < FFT_N) begin
                capture_ram[sample_index[9:0]] <= sample_b_q15;
                if (cap_count == 16'd0) begin
                    y_min <= sample_b_q31;
                    y_max <= sample_b_q31;
                end else begin
                    if (sample_b_q31 < y_min)
                        y_min <= sample_b_q31;
                    if (sample_b_q31 > y_max)
                        y_max <= sample_b_q31;
                end
                y_sum_acc <= y_sum_acc + {{32{sample_b_q15[15]}}, sample_b_q15};
                if (cap_count < FFT_N)
                    cap_count <= cap_count + 16'd1;
            end

            if (capture_finish) begin
                capture_done <= 1'b1;
                capture_done_count <= capture_done_count + 32'd1;
                y_vpp <= y_max - y_min;
                if (cap_count != 16'd0)
                    y_mean <= y_sum_acc / $signed({32'd0, cap_count});
                else
                    y_mean <= 32'sd0;
            end
        end
    end

    reg [3:0] r_state;
    localparam [3:0] R_IDLE     = 4'd0;
    localparam [3:0] R_CFG_FWD  = 4'd1;
    localparam [3:0] R_FEED_FWD = 4'd2;
    localparam [3:0] R_WAIT_FWD = 4'd3;
    localparam [3:0] R_CFG_INV  = 4'd4;
    localparam [3:0] R_FEED_INV = 4'd5;
    localparam [3:0] R_WAIT_INV = 4'd6;

    reg [9:0] feed_index;
    reg [9:0] out_index;
    reg signed [47:0] recon_sum_next;

    reg [15:0] fwd_cfg_tdata;
    reg        fwd_cfg_tvalid;
    wire       fwd_cfg_tready;
    reg [31:0] fwd_s_tdata;
    reg        fwd_s_tvalid;
    wire       fwd_s_tready;
    reg        fwd_s_tlast;
    wire [31:0] fwd_m_tdata;
    wire [7:0]  fwd_m_tuser;
    wire        fwd_m_tvalid;
    wire        fwd_m_tlast;
    wire [7:0]  fwd_status_tdata;
    wire        fwd_status_tvalid;
    wire        fwd_event_frame_started;
    wire        fwd_event_tlast_unexpected;
    wire        fwd_event_tlast_missing;
    wire        fwd_event_fft_overflow;
    wire        fwd_event_data_in_channel_halt;

    reg [15:0] inv_cfg_tdata;
    reg        inv_cfg_tvalid;
    wire       inv_cfg_tready;
    reg [31:0] inv_s_tdata;
    reg        inv_s_tvalid;
    wire       inv_s_tready;
    reg        inv_s_tlast;
    wire [31:0] inv_m_tdata;
    wire [7:0]  inv_m_tuser;
    wire        inv_m_tvalid;
    wire        inv_m_tlast;
    wire [7:0]  inv_status_tdata;
    wire        inv_status_tvalid;
    wire        inv_event_frame_started;
    wire        inv_event_tlast_unexpected;
    wire        inv_event_tlast_missing;
    wire        inv_event_fft_overflow;
    wire        inv_event_data_in_channel_halt;

    wire signed [15:0] ifft_real = inv_m_tdata[15:0];
    wire signed [31:0] ifft_real_scaled = {{11{ifft_real[15]}}, ifft_real, 5'b00000};

    function signed [15:0] sat16;
        input signed [31:0] v;
        begin
            if (v > 32'sd32767)
                sat16 = 16'sd32767;
            else if (v < -32'sd32768)
                sat16 = -16'sd32768;
            else
                sat16 = v[15:0];
        end
    endfunction

    wire signed [15:0] ifft_sample_sat = sat16(ifft_real_scaled);
    wire signed [31:0] ifft_sample_q31 = {{16{ifft_sample_sat[15]}}, ifft_sample_sat};

    xfft_1024_fwd u_adv_xfft_fwd (
        .aclk(clk),
        .aresetn(~rst),
        .s_axis_config_tdata(fwd_cfg_tdata),
        .s_axis_config_tvalid(fwd_cfg_tvalid),
        .s_axis_config_tready(fwd_cfg_tready),
        .s_axis_data_tdata(fwd_s_tdata),
        .s_axis_data_tvalid(fwd_s_tvalid),
        .s_axis_data_tready(fwd_s_tready),
        .s_axis_data_tlast(fwd_s_tlast),
        .m_axis_data_tdata(fwd_m_tdata),
        .m_axis_data_tuser(fwd_m_tuser),
        .m_axis_data_tvalid(fwd_m_tvalid),
        .m_axis_data_tlast(fwd_m_tlast),
        .m_axis_status_tdata(fwd_status_tdata),
        .m_axis_status_tvalid(fwd_status_tvalid),
        .event_frame_started(fwd_event_frame_started),
        .event_tlast_unexpected(fwd_event_tlast_unexpected),
        .event_tlast_missing(fwd_event_tlast_missing),
        .event_fft_overflow(fwd_event_fft_overflow),
        .event_data_in_channel_halt(fwd_event_data_in_channel_halt)
    );

    xfft_1024_inv u_adv_xfft_inv (
        .aclk(clk),
        .aresetn(~rst),
        .s_axis_config_tdata(inv_cfg_tdata),
        .s_axis_config_tvalid(inv_cfg_tvalid),
        .s_axis_config_tready(inv_cfg_tready),
        .s_axis_data_tdata(inv_s_tdata),
        .s_axis_data_tvalid(inv_s_tvalid),
        .s_axis_data_tready(inv_s_tready),
        .s_axis_data_tlast(inv_s_tlast),
        .m_axis_data_tdata(inv_m_tdata),
        .m_axis_data_tuser(inv_m_tuser),
        .m_axis_data_tvalid(inv_m_tvalid),
        .m_axis_data_tlast(inv_m_tlast),
        .m_axis_status_tdata(inv_status_tdata),
        .m_axis_status_tvalid(inv_status_tvalid),
        .event_frame_started(inv_event_frame_started),
        .event_tlast_unexpected(inv_event_tlast_unexpected),
        .event_tlast_missing(inv_event_tlast_missing),
        .event_fft_overflow(inv_event_fft_overflow),
        .event_data_in_channel_halt(inv_event_data_in_channel_halt)
    );

    always @(posedge clk) begin
        if (rst) begin
            r_state <= R_IDLE;
            feed_index <= 10'd0;
            out_index <= 10'd0;
            recon_done <= 1'b0;
            busy <= 1'b0;
            status_flags <= 32'd0;
            x_min <= 32'sd0;
            x_max <= 32'sd0;
            x_mean <= 32'sd0;
            x_vpp <= 32'sd0;
            x_sum_acc <= 48'sd0;
            recon_sum_next <= 48'sd0;
            recon_done_count <= 32'd0;
            fft_overflow_count <= 32'd0;
            ifft_overflow_count <= 32'd0;
            tlast_missing_count <= 32'd0;
            tlast_unexpected_count <= 32'd0;
            fwd_cfg_tdata <= 16'd0;
            fwd_cfg_tvalid <= 1'b0;
            fwd_s_tdata <= 32'd0;
            fwd_s_tvalid <= 1'b0;
            fwd_s_tlast <= 1'b0;
            inv_cfg_tdata <= 16'd0;
            inv_cfg_tvalid <= 1'b0;
            inv_s_tdata <= 32'd0;
            inv_s_tvalid <= 1'b0;
            inv_s_tlast <= 1'b0;
        end else begin
            fwd_cfg_tvalid <= 1'b0;
            fwd_s_tvalid <= 1'b0;
            fwd_s_tlast <= 1'b0;
            inv_cfg_tvalid <= 1'b0;
            inv_s_tvalid <= 1'b0;
            inv_s_tlast <= 1'b0;

            if (fwd_event_fft_overflow)
                fft_overflow_count <= fft_overflow_count + 32'd1;
            if (inv_event_fft_overflow)
                ifft_overflow_count <= ifft_overflow_count + 32'd1;
            if (fwd_event_tlast_missing || inv_event_tlast_missing)
                tlast_missing_count <= tlast_missing_count + 32'd1;
            if (fwd_event_tlast_unexpected || inv_event_tlast_unexpected)
                tlast_unexpected_count <= tlast_unexpected_count + 32'd1;

            case (r_state)
                R_IDLE: begin
                    busy <= 1'b0;
                    if (capture_clear)
                        recon_done <= 1'b0;
                    if (reconstruct_start && capture_done) begin
                        recon_done <= 1'b0;
                        busy <= 1'b1;
                        feed_index <= 10'd0;
                        out_index <= 10'd0;
                        status_flags <= WAVE_FLAGS_VALID | WAVE_FLAGS_H0;
                        fwd_cfg_tdata <= FFT_CONFIG_FWD;
                        r_state <= R_CFG_FWD;
                    end
                end

                R_CFG_FWD: begin
                    busy <= 1'b1;
                    fwd_cfg_tdata <= FFT_CONFIG_FWD;
                    fwd_cfg_tvalid <= 1'b1;
                    if (fwd_cfg_tvalid && fwd_cfg_tready) begin
                        fwd_cfg_tvalid <= 1'b0;
                        feed_index <= 10'd0;
                        fwd_s_tdata <= {16'sd0, capture_ram[10'd0]};
                        fwd_s_tvalid <= 1'b1;
                        fwd_s_tlast <= (FFT_N == 1);
                        r_state <= R_FEED_FWD;
                    end
                end

                R_FEED_FWD: begin
                    busy <= 1'b1;
                    fwd_s_tdata <= {16'sd0, capture_ram[feed_index]};
                    fwd_s_tvalid <= 1'b1;
                    fwd_s_tlast <= (feed_index == FFT_N - 1);
                    if (fwd_s_tvalid && fwd_s_tready) begin
                        if (feed_index == FFT_N - 1) begin
                            fwd_s_tvalid <= 1'b0;
                            fwd_s_tlast <= 1'b0;
                            out_index <= 10'd0;
                            r_state <= R_WAIT_FWD;
                        end else begin
                            feed_index <= feed_index + 10'd1;
                            fwd_s_tdata <= {16'sd0, capture_ram[feed_index + 10'd1]};
                            fwd_s_tlast <= (feed_index + 10'd1 == FFT_N - 1);
                        end
                    end
                end

                R_WAIT_FWD: begin
                    busy <= 1'b1;
                    if (fwd_m_tvalid) begin
                        spectrum_ram[out_index] <= fwd_m_tdata;
                        if (out_index == FFT_N - 1 || fwd_m_tlast) begin
                            feed_index <= 10'd0;
                            inv_cfg_tdata <= FFT_CONFIG_INV;
                            r_state <= R_CFG_INV;
                        end else begin
                            out_index <= out_index + 10'd1;
                        end
                    end
                end

                R_CFG_INV: begin
                    busy <= 1'b1;
                    inv_cfg_tdata <= FFT_CONFIG_INV;
                    inv_cfg_tvalid <= 1'b1;
                    if (inv_cfg_tvalid && inv_cfg_tready) begin
                        inv_cfg_tvalid <= 1'b0;
                        feed_index <= 10'd0;
                        out_index <= 10'd0;
                        x_sum_acc <= 48'sd0;
                        x_min <= 32'sd0;
                        x_max <= 32'sd0;
                        inv_s_tdata <= spectrum_ram[10'd0];
                        inv_s_tvalid <= 1'b1;
                        inv_s_tlast <= (FFT_N == 1);
                        r_state <= R_FEED_INV;
                    end
                end

                R_FEED_INV: begin
                    busy <= 1'b1;
                    inv_s_tdata <= spectrum_ram[feed_index];
                    inv_s_tvalid <= 1'b1;
                    inv_s_tlast <= (feed_index == FFT_N - 1);
                    if (inv_s_tvalid && inv_s_tready) begin
                        if (feed_index == FFT_N - 1) begin
                            inv_s_tvalid <= 1'b0;
                            inv_s_tlast <= 1'b0;
                            out_index <= 10'd0;
                            r_state <= R_WAIT_INV;
                        end else begin
                            feed_index <= feed_index + 10'd1;
                            inv_s_tdata <= spectrum_ram[feed_index + 10'd1];
                            inv_s_tlast <= (feed_index + 10'd1 == FFT_N - 1);
                        end
                    end
                end

                R_WAIT_INV: begin
                    busy <= 1'b1;
                    if (inv_m_tvalid) begin
                        recon_ram[out_index] <= ifft_sample_sat;
                        recon_sum_next = x_sum_acc + {{32{ifft_sample_sat[15]}}, ifft_sample_sat};
                        x_sum_acc <= recon_sum_next;
                        if (out_index == 10'd0) begin
                            x_min <= ifft_sample_q31;
                            x_max <= ifft_sample_q31;
                        end else begin
                            if (ifft_sample_q31 < x_min)
                                x_min <= ifft_sample_q31;
                            if (ifft_sample_q31 > x_max)
                                x_max <= ifft_sample_q31;
                        end
                        if (out_index == FFT_N - 1 || inv_m_tlast) begin
                            x_vpp <= x_max - x_min;
                            x_mean <= recon_sum_next / 48'sd1024;
                            recon_done <= 1'b1;
                            recon_done_count <= recon_done_count + 32'd1;
                            busy <= 1'b0;
                            r_state <= R_IDLE;
                        end else begin
                            out_index <= out_index + 10'd1;
                        end
                    end
                end

                default: r_state <= R_IDLE;
            endcase
        end
    end

    function [7:0] get_byte32;
        input [31:0] v;
        input integer byte_idx;
        begin
            get_byte32 = v[byte_idx*8 +: 8];
        end
    endfunction

    function [15:0] sample_for_frame;
        input [31:0] wave_type;
        input integer sample_idx;
        begin
            if (sample_idx >= FFT_N)
                sample_for_frame = 16'd0;
            else if (wave_type == 32'd1)
                sample_for_frame = recon_ram[sample_idx[9:0]];
            else
                sample_for_frame = capture_ram[sample_idx[9:0]];
        end
    endfunction

    reg [31:0] frame_start_index;
    reg [31:0] frame_flags;
    reg signed [31:0] frame_min;
    reg signed [31:0] frame_max;
    reg signed [31:0] frame_mean;
    reg signed [31:0] frame_vpp;
    reg [7:0] frame_chk;
    reg [15:0] frame_sample;
    integer frame_i;
    integer sample_i;

    always @* begin
        wave_frame = 1024'd0;
        frame_start_index = frame_chunk_index * SAMPLES_PER_CHUNK;
        frame_flags = WAVE_FLAGS_VALID | WAVE_FLAGS_RAW | WAVE_FLAGS_H0 |
                      ((frame_chunk_index >= (frame_chunk_count - 1)) ? WAVE_FLAGS_DONE : 32'd0);
        frame_min = (frame_wave_type == 32'd1) ? x_min : y_min;
        frame_max = (frame_wave_type == 32'd1) ? x_max : y_max;
        frame_mean = (frame_wave_type == 32'd1) ? x_mean : y_mean;
        frame_vpp = (frame_wave_type == 32'd1) ? x_vpp : y_vpp;

        wave_frame[0*8 +: 8] = 8'hA5;
        wave_frame[1*8 +: 8] = 8'h5A;
        wave_frame[2*8 +: 8] = 8'h15;
        wave_frame[3*8 +: 8] = 8'd112;

        for (frame_i = 0; frame_i < 4; frame_i = frame_i + 1) begin
            wave_frame[(4  + frame_i)*8 +: 8] = get_byte32(frame_seq, frame_i);
            wave_frame[(8  + frame_i)*8 +: 8] = get_byte32(frame_wave_type, frame_i);
            wave_frame[(12 + frame_i)*8 +: 8] = get_byte32(frame_chunk_index, frame_i);
            wave_frame[(16 + frame_i)*8 +: 8] = get_byte32(frame_chunk_count, frame_i);
            wave_frame[(20 + frame_i)*8 +: 8] = get_byte32(SAMPLE_RATE_HZ, frame_i);
            wave_frame[(24 + frame_i)*8 +: 8] = get_byte32(FFT_N, frame_i);
            wave_frame[(28 + frame_i)*8 +: 8] = get_byte32(frame_start_index, frame_i);
            wave_frame[(32 + frame_i)*8 +: 8] = get_byte32(frame_min, frame_i);
            wave_frame[(36 + frame_i)*8 +: 8] = get_byte32(frame_max, frame_i);
            wave_frame[(40 + frame_i)*8 +: 8] = get_byte32(frame_mean, frame_i);
            wave_frame[(44 + frame_i)*8 +: 8] = get_byte32(frame_vpp, frame_i);
            wave_frame[(48 + frame_i)*8 +: 8] = get_byte32(frame_flags, frame_i);
        end

        for (sample_i = 0; sample_i < SAMPLES_PER_CHUNK; sample_i = sample_i + 1) begin
            frame_sample = sample_for_frame(frame_wave_type, frame_start_index + sample_i);
            wave_frame[(52 + sample_i*2)*8 +: 8] = frame_sample[7:0];
            wave_frame[(53 + sample_i*2)*8 +: 8] = frame_sample[15:8];
        end

        frame_chk = 8'd0;
        for (frame_i = 0; frame_i < 116; frame_i = frame_i + 1)
            frame_chk = frame_chk ^ wave_frame[frame_i*8 +: 8];
        wave_frame[116*8 +: 8] = frame_chk;
    end

endmodule
