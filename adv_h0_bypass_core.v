`timescale 1ns / 1ps

// Advanced-stage debug core:
// capture ADC CHB y(t) -> shared xfft -> H0 bypass X[k]=Y[k]
// -> shared xfft inverse -> recon RAM. It also packs 0x15 waveform chunks
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
    input  wire        frame_prepare_start,
    output wire [31:0] frame_chunk_count,
    output reg  [1023:0] wave_frame,
    output reg         frame_ready,

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

    // xfft_0xfft_1024_shared config: bit 0 = FWD/INV, bits 20:1 = scale schedule.
    // Use the generated IP demo's default 1024-point radix-2 scaling schedule.
    localparam [23:0] FFT_CONFIG_FWD = 24'h0AAAAD;
    localparam [23:0] FFT_CONFIG_INV = 24'h0AAAAC;
    localparam [31:0] WAVE_FLAGS_DONE  = 32'h00000002;
    localparam [31:0] WAVE_FLAGS_VALID = 32'h00000004;
    localparam [31:0] WAVE_FLAGS_RAW   = 32'h00000008;
    localparam [31:0] WAVE_FLAGS_H0    = 32'h00001000;

    assign frame_chunk_count = (FFT_N + SAMPLES_PER_CHUNK - 1) / SAMPLES_PER_CHUNK;

    (* ram_style = "block" *) reg signed [15:0] capture_ram [0:FFT_N-1];
    (* ram_style = "distributed" *) reg signed [15:0] recon_ram [0:FFT_N-1];
    (* ram_style = "block" *) reg [31:0] spectrum_ram [0:FFT_N-1];

    wire signed [12:0] sample_b_centered = $signed({1'b0, sample_b_raw}) - 13'sd2048;
    wire signed [15:0] sample_b_q15 = {sample_b_centered, 3'b000};
    wire signed [31:0] sample_b_q31 = {{16{sample_b_q15[15]}}, sample_b_q15};

    reg [15:0] cap_count;
    reg signed [47:0] y_sum_acc;
    reg signed [47:0] x_sum_acc;
    reg frame_prep_active;
    reg [9:0] capture_fft_rd_addr;
    reg [9:0] capture_frame_rd_addr;
    wire [9:0] capture_rd_addr = (frame_prep_active || frame_prepare_start) ?
                                  capture_frame_rd_addr : capture_fft_rd_addr;
    reg signed [15:0] capture_rd_data;
    reg [9:0] spectrum_rd_addr;
    reg [31:0] spectrum_rd_data;

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
            capture_rd_data <= 16'sd0;
        end else begin
            capture_rd_data <= capture_ram[capture_rd_addr];

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
    localparam [3:0] R_PRIME_FWD = 4'd7;
    localparam [3:0] R_PRIME_INV = 4'd8;

    reg [9:0] feed_index;
    reg [9:0] out_index;
    reg signed [47:0] recon_sum_next;

    reg [23:0] fft_cfg_tdata;
    reg        fft_cfg_tvalid;
    wire       fft_cfg_tready;
    reg [31:0] fft_s_tdata;
    reg        fft_s_tvalid;
    wire       fft_s_tready;
    reg        fft_s_tlast;
    wire [31:0] fft_m_tdata;
    wire [7:0]  fft_m_tuser;
    wire        fft_m_tvalid;
    wire        fft_m_tlast;
    wire [7:0]  fft_status_tdata;
    wire        fft_status_tvalid;
    wire        fft_event_frame_started;
    wire        fft_event_tlast_unexpected;
    wire        fft_event_tlast_missing;
    wire        fft_event_fft_overflow;
    wire        fft_event_status_channel_halt;
    wire        fft_event_data_in_channel_halt;
    wire        fft_event_data_out_channel_halt;

    wire signed [15:0] ifft_real = fft_m_tdata[15:0];
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

    xfft_0xfft_1024_shared u_adv_xfft_shared (
        .aclk(clk),
        .aresetn(~rst),
        .s_axis_config_tdata(fft_cfg_tdata),
        .s_axis_config_tvalid(fft_cfg_tvalid),
        .s_axis_config_tready(fft_cfg_tready),
        .s_axis_data_tdata(fft_s_tdata),
        .s_axis_data_tvalid(fft_s_tvalid),
        .s_axis_data_tready(fft_s_tready),
        .s_axis_data_tlast(fft_s_tlast),
        .m_axis_data_tdata(fft_m_tdata),
        .m_axis_data_tuser(fft_m_tuser),
        .m_axis_data_tvalid(fft_m_tvalid),
        .m_axis_data_tready(1'b1),
        .m_axis_data_tlast(fft_m_tlast),
        .m_axis_status_tdata(fft_status_tdata),
        .m_axis_status_tvalid(fft_status_tvalid),
        .m_axis_status_tready(1'b1),
        .event_frame_started(fft_event_frame_started),
        .event_tlast_unexpected(fft_event_tlast_unexpected),
        .event_tlast_missing(fft_event_tlast_missing),
        .event_fft_overflow(fft_event_fft_overflow),
        .event_status_channel_halt(fft_event_status_channel_halt),
        .event_data_in_channel_halt(fft_event_data_in_channel_halt),
        .event_data_out_channel_halt(fft_event_data_out_channel_halt)
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
            fft_cfg_tdata <= 24'd0;
            fft_cfg_tvalid <= 1'b0;
            fft_s_tdata <= 32'd0;
            fft_s_tvalid <= 1'b0;
            fft_s_tlast <= 1'b0;
            capture_fft_rd_addr <= 10'd0;
            capture_frame_rd_addr <= 10'd0;
            spectrum_rd_addr <= 10'd0;
            spectrum_rd_data <= 32'd0;
        end else begin
            spectrum_rd_data <= spectrum_ram[spectrum_rd_addr];

            fft_cfg_tvalid <= 1'b0;
            fft_s_tvalid <= 1'b0;
            fft_s_tlast <= 1'b0;

            if (fft_event_fft_overflow) begin
                if (r_state == R_CFG_INV || r_state == R_FEED_INV || r_state == R_WAIT_INV)
                    ifft_overflow_count <= ifft_overflow_count + 32'd1;
                else
                    fft_overflow_count <= fft_overflow_count + 32'd1;
            end
            if (fft_event_tlast_missing)
                tlast_missing_count <= tlast_missing_count + 32'd1;
            if (fft_event_tlast_unexpected)
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
                        fft_cfg_tdata <= FFT_CONFIG_FWD;
                        capture_fft_rd_addr <= 10'd0;
                        r_state <= R_CFG_FWD;
                    end
                end

                R_CFG_FWD: begin
                    busy <= 1'b1;
                    fft_cfg_tdata <= FFT_CONFIG_FWD;
                    fft_cfg_tvalid <= 1'b1;
                    if (fft_cfg_tvalid && fft_cfg_tready) begin
                        fft_cfg_tvalid <= 1'b0;
                        feed_index <= 10'd0;
                        capture_fft_rd_addr <= 10'd0;
                        r_state <= R_PRIME_FWD;
                    end
                end

                R_PRIME_FWD: begin
                    busy <= 1'b1;
                    capture_fft_rd_addr <= 10'd1;
                    r_state <= R_FEED_FWD;
                end

                R_FEED_FWD: begin
                    busy <= 1'b1;
                    fft_s_tdata <= {16'sd0, capture_rd_data};
                    fft_s_tvalid <= 1'b1;
                    fft_s_tlast <= (feed_index == FFT_N - 1);
                    if (fft_s_tvalid && fft_s_tready) begin
                        if (feed_index == FFT_N - 1) begin
                            fft_s_tvalid <= 1'b0;
                            fft_s_tlast <= 1'b0;
                            out_index <= 10'd0;
                            r_state <= R_WAIT_FWD;
                        end else begin
                            feed_index <= feed_index + 10'd1;
                            capture_fft_rd_addr <= feed_index + 10'd2;
                            fft_s_tdata <= {16'sd0, capture_rd_data};
                            fft_s_tlast <= (feed_index + 10'd1 == FFT_N - 1);
                        end
                    end
                end

                R_WAIT_FWD: begin
                    busy <= 1'b1;
                    if (fft_m_tvalid) begin
                        spectrum_ram[out_index] <= fft_m_tdata;
                        if (out_index == FFT_N - 1 || fft_m_tlast) begin
                            feed_index <= 10'd0;
                            fft_cfg_tdata <= FFT_CONFIG_INV;
                            spectrum_rd_addr <= 10'd0;
                            r_state <= R_CFG_INV;
                        end else begin
                            out_index <= out_index + 10'd1;
                        end
                    end
                end

                R_CFG_INV: begin
                    busy <= 1'b1;
                    fft_cfg_tdata <= FFT_CONFIG_INV;
                    fft_cfg_tvalid <= 1'b1;
                    if (fft_cfg_tvalid && fft_cfg_tready) begin
                        fft_cfg_tvalid <= 1'b0;
                        feed_index <= 10'd0;
                        out_index <= 10'd0;
                        x_sum_acc <= 48'sd0;
                        x_min <= 32'sd0;
                        x_max <= 32'sd0;
                        spectrum_rd_addr <= 10'd0;
                        r_state <= R_PRIME_INV;
                    end
                end

                R_PRIME_INV: begin
                    busy <= 1'b1;
                    spectrum_rd_addr <= 10'd1;
                    r_state <= R_FEED_INV;
                end

                R_FEED_INV: begin
                    busy <= 1'b1;
                    fft_s_tdata <= spectrum_rd_data;
                    fft_s_tvalid <= 1'b1;
                    fft_s_tlast <= (feed_index == FFT_N - 1);
                    if (fft_s_tvalid && fft_s_tready) begin
                        if (feed_index == FFT_N - 1) begin
                            fft_s_tvalid <= 1'b0;
                            fft_s_tlast <= 1'b0;
                            out_index <= 10'd0;
                            r_state <= R_WAIT_INV;
                        end else begin
                            feed_index <= feed_index + 10'd1;
                            spectrum_rd_addr <= feed_index + 10'd2;
                            fft_s_tdata <= spectrum_rd_data;
                            fft_s_tlast <= (feed_index + 10'd1 == FFT_N - 1);
                        end
                    end
                end

                R_WAIT_INV: begin
                    busy <= 1'b1;
                    if (fft_m_tvalid) begin
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
                        if (out_index == FFT_N - 1 || fft_m_tlast) begin
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

    function [7:0] xor32_bytes;
        input [31:0] v;
        begin
            xor32_bytes = v[7:0] ^ v[15:8] ^ v[23:16] ^ v[31:24];
        end
    endfunction

    wire [31:0] frame_start_index_now = frame_chunk_index * SAMPLES_PER_CHUNK;
    wire [31:0] frame_flags_now = WAVE_FLAGS_VALID | WAVE_FLAGS_RAW | WAVE_FLAGS_H0 |
                                  ((frame_chunk_index >= (frame_chunk_count - 1)) ? WAVE_FLAGS_DONE : 32'd0);
    wire signed [31:0] frame_min_now = (frame_wave_type == 32'd1) ? x_min : y_min;
    wire signed [31:0] frame_max_now = (frame_wave_type == 32'd1) ? x_max : y_max;
    wire signed [31:0] frame_mean_now = (frame_wave_type == 32'd1) ? x_mean : y_mean;
    wire signed [31:0] frame_vpp_now = (frame_wave_type == 32'd1) ? x_vpp : y_vpp;

    reg [31:0] frame_start_index;
    reg [31:0] frame_flags;
    reg signed [31:0] frame_min;
    reg signed [31:0] frame_max;
    reg signed [31:0] frame_mean;
    reg signed [31:0] frame_vpp;
    reg [7:0] frame_chk;
    reg [15:0] frame_sample;
    reg [31:0] frame_sample_addr;
    reg [5:0] frame_sample_index;
    reg frame_sample_valid;
    integer frame_i;

    always @(posedge clk) begin
        if (rst) begin
            wave_frame <= 1024'd0;
            frame_ready <= 1'b0;
            frame_prep_active <= 1'b0;
            frame_sample_index <= 6'd0;
            frame_start_index <= 32'd0;
            frame_flags <= 32'd0;
            frame_min <= 32'sd0;
            frame_max <= 32'sd0;
            frame_mean <= 32'sd0;
            frame_vpp <= 32'sd0;
            frame_chk <= 8'd0;
            frame_sample <= 16'd0;
            frame_sample_addr <= 32'd0;
            frame_sample_valid <= 1'b0;
        end else begin
            if (frame_prepare_start) begin
                wave_frame <= 1024'd0;
                frame_ready <= 1'b0;
                frame_prep_active <= 1'b1;
                frame_sample_index <= 6'd0;
                frame_sample_valid <= 1'b0;
                frame_start_index <= frame_start_index_now;
                frame_flags <= frame_flags_now;
                frame_min <= frame_min_now;
                frame_max <= frame_max_now;
                frame_mean <= frame_mean_now;
                frame_vpp <= frame_vpp_now;
                if (frame_wave_type == 32'd0)
                    capture_frame_rd_addr <= frame_start_index_now[9:0];

                wave_frame[0*8 +: 8] <= 8'hA5;
                wave_frame[1*8 +: 8] <= 8'h5A;
                wave_frame[2*8 +: 8] <= 8'h15;
                wave_frame[3*8 +: 8] <= 8'd112;

                for (frame_i = 0; frame_i < 4; frame_i = frame_i + 1) begin
                    wave_frame[(4  + frame_i)*8 +: 8] <= get_byte32(frame_seq, frame_i);
                    wave_frame[(8  + frame_i)*8 +: 8] <= get_byte32(frame_wave_type, frame_i);
                    wave_frame[(12 + frame_i)*8 +: 8] <= get_byte32(frame_chunk_index, frame_i);
                    wave_frame[(16 + frame_i)*8 +: 8] <= get_byte32(frame_chunk_count, frame_i);
                    wave_frame[(20 + frame_i)*8 +: 8] <= get_byte32(SAMPLE_RATE_HZ, frame_i);
                    wave_frame[(24 + frame_i)*8 +: 8] <= get_byte32(FFT_N, frame_i);
                    wave_frame[(28 + frame_i)*8 +: 8] <= get_byte32(frame_start_index_now, frame_i);
                    wave_frame[(32 + frame_i)*8 +: 8] <= get_byte32(frame_min_now, frame_i);
                    wave_frame[(36 + frame_i)*8 +: 8] <= get_byte32(frame_max_now, frame_i);
                    wave_frame[(40 + frame_i)*8 +: 8] <= get_byte32(frame_mean_now, frame_i);
                    wave_frame[(44 + frame_i)*8 +: 8] <= get_byte32(frame_vpp_now, frame_i);
                    wave_frame[(48 + frame_i)*8 +: 8] <= get_byte32(frame_flags_now, frame_i);
                end

                frame_chk <= 8'hA5 ^ 8'h5A ^ 8'h15 ^ 8'd112 ^
                             xor32_bytes(frame_seq) ^
                             xor32_bytes(frame_wave_type) ^
                             xor32_bytes(frame_chunk_index) ^
                             xor32_bytes(frame_chunk_count) ^
                             xor32_bytes(SAMPLE_RATE_HZ) ^
                             xor32_bytes(FFT_N) ^
                             xor32_bytes(frame_start_index_now) ^
                             xor32_bytes(frame_min_now) ^
                             xor32_bytes(frame_max_now) ^
                             xor32_bytes(frame_mean_now) ^
                             xor32_bytes(frame_vpp_now) ^
                             xor32_bytes(frame_flags_now);
            end else if (frame_prep_active) begin
                if (!frame_sample_valid) begin
                    if (frame_wave_type == 32'd0)
                        capture_frame_rd_addr <= frame_start_index[9:0] + 10'd1;
                    frame_sample_valid <= 1'b1;
                end else if (frame_sample_index < SAMPLES_PER_CHUNK) begin
                    frame_sample_addr = frame_start_index + frame_sample_index;
                    if (frame_sample_addr >= FFT_N)
                        frame_sample = 16'd0;
                    else if (frame_wave_type == 32'd1)
                        frame_sample = recon_ram[frame_sample_addr[9:0]];
                    else
                        frame_sample = capture_rd_data;

                    if (frame_wave_type == 32'd0)
                        capture_frame_rd_addr <= frame_start_index[9:0] + frame_sample_index[5:0] + 10'd2;
                    wave_frame[(52 + frame_sample_index*2)*8 +: 8] <= frame_sample[7:0];
                    wave_frame[(53 + frame_sample_index*2)*8 +: 8] <= frame_sample[15:8];
                    frame_chk <= frame_chk ^ frame_sample[7:0] ^ frame_sample[15:8];
                    frame_sample_index <= frame_sample_index + 6'd1;
                end else begin
                    wave_frame[116*8 +: 8] <= frame_chk;
                    frame_ready <= 1'b1;
                    frame_prep_active <= 1'b0;
                    frame_sample_valid <= 1'b0;
                end
            end
        end
    end

endmodule
