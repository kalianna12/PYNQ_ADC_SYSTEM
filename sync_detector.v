`timescale 1ns / 1ps

module sync_detector #(
    parameter integer DEFAULT_SAMPLE_COUNT = 4096,
    parameter integer CORDIC_INPUT_SHIFT   = 9,
    parameter         CORDIC_XY_SWAP       = 1'b0,
    parameter         CORDIC_OUT_SWAP      = 1'b0,
    parameter         PHASE_SIGN_INVERT    = 1'b0
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,

    input  wire        sample_valid,
    input  wire [11:0] sample_a_raw,
    input  wire [11:0] sample_b_raw,

    input  wire [11:0] phase_step_cfg,
    input  wire [15:0] sample_count_cfg,

    output reg         busy,
    output reg         done,
    output reg  [31:0] amp_a_code,
    output reg  [31:0] amp_b_code,
    output reg signed [31:0] phase_deg_x10,
    output reg signed [31:0] phase_a_raw,
    output reg signed [31:0] phase_b_raw,
    output reg signed [31:0] phase_diff_raw,
    output reg  [31:0] debug_flags
);

    // CORDIC debug switches:
    // CORDIC_XY_SWAP=0 assumes X/I is tdata[31:0] and Y/Q is tdata[63:32].
    // CORDIC_OUT_SWAP=0 assumes magnitude is dout[31:0] and phase is dout[63:32].
    // PHASE_SIGN_INVERT=0 reports phase_b - phase_a.
    // Hardware check: feed A/B from the same 10 kHz 2 Vpp sine, phase_diff
    // should be near 0 deg. If phase is near +/-90 deg or unstable, check
    // CORDIC_XY_SWAP, Q sign, and the sin/cos ROM alignment. If magnitude and
    // phase look exchanged, check CORDIC_OUT_SWAP. If same-source is near 0
    // but an RC test has the opposite sign, check PHASE_SIGN_INVERT. Do not
    // solve phase sign by changing XDC pin constraints.

    localparam [3:0] ST_IDLE   = 4'd0;
    localparam [3:0] ST_ACCUM  = 4'd1;
    localparam [3:0] ST_SEND_A = 4'd2;
    localparam [3:0] ST_WAIT_A = 4'd3;
    localparam [3:0] ST_SEND_B = 4'd4;
    localparam [3:0] ST_WAIT_B = 4'd5;
    localparam [3:0] ST_DONE   = 4'd6;

    localparam signed [33:0] PI_Q29     = 34'sd1686629713;
    localparam signed [33:0] TWO_PI_Q29 = 34'sd3373259426;
    localparam signed [33:0] RAD_TO_DEGX10_Q32 = 34'sd4584;
    localparam [15:0] DEFAULT_SAMPLE_COUNT_U16 = DEFAULT_SAMPLE_COUNT;

    localparam [31:0] DBG_ACCUM_ACTIVE  = 32'h00000001;
    localparam [31:0] DBG_CORDIC_A_SENT = 32'h00000002;
    localparam [31:0] DBG_CORDIC_B_SENT = 32'h00000004;
    localparam [31:0] DBG_RESULT_VALID  = 32'h00000008;
    localparam [31:0] DBG_INPUT_CLIPPED = 32'h00000010;

    reg [3:0] state = ST_IDLE;

    reg [11:0] phase_acc = 12'd0;
    wire [11:0] lut_addra = phase_acc[11:0];
    wire        lut_ena = (state == ST_ACCUM) && sample_valid;
    wire signed [15:0] sin_q15;
    wire signed [15:0] cos_q15;

    reg        valid_d0 = 1'b0;
    reg        valid_d1 = 1'b0;
    reg        valid_d2 = 1'b0;
    reg signed [12:0] center_a_d0 = 13'sd0;
    reg signed [12:0] center_a_d1 = 13'sd0;
    reg signed [12:0] center_a_d2 = 13'sd0;
    reg signed [12:0] center_b_d0 = 13'sd0;
    reg signed [12:0] center_b_d1 = 13'sd0;
    reg signed [12:0] center_b_d2 = 13'sd0;

    reg [15:0] accum_count = 16'd0;
    reg signed [47:0] i_acc_a = 48'sd0;
    reg signed [47:0] q_acc_a = 48'sd0;
    reg signed [47:0] i_acc_b = 48'sd0;
    reg signed [47:0] q_acc_b = 48'sd0;

    reg signed [31:0] cordic_x_a = 32'sd0;
    reg signed [31:0] cordic_y_a = 32'sd0;
    reg signed [31:0] cordic_x_b = 32'sd0;
    reg signed [31:0] cordic_y_b = 32'sd0;
    reg        cordic_in_valid = 1'b0;
    reg [63:0] cordic_in_data = 64'd0;
    wire       cordic_in_ready;
    wire       cordic_out_valid;
    wire [63:0] cordic_out_data;

    wire [15:0] target_count =
        (sample_count_cfg == 16'd0) ? DEFAULT_SAMPLE_COUNT_U16 : sample_count_cfg;

    wire signed [12:0] sample_a_centered = {1'b0, sample_a_raw} - 13'sd2048;
    wire signed [12:0] sample_b_centered = {1'b0, sample_b_raw} - 13'sd2048;

    wire signed [12:0] center_a_aligned = center_a_d2;
    wire signed [12:0] center_b_aligned = center_b_d2;
    wire signed [28:0] mult_a_i = center_a_aligned * cos_q15;
    wire signed [28:0] mult_a_q = center_a_aligned * sin_q15;
    wire signed [28:0] mult_b_i = center_b_aligned * cos_q15;
    wire signed [28:0] mult_b_q = center_b_aligned * sin_q15;

    wire signed [47:0] mult_a_i_ext = {{19{mult_a_i[28]}}, mult_a_i};
    wire signed [47:0] mult_a_q_ext = {{19{mult_a_q[28]}}, mult_a_q};
    wire signed [47:0] mult_b_i_ext = {{19{mult_b_i[28]}}, mult_b_i};
    wire signed [47:0] mult_b_q_ext = {{19{mult_b_q[28]}}, mult_b_q};

    wire signed [47:0] i_acc_a_next = i_acc_a + mult_a_i_ext;
    wire signed [47:0] q_acc_a_next = q_acc_a - mult_a_q_ext;
    wire signed [47:0] i_acc_b_next = i_acc_b + mult_b_i_ext;
    wire signed [47:0] q_acc_b_next = q_acc_b - mult_b_q_ext;

    wire signed [31:0] cordic_mag_out =
        CORDIC_OUT_SWAP ? cordic_out_data[63:32] : cordic_out_data[31:0];
    wire signed [31:0] cordic_phase_out =
        CORDIC_OUT_SWAP ? cordic_out_data[31:0] : cordic_out_data[63:32];

    wire signed [33:0] phase_diff_ba_wide =
        {{2{cordic_phase_out[31]}}, cordic_phase_out} -
        {{2{phase_a_raw[31]}}, phase_a_raw};
    wire signed [33:0] phase_diff_selected =
        PHASE_SIGN_INVERT ? -phase_diff_ba_wide : phase_diff_ba_wide;
    wire signed [33:0] phase_diff_wrapped_now = wrap_phase_diff(phase_diff_selected);
    wire signed [31:0] phase_deg_x10_now = phase_to_deg_x10(phase_diff_wrapped_now);

    function signed [31:0] scale_for_cordic;
        input signed [47:0] value;
        reg signed [47:0] shifted;
        begin
            shifted = value >>> CORDIC_INPUT_SHIFT;
            if (shifted > 48'sd1073741823)
                scale_for_cordic = 32'sh3fffffff;
            else if (shifted < -48'sd1073741824)
                scale_for_cordic = 32'shc0000000;
            else
                scale_for_cordic = shifted[31:0];
        end
    endfunction

    function signed [33:0] wrap_phase_diff;
        input signed [33:0] phase_in;
        reg signed [33:0] phase_tmp;
        begin
            phase_tmp = phase_in;
            if (phase_tmp > PI_Q29)
                phase_tmp = phase_tmp - TWO_PI_Q29;
            else if (phase_tmp < -PI_Q29)
                phase_tmp = phase_tmp + TWO_PI_Q29;
            wrap_phase_diff = phase_tmp;
        end
    endfunction

    function signed [31:0] phase_to_deg_x10;
        input signed [33:0] phase_rad_q29;
        reg signed [67:0] product;
        begin
            product = phase_rad_q29 * RAD_TO_DEGX10_Q32;
            phase_to_deg_x10 = product >>> 32;
        end
    endfunction

    sin_lut_bram u_sin_lut_bram (
        .clka(clk),
        .ena(lut_ena),
        .addra(lut_addra),
        .douta(sin_q15)
    );

    cos_lut_bram u_cos_lut_bram (
        .clka(clk),
        .ena(lut_ena),
        .addra(lut_addra),
        .douta(cos_q15)
    );

    cordic_vectoring_mag_phase u_cordic (
        .aclk(clk),
        .s_axis_cartesian_tvalid(cordic_in_valid),
        .s_axis_cartesian_tready(cordic_in_ready),
        .s_axis_cartesian_tdata(cordic_in_data),
        .m_axis_dout_tvalid(cordic_out_valid),
        .m_axis_dout_tdata(cordic_out_data)
    );

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            amp_a_code <= 32'd0;
            amp_b_code <= 32'd0;
            phase_deg_x10 <= 32'sd0;
            phase_a_raw <= 32'sd0;
            phase_b_raw <= 32'sd0;
            phase_diff_raw <= 32'sd0;
            debug_flags <= 32'd0;
            phase_acc <= 12'd0;
            valid_d0 <= 1'b0;
            valid_d1 <= 1'b0;
            valid_d2 <= 1'b0;
            center_a_d0 <= 13'sd0;
            center_a_d1 <= 13'sd0;
            center_a_d2 <= 13'sd0;
            center_b_d0 <= 13'sd0;
            center_b_d1 <= 13'sd0;
            center_b_d2 <= 13'sd0;
            accum_count <= 16'd0;
            i_acc_a <= 48'sd0;
            q_acc_a <= 48'sd0;
            i_acc_b <= 48'sd0;
            q_acc_b <= 48'sd0;
            cordic_x_a <= 32'sd0;
            cordic_y_a <= 32'sd0;
            cordic_x_b <= 32'sd0;
            cordic_y_b <= 32'sd0;
            cordic_in_valid <= 1'b0;
            cordic_in_data <= 64'd0;
        end else begin
            if (start) begin
                state <= ST_ACCUM;
                busy <= 1'b1;
                done <= 1'b0;
                amp_a_code <= 32'd0;
                amp_b_code <= 32'd0;
                phase_deg_x10 <= 32'sd0;
                phase_a_raw <= 32'sd0;
                phase_b_raw <= 32'sd0;
                phase_diff_raw <= 32'sd0;
                debug_flags <= DBG_ACCUM_ACTIVE;
                phase_acc <= 12'd0;
                valid_d0 <= 1'b0;
                valid_d1 <= 1'b0;
                valid_d2 <= 1'b0;
                center_a_d0 <= 13'sd0;
                center_a_d1 <= 13'sd0;
                center_a_d2 <= 13'sd0;
                center_b_d0 <= 13'sd0;
                center_b_d1 <= 13'sd0;
                center_b_d2 <= 13'sd0;
                accum_count <= 16'd0;
                i_acc_a <= 48'sd0;
                q_acc_a <= 48'sd0;
                i_acc_b <= 48'sd0;
                q_acc_b <= 48'sd0;
                cordic_x_a <= 32'sd0;
                cordic_y_a <= 32'sd0;
                cordic_x_b <= 32'sd0;
                cordic_y_b <= 32'sd0;
                cordic_in_valid <= 1'b0;
                cordic_in_data <= 64'd0;
            end else begin
                case (state)
                    ST_IDLE: begin
                        busy <= 1'b0;
                        cordic_in_valid <= 1'b0;
                        valid_d0 <= 1'b0;
                        valid_d1 <= 1'b0;
                        valid_d2 <= 1'b0;
                    end

                    ST_ACCUM: begin
                        // sin_lut_bram/cos_lut_bram are synchronous ROMs with
                        // 2-clock read latency; centered samples are delayed
                        // by 2 clocks to align with sin_q15/cos_q15.
                        if (sample_valid) begin
                            phase_acc <= phase_acc + phase_step_cfg;
                            center_a_d0 <= sample_a_centered;
                            center_b_d0 <= sample_b_centered;
                            valid_d0 <= 1'b1;
                            if (sample_a_raw == 12'd0 || sample_a_raw == 12'd4095 ||
                                sample_b_raw == 12'd0 || sample_b_raw == 12'd4095)
                                debug_flags <= debug_flags | DBG_INPUT_CLIPPED;
                        end else begin
                            valid_d0 <= 1'b0;
                        end

                        valid_d1 <= valid_d0;
                        valid_d2 <= valid_d1;
                        center_a_d1 <= center_a_d0;
                        center_a_d2 <= center_a_d1;
                        center_b_d1 <= center_b_d0;
                        center_b_d2 <= center_b_d1;

                        if (valid_d2) begin
                            i_acc_a <= i_acc_a_next;
                            q_acc_a <= q_acc_a_next;
                            i_acc_b <= i_acc_b_next;
                            q_acc_b <= q_acc_b_next;

                            if (accum_count >= target_count - 16'd1) begin
                                cordic_x_a <= scale_for_cordic(i_acc_a_next);
                                cordic_y_a <= scale_for_cordic(q_acc_a_next);
                                cordic_x_b <= scale_for_cordic(i_acc_b_next);
                                cordic_y_b <= scale_for_cordic(q_acc_b_next);
                                state <= ST_SEND_A;
                                debug_flags <= (debug_flags & ~DBG_ACCUM_ACTIVE);
                            end else begin
                                accum_count <= accum_count + 16'd1;
                            end
                        end
                    end

                    ST_SEND_A: begin
                        if (!cordic_in_valid) begin
                            cordic_in_valid <= 1'b1;
                            cordic_in_data <= CORDIC_XY_SWAP ?
                                {cordic_x_a, cordic_y_a} : {cordic_y_a, cordic_x_a};
                        end else if (cordic_in_ready) begin
                            cordic_in_valid <= 1'b0;
                            debug_flags <= debug_flags | DBG_CORDIC_A_SENT;
                            state <= ST_WAIT_A;
                        end
                    end

                    ST_WAIT_A: begin
                        cordic_in_valid <= 1'b0;
                        if (cordic_out_valid) begin
                            amp_a_code <= {1'b0, cordic_mag_out[30:0]};
                            phase_a_raw <= cordic_phase_out;
                            state <= ST_SEND_B;
                        end
                    end

                    ST_SEND_B: begin
                        if (!cordic_in_valid) begin
                            cordic_in_valid <= 1'b1;
                            cordic_in_data <= CORDIC_XY_SWAP ?
                                {cordic_x_b, cordic_y_b} : {cordic_y_b, cordic_x_b};
                        end else if (cordic_in_ready) begin
                            cordic_in_valid <= 1'b0;
                            debug_flags <= debug_flags | DBG_CORDIC_B_SENT;
                            state <= ST_WAIT_B;
                        end
                    end

                    ST_WAIT_B: begin
                        cordic_in_valid <= 1'b0;
                        if (cordic_out_valid) begin
                            amp_b_code <= {1'b0, cordic_mag_out[30:0]};
                            phase_b_raw <= cordic_phase_out;
                            phase_diff_raw <= phase_diff_wrapped_now[31:0];
                            phase_deg_x10 <= phase_deg_x10_now;
                            debug_flags <= debug_flags | DBG_RESULT_VALID;
                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= ST_DONE;
                        end
                    end

                    ST_DONE: begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        cordic_in_valid <= 1'b0;
                    end

                    default: begin
                        state <= ST_IDLE;
                        busy <= 1'b0;
                        cordic_in_valid <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
