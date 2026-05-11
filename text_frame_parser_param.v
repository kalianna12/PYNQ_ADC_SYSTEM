module text_frame_parser_param #(
    parameter [7:0] EXPECT_TYPE = 8'hE2
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        parse_en,
    input  wire [1023:0] frame,
    output reg         valid,
    output reg  [31:0] seq,
    output reg  [31:0] text_len,
    output reg  [831:0] text_bytes
);

    integer i;
    reg [7:0] checksum;
    reg [31:0] capped_len;

    always @(posedge clk) begin
        if (rst) begin
            valid <= 1'b0;
            seq <= 32'd0;
            text_len <= 32'd0;
            text_bytes <= 832'd0;
        end else begin
            valid <= 1'b0;

            if (parse_en) begin
                checksum = 8'd0;
                for (i = 0; i < 116; i = i + 1) begin
                    checksum = checksum ^ frame[i*8 +: 8];
                end

                if (frame[0*8 +: 8] == 8'hA5 &&
                    frame[1*8 +: 8] == 8'h5A &&
                    frame[2*8 +: 8] == EXPECT_TYPE &&
                    frame[3*8 +: 8] == 8'd112 &&
                    frame[116*8 +: 8] == checksum) begin

                    seq <= {frame[7*8 +: 8], frame[6*8 +: 8], frame[5*8 +: 8], frame[4*8 +: 8]};
                    capped_len = {frame[11*8 +: 8], frame[10*8 +: 8], frame[9*8 +: 8], frame[8*8 +: 8]};
                    if (capped_len > 32'd104) begin
                        capped_len = 32'd104;
                    end
                    text_len <= capped_len;

                    for (i = 0; i < 104; i = i + 1) begin
                        text_bytes[i*8 +: 8] <= (i < capped_len) ? frame[(12 + i)*8 +: 8] : 8'd0;
                    end

                    valid <= 1'b1;
                end
            end
        end
    end

endmodule
