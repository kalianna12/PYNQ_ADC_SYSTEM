module text_frame_builder_param #(
    parameter [7:0] FRAME_TYPE = 8'hE1
) (
    input  wire [31:0]  seq,
    input  wire [31:0]  text_len,
    input  wire [831:0] text_bytes,
    output reg  [1023:0] frame
);

    integer i;
    reg [7:0] checksum;
    reg [31:0] capped_len;

    always @* begin
        frame = 1024'd0;
        capped_len = (text_len > 32'd104) ? 32'd104 : text_len;

        frame[0*8 +: 8] = 8'hA5;
        frame[1*8 +: 8] = 8'h5A;
        frame[2*8 +: 8] = FRAME_TYPE;
        frame[3*8 +: 8] = 8'd112;

        frame[4*8 +: 8] = seq[7:0];
        frame[5*8 +: 8] = seq[15:8];
        frame[6*8 +: 8] = seq[23:16];
        frame[7*8 +: 8] = seq[31:24];

        frame[8*8 +: 8]  = capped_len[7:0];
        frame[9*8 +: 8]  = capped_len[15:8];
        frame[10*8 +: 8] = capped_len[23:16];
        frame[11*8 +: 8] = capped_len[31:24];

        for (i = 0; i < 104; i = i + 1) begin
            frame[(12 + i)*8 +: 8] = (i < capped_len) ? text_bytes[i*8 +: 8] : 8'd0;
        end

        checksum = 8'd0;
        for (i = 0; i < 116; i = i + 1) begin
            checksum = checksum ^ frame[i*8 +: 8];
        end
        frame[116*8 +: 8] = checksum;
    end

endmodule
