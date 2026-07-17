/*
         [1, 2, 1]
kernal = [2, 4, 2]
         [1, 2, 1]
pixel_out = 1/16 * kernal * pixels_3x3_in

The two image line buffers are implemented with the existing image_line_fifo IP.
This keeps the 640x24x2 storage out of fabric registers and makes placement
much easier on the PGL50H design.
*/
`define GAUSSIAN_CORE_ENABLE

module image_gaussian_filter_3x3 #(
    parameter DW     = 24,
    parameter IW     = 640,
    parameter IH     = 480,
    parameter H_SYNC = 200
)(
    input               rst_n,

    input               pclk,
    input               i_hsync,
    input               i_vsync,
    input               i_de,
    input  [DW-1:0]     i_pixels,

    output              o_hsync,
    output              o_vsync,
    output              o_de,
    output [DW-1:0]     o_pixels
);

function integer clogb2;
    input integer depth;
    integer value;
    begin
        value = depth - 1;
        for (clogb2 = 0; value > 0; clogb2 = clogb2 + 1)
            value = value >> 1;
        if (clogb2 == 0)
            clogb2 = 1;
    end
endfunction

localparam COL_DW = clogb2(IW);
localparam ROW_DW = clogb2(IH + 1);

wire [DW-1:0] zero_pixel = {DW{1'b0}};

reg hsync_d1;
reg vsync_d1;
wire vsync_pos   = ~vsync_d1 && i_vsync;
wire hsync_neg   =  hsync_d1 && ~i_hsync;
wire pixel_valid = i_hsync && i_de;

reg [COL_DW-1:0] col_cnt;
reg [ROW_DW-1:0] row_cnt;

reg flush_active;
reg flush_right_pending;
reg [COL_DW-1:0] flush_col_cnt;
reg line_right_pending;
reg out_line_active;

wire fifo0_wr_en = pixel_valid && !flush_active;
wire fifo0_rd_en = (pixel_valid && (row_cnt != {ROW_DW{1'b0}})) || flush_active;
wire fifo1_rd_en = (pixel_valid && (row_cnt >= 2)) || flush_active;

wire [23:0] fifo0_rd_data;
wire [23:0] fifo1_rd_data;
wire        fifo0_wr_full;
wire        fifo0_rd_empty;
wire        fifo1_wr_full;
wire        fifo1_rd_empty;
wire        unused_af0;
wire        unused_ae0;
wire        unused_af1;
wire        unused_ae1;

reg         stage_valid;
reg         stage_flush;
reg [DW-1:0] stage_pixel;
reg [COL_DW-1:0] stage_col;
reg [ROW_DW-1:0] stage_row;

wire fifo1_wr_en = stage_valid && !stage_flush && (stage_row != {ROW_DW{1'b0}});

image_line_fifo u_line_fifo0 (
    .clk          (pclk),
    .rst          (~rst_n),
    .wr_en        (fifo0_wr_en),
    .wr_data      (i_pixels[23:0]),
    .wr_full      (fifo0_wr_full),
    .almost_full  (unused_af0),
    .rd_en        (fifo0_rd_en),
    .rd_data      (fifo0_rd_data),
    .rd_empty     (fifo0_rd_empty),
    .almost_empty (unused_ae0)
);

image_line_fifo u_line_fifo1 (
    .clk          (pclk),
    .rst          (~rst_n),
    .wr_en        (fifo1_wr_en),
    .wr_data      (fifo0_rd_data),
    .wr_full      (fifo1_wr_full),
    .almost_full  (unused_af1),
    .rd_en        (fifo1_rd_en),
    .rd_data      (fifo1_rd_data),
    .rd_empty     (fifo1_rd_empty),
    .almost_empty (unused_ae1)
);

reg [DW-1:0] p00, p01, p02;
reg [DW-1:0] p10, p11, p12;
reg [DW-1:0] p20, p21, p22;

reg          kernal_i_hsync;
reg          kernal_i_vsync;
reg          kernal_i_de;
reg [DW-1:0] kernal_p00, kernal_p01, kernal_p02;
reg [DW-1:0] kernal_p10, kernal_p11, kernal_p12;
reg [DW-1:0] kernal_p20, kernal_p21, kernal_p22;

wire [DW-1:0] line0_pixel = (stage_row == {ROW_DW{1'b0}}) ? zero_pixel : fifo0_rd_data[DW-1:0];
wire [DW-1:0] line1_pixel = (stage_row < 2) ? zero_pixel : fifo1_rd_data[DW-1:0];

image_gaussian_kernal_3x3 #(
    .DW (DW),
    .IW (IW),
    .IH (IH)
) u_image_gaussian_kernal_3x3 (
    .rst_n   (rst_n),
    .pclk    (pclk),
    .i_hsync (kernal_i_hsync),
    .i_vsync (kernal_i_vsync),
    .i_de    (kernal_i_de),
    .i_p00   (kernal_p00),
    .i_p01   (kernal_p01),
    .i_p02   (kernal_p02),
    .i_p10   (kernal_p10),
    .i_p11   (kernal_p11),
    .i_p12   (kernal_p12),
    .i_p20   (kernal_p20),
    .i_p21   (kernal_p21),
    .i_p22   (kernal_p22),
    .o_hsync (o_hsync),
    .o_vsync (o_vsync),
    .o_de    (o_de),
    .o_pixels(o_pixels)
);

always @(posedge pclk) begin
    if (~rst_n) begin
        hsync_d1            <= 1'b0;
        vsync_d1            <= 1'b0;
        col_cnt             <= {COL_DW{1'b0}};
        row_cnt             <= {ROW_DW{1'b0}};
        flush_active        <= 1'b0;
        flush_right_pending <= 1'b0;
        flush_col_cnt       <= {COL_DW{1'b0}};
        line_right_pending  <= 1'b0;
        out_line_active     <= 1'b0;
        stage_valid         <= 1'b0;
        stage_flush         <= 1'b0;
        stage_pixel         <= {DW{1'b0}};
        stage_col           <= {COL_DW{1'b0}};
        stage_row           <= {ROW_DW{1'b0}};
        p00                 <= {DW{1'b0}};
        p01                 <= {DW{1'b0}};
        p02                 <= {DW{1'b0}};
        p10                 <= {DW{1'b0}};
        p11                 <= {DW{1'b0}};
        p12                 <= {DW{1'b0}};
        p20                 <= {DW{1'b0}};
        p21                 <= {DW{1'b0}};
        p22                 <= {DW{1'b0}};
        kernal_i_hsync      <= 1'b0;
        kernal_i_vsync      <= 1'b0;
        kernal_i_de         <= 1'b0;
        kernal_p00          <= {DW{1'b0}};
        kernal_p01          <= {DW{1'b0}};
        kernal_p02          <= {DW{1'b0}};
        kernal_p10          <= {DW{1'b0}};
        kernal_p11          <= {DW{1'b0}};
        kernal_p12          <= {DW{1'b0}};
        kernal_p20          <= {DW{1'b0}};
        kernal_p21          <= {DW{1'b0}};
        kernal_p22          <= {DW{1'b0}};
    end
    else begin
        hsync_d1       <= i_hsync;
        vsync_d1       <= i_vsync;
        kernal_i_hsync <= out_line_active;
        kernal_i_de    <= 1'b0;
        kernal_i_vsync <= i_vsync | flush_active | stage_flush;

        stage_valid <= pixel_valid | flush_active;
        stage_flush <= flush_active;
        stage_pixel <= flush_active ? zero_pixel : i_pixels;
        stage_col   <= flush_active ? flush_col_cnt : col_cnt;
        stage_row   <= flush_active ? IH[ROW_DW-1:0] : row_cnt;

        if (vsync_pos) begin
            col_cnt             <= {COL_DW{1'b0}};
            row_cnt             <= {ROW_DW{1'b0}};
            flush_active        <= 1'b0;
            flush_right_pending <= 1'b0;
            flush_col_cnt       <= {COL_DW{1'b0}};
            line_right_pending  <= 1'b0;
            out_line_active     <= 1'b0;
            stage_valid         <= 1'b0;
            stage_flush         <= 1'b0;
            p00                 <= {DW{1'b0}};
            p01                 <= {DW{1'b0}};
            p02                 <= {DW{1'b0}};
            p10                 <= {DW{1'b0}};
            p11                 <= {DW{1'b0}};
            p12                 <= {DW{1'b0}};
            p20                 <= {DW{1'b0}};
            p21                 <= {DW{1'b0}};
            p22                 <= {DW{1'b0}};
        end
        else begin
            if (pixel_valid && !flush_active && (col_cnt != IW - 1))
                col_cnt <= col_cnt + 1'b1;

            if (flush_active && (flush_col_cnt != IW - 1))
                flush_col_cnt <= flush_col_cnt + 1'b1;
        end

        if (!vsync_pos && stage_valid) begin
            if ((stage_row != {ROW_DW{1'b0}}) && (stage_col != {COL_DW{1'b0}})) begin
                kernal_p00     <= p01;
                kernal_p01     <= p02;
                kernal_p02     <= line1_pixel;
                kernal_p10     <= p11;
                kernal_p11     <= p12;
                kernal_p12     <= line0_pixel;
                kernal_p20     <= p21;
                kernal_p21     <= p22;
                kernal_p22     <= stage_pixel;
                kernal_i_hsync <= 1'b1;
                kernal_i_de    <= 1'b1;
                out_line_active <= 1'b1;
            end

            p00 <= p01;
            p01 <= p02;
            p02 <= line1_pixel;
            p10 <= p11;
            p11 <= p12;
            p12 <= line0_pixel;
            p20 <= p21;
            p21 <= p22;
            p22 <= stage_pixel;

            if (stage_col == IW - 1)
                line_right_pending <= !stage_flush;

            if (stage_flush) begin
                if (stage_col == IW - 1) begin
                    stage_valid <= 1'b0;
                    flush_active <= 1'b0;
                    flush_right_pending <= 1'b1;
                end
            end
        end
        else if (!vsync_pos && line_right_pending) begin
            if (row_cnt != {ROW_DW{1'b0}}) begin
                kernal_p00     <= p01;
                kernal_p01     <= p02;
                kernal_p02     <= zero_pixel;
                kernal_p10     <= p11;
                kernal_p11     <= p12;
                kernal_p12     <= zero_pixel;
                kernal_p20     <= p21;
                kernal_p21     <= p22;
                kernal_p22     <= zero_pixel;
                kernal_i_hsync <= 1'b1;
                kernal_i_de    <= 1'b1;
            end

            line_right_pending <= 1'b0;
            out_line_active    <= 1'b0;
            col_cnt            <= {COL_DW{1'b0}};
            p00                <= {DW{1'b0}};
            p01                <= {DW{1'b0}};
            p02                <= {DW{1'b0}};
            p10                <= {DW{1'b0}};
            p11                <= {DW{1'b0}};
            p12                <= {DW{1'b0}};
            p20                <= {DW{1'b0}};
            p21                <= {DW{1'b0}};
            p22                <= {DW{1'b0}};

            if (row_cnt == IH - 1) begin
                flush_active        <= 1'b1;
                flush_right_pending <= 1'b0;
                flush_col_cnt       <= {COL_DW{1'b0}};
            end
            else begin
                row_cnt <= row_cnt + 1'b1;
            end
        end
        else if (!vsync_pos && flush_right_pending) begin
            kernal_p00           <= p01;
            kernal_p01           <= p02;
            kernal_p02           <= zero_pixel;
            kernal_p10           <= p11;
            kernal_p11           <= p12;
            kernal_p12           <= zero_pixel;
            kernal_p20           <= p21;
            kernal_p21           <= p22;
            kernal_p22           <= zero_pixel;
            kernal_i_hsync       <= 1'b1;
            kernal_i_de          <= 1'b1;
            out_line_active      <= 1'b0;
            flush_active         <= 1'b0;
            flush_right_pending  <= 1'b0;
            flush_col_cnt        <= {COL_DW{1'b0}};
            col_cnt              <= {COL_DW{1'b0}};
            row_cnt              <= {ROW_DW{1'b0}};
            p00                  <= {DW{1'b0}};
            p01                  <= {DW{1'b0}};
            p02                  <= {DW{1'b0}};
            p10                  <= {DW{1'b0}};
            p11                  <= {DW{1'b0}};
            p12                  <= {DW{1'b0}};
            p20                  <= {DW{1'b0}};
            p21                  <= {DW{1'b0}};
            p22                  <= {DW{1'b0}};
        end
        else if (!vsync_pos && hsync_neg) begin
            if ((col_cnt != {COL_DW{1'b0}}) && (col_cnt != IW - 1)) begin
                col_cnt <= {COL_DW{1'b0}};
                p00     <= {DW{1'b0}};
                p01     <= {DW{1'b0}};
                p02     <= {DW{1'b0}};
                p10     <= {DW{1'b0}};
                p11     <= {DW{1'b0}};
                p12     <= {DW{1'b0}};
                p20     <= {DW{1'b0}};
                p21     <= {DW{1'b0}};
                p22     <= {DW{1'b0}};
                out_line_active <= 1'b0;
                if (row_cnt != IH - 1)
                    row_cnt <= row_cnt + 1'b1;
            end
        end
    end
end

endmodule
