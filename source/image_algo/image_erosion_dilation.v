// Binary morphology: 3x3 erosion or dilation over a 1-bit match stream.

module image_erosion_dilation #(
    parameter IW            = 640,
    parameter IH            = 480,
    parameter H_SYNC        = 200,
    parameter E_Dn          = 1,    // 1 - erosion/min, 0 - dilation/max
    // Keep the original misspelled parameter for existing project instances.
    parameter CAHCE_ENABLE  = 0,
    parameter CACHE_ENABLE  = CAHCE_ENABLE
)(
    input               rst_n,

    input               pclk,
    input               i_hsync,
    input               i_vsync,
    input               i_de,
    input               i_match,

    output reg          o_hsync,
    output reg          o_vsync,
    output reg          o_de,
    output reg          o_match,

    input               rd_clk,
    input  [9:0]        i_rd_row,
    input  [9:0]        i_rd_col,
    input               i_rd_valid,
    output              o_rd_ready,
    output [31:0]       o_rd_32pix
);

localparam RD_BLOCKS_RAW     = (IW + 31) / 32;
localparam RD_BLOCKS_PER_ROW = (RD_BLOCKS_RAW > 16) ? 16 : RD_BLOCKS_RAW;
localparam [3:0] RD_COL_MAX  = RD_BLOCKS_PER_ROW - 1;

wire pixel_valid = i_hsync && i_de;

reg vsync_d1;
reg hsync_d1;

wire vsync_pos = ~vsync_d1 && i_vsync;
wire hsync_neg =  hsync_d1 && ~i_hsync;

reg [9:0] h_count;
reg [9:0] v_count;

// Two 1-bit previous-line stores form the vertical part of the 3x3 window.
// They are intentionally small here: post_image_process instantiates this
// module at half resolution, so each instance uses only a few hundred bits.
reg [IW-1:0] line0;
reg [IW-1:0] line1;

wire h_count_in_range = (h_count < IW);
wire [9:0] safe_h_count = h_count_in_range ? h_count : 10'd0;
wire line0_pixel = (v_count != 10'd0 && h_count_in_range) ? line0[safe_h_count] : 1'b0;
wire line1_pixel = (v_count >  10'd1 && h_count_in_range) ? line1[safe_h_count] : 1'b0;

reg p00, p01, p02;
reg p10, p11, p12;
reg p20, p21, p22;

wire n00 = p01;
wire n01 = p02;
wire n02 = line1_pixel;
wire n10 = p11;
wire n11 = p12;
wire n12 = line0_pixel;
wire n20 = p21;
wire n21 = p22;
wire n22 = i_match;

wire erosion_match  = n00 & n01 & n02 & n10 & n11 & n12 & n20 & n21 & n22;
wire dilation_match = n00 | n01 | n02 | n10 | n11 | n12 | n20 | n21 | n22;
wire morph_match    = (E_Dn != 0) ? erosion_match : dilation_match;

reg [9:0] out_row;
reg [9:0] out_col;

always @(posedge pclk or negedge rst_n) begin
    if (~rst_n) begin
        vsync_d1 <= 1'b0;
        hsync_d1 <= 1'b0;
    end
    else begin
        vsync_d1 <= i_vsync;
        hsync_d1 <= i_hsync;
    end
end

always @(posedge pclk or negedge rst_n) begin
    if (~rst_n) begin
        h_count <= 10'd0;
        v_count <= 10'd0;
    end
    else if (vsync_pos) begin
        h_count <= 10'd0;
        v_count <= 10'd0;
    end
    else if (hsync_neg) begin
        h_count <= 10'd0;
        v_count <= v_count + 10'd1;
    end
    else if (pixel_valid) begin
        h_count <= h_count + 10'd1;
    end
end

always @(posedge pclk) begin
    if (pixel_valid) begin
        line1[h_count] <= line0[h_count];
        line0[h_count] <= i_match;
    end
end

always @(posedge pclk or negedge rst_n) begin
    if (~rst_n) begin
        p00 <= 1'b0; p01 <= 1'b0; p02 <= 1'b0;
        p10 <= 1'b0; p11 <= 1'b0; p12 <= 1'b0;
        p20 <= 1'b0; p21 <= 1'b0; p22 <= 1'b0;
    end
    else if (vsync_pos || hsync_neg) begin
        p00 <= 1'b0; p01 <= 1'b0; p02 <= 1'b0;
        p10 <= 1'b0; p11 <= 1'b0; p12 <= 1'b0;
        p20 <= 1'b0; p21 <= 1'b0; p22 <= 1'b0;
    end
    else if (pixel_valid) begin
        p00 <= n00; p01 <= n01; p02 <= n02;
        p10 <= n10; p11 <= n11; p12 <= n12;
        p20 <= n20; p21 <= n21; p22 <= n22;
    end
end

always @(posedge pclk or negedge rst_n) begin
    if (~rst_n) begin
        o_vsync <= 1'b0;
        o_hsync <= 1'b0;
        o_de    <= 1'b0;
        o_match <= 1'b0;
        out_row <= 10'd0;
        out_col <= 10'd0;
    end
    else begin
        o_vsync <= i_vsync;
        o_hsync <= i_hsync;
        o_de    <= pixel_valid;

        if (pixel_valid) begin
            o_match <= morph_match;
            out_row <= v_count;
            out_col <= h_count;
        end
        else begin
            o_match <= 1'b0;
        end
    end
end

// The BRAM stores one binary pixel per write address and reads 32 pixels per
// block. External read coordinates remain full-size HDMI coordinates, matching
// image_color_filter: row/column are divided by two at the read interface.
wire        hbi_wr_en   = o_de;
wire        hbi_wr_data = o_match;
wire [16:0] hbi_wr_addr = {out_row[7:0], out_col[8:0]};

wire [31:0] hbi_rd_data;
reg  [31:0] hbi_rd_cache [0:15];
reg  [7:0]  hbi_rd_row;
reg  [3:0]  hbi_issue_col;
reg  [3:0]  hbi_return_col;
reg         hbi_issue_active;
reg         hbi_data_pending;
reg         hbi_row_valid;
reg         hbi_rd_ready;
reg         i_rd_valid_d1;

wire [11:0] hbi_rd_addr = {hbi_rd_row, hbi_issue_col};
wire i_rd_valid_start = i_rd_valid && ~i_rd_valid_d1;
wire requested_row_hit = hbi_row_valid && (hbi_rd_row == i_rd_row[8:1]);
wire [3:0] requested_col = i_rd_col[9:6];
wire requested_col_valid = (requested_col <= RD_COL_MAX);

generate
    if (CACHE_ENABLE > 0) begin : g_cache
        Binary_Image u_binary_image (
            .wr_data (hbi_wr_data),
            .wr_addr (hbi_wr_addr),
            .wr_en   (hbi_wr_en),
            .wr_clk  (pclk),
            .wr_rst  (~rst_n),
            .rd_data (hbi_rd_data),
            .rd_addr (hbi_rd_addr),
            .rd_clk  (rd_clk),
            .rd_rst  (~rst_n)
        );

        always @(posedge rd_clk or negedge rst_n) begin
            if (~rst_n) begin
                i_rd_valid_d1   <= 1'b0;
                hbi_rd_row      <= 8'd0;
                hbi_issue_col   <= 4'd0;
                hbi_return_col  <= 4'd0;
                hbi_issue_active <= 1'b0;
                hbi_data_pending <= 1'b0;
                hbi_row_valid   <= 1'b0;
                hbi_rd_ready    <= 1'b0;
            end
            else begin
                i_rd_valid_d1 <= i_rd_valid;

                if (i_rd_valid_start && !requested_row_hit) begin
                    hbi_rd_row       <= i_rd_row[8:1];
                    hbi_issue_col    <= 4'd0;
                    hbi_issue_active <= 1'b1;
                    hbi_data_pending <= 1'b0;
                    hbi_row_valid    <= 1'b0;
                    hbi_rd_ready     <= 1'b0;
                end
                else begin
                    if (hbi_issue_active) begin
                        hbi_return_col   <= hbi_issue_col;
                        hbi_data_pending <= 1'b1;

                        if (hbi_issue_col == RD_COL_MAX)
                            hbi_issue_active <= 1'b0;
                        else
                            hbi_issue_col <= hbi_issue_col + 4'd1;
                    end
                    else begin
                        hbi_data_pending <= 1'b0;
                    end

                    if (hbi_data_pending) begin
                        hbi_rd_cache[hbi_return_col] <= hbi_rd_data;
                        if (hbi_return_col == RD_COL_MAX) begin
                            hbi_row_valid <= 1'b1;
                            hbi_rd_ready  <= 1'b1;
                        end
                    end
                end
            end
        end

        assign o_rd_ready = requested_row_hit && hbi_rd_ready;
        assign o_rd_32pix = (requested_row_hit && hbi_rd_ready && requested_col_valid)
                           ? hbi_rd_cache[requested_col]
                           : 32'd0;
    end
    else begin : g_no_cache
        assign hbi_rd_data = 32'd0;
        assign o_rd_ready  = 1'b0;
        assign o_rd_32pix  = 32'd0;
    end
endgenerate

endmodule
