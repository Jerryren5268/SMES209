// YUV color range filter — pixel-level matching within YUV min/max, outputs binary mask + cached readout

module image_color_filter #(
    parameter IW            = 640,
    parameter IH            = 480,
    parameter Y_MAX         = 8'd255,
    parameter Y_MIN         = 8'd0,
    parameter U_MAX         = 8'd146,
    parameter U_MIN         = 8'd128,
    parameter V_MAX         = 8'd158,
    parameter V_MIN         = 8'd135,
    // Keep the original misspelled parameter for existing project instances.
    parameter CAHCE_ENABLE  = 0,
    parameter OUT_THRESHOLD = 4,
    parameter OUT_DIV2      = 1,
    parameter CACHE_ENABLE  = CAHCE_ENABLE
)(
    input               rst_n,

    input               pclk,
    input               i_hsync,
    input               i_vsync,
    input               i_de,
    input  [23:0]       i_pixels,

    output              o_hsync,
    output              o_vsync,
    output              o_de,
    output              o_match,

    input               rd_clk,
    input  [9:0]        i_rd_row,
    input  [9:0]        i_rd_col,
    input               i_rd_valid,
    output              o_rd_ready,
    output [31:0]       o_rd_32pix
);

localparam RD_BLOCKS_PER_ROW   = (IW + 63) / 64;
localparam [3:0] RD_COL_MAX    = RD_BLOCKS_PER_ROW - 1;

wire [7:0] i_y = i_pixels[23:16];
wire [7:0] i_u = i_pixels[15:8];
wire [7:0] i_v = i_pixels[7:0];

wire i_y_match = (i_y >= Y_MIN) && (i_y <= Y_MAX);
wire i_u_match = (i_u >= U_MIN) && (i_u <= U_MAX);
wire i_v_match = (i_v >= V_MIN) && (i_v <= V_MAX);
wire i_match   = i_y_match && i_u_match && i_v_match;
wire pixel_valid = i_hsync && i_de;

reg vsync_d1;
reg hsync_d1;
reg de_d1;
reg match_d1;

wire vsync_pos = ~vsync_d1 && i_vsync;
wire hsync_neg =  hsync_d1 && ~i_hsync;

reg [9:0] h_count;
reg [9:0] v_count;

// Local one-row storage for the 2x2 window. Binary_Image is reserved for
// the half-size frame cache used by the HDMI read interface.
reg  [IW-1:0] match_line;
wire        h_count_in_range = (h_count < IW);
wire [9:0]  safe_h_count = h_count_in_range ? h_count : 10'd0;
wire        match_01 = (v_count != 10'd0 && h_count_in_range)
                         ? match_line[safe_h_count]
                         : 1'b0;
reg          match_00;
reg          match_10;
wire [2:0]   match_sum = {2'b0, match_00} + {2'b0, match_01}
                               + {2'b0, match_10} + {2'b0, i_match};

reg half_hsync;
reg half_vsync;
reg half_de;
reg half_match;
reg [9:0] half_row;
reg [9:0] half_col;

always @(posedge pclk or negedge rst_n) begin
    if (~rst_n) begin
        vsync_d1  <= 1'b0;
        hsync_d1  <= 1'b0;
        de_d1     <= 1'b0;
        match_d1  <= 1'b0;
    end
    else begin
        vsync_d1  <= i_vsync;
        hsync_d1  <= i_hsync;
        de_d1     <= i_de;
        match_d1  <= i_match;
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

// The old bit remains visible during the active edge and is then replaced
// by the current-row result for use on the next line.
always @(posedge pclk) begin
    if (pixel_valid)
        match_line[h_count] <= i_match;
end

always @(posedge pclk or negedge rst_n) begin
    if (~rst_n) begin
        match_00 <= 1'b0;
        match_10 <= 1'b0;
    end
    else if (vsync_pos || hsync_neg) begin
        match_00 <= 1'b0;
        match_10 <= 1'b0;
    end
    else if (pixel_valid) begin
        match_00 <= match_01;
        match_10 <= i_match;
    end
end

// The selected point is the lower-right pixel of each complete 2x2 block.
always @(posedge pclk or negedge rst_n) begin
    if (~rst_n) begin
        half_vsync <= 1'b0;
        half_hsync <= 1'b0;
        half_de    <= 1'b0;
        half_match <= 1'b0;
        half_row   <= 10'd0;
        half_col   <= 10'd0;
    end
    else begin
        half_vsync <= i_vsync;
        half_hsync <= i_hsync && v_count[0];
        half_de    <= pixel_valid && v_count[0] && h_count[0];

        if (pixel_valid && v_count[0] && h_count[0]) begin
            half_match <= (match_sum >= OUT_THRESHOLD[2:0]);
            half_row   <= v_count;
            half_col   <= h_count;
        end
        else begin
            half_match <= 1'b0;
        end
    end
end

generate
    if (OUT_DIV2 == 0) begin : g_full_output
        assign o_vsync = vsync_d1;
        assign o_hsync = hsync_d1;
        assign o_de    = de_d1;
        assign o_match = match_d1;
    end
    else begin : g_half_output
        assign o_vsync = half_vsync;
        assign o_hsync = half_hsync;
        assign o_de    = half_de;
        assign o_match = half_match;
    end
endgenerate

// The BRAM stores the half-size binary image in rows padded to 512 pixels.
wire        hbi_wr_en   = half_de;
wire        hbi_wr_data = half_match;
wire [16:0] hbi_wr_addr = {half_row[8:1], half_col[9:1]};

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
        Binary_Image u_half_binary_image (
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
                i_rd_valid_d1 <= 1'b0;
                hbi_rd_row    <= 8'd0;
                hbi_issue_col <= 4'd0;
                hbi_return_col <= 4'd0;
                hbi_issue_active <= 1'b0;
                hbi_data_pending <= 1'b0;
                hbi_row_valid <= 1'b0;
                hbi_rd_ready  <= 1'b0;
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





// YUV color range filter — pixel-level matching within YUV min/max, outputs binary mask + cached readout

// module image_color_filter #(
// 	// 图像基本参数
// 	parameter	IW		=	640			,	// SOURCE 图像宽（image width）
// 	parameter	IH		=	480			,	// SOURCE 图像高（image height）
// 	parameter	Y_MAX	= 	8'd255		,	// YUV 目标颜色范围, 闭合区间, [min, max]
// 	parameter	Y_MIN	= 	8'd128		,
// 	parameter	U_MAX	= 	8'd255		,
// 	parameter	U_MIN	= 	8'd128		,
// 	parameter	V_MAX	= 	8'd255		,
// 	parameter	V_MIN	= 	8'd128		,
// 	parameter	CAHCE_ENABLE  = 0		,
// 	parameter 	OUT_THRESHOLD = 4		,	// 3 or 4, in 2x2 patch
// 	parameter	OUT_DIV2  = 1				// enable half image or not
// )
// (
// 	// 系统信号
// 	input 				rst_n			,	// 复位（reset）

// 	// source 视频信号
// 	input 				pclk			,	// input 像素时钟输出（pixel clock）
// 	input 				i_hsync			,	// input 行同步信号（数据有效输出中标志）
// 	input 				i_vsync			,	// input 场同步信号
// 	input				i_de			,	// input 像素数据有效位
// 	input [23:0] 		i_pixels		,	// input 像素数据输入, YUV888

// 	// 处理后输出视频信号
// 	output 				o_hsync			,	// output 行同步信号（数据有效输出中标志）
// 	output 		    	o_vsync			,	// output 场同步信号, 对 i_vsync 打一拍
// 	output 				o_de			,	// output 像素数据有效位
// 	output     		    o_match			,	// output 当前像素是否匹配颜色范围

// 	// 读取缓存的二值化图片
// 	input				rd_clk 			,   // read clock
// 	input [9:0]			i_rd_row		,	// each read action by row index only
// 	input [9:0]			i_rd_col		,	// col[9:6] will decide which part of data to output
// 	input 				i_rd_valid		,	// request to take read action
// 	output 				o_rd_ready		,   // callback to requester, data is ready to read out
// 	output [31:0]		o_rd_32pix			// offset by col[5:1], coz only cache 2x2 patch
// );

// // TODO 
// reg o_hsync;
// reg o_vsync;
// reg o_de;
// reg o_match;
// always @(posedge pclk) begin
// 	o_vsync	<= i_vsync;
// 	o_hsync <= i_hsync;
// 	o_de 	<= i_de;
// 	o_match	<= 1'b0;
// end

// assign o_rd_ready = 1'b1; // always ready to read
// assign o_rd_32pix = 32'h0; // always return 0


// endmodule