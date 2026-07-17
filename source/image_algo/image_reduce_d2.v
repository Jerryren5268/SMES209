// 1/2 image downscale — halves resolution by 2x2 median pooling

module image_reduce_d2 #(

    // 目的视频参数
	parameter	IMG_DW		=	16		,  // 目的视频输出像素数据位宽（data width of source）

	// 图像基本参数
	parameter	IW			=	640		,   // SOURCE 图像宽（image width）
	parameter	IH			=	480		,   // SOURCE 图像高（image height）
	parameter	OW			=	IW / 2	,   // OUTPUT 图像宽（image width）
	parameter	OH			=	IH / 2	,   // OUTPUT图像高（image height）
	parameter	OW_DW    	=	9		,   // OW 的数字宽度
	parameter	IW_DW    	=	OW_DW + 1	,   // IW 的数字宽度
	parameter	IH_DW    	=	10		    // IH 的数字宽度
)
(
	// 系统信号
	rst_n			,	// 复位（reset）

	// source 视频信号
	pclk			,	// input 像素时钟输出（pixel clock）
	i_hsync			,	// input 行同步信号（数据有效输出中标志）
	i_vsync			,	// input 场同步信号
	i_de			,	// input 像素数据有效位
	i_pixels		,	// input 像素数据输出

	// 处理后输出视频信号
	o_hsync			,	// output 行同步信号（数据有效输出中标志）
	o_vsync			,	// output 场同步信号
	o_de			,	// output 像素数据有效位
	o_pixels			// output 像素数据输出
);


	// *******************************************端口声明***************************************
	// 系统信号
	input					rst_n			;	// 复位（reset）

	// source 视频信号
	input					pclk			;	// source 像素时钟输出（pixel clock）
	input					i_hsync			;	// source 行同步信号
	input					i_vsync			;	// source 场同步信号
	input 					i_de			;	// source 像素数据有效位
	input		[IMG_DW-1:0]	i_pixels		;	// source 像素数据输入

	// 处理后输出视频信号
	output reg				o_hsync			;	// output, 行同步信号, 当前输出行有效
	output reg				o_vsync			;	// output, 场同步信号 打1拍
	output reg				o_de			;	// output, 像素数据有效位
	output reg  [IMG_DW-1:0]	o_pixels		;	// output, 像素数据输出

	// *******************************************************************************************


	// ******************************************内部信号声明*************************************
	reg                         	hsync_d1		;	// 行同步信号 打1拍
	wire						vsync_pos		;	// **场**同步信号 上升沿
	wire						hsync_neg		;	// **行**同步信号 下降沿

	reg		[IW_DW-1:0]		col_cnt 		;	// 当前输入列索引
	reg		[IH_DW-1:0]		row_cnt 		;	// 当前输入行索引

	reg		[IMG_DW-1:0]	line_buf [0:IW-1]	;	// 上一行像素缓存
	reg		[IMG_DW-1:0]	left_upper_pixel	;	// 上一行上一列像素
	reg		[IMG_DW-1:0]	left_curr_pixel		;	// 当前行上一列像素

	wire						pixel_valid		;	// 当前输入像素有效
	// *******************************************************************************************

	assign	vsync_pos	=	~o_vsync && i_vsync;	// 01
	assign	hsync_neg	=	hsync_d1 && ~i_hsync;	// 10
	assign	pixel_valid	=	i_de & i_hsync;

	// RGB565 分通道排序，返回第 2/3 小值的平均值，作为 2x2 中值池化结果
	function [4:0] median4_avg_5;
		input [4:0] a;
		input [4:0] b;
		input [4:0] c;
		input [4:0] d;
		reg   [4:0] x0;
		reg   [4:0] x1;
		reg   [4:0] x2;
		reg   [4:0] x3;
		reg   [4:0] t;
		begin
			x0 = a;
			x1 = b;
			x2 = c;
			x3 = d;

			if (x0 > x1) begin t = x0; x0 = x1; x1 = t; end
			if (x2 > x3) begin t = x2; x2 = x3; x3 = t; end
			if (x0 > x2) begin t = x0; x0 = x2; x2 = t; end
			if (x1 > x3) begin t = x1; x1 = x3; x3 = t; end
			if (x1 > x2) begin t = x1; x1 = x2; x2 = t; end

			median4_avg_5 = ({1'b0, x1} + {1'b0, x2}) >> 1;
		end
	endfunction

	function [5:0] median4_avg_6;
		input [5:0] a;
		input [5:0] b;
		input [5:0] c;
		input [5:0] d;
		reg   [5:0] x0;
		reg   [5:0] x1;
		reg   [5:0] x2;
		reg   [5:0] x3;
		reg   [5:0] t;
		begin
			x0 = a;
			x1 = b;
			x2 = c;
			x3 = d;

			if (x0 > x1) begin t = x0; x0 = x1; x1 = t; end
			if (x2 > x3) begin t = x2; x2 = x3; x3 = t; end
			if (x0 > x2) begin t = x0; x0 = x2; x2 = t; end
			if (x1 > x3) begin t = x1; x1 = x3; x3 = t; end
			if (x1 > x2) begin t = x1; x1 = x2; x2 = t; end

			median4_avg_6 = ({1'b0, x1} + {1'b0, x2}) >> 1;
		end
	endfunction

	// 场/行同步信号延迟，用于边沿检测和输出同步
	always @(posedge pclk)
	begin
		if(~rst_n)
		begin
			o_vsync  <= 1'b0;
			hsync_d1 <= 1'b0;
		end
		else
		begin
			o_vsync  <= i_vsync;
			hsync_d1 <= i_hsync;
		end
	end

	// 输入行列计数
	always @(posedge pclk)
	begin
		if(~rst_n)
		begin
			row_cnt <= {IH_DW{1'b0}};
			col_cnt <= {IW_DW{1'b0}};
		end
		else if(vsync_pos)
		begin
			row_cnt <= {IH_DW{1'b0}};
			col_cnt <= {IW_DW{1'b0}};
		end
		else if(hsync_neg)
		begin
			col_cnt <= {IW_DW{1'b0}};
			if(row_cnt == IH - 1)
				row_cnt <= {IH_DW{1'b0}};
			else
				row_cnt <= row_cnt + 1'b1;
		end
		else if(pixel_valid)
		begin
			if(col_cnt == IW - 1)
				col_cnt <= col_cnt;
			else
				col_cnt <= col_cnt + 1'b1;
		end
	end

	// 行缓存和 2x2 窗口缓存
	always @(posedge pclk)
	begin
		if(~rst_n)
		begin
			left_upper_pixel <= {IMG_DW{1'b0}};
			left_curr_pixel  <= {IMG_DW{1'b0}};
		end
		else if(vsync_pos | hsync_neg)
		begin
			left_upper_pixel <= {IMG_DW{1'b0}};
			left_curr_pixel  <= {IMG_DW{1'b0}};
		end
		else if(pixel_valid)
		begin
			left_upper_pixel <= line_buf[col_cnt];
			left_curr_pixel  <= i_pixels;
			line_buf[col_cnt] <= i_pixels;
		end
	end

	// 输出 2x2 中值池化结果：只在奇数行、奇数列输出
	always @(posedge pclk)
	begin
		if(~rst_n)
		begin
			o_hsync <= 1'b0;
			o_de     <= 1'b0;
			o_pixels <= {IMG_DW{1'b0}};
		end
		else
		begin
			o_hsync <= i_hsync & row_cnt[0];
			o_de    <= pixel_valid & row_cnt[0] & col_cnt[0];

			if(pixel_valid & row_cnt[0] & col_cnt[0])
			begin
					o_pixels <= {
						median4_avg_5(left_upper_pixel[15:11], line_buf[col_cnt][15:11], left_curr_pixel[15:11], i_pixels[15:11]),
						median4_avg_6(left_upper_pixel[10:5],  line_buf[col_cnt][10:5],  left_curr_pixel[10:5],  i_pixels[10:5]),
						median4_avg_5(left_upper_pixel[4:0],   line_buf[col_cnt][4:0],   left_curr_pixel[4:0],   i_pixels[4:0])
					};
			end
			else if(vsync_pos)
			begin
				o_pixels <= {IMG_DW{1'b0}};
			end
		end
	end

endmodule
//1/2 bilinear image downscale — halves resolution by decimating alternate rows and columns

// module image_reduce_d2 #(

//     // 目的视频参数
// 	parameter	IMG_DW		=	16		,  // 目的视频输出像素数据位宽（data width of source）
	
// 	// 图像基本参数
// 	parameter	IW			=	640		,	// SOURCE 图像宽（image width）
// 	parameter	IH			=	480		,	// SOURCE 图像高（image height）
// 	parameter	OW			=	IW / 2	,	// OUTPUT 图像宽（image width）
// 	parameter	OH			=	IH / 2	,	// OUTPUT图像高（image height）
// 	parameter	OW_DW    	=   9			// OW 的数字宽度
// )
// (
// 	// 系统信号
// 	rst_n			,	// 复位（reset）
	
// 	// source 视频信号
// 	pclk			,	// input 像素时钟输出（pixel clock）
// 	i_hsync			,	// input 行同步信号（数据有效输出中标志）
// 	i_vsync			,	// input 场同步信号
// 	i_de			,	// input 像素数据有效位
// 	i_pixels		,	// input 像素数据输出

// 	// 处理后输出视频信号
// 	o_hsync			,	// output 行同步信号（数据有效输出中标志）
// 	o_vsync			,	// output 场同步信号
// 	o_de			,	// output 像素数据有效位
// 	o_pixels		,	// output 像素数据输出
// );
	
	
// 	// *******************************************端口声明***************************************
// 	// 系统信号
// 	input						rst_n			;	// 复位（reset）
	
// 	// source 视频信号
// 	input						pclk			;	// source 像素时钟输出（pixel clock）
// 	input						i_hsync			;	// source 行同步信号
// 	input						i_vsync			;	// source 场同步信号
// 	input 						i_de			;	// source 像素数据有效位
// 	input		[IMG_DW-1:0]	i_pixels		;	// source 像素数据输入

// 	// 处理后输出视频信号
// 	output 						o_hsync			;	// output, 行同步信号 打1拍, 并当前行输出有效
// 	output reg					o_vsync			;	// output, 场同步信号 打1拍
// 	output 						o_de			;	// output, 像素数据有效位
// 	output reg  [IMG_DW-1:0]	o_pixels		;	// output, 像素数据输出

// 	// *******************************************************************************************
	
	
// 	// ******************************************内部信号声明*************************************
//     reg                         de_d1           ;
// 	reg                        	hsync_d1		;	// 行同步信号 打1拍
// 	wire						vsync_pos		;	// **场**同步信号 上升沿
// 	wire						vsync_neg		;	// **场**同步信号 下降沿
// 	wire						hsync_pos		;	// **行**同步信号 上升沿
// 	wire						hsync_neg		;	// **行**同步信号 下降沿
// 	// *******************************************************************************************
	
// 	// 目的视频场/行同步信号 打1拍（用于检测目的视频场同步信号上升沿、下降沿）
// 	always @(posedge pclk)
// 	begin
// 		if(~rst_n) 
// 		begin
// 			o_vsync <= 1'b0;
// 			hsync_d1 <= 1'b0;
//             de_d1   <= 1'b0;
// 		end 
// 		else
// 		begin
// 			o_vsync <= i_vsync;
// 			hsync_d1 <= i_hsync;
//             de_d1   <= i_de;
// 		end 
// 	end
	
// 	// 目的视频场/行同步信号 上升沿、下降沿
// 	assign	vsync_pos	=	~o_vsync && i_vsync;	// 01
// 	assign	vsync_neg	=	o_vsync && ~i_vsync;	// 10
// 	assign	hsync_pos	=	~hsync_d1 && i_hsync;	// 01
// 	assign	hsync_neg	=	hsync_d1 && ~i_hsync;	// 10

// 	// 当前列索引
// 	reg [OW_DW-1:0]				col_idx 			;

// 	// 当前**行**是否需要输出 
// 	reg 						row_enable			; // 0 - 缓存, 1 - 输出
// 	// 当前**列**是否需要输出 
// 	reg 						col_enable			; // 0 - 缓存, 1 - 输出

// 	assign	o_de 		= 		row_enable & col_enable & de_d1	;
// 	assign  o_hsync	    = 		row_enable & hsync_d1	;

// 	always @(posedge pclk)
// 	begin
// 		if(~rst_n) 
// 		begin
//             o_pixels <= {IMG_DW{1'b0}};
// 		end 
// 		else
// 		begin
// 			o_pixels <= i_pixels;
// 		end 
// 	end

// 	always @(posedge pclk)
// 	begin
// 		if(~rst_n)
// 			row_enable <= 1'b0;
// 		else if (vsync_pos)
// 			row_enable <= 1'b0;
// 		else if (hsync_neg)
// 			row_enable <= ~row_enable;
// 	end

// 	always @(posedge pclk)
// 	begin
// 		if(~rst_n) 
// 		begin 
// 			col_enable <= 1'b0;
// 			col_idx	   <= {OW_DW{1'b0}};
// 		end 
// 		else if (vsync_pos | hsync_neg) 
// 		begin
// 			col_enable <= 1'b0;
// 			col_idx	   <= {OW_DW{1'b0}};
// 		end 
// 		else if (i_de & i_hsync)
// 		begin
// 			col_enable <= ~col_enable;
// 			if (row_enable & col_enable) begin
// 				col_idx	<= col_idx + 1'b1;
// 			end 
// 			else if (~row_enable & col_enable) begin
// 				col_idx	<= col_idx + 1'b1;
// 			end 
// 		end 
// 	end
	
// endmodule
