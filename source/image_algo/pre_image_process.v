// Image pre-processing pipeline: 1/2 downscale → RGB565 to YUV → 3x3 Gaussian filter
// Dual output: 16-bit RGB565 for DDR write path, 24-bit YUV888 for algorithm pipeline

`define OUTPUT_NORMAL
// `define OUTPUT_GRAY
// `define OUTPUT_GF_GRAY

`define ENDIAN_GF_YUV

module pre_image_process #(
   // 图像基本参数
   parameter	IW = 12'd1280  ,	// SOURCE 图像宽（image width）
   parameter	IH	= 12'd720      // SOURCE 图像高（image height）
)
(
   // 系统信号
   input rst_n			         ,	// 复位（reset）

   // source 视频信号
   input pclk			         ,	// input 像素时钟输出（pixel clock）
   input i_hsync		         ,	// input 行同步信号（数据有效输出中标志）
   input i_vsync		         ,	// input 场同步信号
   input i_de			         ,	// input 像素数据有效位
   input [15:0] i_pixels		 ,	// input 像素数据输出

   // 针对显示输出视频信号
   output o_hsync		         ,	// output 行同步信号（数据有效输出中标志）
   output o_vsync		         ,	// output 场同步信号
   output o_de			         ,	// output 像素数据有效位
   output [15:0] o_pixels		 , // output 像素数据输出

   // pipeline末端输出视频信号
   output e_hsync		         ,	// output 行同步信号（数据有效输出中标志）
   output e_vsync		         ,	// output 场同步信号
   output e_de			         ,	// output 像素数据有效位
   output [23:0] e_pixels			// output 像素数据输出 YUV888
);


/************************ step1, image reduce ************************/

localparam IW_HALF = IW / 2;
localparam IH_HALF = IH / 2;

wire [15:0] o1_image_pixels	;
wire 		o1_hsync		;
wire 		o1_vsync		;
wire 		o1_de			;

image_reduce_d2 #(
   	.IMG_DW        ( 16      	),  // 目的视频输出像素数据位宽（data width of source）
	.IW            ( IW		 	),  // SOURCE 图像宽（image width）
	.IH            ( IH      	),  // SOURCE 图像高（image height）
	.OW_DW         ( 10      	)	  // OW 的数字宽度, 1280 /2 = 640
)
u_image_reduce (
	// 系统信号
	.rst_n			(rst_n    	),	// 复位（reset）
	
	// source 视频信号
	.pclk			(pclk	  	),	// input 像素时钟输出（pixel clock）
	.i_hsync		(i_hsync	),	// input 行同步信号（数据有效输出中标志）
	.i_vsync		(i_vsync	),	// input 场同步信号
	.i_de			(i_de		),	// input 像素数据有效位
	.i_pixels		(i_pixels	),	// input 像素数据输出

	// 处理后输出视频信号
	.o_hsync		(o1_hsync	),	// output 行同步信号（数据有效输出中标志）
	.o_vsync		(o1_vsync	),	// output 场同步信号
	.o_de			(o1_de		),	// output 像素数据有效位
	.o_pixels		(o1_image_pixels)	// output 像素数据输出
);

///////////////////////////////////////////////////////////////////////


/************************ step2, RGB TO YUV   ************************/

wire [15:0] o2_image_pixels;
wire [ 7:0] o2_image_y;
wire [ 7:0] o2_image_u;
wire [ 7:0] o2_image_v;
wire [ 7:0] o2_image_gray;

rgb565_to_yuv u_rgb565_to_yuv
(
	// input 
	.i_rgb565			(o1_image_pixels),
	// output
	.o_y				(o2_image_y		), // output [7:0] 
	.o_u				(o2_image_u		), // output [7:0] 
	.o_v				(o2_image_v		), // output [7:0] 
	.o_gray    			(o2_image_gray	)  // output [7:0] 
);

// format as RGB565
// assign o2_image_pixels = {o2_image_gray[7:3], o2_image_gray[7:2], o2_image_gray[7:3]}; 
assign o2_image_pixels = {o2_image_y[7:3], o2_image_y[7:2], o2_image_y[7:3]}; 

///////////////////////////////////////////////////////////////////////


/************************ step3, Gaussian Filter   ************************/

wire [23:0] i3_pixels = {o2_image_y, o2_image_u, o2_image_v};

wire 		o3_hsync		;
wire 		o3_vsync		;
wire 		o3_de			;
wire [23:0] o3_pixels 		;

image_gaussian_filter_3x3 #(
	// 图像基本参数
	.IW				(IW_HALF	),	// 图像宽（image width）
	.IH				(IH_HALF	),	// 图像高（image height）
	.DW				(24			)	// 源视频输出像素数据位宽（data width of source）
) 
u_image_gaussian_filter (
	.rst_n			(rst_n		),
	// source 视频信号
	.pclk			(pclk		),	// input 像素时钟输出（pixel clock）
	.i_hsync		(o1_hsync	),	// input 行同步信号（数据有效输出中标志）
	.i_vsync		(o1_vsync	),	// input 场同步信号
	.i_de			(o1_de		),	// input 像素数据有效位
	.i_pixels		(i3_pixels	),	// input 像素数据输出

	// 处理后输出视频信号
	.o_hsync		(o3_hsync	),	// output 行同步信号（数据有效输出中标志）
	.o_vsync		(o3_vsync	),	// output 场同步信号
	.o_de			(o3_de		),	// output 像素数据有效位
	.o_pixels		(o3_pixels	)	// output 像素数据输出
);

// format as RGB565
wire [15:0] o3_image_pixels = {o3_pixels[23:19], o3_pixels[23:18], o3_pixels[23:19]}; 


///////////////////////////////////////////////////////////////////////



/************************ END FOR OUTPUT   ************************/
`ifdef OUTPUT_NORMAL
assign o_hsync 		= o1_hsync			; // output 行同步信号（数据有效输出中标志）
assign o_vsync		= o1_vsync			; // output 场同步信号
assign o_de			= o1_de				; // output 像素数据有效位
assign o_pixels		= o1_image_pixels 	; // output 像素数据输出
`endif 

`ifdef OUTPUT_GRAY
assign o_hsync 		= o1_hsync			; // output 行同步信号（数据有效输出中标志）
assign o_vsync		= o1_vsync			; // output 场同步信号
assign o_de			= o1_de				; // output 像素数据有效位
assign o_pixels		= o2_image_pixels 	; // output 像素数据输出
`endif 

`ifdef OUTPUT_GF_GRAY
assign o_hsync 		= o3_hsync			; // output 行同步信号（数据有效输出中标志）
assign o_vsync		= o3_vsync			; // output 场同步信号
assign o_de			= o3_de				; // output 像素数据有效位
assign o_pixels		= o3_image_pixels 	; // output 像素数据输出
`endif 

`ifdef ENDIAN_GF_YUV
assign e_hsync 		= o3_hsync			; // output 行同步信号（数据有效输出中标志）
assign e_vsync		= o3_vsync			; // output 场同步信号
assign e_de			= o3_de				; // output 像素数据有效位
assign e_pixels		= o3_pixels 		; // output 像素数据输出, YUV888
`else 
assign e_hsync 		= o1_hsync			; // output 行同步信号（数据有效输出中标志）
assign e_vsync		= o1_vsync			; // output 场同步信号
assign e_de			= o1_de				; // output 像素数据有效位
assign e_pixels		= i3_pixels 		; // output 像素数据输出, YUV888
`endif 

endmodule
