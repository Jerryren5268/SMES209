// Image post-processing orchestrator — 3 color-filter channels, erosion/dilation, bounding box, result mux
// Reads cached results via rd_clk domain and outputs per 32-pixel blocks to hdmi_wrapper

// define 3 color filters, at office
// BLACK
`define CF0_Y_MIN	8'd0
`define CF0_Y_MAX	8'd100
`define CF0_U_MIN	8'd114
`define CF0_U_MAX	8'd130
`define CF0_V_MIN	8'd114
`define CF0_V_MAX	8'd130
// RED
`define CF1_Y_MIN	8'd0
`define CF1_Y_MAX	8'd255
`define CF1_U_MIN	8'd130
`define CF1_U_MAX	8'd160
`define CF1_V_MIN	8'd130
`define CF1_V_MAX	8'd180
// GREEN
`define CF2_Y_MIN	8'd0
`define CF2_Y_MAX	8'd255
`define CF2_U_MIN	8'd90
`define CF2_U_MAX	8'd112
`define CF2_V_MIN	8'd110
`define CF2_V_MAX	8'd160

// define 3 color filters, at home
// // BLACK
// `define CF0_Y_MIN	8'd0
// `define CF0_Y_MAX	8'd60
// `define CF0_U_MIN	8'd118
// `define CF0_U_MAX	8'd128
// `define CF0_V_MIN	8'd118
// `define CF0_V_MAX	8'd128
// // RED
// `define CF1_Y_MIN	8'd30
// `define CF1_Y_MAX	8'd80
// `define CF1_U_MIN	8'd126
// `define CF1_U_MAX	8'd141
// `define CF1_V_MIN	8'd127
// `define CF1_V_MAX	8'd155
// // GREEN
// `define CF2_Y_MIN	8'd30
// `define CF2_Y_MAX	8'd61
// `define CF2_U_MIN	8'd107
// `define CF2_U_MAX	8'd119
// `define CF2_V_MIN	8'd123
// `define CF2_V_MAX	8'd137


`define ENABLE_EROSION
// ENABLE_DILATION, need to ENABLE_EROSION also
`define ENABLE_DILATION 


module post_image_process #(
   // 图像基本参数
   parameter	IW  = 10'd640  ,   // SOURCE 图像宽（image width）
   parameter	IH	= 10'd360      // SOURCE 图像高（image height）
)
(
    // 系统信号
    input rst_n			         ,	// 复位（reset）

    // source 视频信号
    input pclk			         ,	// input 像素时钟输出（pixel clock）
    input i_hsync		         ,	// input 行同步信号（数据有效输出中标志）
    input i_vsync		         ,	// input 场同步信号
    input i_de			         ,	// input 像素数据有效位
    input [23:0] i_pixels		 ,	// input 像素数据输出, YUV888

    // 读取结果
    input				rd_clk 			,   // read clock
    input [9:0]			i_rd_row		,	// each read action by row index only
    input [9:0]			i_rd_col		,	// col[9:6] will decide which part of data to output
    input 				i_rd_valid		,	// request to take read action
    output reg 			o_rd_ready		,   // callback to requester, data is ready to read out

    // 输出3路: 识别目标 (Recognized Target) 结果
    input  [1:0]        i_rt_mode       ,   // input [1:0], select mode for rt0 ~ rt2:
                                            //    0 - only color filter, 
                                            //    1 - color filter + erosion, 
                                            //    2 - color filter + erosion + dilation
    output reg [31:0]	o_rt0_32pix		,	// offset by col[5:1], coz only cache 2x2 patch
    output reg [31:0]	o_rt1_32pix		,
    output reg [31:0]	o_rt2_32pix		,

    // 输出3路: bounding box 结果, 针对上述3路的 识别目标
    output [39:0]       o_bb0_xxyy      ,   // output [39:0], { x_min[9:0], x_max[9:0], y_min[9:0], y_max[9:0] }
    output [39:0]       o_bb1_xxyy      ,   // output [39:0],
    output [39:0]       o_bb2_xxyy          // output [39:0],
);

    // input size for bounding box, that is output size of color filter also
    localparam HALF_IW = IW / 2 ;
    localparam HALF_IH = IH / 2 ;

    // ROI (Region Of Interest) for bounding boxing, fixed as 1st version.
    // TODO - improve later
    localparam [9:0] BB_X_START = 10'd0;
    localparam [9:0] BB_X_END   = HALF_IW - 1;
    localparam [9:0] BB_Y_START = 10'd0;
    localparam [9:0] BB_Y_END   = HALF_IH - 1;

    // MODE FOR Recognized Target Outputs
    localparam RT_OUT_MODE_CF = 2'b00;
    localparam RT_OUT_MODE_ER = 2'b01;
    localparam RT_OUT_MODE_DL = 2'b10;


// ******************************* Recognized Target Output *************************************

    wire  		    cf0_rd_ready 	;
    wire  		    cf1_rd_ready 	;
    wire  		    cf2_rd_ready 	;
    wire [31:0]		o_cf0_32pix		;
    wire [31:0]		o_cf1_32pix		;
    wire [31:0]		o_cf2_32pix		;

    wire  		    er0_rd_ready 	;
    wire  		    er1_rd_ready 	;
    wire  		    er2_rd_ready 	;
    wire [31:0]		o_er0_32pix		;
    wire [31:0]		o_er1_32pix		;
    wire [31:0]		o_er2_32pix		;

    wire  		    dl0_rd_ready 	;
    wire  		    dl1_rd_ready 	;
    wire  		    dl2_rd_ready 	;
    wire [31:0]		o_dl0_32pix		;
    wire [31:0]		o_dl1_32pix		;
    wire [31:0]		o_dl2_32pix		;

    always @(*) begin
        if (i_rt_mode == RT_OUT_MODE_ER) begin
            o_rt0_32pix <= o_er0_32pix;
            o_rt1_32pix <= o_er1_32pix;
            o_rt2_32pix <= o_er2_32pix;
        end 
        else if (i_rt_mode == RT_OUT_MODE_DL) begin
            o_rt0_32pix <= o_dl0_32pix;
            o_rt1_32pix <= o_dl1_32pix;
            o_rt2_32pix <= o_dl2_32pix;
        end 
        else begin
            o_rt0_32pix <= o_cf0_32pix;
            o_rt1_32pix <= o_cf1_32pix;
            o_rt2_32pix <= o_cf2_32pix;
        end 
    end 

    always @(*) begin
        if (i_rt_mode == RT_OUT_MODE_ER) begin
            o_rd_ready = er0_rd_ready & er1_rd_ready & er2_rd_ready  ;
        end 
        else if (i_rt_mode == RT_OUT_MODE_DL) begin
            o_rd_ready = dl0_rd_ready & dl1_rd_ready & dl2_rd_ready  ;
        end 
        else begin
            o_rd_ready = cf0_rd_ready & cf1_rd_ready & cf2_rd_ready  ;
        end 
    end 


// ********************************** Color Filter 0 *************************************

    wire        cf0_o_hsync             ;
    wire        cf0_o_vsync             ;
    wire        cf0_o_de                ;
    wire        cf0_o_match             ;

    image_color_filter #(
        // 图像基本参数
        .IW				(IW			),	// 图像宽（image width）
        .IH				(IH			),	// 图像高（image height）
        .Y_MAX			(`CF0_Y_MAX	),	// YUV 目标颜色范围, 闭合区间, [min, max]
        .Y_MIN			(`CF0_Y_MIN	),
        .U_MAX			(`CF0_U_MAX	),
        .U_MIN			(`CF0_U_MIN	),
        .V_MAX			(`CF0_V_MAX	),
        .V_MIN			(`CF0_V_MIN	),
        // .CAHCE_ENABLE  	(0			),
        .CAHCE_ENABLE  	(1			),
        .OUT_DIV2		(1			)
    ) u_image_cf0 (
        .rst_n			(rst_n		),
        // source 视频信号
        .pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
        .i_hsync		(i_hsync	),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(i_vsync	),	// input 场同步信号
        .i_de			(i_de	    ),	// input 像素数据有效位
        .i_pixels		(i_pixels	),	// input 像素数据输出
        // 处理后输出视频信号
	    .o_hsync		(cf0_o_hsync),	// output 行同步信号（数据有效输出中标志）
	    .o_vsync		(cf0_o_vsync),	// output 场同步信号, 对 i_vsync 打一拍
	    .o_de			(cf0_o_de   ),	// output 像素数据有效位
	    .o_match		(cf0_o_match),	// output 当前像素是否匹配颜色范围
        // 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(cf0_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_cf0_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
    );


// ********************************** Color Filter 1 *************************************

    wire        cf1_o_hsync             ;
    wire        cf1_o_vsync             ;
    wire        cf1_o_de                ;
    wire        cf1_o_match             ;

    image_color_filter #(
        // 图像基本参数
        .IW				(IW			),	// 图像宽（image width）
        .IH				(IH			),	// 图像高（image height）
        .Y_MAX			(`CF1_Y_MAX	),	// YUV 目标颜色范围, 闭合区间, [min, max]
        .Y_MIN			(`CF1_Y_MIN	),
        .U_MAX			(`CF1_U_MAX	),
        .U_MIN			(`CF1_U_MIN	),
        .V_MAX			(`CF1_V_MAX	),
        .V_MIN			(`CF1_V_MIN	),
        // .CAHCE_ENABLE  	(0			),
        .CAHCE_ENABLE  	(1			),
        .OUT_DIV2		(1			)
    ) u_image_cf1 (
        .rst_n			(rst_n		),
        // source 视频信号
        .pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
        .i_hsync		(i_hsync	),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(i_vsync	),	// input 场同步信号
        .i_de			(i_de	    ),	// input 像素数据有效位
        .i_pixels		(i_pixels	),	// input 像素数据输出
        // 处理后输出视频信号
	    .o_hsync		(cf1_o_hsync),	// output 行同步信号（数据有效输出中标志）
	    .o_vsync		(cf1_o_vsync),	// output 场同步信号, 对 i_vsync 打一拍
	    .o_de			(cf1_o_de   ),	// output 像素数据有效位
	    .o_match		(cf1_o_match),	// output 当前像素是否匹配颜色范围
        // 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(cf1_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_cf1_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
    );


// ********************************** Color Filter 2 *************************************

    wire        cf2_o_hsync             ;
    wire        cf2_o_vsync             ;
    wire        cf2_o_de                ;
    wire        cf2_o_match             ;

    image_color_filter #(
        // 图像基本参数
        .IW				(IW			),	// 图像宽（image width）
        .IH				(IH			),	// 图像高（image height）
        .Y_MAX			(`CF2_Y_MAX	),	// YUV 目标颜色范围, 闭合区间, [min, max]
        .Y_MIN			(`CF2_Y_MIN	),
        .U_MAX			(`CF2_U_MAX	),
        .U_MIN			(`CF2_U_MIN	),
        .V_MAX			(`CF2_V_MAX	),
        .V_MIN			(`CF2_V_MIN	),
        // .CAHCE_ENABLE  	(0			),
        .CAHCE_ENABLE  	(1			),
        .OUT_DIV2		(1			)
    ) u_image_cf2 (
        .rst_n			(rst_n		),
        // source 视频信号
        .pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
        .i_hsync		(i_hsync	),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(i_vsync	),	// input 场同步信号
        .i_de			(i_de	    ),	// input 像素数据有效位
        .i_pixels		(i_pixels	),	// input 像素数据输出
        // 处理后输出视频信号
	    .o_hsync		(cf2_o_hsync),	// output 行同步信号（数据有效输出中标志）
	    .o_vsync		(cf2_o_vsync),	// output 场同步信号, 对 i_vsync 打一拍
	    .o_de			(cf2_o_de   ),	// output 像素数据有效位
	    .o_match		(cf2_o_match),	// output 当前像素是否匹配颜色范围
        // 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(cf2_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_cf2_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
    );


// ********************************** Erosion 0 *************************************

`ifdef ENABLE_EROSION
    wire   		er_o0_match				 ;
	wire   		er_o0_hsync				 ;
	wire   		er_o0_vsync				 ;
	wire   		er_o0_de				 ;
    
    image_erosion_dilation #(
		// 图像基本参数
		.IW				(HALF_IW	),	// 图像宽（image width）
		.IH				(HALF_IH	),	// 图像高（image height）
		.E_Dn    		(1			),	// 1 - erosion, 0 - dilation
		.CAHCE_ENABLE  	(1			)
	) u0_image_erosion (
		.rst_n			(rst_n		),
		// source 视频信号
		.pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
		.i_hsync		(cf0_o_hsync),	// input 行同步信号（数据有效输出中标志）
		.i_vsync		(cf0_o_vsync),	// input 场同步信号
		.i_de			(cf0_o_de	),	// input 像素数据有效位
		.i_match		(cf0_o_match),	// input 像素是否匹配目标

		// 处理后输出视频信号
		.o_hsync		(er_o0_hsync),	// output 行同步信号（数据有效输出中标志）
		.o_vsync		(er_o0_vsync),	// output 场同步信号
		.o_de			(er_o0_de	),	// output 像素数据有效位
		.o_match		(er_o0_match),	// output 像素是否匹配目标

		// 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(er0_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_er0_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
	);
`endif 

// ********************************** Erosion 1 *************************************


`ifdef ENABLE_EROSION
    wire   		er_o1_match				 ;
	wire   		er_o1_hsync				 ;
	wire   		er_o1_vsync				 ;
	wire   		er_o1_de				 ;
    
    image_erosion_dilation #(
		// 图像基本参数
		.IW				(HALF_IW	),	// 图像宽（image width）
		.IH				(HALF_IH	),	// 图像高（image height）
		.E_Dn    		(1			),	// 1 - erosion, 0 - dilation
		.CAHCE_ENABLE  	(1			)
	) u1_image_erosion (
		.rst_n			(rst_n		),
		// source 视频信号
		.pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
		.i_hsync		(cf1_o_hsync),	// input 行同步信号（数据有效输出中标志）
		.i_vsync		(cf1_o_vsync),	// input 场同步信号
		.i_de			(cf1_o_de	),	// input 像素数据有效位
		.i_match		(cf1_o_match),	// input 像素是否匹配目标

		// 处理后输出视频信号
		.o_hsync		(er_o1_hsync),	// output 行同步信号（数据有效输出中标志）
		.o_vsync		(er_o1_vsync),	// output 场同步信号
		.o_de			(er_o1_de	),	// output 像素数据有效位
		.o_match		(er_o1_match),	// output 像素是否匹配目标

		// 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(er1_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_er1_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
	);
`endif 

// ********************************** Erosion 2 *************************************


`ifdef ENABLE_EROSION
    wire   		er_o2_match				 ;
	wire   		er_o2_hsync				 ;
	wire   		er_o2_vsync				 ;
	wire   		er_o2_de				 ;
    
    image_erosion_dilation #(
		// 图像基本参数
		.IW				(HALF_IW	),	// 图像宽（image width）
		.IH				(HALF_IH	),	// 图像高（image height）
		.E_Dn    		(1			),	// 1 - erosion, 0 - dilation
        // .E_Dn    		(0			),	// 1 - erosion, 0 - dilation
		// .CAHCE_ENABLE  	(0			)
		.CAHCE_ENABLE  	(1			)
	) u2_image_erosion (
		.rst_n			(rst_n		),
		// source 视频信号
		.pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
		.i_hsync		(cf2_o_hsync),	// input 行同步信号（数据有效输出中标志）
		.i_vsync		(cf2_o_vsync),	// input 场同步信号
		.i_de			(cf2_o_de	),	// input 像素数据有效位
		.i_match		(cf2_o_match),	// input 像素是否匹配目标

		// 处理后输出视频信号
		.o_hsync		(er_o2_hsync),	// output 行同步信号（数据有效输出中标志）
		.o_vsync		(er_o2_vsync),	// output 场同步信号
		.o_de			(er_o2_de	),	// output 像素数据有效位
		.o_match		(er_o2_match),	// output 像素是否匹配目标

		// 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(er2_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_er2_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
	);
`endif 


// ********************************** Dilation 0 *************************************

`ifdef ENABLE_DILATION
    wire   		dl_o0_match				 ;
	wire   		dl_o0_hsync				 ;
	wire   		dl_o0_vsync				 ;
	wire   		dl_o0_de				 ;
    
    image_erosion_dilation #(
		// 图像基本参数
		.IW				(HALF_IW	),	// 图像宽（image width）
		.IH				(HALF_IH	),	// 图像高（image height）
        .E_Dn    		(0			),	// 1 - erosion, 0 - dilation
		.CAHCE_ENABLE  	(1			)
	) u0_image_dilation (
		.rst_n			(rst_n		),
		// source 视频信号
		.pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
		.i_hsync		(er_o0_hsync),	// input 行同步信号（数据有效输出中标志）
		.i_vsync		(er_o0_vsync),	// input 场同步信号
		.i_de			(er_o0_de	),	// input 像素数据有效位
		.i_match		(er_o0_match),	// input 像素是否匹配目标

		// 处理后输出视频信号
		.o_hsync		(dl_o0_hsync),	// output 行同步信号（数据有效输出中标志）
		.o_vsync		(dl_o0_vsync),	// output 场同步信号
		.o_de			(dl_o0_de	),	// output 像素数据有效位
		.o_match		(dl_o0_match),	// output 像素是否匹配目标

		// 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(dl0_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_dl0_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
	);
`endif 


// ********************************** Dilation 1 *************************************

`ifdef ENABLE_DILATION
    wire   		dl_o1_match				 ;
	wire   		dl_o1_hsync				 ;
	wire   		dl_o1_vsync				 ;
	wire   		dl_o1_de				 ;
    
    image_erosion_dilation #(
		// 图像基本参数
		.IW				(HALF_IW	),	// 图像宽（image width）
		.IH				(HALF_IH	),	// 图像高（image height）
        .E_Dn    		(0			),	// 1 - erosion, 0 - dilation
		.CAHCE_ENABLE  	(1			)
	) u1_image_dilation (
		.rst_n			(rst_n		),
		// source 视频信号
		.pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
		.i_hsync		(er_o1_hsync),	// input 行同步信号（数据有效输出中标志）
		.i_vsync		(er_o1_vsync),	// input 场同步信号
		.i_de			(er_o1_de	),	// input 像素数据有效位
		.i_match		(er_o1_match),	// input 像素是否匹配目标

		// 处理后输出视频信号
		.o_hsync		(dl_o1_hsync),	// output 行同步信号（数据有效输出中标志）
		.o_vsync		(dl_o1_vsync),	// output 场同步信号
		.o_de			(dl_o1_de	),	// output 像素数据有效位
		.o_match		(dl_o1_match),	// output 像素是否匹配目标

		// 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(dl1_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_dl1_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
	);
`endif 


// ********************************** Dilation 2 *************************************

`ifdef ENABLE_DILATION
    wire   		dl_o2_match				 ;
	wire   		dl_o2_hsync				 ;
	wire   		dl_o2_vsync				 ;
	wire   		dl_o2_de				 ;
    
    image_erosion_dilation #(
		// 图像基本参数
		.IW				(HALF_IW	),	// 图像宽（image width）
		.IH				(HALF_IH	),	// 图像高（image height）
        .E_Dn    		(0			),	// 1 - erosion, 0 - dilation
		.CAHCE_ENABLE  	(1			)
	) u2_image_dilation (
		.rst_n			(rst_n		),
		// source 视频信号
		.pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
		.i_hsync		(er_o2_hsync),	// input 行同步信号（数据有效输出中标志）
		.i_vsync		(er_o2_vsync),	// input 场同步信号
		.i_de			(er_o2_de	),	// input 像素数据有效位
		.i_match		(er_o2_match),	// input 像素是否匹配目标

		// 处理后输出视频信号
		.o_hsync		(dl_o2_hsync),	// output 行同步信号（数据有效输出中标志）
		.o_vsync		(dl_o2_vsync),	// output 场同步信号
		.o_de			(dl_o2_de	),	// output 像素数据有效位
		.o_match		(dl_o2_match),	// output 像素是否匹配目标

		// 读取缓存的二值化图片
        .rd_clk 		(rd_clk	    ),   // input, read clock
        .i_rd_row		(i_rd_row   ),	 // input [9:0], each read action by row index only
        .i_rd_col		(i_rd_col   ),	 // input [9:0], col[9:6] will decide which part of data to output
        .i_rd_valid		(i_rd_valid ),  // request to take read action
        .o_rd_ready		(dl2_rd_ready),	 // output, callback to requester, data is ready to read out
        .o_rd_32pix		(o_dl2_32pix)	 // output [31:0], offset by col[5:1], coz only cache 2x2 patch
	);
`endif 


// ********************************** Bounding Box 0 *************************************

	wire [9:0]  		bb0_x_min 	;
    wire [9:0]  		bb0_x_max 	;
    wire [9:0]  		bb0_y_min 	;
    wire [9:0]  		bb0_y_max 	;

    image_bounding_box #(
        // 图像基本参数
        .IW				(HALF_IW	),	// 图像宽（image width）
        .IH				(HALF_IH	)   // 图像高（image height）
    ) u_image_bb0 (
        .rst_n			(rst_n		),
        // source 视频信号
        .pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
`ifdef ENABLE_EROSION
`ifdef ENABLE_DILATION
        .i_hsync		(dl_o0_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(dl_o0_vsync),	// input 场同步信号
        .i_de			(dl_o0_de	),	// input 像素数据有效位
        .i_match		(dl_o0_match),	// input 像素数据输出
`else
        .i_hsync		(er_o0_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(er_o0_vsync),	// input 场同步信号
        .i_de			(er_o0_de	),	// input 像素数据有效位
        .i_match		(er_o0_match),	// input 像素数据输出
`endif // ENABLE_DILATION
`else
        .i_hsync		(cf0_o_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(cf0_o_vsync),	// input 场同步信号
        .i_de			(cf0_o_de	),	// input 像素数据有效位
        .i_match		(cf0_o_match),	// input 像素数据输出
`endif 
        // 输入：设置 ROI (Region Of Interest)
	    .i_Xstart		(BB_X_START ),	// input [9:0], start/end position in horizontal 
	    .i_Xend			(BB_X_END   ),	// input [9:0]
	    .i_Ystart		(BB_Y_START ),	// input [9:0], start/end position in vertical 
	    .i_Yend			(BB_Y_END   ),	// input [9:0]

	// 输出：识别目标边界
	    .o_Xmin		    (bb0_x_min  ),	// output [9:0], start/end position in horizontal 
	    .o_Xmax			(bb0_x_max  ),	// output [9:0]
	    .o_Ymin  		(bb0_y_min  ),	// output [9:0], start/end position in vertical 
	    .o_Ymax			(bb0_y_max  )   // output [9:0]
    );

// ********************************** Bounding Box 1 *************************************

	wire [9:0]  		bb1_x_min 	;
    wire [9:0]  		bb1_x_max 	;
    wire [9:0]  		bb1_y_min 	;
    wire [9:0]  		bb1_y_max 	;

    image_bounding_box #(
        // 图像基本参数
        .IW				(HALF_IW	),	// 图像宽（image width）
        .IH				(HALF_IH	)   // 图像高（image height）
    ) u_image_bb1 (
        .rst_n			(rst_n		),
        // source 视频信号
        .pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
`ifdef ENABLE_EROSION
`ifdef ENABLE_DILATION
        .i_hsync		(dl_o1_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(dl_o1_vsync),	// input 场同步信号
        .i_de			(dl_o1_de	),	// input 像素数据有效位
        .i_match		(dl_o1_match),	// input 像素数据输出
`else
        .i_hsync		(er_o1_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(er_o1_vsync),	// input 场同步信号
        .i_de			(er_o1_de	),	// input 像素数据有效位
        .i_match		(er_o1_match),	// input 像素数据输出
`endif // ENABLE_DILATION
`else
        .i_hsync		(cf1_o_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(cf1_o_vsync),	// input 场同步信号
        .i_de			(cf1_o_de	),	// input 像素数据有效位
        .i_match		(cf1_o_match),	// input 像素数据输出
`endif 
        // 输入：设置 ROI (Region Of Interest)
	    .i_Xstart		(BB_X_START ),	// input [9:0], start/end position in horizontal 
	    .i_Xend			(BB_X_END   ),	// input [9:0]
	    .i_Ystart		(BB_Y_START ),	// input [9:0], start/end position in vertical 
	    .i_Yend			(BB_Y_END   ),	// input [9:0]
	    // 输出：识别目标边界
	    .o_Xmin		    (bb1_x_min  ),	// output [9:0], start/end position in horizontal 
	    .o_Xmax			(bb1_x_max  ),	// output [9:0]
	    .o_Ymin  		(bb1_y_min  ),	// output [9:0], start/end position in vertical 
	    .o_Ymax			(bb1_y_max  )   // output [9:0]
    );


// ********************************** Bounding Box 2 *************************************

	wire [9:0]  		bb2_x_min 	;
    wire [9:0]  		bb2_x_max 	;
    wire [9:0]  		bb2_y_min 	;
    wire [9:0]  		bb2_y_max 	;

    image_bounding_box #(
        // 图像基本参数
        .IW				(HALF_IW	),	// 图像宽（image width）
        .IH				(HALF_IH	)   // 图像高（image height）
    ) u_image_bb2 (
        .rst_n			(rst_n		),
        // source 视频信号
        .pclk			(pclk   	),	// input 像素时钟输出（pixel clock）
`ifdef ENABLE_EROSION
`ifdef ENABLE_DILATION
        .i_hsync		(dl_o2_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(dl_o2_vsync),	// input 场同步信号
        .i_de			(dl_o2_de	),	// input 像素数据有效位
        .i_match		(dl_o2_match),	// input 像素数据输出
`else
        .i_hsync		(er_o2_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(er_o2_vsync),	// input 场同步信号
        .i_de			(er_o2_de	),	// input 像素数据有效位
        .i_match		(er_o2_match),	// input 像素数据输出
`endif //ENABLE_DILATION
`else
        .i_hsync		(cf2_o_hsync),	// input 行同步信号（数据有效输出中标志）
        .i_vsync		(cf2_o_vsync),	// input 场同步信号
        .i_de			(cf2_o_de	),	// input 像素数据有效位
        .i_match		(cf2_o_match),	// input 像素数据输出
`endif 
        // 输入：设置 ROI (Region Of Interest)
	    .i_Xstart		(BB_X_START ),	// input [9:0], start/end position in horizontal 
	    .i_Xend			(BB_X_END   ),	// input [9:0]
	    .i_Ystart		(BB_Y_START ),	// input [9:0], start/end position in vertical 
	    .i_Yend			(BB_Y_END   ),	// input [9:0]
	    // 输出：识别目标边界
	    .o_Xmin		    (bb2_x_min  ),	// output [9:0], start/end position in horizontal 
	    .o_Xmax			(bb2_x_max  ),	// output [9:0]
	    .o_Ymin  		(bb2_y_min  ),	// output [9:0], start/end position in vertical 
	    .o_Ymax			(bb2_y_max  )   // output [9:0]
    );


// ***************************** Output for Bounding Box **********************************
    
    assign o_bb0_xxyy = {bb0_x_min, bb0_x_max, bb0_y_min, bb0_y_max};

    assign o_bb1_xxyy = {bb1_x_min, bb1_x_max, bb1_y_min, bb1_y_max};

    assign o_bb2_xxyy = {bb2_x_min, bb2_x_max, bb2_y_min, bb2_y_max};


endmodule
