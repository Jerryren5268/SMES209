// RGB565 to YUV444 color space converter (combinational, fixed-point coefficients)
// Also outputs grayscale: Gray = (R*76 + G*150 + B*30) >> 8
// YUV matrix uses self-defined coefficients (see matrix below)

/*
self-define YUV
[Y]    [1/4   1/2   1/4]   [R]   [ 0 ]
[U] =  [1/4  -1/2   1/4] x [G] + [128]
[V]    [1/2     0  -1/2]   [B]   [128]

[R]    [1/4   1/2   1/4]   ([Y]   [ 0 ])
[G] =  [1/4  -1/2   1/4] x ([U] - [128])
[B]    [1/2     0  -1/2]   ([V]   [128])

Note that: Y, U, V value range: 0 ~ 255
*/

// `define GRAY_ENABLE

module rgb565_to_yuv 
(
	// input 
	input [15:0] 		i_rgb565,
	// output
	output [7:0] 		o_y		,
	output [7:0] 		o_u		, 
	output [7:0] 		o_v		,
	output [7:0] 		o_gray
);

wire [15:0] r = {8'b0, i_rgb565[15:11], i_rgb565[15:13]};
wire [15:0] g = {8'b0, i_rgb565[10: 5], i_rgb565[10: 9]};
wire [15:0] b = {8'b0, i_rgb565[ 4: 0], i_rgb565[ 4: 2]};

`ifdef GRAY_ENABLE

// 典型灰度转换公式 
// Gray = R*0.299 + G*0.587 + B*0.114 =(R*77 + G*150 + B*29) >>8
// 
// 77 = 64 + 8 + 4 + 1
// 150 = 128 + 16 + 4 + 2
// 29 = 16 + 8 + 4 + 1
wire [15:0]  r77  = (r << 6) + (r << 3) + (r << 2) + r;
wire [15:0]  g150 = (g << 7) + (g << 4) + (g << 2) + (g << 1);
wire [15:0]  b29  = (b << 4) + (b << 3) + (b << 2) + b;
wire [15:0]  gray_sum = r77 + g150 + b29;
assign o_gray = gray_sum[15:8];

`endif 

// RGB到YUV转换  
// Y  = 0.299 * R + 0.587 * G + 0.114 * B  
// Cb = -0.1687 * R - 0.3313 * G + 0.5 * B + 128  
// Cr = 0.5 * R - 0.4187 * G - 0.0813 * B + 128
// 简化版本
// Y = 1/4 * r + 1/2 * g + 1/4 * b 			  // 近似Y值，注意这里使用整数运算  
// U = 1/4 * r - 1/2 * g + 1/4 * b + 128	  // 近似Cb值
// V = 1/2 * r -  0  * g - 1/2 * b + 128 	  // 近似Cr值

assign o_y = {2'b0, r[7:2]} + {2'b0, b[7:2]} + {1'b0, g[7:1]};
assign o_u = {2'b0, r[7:2]} + {2'b0, b[7:2]} - {1'b0, g[7:1]} + 8'd128;
assign o_v = {1'b0, r[7:1]} - {1'b0, b[7:1]} 				  + 8'd128;


endmodule
