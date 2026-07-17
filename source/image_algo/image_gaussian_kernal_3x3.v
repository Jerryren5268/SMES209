// Gaussian 3x3 kernel convolution — computes weighted sum of 9 pixels with coef [1,2,1; 2,4,2; 1,2,1]/16
// Output = sum(data[i][j] * kernel[i][j]) / 16 (combinational, 3 pipeline stages for timing)

/*
         [1, 2, 1]
kernal = [2, 4, 2]
	     [1, 2, 1]
pixel_out = 1/16 * kernal * pixels_3x3_in
*/

module image_gaussian_kernal_3x3 #(
	// 图像基本参数
	parameter	DW			=	24			    ,   // 像素数据位宽 (data width of source), defualt as YUV (888)
	parameter	IW			=	640				,	// SOURCE 图像宽 (image width)
	parameter	IH			=	480					// SOURCE 图像高 (image height)
)
(
	// 系统信号
	input 				rst_n			,	// 复位（reset）

	// source 视频信号
	input 				pclk			,	// input 像素时钟输出（pixel clock）
	input 				i_hsync			,	// input 行同步信号（数据有效输出中标志）
	input 				i_vsync			,	// input 场同步信号
	input				i_de			,	// input 像素数据有效位
	input [DW-1:0] 		i_p00			,	// input 像素数据
	input [DW-1:0] 		i_p01			,
	input [DW-1:0] 		i_p02			,
	input [DW-1:0] 		i_p10			,
	input [DW-1:0] 		i_p11			,
	input [DW-1:0] 		i_p12			,
	input [DW-1:0] 		i_p20			,
	input [DW-1:0] 		i_p21			,
	input [DW-1:0] 		i_p22			,
	// 处理后输出视频信号
	output 				o_hsync			,	// output 行同步信号（数据有效输出中标志）
	output 		    	o_vsync			,	// output 场同步信号, 对 i_vsync 打一拍
	output 				o_de			,	// output 像素数据有效位
	output reg [DW-1:0]	o_pixels			// output 像素数据
);

	// *******************************************************************************************
	// 同步信号延迟：卷积输出经过 3 级流水线，因此 hsync/vsync/de 也延迟 3 拍
	reg [2:0] hsync_d;
	reg [2:0] vsync_d;
	reg [2:0] de_d;

	assign o_hsync = hsync_d[2];
	assign o_vsync = vsync_d[2];
	assign o_de    = de_d[2];
	// *******************************************************************************************

	// *******************************************************************************************
	// 第 1 级流水：分别计算三行的加权和
	// row0: 1*p00 + 2*p01 + 1*p02
	// row1: 2*p10 + 4*p11 + 2*p12
	// row2: 1*p20 + 2*p21 + 1*p22
	reg [11:0] row0_y, row1_y, row2_y;
	reg [11:0] row0_u, row1_u, row2_u;
	reg [11:0] row0_v, row1_v, row2_v;

	// 第 2 级流水：三行相加得到完整 3x3 加权和
	reg [11:0] sum_y;
	reg [11:0] sum_u;
	reg [11:0] sum_v;
	// *******************************************************************************************

	// *******************************************************************************************
	// 3 级流水卷积计算
	always @(posedge pclk)
	begin
		if(~rst_n)
		begin
			hsync_d  <= 3'b000;
			vsync_d  <= 3'b000;
			de_d     <= 3'b000;

			row0_y   <= 12'd0;
			row1_y   <= 12'd0;
			row2_y   <= 12'd0;
			row0_u   <= 12'd0;
			row1_u   <= 12'd0;
			row2_u   <= 12'd0;
			row0_v   <= 12'd0;
			row1_v   <= 12'd0;
			row2_v   <= 12'd0;

			sum_y    <= 12'd0;
			sum_u    <= 12'd0;
			sum_v    <= 12'd0;

			o_pixels <= {DW{1'b0}};
		end
		else
		begin
			// 控制信号与像素数据保持同样的 3 拍延迟
			hsync_d <= {hsync_d[1:0], i_hsync};
			vsync_d <= {vsync_d[1:0], i_vsync};
			de_d    <= {de_d[1:0], i_de};

			// stage 1: 对 Y/U/V 三个通道分别计算每一行的加权和
			row0_y <= {4'b0, i_p00[23:16]} + {3'b0, i_p01[23:16], 1'b0} + {4'b0, i_p02[23:16]};
			row1_y <= {3'b0, i_p10[23:16], 1'b0} + {2'b0, i_p11[23:16], 2'b0} + {3'b0, i_p12[23:16], 1'b0};
			row2_y <= {4'b0, i_p20[23:16]} + {3'b0, i_p21[23:16], 1'b0} + {4'b0, i_p22[23:16]};

			row0_u <= {4'b0, i_p00[15:8]} + {3'b0, i_p01[15:8], 1'b0} + {4'b0, i_p02[15:8]};
			row1_u <= {3'b0, i_p10[15:8], 1'b0} + {2'b0, i_p11[15:8], 2'b0} + {3'b0, i_p12[15:8], 1'b0};
			row2_u <= {4'b0, i_p20[15:8]} + {3'b0, i_p21[15:8], 1'b0} + {4'b0, i_p22[15:8]};

			row0_v <= {4'b0, i_p00[7:0]} + {3'b0, i_p01[7:0], 1'b0} + {4'b0, i_p02[7:0]};
			row1_v <= {3'b0, i_p10[7:0], 1'b0} + {2'b0, i_p11[7:0], 2'b0} + {3'b0, i_p12[7:0], 1'b0};
			row2_v <= {4'b0, i_p20[7:0]} + {3'b0, i_p21[7:0], 1'b0} + {4'b0, i_p22[7:0]};

			// stage 2: 三行累加，最大值 255*16=4080，12 bit 足够表示
			sum_y <= row0_y + row1_y + row2_y;
			sum_u <= row0_u + row1_u + row2_u;
			sum_v <= row0_v + row1_v + row2_v;

			// stage 3: 除以 16，右移 4 bit，重新拼接为 YUV888
			o_pixels <= {sum_y[11:4], sum_u[11:4], sum_v[11:4]};
		end
	end
	// *******************************************************************************************

endmodule
