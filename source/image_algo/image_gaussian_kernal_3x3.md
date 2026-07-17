# `image_gaussian_kernal_3x3.v` 实现说明：3x3 高斯卷积核计算模块

## 目标

`image_gaussian_kernal_3x3.v` 用于实现 **3x3 高斯卷积核的乘加计算**。

该模块本身不负责生成 3x3 窗口，也不负责行缓存或边界补 0。它只接收上一级已经准备好的 9 个窗口像素：

```text
i_p00  i_p01  i_p02
i_p10  i_p11  i_p12
i_p20  i_p21  i_p22
```

然后按照高斯卷积核：

```text
         [1, 2, 1]
kernal = [2, 4, 2]
         [1, 2, 1]
```

计算：

```text
out = (i_p00 + 2*i_p01 + i_p02
     + 2*i_p10 + 4*i_p11 + 2*i_p12
     + i_p20 + 2*i_p21 + i_p22) / 16
```

其中 `i_p11` 是 3x3 窗口中心像素。

---

## 模块定位

整个高斯滤波分为两层：

```text
输入视频流
   |
   v
行缓存 + 3x3 窗口生成模块
   |
   +-- i_p00 i_p01 i_p02
   +-- i_p10 i_p11 i_p12
   +-- i_p20 i_p21 i_p22
   |
   v
image_gaussian_kernal_3x3
   |
   v
滤波后的像素 o_pixels
```

本模块只做 kernel 计算：

1. 接收 9 个输入像素。
2. 对每个像素乘以对应高斯权重。
3. 对 Y/U/V 三个通道分别求和。
4. 右移 4 位，相当于除以 16。
5. 输出滤波后的 YUV888 像素。

---

## 模块接口

```verilog
module image_gaussian_kernal_3x3 #(
    parameter DW = 24,
    parameter IW = 640,
    parameter IH = 480
)(
    input               rst_n,
    input               pclk,
    input               i_hsync,
    input               i_vsync,
    input               i_de,

    input [DW-1:0]      i_p00,
    input [DW-1:0]      i_p01,
    input [DW-1:0]      i_p02,
    input [DW-1:0]      i_p10,
    input [DW-1:0]      i_p11,
    input [DW-1:0]      i_p12,
    input [DW-1:0]      i_p20,
    input [DW-1:0]      i_p21,
    input [DW-1:0]      i_p22,

    output              o_hsync,
    output              o_vsync,
    output              o_de,
    output reg [DW-1:0] o_pixels
);
```

`IW` 和 `IH` 当前只保留接口一致性，kernel 计算本身不使用图像尺寸。

---

## 输入数据格式

当前项目中高斯滤波数据应为 **YUV888 / YUV444**：

```text
DW = 24

[23:16] Y
[15:8]  U
[7:0]   V
```

本模块内部固定按上述位段取 Y/U/V，因此实际要求 `DW=24`。不能把整个 24 bit 像素直接当作一个整数做卷积。

正确做法：

```text
Y 通道单独卷积
U 通道单独卷积
V 通道单独卷积
```

即：

```text
Y_out = gaussian3x3(i_p00[23:16], i_p01[23:16], ..., i_p22[23:16])
U_out = gaussian3x3(i_p00[15:8],  i_p01[15:8],  ..., i_p22[15:8])
V_out = gaussian3x3(i_p00[7:0],   i_p01[7:0],   ..., i_p22[7:0])
```

最后输出：

```verilog
o_pixels <= {sum_y[11:4], sum_u[11:4], sum_v[11:4]};
```

---

## 单通道卷积计算

对于任意一个 8 bit 通道，输入窗口为：

```text
d00  d01  d02
d10  d11  d12
d20  d21  d22
```

对应高斯卷积为：

```text
sum = d00 + 2*d01 + d02
    + 2*d10 + 4*d11 + 2*d12
    + d20 + 2*d21 + d22

out = sum / 16
```

硬件中乘以 2、乘以 4 使用左移实现：

```text
2*x = x << 1
4*x = x << 2
```

除以 16 使用右移 4 位实现：

```verilog
out = sum[11:4];
```

---

## 位宽分析

单个通道为 8 bit，最大值 255。

高斯卷积核权重和为：

```text
1 + 2 + 1 + 2 + 4 + 2 + 1 + 2 + 1 = 16
```

最大加权和为：

```text
255 * 16 = 4080
```

4080 需要 12 bit 表示：

```text
2^12 = 4096
```

因此当前 RTL 使用 12 bit 中间寄存器：

```verilog
reg [11:0] row0_y, row1_y, row2_y;
reg [11:0] row0_u, row1_u, row2_u;
reg [11:0] row0_v, row1_v, row2_v;
reg [11:0] sum_y;
reg [11:0] sum_u;
reg [11:0] sum_v;
```

---

## 三阶段流水线

当前 RTL 是 3 拍流水。

### 第 1 拍：每行加权和

分别计算三行的加权和：

```verilog
row0_y <= 1*p00_y + 2*p01_y + 1*p02_y;
row1_y <= 2*p10_y + 4*p11_y + 2*p12_y;
row2_y <= 1*p20_y + 2*p21_y + 1*p22_y;
```

U/V 通道同理。

### 第 2 拍：三行求和

```verilog
sum_y <= row0_y + row1_y + row2_y;
sum_u <= row0_u + row1_u + row2_u;
sum_v <= row0_v + row1_v + row2_v;
```

### 第 3 拍：除以 16 并输出

```verilog
o_pixels <= {sum_y[11:4], sum_u[11:4], sum_v[11:4]};
```

---

## 同步信号延迟

由于像素数据经过 3 拍流水，`i_hsync / i_vsync / i_de` 也延迟 3 拍：

```verilog
reg [2:0] hsync_d;
reg [2:0] vsync_d;
reg [2:0] de_d;

assign o_hsync = hsync_d[2];
assign o_vsync = vsync_d[2];
assign o_de    = de_d[2];
```

这样可以保证：

```text
o_pixels 与 o_de / o_hsync / o_vsync 对齐
```

---

## 边界处理说明

本模块不直接处理图像边界。边界补 0 由上一级窗口生成模块完成。

也就是说，`image_gaussian_kernal_3x3` 只相信输入的 `i_p00 ~ i_p22` 已经是完整窗口。如果某个边界位置需要补 0，上一级应直接把对应的 `i_pXX` 传入 0。

---

## 注意事项

1. 当前 RTL 实际要求 `DW=24`，对应 YUV888。
2. Y/U/V 三个通道必须分开计算。
3. 中间加权和使用 12 bit，避免溢出。
4. 除以 16 使用右移 4 位。
5. `i_hsync / i_vsync / i_de` 必须和像素数据同样延迟 3 拍。
6. 本模块不生成 3x3 窗口，也不做边界补 0。
7. `kernal` 是原文件名中的拼写，为保持代码一致，文档沿用该名字。
