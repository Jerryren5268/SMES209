# `image_reduce_d2.v` 实现说明：RGB565 三通道 2×2 中间均值池化下采样

## 目标

`source/image_algo/image_reduce_d2.v` 已实现图像宽、高各缩小为原来的 `1/2`：

- 输入图像尺寸：`IW × IH`
- 输出图像尺寸：`OW × OH = IW/2 × IH/2`
- 每个输出像素对应输入图像中的一个 `2×2` 区域
- 输入像素按 RGB565 格式处理
- R/G/B 三个通道分别对 `2×2` 窗口排序，取第 2 小和第 3 小的平均值，再重新拼成 RGB565 输出

当前项目中 `pre_image_process.v` 传入：

```verilog
.IMG_DW (16)
```

因此 `i_pixels` 和 `o_pixels` 都是 16 bit RGB565：

```text
[15:11] R，5 bit
[10:5]  G，6 bit
[4:0]   B，5 bit
```

---

## 为什么要三通道分别处理

RGB 彩色像素不能直接把整个 16 bit 当成一个数比较。

例如：

```text
红色 RGB565 = 16'hF800
绿色 RGB565 = 16'h07E0
蓝色 RGB565 = 16'h001F
```

如果直接比较 16 bit 数值，比较结果只是编码大小，不代表颜色中间值。因此当前实现对三个通道分别排序，并取中间两个数的平均值：

```text
R_out = avg_mid(R00, R01, R10, R11)
G_out = avg_mid(G00, G01, G10, G11)
B_out = avg_mid(B00, B01, B10, B11)
```

其中 `avg_mid` 表示：先排序，再取第 2 小和第 3 小的平均值。

最后输出：

```verilog
o_pixels = {R_out, G_out, B_out};
```

---

## 2×2 窗口

`2×2` 输入窗口如下：

```text
p00  p01
p10  p11
```

在代码中的对应关系为：

```text
p00 = left_upper_pixel      // 上一行上一列
p01 = line_buf[col_cnt]     // 上一行当前列
p10 = left_curr_pixel       // 当前行上一列
p11 = i_pixels              // 当前行当前列
```

每个 `pXX` 都是一个 RGB565 像素。

当当前输入位置满足：

```verilog
row_cnt[0] == 1'b1
col_cnt[0] == 1'b1
```

说明当前像素是一个 `2×2` 块的右下角，此时 4 个像素已经完整，可以输出一个缩小后的像素。

---

## 中间均值定义

因为 `2×2` 有 4 个数，严格数学意义上偶数个样本的中位数通常取中间两个数的平均值。当前硬件实现采用：先排序，再取第 2 小和第 3 小的平均值。

```text
sort(a, b, c, d) = s0 <= s1 <= s2 <= s3
输出 = (s1 + s2) / 2
```

相比原先“只取第 2 小值”的 lower median，这种方式更接近标准中位数定义，输出会稍微更平滑。

---

## RGB565 中间均值池化输出

当前输出像素在代码中按 5/6/5 三通道分别计算：

```verilog
o_pixels <= {
    median4_avg_5(left_upper_pixel[15:11], line_buf[col_cnt][15:11], left_curr_pixel[15:11], i_pixels[15:11]),
    median4_avg_6(left_upper_pixel[10:5],  line_buf[col_cnt][10:5],  left_curr_pixel[10:5],  i_pixels[10:5]),
    median4_avg_5(left_upper_pixel[4:0],   line_buf[col_cnt][4:0],   left_curr_pixel[4:0],   i_pixels[4:0])
};
```

其中：

- R 通道是 5 bit，使用 `median4_avg_5`
- G 通道是 6 bit，使用 `median4_avg_6`
- B 通道是 5 bit，使用 `median4_avg_5`

---

## 中值计算函数

模块内部实现了两个组合函数：

```verilog
function [4:0] median4_avg_5;
function [5:0] median4_avg_6;
```

两者逻辑相同，只是位宽不同。比较交换顺序为：

```verilog
if (x0 > x1) swap(x0, x1);
if (x2 > x3) swap(x2, x3);
if (x0 > x2) swap(x0, x2);
if (x1 > x3) swap(x1, x3);
if (x1 > x2) swap(x1, x2);
```

排序完成后：

```text
x0 <= x1 <= x2 <= x3
```

函数返回 `(x1 + x2) / 2`，即第 2 小值和第 3 小值的平均值。

代码中为了避免加法溢出，会先把两个中间值扩展 1 bit 再相加，然后右移 1 位：

```verilog
median4_avg_5 = ({1'b0, x1} + {1'b0, x2}) >> 1;
median4_avg_6 = ({1'b0, x1} + {1'b0, x2}) >> 1;
```

---

## 主要接口参数

原有参数保留，并新增输入行列计数位宽参数：

```verilog
parameter IMG_DW = 16,
parameter IW     = 640,
parameter IH     = 480,
parameter OW     = IW / 2,
parameter OH     = IH / 2,
parameter OW_DW  = 9,
parameter IW_DW  = OW_DW + 1,
parameter IH_DW  = 10
```

说明：

- `IMG_DW` 当前应为 `16`，对应 RGB565。
- `OW_DW` 用于输出宽度相关位宽。
- `IW_DW` 用于输入列计数，默认是 `OW_DW + 1`。
- `IH_DW` 用于输入行计数，默认可覆盖常见 `480/720` 等输入高度。

---

## 行缓存设计

中间均值池化需要同时拿到相邻两行、相邻两列的 4 个像素，因此模块内部使用一行缓存保存上一行 RGB565 像素：

```verilog
reg [IMG_DW-1:0] line_buf [0:IW-1];
```

每个输入有效像素到来时：

```verilog
left_upper_pixel <= line_buf[col_cnt];
left_curr_pixel  <= i_pixels;
line_buf[col_cnt] <= i_pixels;
```

含义：

- `left_upper_pixel` 保存上一行上一列像素
- `line_buf[col_cnt]` 在输出计算时提供上一行当前列像素
- `left_curr_pixel` 保存当前行上一列像素
- `i_pixels` 是当前行当前列像素

---

## 输出信号

输出像素有效条件：

```verilog
o_de <= pixel_valid & row_cnt[0] & col_cnt[0];
```

输出行同步条件：

```verilog
o_hsync <= i_hsync & row_cnt[0];
```

输出场同步：

```verilog
o_vsync <= i_vsync;
```

其中 `o_vsync` 在时钟中打一拍输出。

输出时序：

- 输入第 0 行只写入行缓存，不输出。
- 输入第 1 行在奇数列输出第 0/1 行组成的中间均值池化结果。
- 输入第 2 行只写入行缓存，不输出。
- 输入第 3 行在奇数列输出第 2/3 行组成的中间均值池化结果。
- 后续行同理。

---

## 边界处理

默认输入宽高为偶数：

```verilog
IW = 640 或 1280
IH = 480 或 720
```

若输入宽或高为奇数：

- 最后一列不能组成完整 `2×2`，会被丢弃。
- 最后一行不能组成完整 `2×2`，会被丢弃。

当前项目使用的常见输入尺寸均为偶数，因此不需要补边。

---

## 资源和时序说明

1. 行缓存资源为 `IW × IMG_DW` bit。
2. 当前行缓存写法是寄存器数组，综合工具可能实现为寄存器或 RAM，取决于器件和综合策略。
3. RGB565 三通道分别比较，R/B 各 5 bit，G 为 6 bit。
4. 每个通道的中间均值计算是组合比较网络，共 5 组比较交换，之后对中间两个值做一次加法和右移 1 位。
5. 输出仅在奇数行、奇数列有效，因此输出宽高均为输入的 `1/2`。
6. `o_hsync` 只在输出行对应的输入行中有效，`o_de` 只在输出像素周期有效。

---

## 已替换的旧逻辑

旧版本是隔行隔列抽点：

- `row_enable` 每行翻转一次
- `col_enable` 每个有效像素翻转一次
- `o_pixels` 直接输出 `i_pixels`

当前版本已经替换为 RGB565 三通道 `2×2` 中间均值池化，不再直接抽取单个输入像素，也不再把整个 16 bit RGB565 当作一个数比较。
