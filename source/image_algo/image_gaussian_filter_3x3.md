# `image_gaussian_filter_3x3.v` 实现说明：FIFO 行缓存窗口生成 + `image_gaussian_kernal_3x3` 卷积计算

## 目标

`source/image_algo/image_gaussian_filter_3x3.v` 用于实现完整的 **3x3 高斯滤波**。

当前实现把高斯滤波拆成两层：

1. `image_gaussian_filter_3x3.v`
   - 接收连续视频流 `i_pixels`。
   - 复用已有 `image_line_fifo` IP 作为两级行缓存。
   - 生成 3x3 窗口 `p00 ~ p22`。
   - 处理图像边界补 0。
   - 把对齐后的窗口送入卷积核模块。
2. `image_gaussian_kernal_3x3.v`
   - 只负责 3x3 高斯卷积核乘加计算。
   - 对 Y/U/V 三个 8 bit 通道分别计算。
   - 内部使用 3 级流水线。
   - 输出滤波后的 `o_pixels` 和延迟对齐后的同步信号。

这样拆分后，窗口生成逻辑和卷积计算逻辑互相独立。后续如果要替换卷积核，只需要修改 `image_gaussian_kernal_3x3.v`；如果要调整行缓存、窗口或边界策略，只需要修改 `image_gaussian_filter_3x3.v`。

---

## 模块接口

```verilog
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
```

注意：
- 当前工程中 `DW` 实际按 `24` 使用，对应 YUV888。
- `H_SYNC` 保留兼容旧接口，当前 RTL 不依赖它生成时序。
- `o_hsync / o_vsync / o_de / o_pixels` 直接来自 `image_gaussian_kernal_3x3` 子模块。

---

## 总体数据流

```text
输入视频流 i_pixels
        |
        v
image_line_fifo #0          保存上一行
        |
        v
image_line_fifo #1          保存上上一行
        |
        v
stage_* 对齐寄存器          对齐 FIFO 读出数据、当前像素、行列坐标
        |
        v
p00 ~ p22                  3x3 滑动窗口
        |
        v
kernal_p00 ~ kernal_p22    送入 kernel 的窗口寄存器
        |
        v
image_gaussian_kernal_3x3
        |
        v
输出视频流 o_pixels / o_de / o_hsync / o_vsync
```

关键点：
- `u_line_fifo0` 接收当前行像素，读出上一行同列像素。
- `u_line_fifo1` 接收 `u_line_fifo0` 的读出数据，读出上上一行同列像素。
- `stage_valid / stage_flush / stage_pixel / stage_col / stage_row` 用于对齐当前像素、FIFO 读出像素和坐标。
- `p00 ~ p22` 是横向滑动的 3x3 窗口缓存。
- `kernal_i_hsync / kernal_i_vsync / kernal_i_de` 与 `kernal_p00 ~ kernal_p22` 同周期送入 kernel。

---

## 为什么使用 `image_line_fifo`

旧的直接寄存器数组写法在当前综合器中容易被展开成大量分散触发器/逻辑单元。以 `IW=640, DW=24` 计算，两行缓存约为：

```text
640 * 24 * 2 = 30720 bit
```

这些 bit 如果落在 fabric registers 上，会显著增加布局布线压力。当前实现改为复用工程已有的 `image_line_fifo` IP，目标是让两行图像缓存落到 FIFO/BRAM 资源，而不是普通寄存器阵列。

---

## 高斯卷积核

卷积核为：

```text
         [1, 2, 1]
kernal = [2, 4, 2]
         [1, 2, 1]
```

计算公式为：

```text
out = (p00 + 2*p01 + p02
     + 2*p10 + 4*p11 + 2*p12
     + p20 + 2*p21 + p22) / 16
```

该乘加计算不在 `image_gaussian_filter_3x3.v` 中直接完成，而是通过实例化：

```verilog
image_gaussian_kernal_3x3 #(
    .DW (DW),
    .IW (IW),
    .IH (IH)
) u_image_gaussian_kernal_3x3 (...);
```

---

## 输入数据格式

当前高斯滤波输入为 YUV888 / YUV444：

```text
DW = 24

[23:16] Y
[15:8]  U
[7:0]   V
```

`image_gaussian_kernal_3x3.v` 对三个通道分别计算：

```text
Y_out = gaussian3x3(p00[23:16], p01[23:16], ..., p22[23:16])
U_out = gaussian3x3(p00[15:8],  p01[15:8],  ..., p22[15:8])
V_out = gaussian3x3(p00[7:0],   p01[7:0],   ..., p22[7:0])
```

最后输出：

```verilog
o_pixels = {Y_out, U_out, V_out};
```

因此不能把整个 24 bit 像素当成一个整数做卷积。

---

## FIFO 行缓存设计

3x3 窗口需要同时使用三行像素：

```text
上上一行
上一行
当前行
```

当前实现使用两个 `image_line_fifo` 串联：

```verilog
image_line_fifo u_line_fifo0 (...);
image_line_fifo u_line_fifo1 (...);
```

读写关系：

```text
i_pixels      -> u_line_fifo0 写入
u_line_fifo0  -> 读出上一行像素 fifo0_rd_data
fifo0_rd_data -> u_line_fifo1 写入
u_line_fifo1  -> 读出上上一行像素 fifo1_rd_data
```

正常输入阶段：

```verilog
fifo0_wr_en = pixel_valid && !flush_active;
fifo0_rd_en = (pixel_valid && row_cnt != 0) || flush_active;
fifo1_rd_en = (pixel_valid && row_cnt >= 2) || flush_active;
fifo1_wr_en = stage_valid && !stage_flush && stage_row != 0;
```

`stage_*` 寄存器用于对齐同步 FIFO 读出的上一行/上上一行数据：

```verilog
stage_valid <= pixel_valid | flush_active;
stage_flush <= flush_active;
stage_pixel <= flush_active ? zero_pixel : i_pixels;
stage_col   <= flush_active ? flush_col_cnt : col_cnt;
stage_row   <= flush_active ? IH[ROW_DW-1:0] : row_cnt;
```

---

## 3x3 窗口生成

窗口寄存器：

```verilog
reg [DW-1:0] p00, p01, p02;
reg [DW-1:0] p10, p11, p12;
reg [DW-1:0] p20, p21, p22;
```

对应窗口：

```text
p00  p01  p02    上上一行
p10  p11  p12    上一行
p20  p21  p22    当前行
```

每个有效像素周期窗口向左移动一列：

```verilog
p00 <= p01;
p01 <= p02;
p02 <= line1_pixel;

p10 <= p11;
p11 <= p12;
p12 <= line0_pixel;

p20 <= p21;
p21 <= p22;
p22 <= stage_pixel;
```

当 `stage_row != 0` 且 `stage_col != 0` 时，可以形成以 `p11` 为中心的有效窗口，并送入 kernel：

```verilog
kernal_p00 <= p01;
kernal_p01 <= p02;
kernal_p02 <= line1_pixel;
kernal_p10 <= p11;
kernal_p11 <= p12;
kernal_p12 <= line0_pixel;
kernal_p20 <= p21;
kernal_p21 <= p22;
kernal_p22 <= stage_pixel;
kernal_i_hsync <= 1'b1;
kernal_i_de    <= 1'b1;
```

---

## 边界补 0

本模块采用 **zero padding**。

### 上边界

第 0 行和第 1 行缺少上方缓存数据：

```verilog
line0_pixel = (stage_row == 0) ? zero_pixel : fifo0_rd_data;
line1_pixel = (stage_row < 2)  ? zero_pixel : fifo1_rd_data;
```

### 左边界

每行开始时清空 `p00 ~ p22`，因此左侧缺失像素自然为 0。

### 右边界

每行最后一个真实像素处理后，`line_right_pending` 触发额外一个右侧补 0 周期，用于输出最后一列。

### 下边界

最后一行输入完成后，`flush_active` 触发额外扫描一行。该阶段当前行视为全 0，用于输出最后一行作为中心行时的滤波结果。

---

## 同步信号和流水线延迟

`image_gaussian_filter_3x3.v` 负责在窗口有效时产生：

```verilog
kernal_i_hsync <= 1'b1;
kernal_i_de    <= 1'b1;
kernal_i_vsync <= i_vsync | flush_active | stage_flush;
```

`image_gaussian_kernal_3x3.v` 内部还有 3 级流水线，因此：

- `o_pixels` 比送入 kernel 的窗口晚 3 个 `pclk` 输出。
- `o_hsync / o_vsync / o_de` 也在 kernel 模块内部延迟 3 拍。
- 最终输出同步信号和输出像素已经对齐。

---

## 输出尺寸

该模块目标是保持图像尺寸不变：

```text
输出宽度 = IW
输出高度 = IH
```

如果该模块放在 `image_reduce_d2` 之后，例如输入原图为 `1280 x 720`，先经过 1/2 下采样，则高斯滤波输入尺寸通常为：

```text
IW = 640
IH = 360
```

输出仍为：

```text
640 x 360
```

---

## 当前实现分工

| 模块 | 职责 |
|---|---|
| `image_gaussian_filter_3x3.v` | FIFO 行缓存、行列计数、3x3 窗口生成、边界补 0 |
| `image_gaussian_kernal_3x3.v` | 3x3 高斯卷积核乘加、YUV 分通道计算、3 拍流水线同步输出 |
| `image_line_fifo` | 24 bit 像素行缓存，避免将整行缓存综合成大量 fabric registers |

---

## 仿真验证建议

1. **输出数量检查**

   输入 `IW x IH` 个有效像素后，输出有效像素数量应仍为：

   ```text
   IW * IH
   ```

2. **纯色图测试**

   中间区域应基本保持原值，边界由于补 0 会略暗。

3. **单点测试**

   单个亮点经过滤波后应扩散成近似：

   ```text
   1 2 1
   2 4 2
   1 2 1
   ```

4. **综合/布线检查**

   重点确认综合后的实例中不再出现大量高斯行缓存 fabric register。预期应主要看到 `image_line_fifo` 相关 IP 实例。

---

## 注意事项

1. 当前工程按 `DW=24` 使用，对应 YUV888。
2. `image_line_fifo` IP 的数据宽度为 24 bit，因此该实现不支持任意 `DW` 配置。
3. `kernal_i_hsync / kernal_i_vsync / kernal_i_de` 必须和 `kernal_p00 ~ kernal_p22` 同周期送入。
4. 本模块使用补 0 边界，边缘像素可能比原图略暗。
5. 该实现的主要硬件目标是降低布线压力，而不是增加算法复杂度。
