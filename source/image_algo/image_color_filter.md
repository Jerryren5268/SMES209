# `image_color_filter.v` 实现说明：YUV 阈值过滤 + 2x2 投票 + 可选缓存读出

## 目标

`image_color_filter` 是一个 YUV 颜色范围过滤模块，用于在像素级别匹配指定的 YUV 范围，并输出二值化掩膜。

当前实现支持：

1. YUV888 分通道阈值匹配。
2. 全分辨率单像素输出，或 2x2 投票后的半分辨率输出。
3. 使用本地 `match_line` 保存上一行二值结果，用于组成 2x2 窗口。
4. 可选将半分辨率二值图写入 `Binary_Image`，供 `rd_clk` 域按行读取。

---

## 模块接口

```verilog
module image_color_filter #(
    parameter IW            = 640,
    parameter IH            = 480,
    parameter Y_MAX         = 8'd255,
    parameter Y_MIN         = 8'd0,
    parameter U_MAX         = 8'd146,
    parameter U_MIN         = 8'd128,
    parameter V_MAX         = 8'd158,
    parameter V_MIN         = 8'd135,
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
```

说明：
- `CAHCE_ENABLE` 是工程中已有的拼写，当前模块保留它兼容旧实例。
- `CACHE_ENABLE` 是正确拼写，默认等于 `CAHCE_ENABLE`。
- 当前工程实例仍可以使用 `.CAHCE_ENABLE(1)`。

---

## 数据流

```text
i_pixels
  |
  v
Y/U/V 分量拆分与范围比较
  |
  v
i_match 单像素匹配结果
  |
  +--> OUT_DIV2=0: 延迟 1 拍后全分辨率输出
  |
  v
match_line + match_00/match_10
  |
  v
2x2 投票
  |
  +--> OUT_DIV2=1: 半分辨率输出
  |
  v
Binary_Image 可选缓存
  |
  v
rd_clk 域整行读入 hbi_rd_cache，并输出 o_rd_32pix
```

---

## YUV 阈值匹配

输入像素格式为 YUV888：

```text
[23:16] Y
[15:8]  U
[7:0]   V
```

当前像素同时满足三个闭区间时，`i_match=1`：

```verilog
wire i_y_match = (i_y >= Y_MIN) && (i_y <= Y_MAX);
wire i_u_match = (i_u >= U_MIN) && (i_u <= U_MAX);
wire i_v_match = (i_v >= V_MIN) && (i_v <= V_MAX);
wire i_match   = i_y_match && i_u_match && i_v_match;
```

---

## 坐标计数

`h_count` 和 `v_count` 只在有效像素流中推进：

```verilog
wire pixel_valid = i_hsync && i_de;
```

行为：
- `vsync_pos` 时清零行列计数。
- `hsync_neg` 时列计数清零，行计数加 1。
- `pixel_valid` 时列计数加 1。

---

## 2x2 窗口与投票

当前实现使用一行本地二值缓存：

```verilog
reg [IW-1:0] match_line;
```

它只保存上一行的单像素匹配结果，不用于整帧缓存。

2x2 窗口定义：

```text
match_00  match_01    上一行
match_10  i_match     当前行
```

来源：
- `match_01`：上一行当前列，来自 `match_line[safe_h_count]`。`safe_h_count` 用于保护行尾空白期，避免 `h_count == IW` 时访问越界 bit。
- `match_00`：上一行左一列，由上一拍 `match_01` 保存。
- `match_10`：当前行左一列，由上一拍 `i_match` 保存。
- `i_match`：当前行当前列。

投票：

```verilog
match_sum = match_00 + match_01 + match_10 + i_match;
half_match <= (match_sum >= OUT_THRESHOLD[2:0]);
```

`OUT_THRESHOLD` 建议使用 1 到 4：
- `4`：四个像素全匹配才输出 1。
- `3`：至少三个像素匹配。
- `2`：至少两个像素匹配。
- `1`：至少一个像素匹配。

---

## 输出模式

### `OUT_DIV2=0`

输出全分辨率单像素匹配结果。同步信号和匹配结果统一延迟 1 拍：

```verilog
assign o_vsync = vsync_d1;
assign o_hsync = hsync_d1;
assign o_de    = de_d1;
assign o_match = match_d1;
```

### `OUT_DIV2=1`

输出 2x2 投票后的半分辨率结果。当前实现选择每个完整 2x2 块的右下角位置：

```verilog
half_hsync <= i_hsync && v_count[0];
half_de    <= pixel_valid && v_count[0] && h_count[0];
```

当 `v_count[0] && h_count[0]` 为 1 时，四个像素已经到齐，输出一个投票结果。

---

## 半分辨率缓存写入

当启用缓存时，模块实例化 `Binary_Image`：

```verilog
Binary_Image u_half_binary_image (...);
```

写入内容为半分辨率二值图：

```verilog
wire        hbi_wr_en   = half_de;
wire        hbi_wr_data = half_match;
wire [16:0] hbi_wr_addr = {half_row[8:1], half_col[9:1]};
```

地址含义：
- `half_row[8:1]`：半分辨率行地址。
- `half_col[9:1]`：半分辨率列地址。

BRAM 行按 512 像素对齐，适配 `Binary_Image` 的 1-bit 写入、32-bit 读出配置。

---

## 缓存读出

读出逻辑工作在 `rd_clk` 域。

读请求流程：

1. 检测 `i_rd_valid` 上升沿。
2. 如果请求行未命中当前缓存行，则从 `Binary_Image` 顺序读取整行 32-bit block。
3. 读出的 block 存入本地 `hbi_rd_cache[0:15]`。
4. 整行读完后置位 `hbi_row_valid` 和 `hbi_rd_ready`。
5. 请求方通过 `i_rd_col[9:6]` 选择需要的 32-bit block。

关键输出：

```verilog
assign o_rd_ready = requested_row_hit && hbi_rd_ready;
assign o_rd_32pix = (requested_row_hit && hbi_rd_ready && requested_col_valid)
                   ? hbi_rd_cache[requested_col]
                   : 32'd0;
```

注意：

- 等待 `o_rd_ready` 期间，请求方应保持 `i_rd_row` 稳定。
- 当前 HDMI 叠加路径不会用 `o_rd_ready` 门控像素输出，因此 RTL 在请求行未准备好或列块越界时输出 0，避免旧值或未初始化值形成雪花点。`hbi_rd_cache` 不做整数组复位/清零，以减少高扇出清零网络。

---

## 资源与布线注意事项

- `match_line` 是 `IW` bit 的本地上一行缓存，不做整行复位；第一行通过 `v_count` gating 补 0。对当前三路 color filter 来说约为 `3 * 640` bit，规模远小于 Gaussian 的 24-bit 双行缓存。
- `Binary_Image` 用于半分辨率整帧缓存和跨时钟读出。
- 若后续布线仍紧张，可以考虑将 `match_line` 也替换为专用 1-bit FIFO/BRAM 行缓存。

---

## 仿真验证建议

1. **全匹配测试**
   - 设置 Y/U/V 阈值覆盖所有输入像素。
   - `OUT_THRESHOLD=4` 时，2x2 全匹配区域应输出 1。

2. **无匹配测试**
   - 设置阈值不覆盖输入像素。
   - 输出应全为 0。

3. **阈值测试**
   - 分别测试 `OUT_THRESHOLD=1/2/3/4`。
   - 输出应随阈值增大而更严格。

4. **缓存读测试**
   - `CACHE_ENABLE=1`。
   - 请求行后等待 `o_rd_ready`，再检查 `o_rd_32pix`。

---

## 当前实现分工

| 模块/结构 | 职责 |
|---|---|
| `image_color_filter.v` | YUV 阈值匹配、2x2 投票、半分辨率输出 |
| `match_line` | 保存上一行二值匹配结果，用于 2x2 窗口 |
| `Binary_Image` | 可选半分辨率二值图缓存 |
| `hbi_rd_cache` | `rd_clk` 域整行 block 缓存 |
