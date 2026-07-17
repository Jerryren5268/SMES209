# `image_erosion_dilation.v` 实现说明

## 功能概述

`image_erosion_dilation` 对二值图像流 `i_match` 做 3x3 形态学处理。模块通过参数 `E_Dn` 选择当前功能：

- `E_Dn = 1`：腐蚀（erosion/min），3x3 窗口内 9 个像素全部为 1 时输出 1。
- `E_Dn = 0`：膨胀（dilation/max），3x3 窗口内任意一个像素为 1 时输出 1。

该模块在 `post_image_process.v` 中接在 `image_color_filter` 后面使用。当前工程中先做腐蚀去除孤立噪点，再可选做膨胀补回目标区域。

---

## 参数说明

```verilog
parameter IW            = 640,
parameter IH            = 480,
parameter H_SYNC        = 200,
parameter E_Dn          = 1,
parameter CAHCE_ENABLE  = 0,
parameter CACHE_ENABLE  = CAHCE_ENABLE
```

- `IW/IH`：输入二值图像流尺寸。
- `H_SYNC`：保留旧接口参数，当前核心逻辑未使用。
- `E_Dn`：形态学模式选择，`1` 为腐蚀，`0` 为膨胀。
- `CAHCE_ENABLE`：工程中原有拼写，继续保留，避免影响已有例化。
- `CACHE_ENABLE`：正确拼写的内部兼容别名，默认等于 `CAHCE_ENABLE`。

---

## 视频输入输出

输入为同步二值像素流：

```verilog
input pclk;
input i_hsync;
input i_vsync;
input i_de;
input i_match;
```

输出为处理后的同步二值像素流：

```verilog
output reg o_hsync;
output reg o_vsync;
output reg o_de;
output reg o_match;
```

`pixel_valid` 定义为：

```verilog
wire pixel_valid = i_hsync && i_de;
```

当前实现中：

- `o_hsync` 跟随 `i_hsync`。
- `o_vsync` 跟随 `i_vsync`。
- `o_de` 在 `pixel_valid` 时有效。
- `o_match` 在 `pixel_valid` 时输出腐蚀或膨胀结果，否则输出 0。

---

## 行列计数

模块在 `pclk` 域维护当前输入像素坐标：

```verilog
reg [9:0] h_count;
reg [9:0] v_count;
```

计数规则：

- `vsync_pos` 到来时，`h_count/v_count` 清 0。
- `hsync_neg` 到来时，`h_count` 清 0，`v_count` 加 1。
- 每个 `pixel_valid` 像素到来时，`h_count` 加 1。

该坐标同时用于 3x3 行缓存读写和 `Binary_Image` 缓存写地址生成。

---

## 3x3 窗口生成

模块使用两个 1-bit 行缓存保存前两行二值结果：

```verilog
reg [IW-1:0] line0;  // 上一行
reg [IW-1:0] line1;  // 上上一行
```

再配合 3 行移位寄存器组成 3x3 窗口：

```text
line1 -> p00 p01 p02
line0 -> p10 p11 p12
input -> p20 p21 p22
```

每个有效像素到来时：

```verilog
line1[h_count] <= line0[h_count];
line0[h_count] <= i_match;
```

同时窗口横向移位：

```verilog
p00 <= n00; p01 <= n01; p02 <= n02;
p10 <= n10; p11 <= n11; p12 <= n12;
p20 <= n20; p21 <= n21; p22 <= n22;
```

其中 `n22` 为当前输入像素 `i_match`，`n12` 为上一行当前列像素，`n02` 为上上一行当前列像素。

---

## 边界处理

左边界和上边界按 0 补齐：

- 每行结束时清空 `p00~p22`，下一行左侧自动补 0。
- 第一行没有上一行数据，`line0_pixel` 输出 0。
- 前两行没有上上一行数据，`line1_pixel` 输出 0。

当前实现是流式锚点窗口：当前输入像素进入窗口右下角 `p22` 后立即计算输出。因此模块不会在行尾或帧尾额外插入补 0 像素，输出像素数量与输入有效像素数量一致。

为了避免上板时行尾空白期出现动态索引越界，RTL 对行缓存读地址做了保护：

```verilog
wire h_count_in_range = (h_count < IW);
wire [9:0] safe_h_count = h_count_in_range ? h_count : 10'd0;
```

只有 `h_count` 在有效范围内时，才读取 `line0/line1`。

---

## 腐蚀/膨胀计算

3x3 窗口的 9 个点为 `n00~n22`。

腐蚀逻辑：

```verilog
wire erosion_match =
    n00 & n01 & n02 &
    n10 & n11 & n12 &
    n20 & n21 & n22;
```

膨胀逻辑：

```verilog
wire dilation_match =
    n00 | n01 | n02 |
    n10 | n11 | n12 |
    n20 | n21 | n22;
```

最终输出选择：

```verilog
wire morph_match = (E_Dn != 0) ? erosion_match : dilation_match;
```

---

## 二值图缓存

当 `CACHE_ENABLE > 0` 时，模块会把处理后的 `o_match` 写入 `Binary_Image` IP，供 HDMI 侧按行读取。

写侧位于 `pclk` 域：

```verilog
wire        hbi_wr_en   = o_de;
wire        hbi_wr_data = o_match;
wire [16:0] hbi_wr_addr = {out_row[7:0], out_col[8:0]};
```

读侧位于 `rd_clk` 域，使用 32-bit block 读出：

```verilog
wire [11:0] hbi_rd_addr = {hbi_rd_row, hbi_issue_col};
```

读接口与 `image_color_filter` 保持一致：

- 外部 `i_rd_row/i_rd_col` 仍按 HDMI 全尺寸坐标输入。
- 实际读行使用 `i_rd_row[8:1]`。
- 实际读块使用 `i_rd_col[9:6]`。
- 每个 block 含 32 个二值像素。

---

## 防花屏处理

HDMI 侧当前没有使用 `postp_o_rd_ready` 门控显示像素，因此缓存读出必须避免暴露旧数据或未初始化数据。

当前 RTL 做了两点保护：

1. `hbi_rd_cache[0:15]` 不做整数组复位/清零，避免生成高扇出清零网络，降低综合布线压力。
2. 当请求行尚未 ready，或请求列块超出当前行有效范围时，`o_rd_32pix` 输出 0，因此未初始化或旧 cache 不会被 HDMI 侧显示出来。

对应逻辑：

```verilog
assign o_rd_ready = requested_row_hit && hbi_rd_ready;
assign o_rd_32pix = (requested_row_hit && hbi_rd_ready && requested_col_valid)
                   ? hbi_rd_cache[requested_col]
                   : 32'd0;
```

这样即使 HDMI 显示侧提前读取，也只会表现为“不叠加目标点”，不会把旧 cache 或随机值显示成雪花点。

---

## 资源说明

该模块的 3x3 行缓存是 1-bit 二值缓存：

```verilog
reg [IW-1:0] line0;
reg [IW-1:0] line1;
```

在当前 `post_image_process` 中，模块输入为半分辨率 `HALF_IW=320`，所以每个实例只需要约 `2 * 320` bit 的行缓存。相比 Gaussian 24-bit 双行缓存，布线压力很小。

帧级显示缓存继续复用已有 `Binary_Image` IP。

---

## 验证记录

已执行语法检查：

```powershell
iverilog -g2012 -t null source\image_algo\image_erosion_dilation.v ipcore\Binary_Image\Binary_Image.v
```

也用临时小尺寸 testbench 验证过：

- `E_Dn=1` 时执行 3x3 与运算。
- `E_Dn=0` 时执行 3x3 或运算。

`post_image_process` 集成检查可使用临时 `Binary_Image` stub 进行。直接使用厂商 IP 仿真模型时，本地 `iverilog` 会缺少 Pango 原语 `GTP_DRM18K`，这是厂商仿真库依赖，不是本模块语法错误。
