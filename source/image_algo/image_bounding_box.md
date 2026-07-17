# `image_bounding_box.v` 实现说明

## 功能概述

`image_bounding_box` 对一路 1-bit 目标流做逐帧包围框统计。模块在一帧内扫描 `i_match`，只统计 ROI 内的目标像素，记录：

```text
Xmin, Xmax, Ymin, Ymax
```

在下一帧 `i_vsync` 上升沿到来时，模块把上一帧统计结果锁存到输出。若上一帧没有检测到目标，当前实现会保持上一帧输出，不再清成零框。这样可以避免目标短暂丢检时 HDMI 绿框闪烁或消失。

## 接口

```verilog
module image_bounding_box #(
    parameter IW = 640,
    parameter IH = 480
)(
    input           rst_n,
    input           pclk,
    input           i_hsync,
    input           i_vsync,
    input           i_de,
    input           i_match,

    input  [9:0]    i_Xstart,
    input  [9:0]    i_Xend,
    input  [9:0]    i_Ystart,
    input  [9:0]    i_Yend,

    output reg [9:0] o_Xmin,
    output reg [9:0] o_Xmax,
    output reg [9:0] o_Ymin,
    output reg [9:0] o_Ymax
);
```

- `i_match`：当前像素是否为目标。
- `i_hsync && i_de`：当前输入像素有效。
- `i_Xstart/i_Xend/i_Ystart/i_Yend`：ROI 搜索区域。
- 输出坐标属于模块输入图像坐标。当前工程中 `post_image_process` 以半分辨率尺寸例化该模块，HDMI 侧再左移一位放大到显示坐标。

## 坐标计数

当前实现参考扫描线计数方式：

- `h_count` 只在 `pixel_valid = i_hsync && i_de` 时递增。
- `row_had_data` 标记当前行是否真正出现过有效像素。
- `hsync_neg` 到来时，只有 `row_had_data=1` 才递增 `v_count`。
- `vsync_pos` 到来时，`h_count/v_count/row_had_data` 清零。

这样可以适配半分辨率或稀疏 `i_de` 的视频流，避免把没有有效数据的 hsync 行也计入 Y 坐标。

## ROI 与命中条件

ROI 会先裁剪到图像边界：

```verilog
roi_x_start = min(i_Xstart, IW - 1);
roi_x_end   = min(i_Xend,   IW - 1);
roi_y_start = min(i_Ystart, IH - 1);
roi_y_end   = min(i_Yend,   IH - 1);
```

目标像素条件为：

```verilog
target_pixel = pixel_valid && i_match && pixel_in_roi;
```

只有满足该条件的像素会参与 min/max 更新。

## 帧内统计

模块使用以下寄存器统计当前帧：

```verilog
reg       match_found;
reg [9:0] cur_x_min;
reg [9:0] cur_x_max;
reg [9:0] cur_y_min;
reg [9:0] cur_y_max;
```

第一个目标像素会初始化四个边界；后续目标像素继续扩展边界：

```text
cur_x_min = min(cur_x_min, h_count)
cur_x_max = max(cur_x_max, h_count)
cur_y_min = min(cur_y_min, v_count)
cur_y_max = max(cur_y_max, v_count)
```

## 帧边界行为

在 `vsync_pos` 到来时：

- 若 `match_found=1`，输出上一帧统计到的 bbox。
- 若 `match_found=0`，保持原输出不变。
- 随后清空当前帧统计寄存器，开始统计新一帧。

该 sticky 行为是为了板上显示稳定：当某一帧颜色阈值、腐蚀膨胀或光照变化导致目标短暂丢失时，绿框不会立刻消失。

## HDMI 叠加侧

`hdmi_wrapper.v` 中还做了显示侧稳定：

- `postp_bb*_xxyy` 先跨到 `hdmi_pix_clk` 域。
- `postp_bb*_draw` 只在 HDMI 帧开始 `vs_start` 更新，整帧扫描期间保持不变。
- 无效 bbox 不会主动清掉 `postp_bb*_draw`，因此显示会保持上一帧有效框。
- 绿色框优先级高于蓝色目标高亮。

## 验证

已运行：

```powershell
iverilog -g2012 -t null source\image_algo\image_bounding_box.v
iverilog -g2012 -DSIMULATION -s hdmi_wrapper -t null source\hdmi\hdmi_wrapper.v source\hdmi\vesa_timer.v source\hdmi\hdmi_output_adapter.v
cmd /c source\sim\run_target_test_1_post.bat
```

`target_test_1.txt` 仿真仍输出有效 bbox：

```text
bb0 scaled: x=4..636   y=4..328
bb1 scaled: x=350..398 y=136..168
bb2 scaled: x=134..638 y=10..358
```
