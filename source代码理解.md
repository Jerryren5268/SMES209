# source 代码理解文档

本文档用于说明当前工程 `source/` 目录下 Verilog 代码的整体架构、主要模块、数据流和各子系统职责。

## 1. 项目整体定位

`source/` 目录实现的是一个基于 FPGA 的实时图像处理与显示系统。系统从 OV5640 摄像头采集图像，经过预处理、DDR3 帧缓存、颜色识别、形态学处理和目标框计算，最后通过 HDMI 输出视频画面，并可叠加识别结果。

一句话概括：

> 双 OV5640 摄像头输入 + FPGA 实时图像预处理 + DDR3 帧缓存 + YUV 颜色识别 / 腐蚀膨胀 / Bounding Box + HDMI 叠加显示 + RISC-V SoC 控制。

## 2. 顶层数据流

系统主数据流如下：

```text
OV5640 摄像头 x2
    ↓
ov_camera_adapter
    ↓
摄像头选择 / 像素时钟选择
    ↓
pre_image_process
    ├── RGB565 图像 → img_ddr_writer → ddr3_axi4_adapter → DDR3
    └── YUV888 图像 → post_image_process → 颜色识别 / 形态学 / 目标框

DDR3 → hdmi_wrapper ← post_image_process 结果
    ↓
HDMI 输出
```

更完整的结构图：

```text
             ┌──────────────────────┐
             │      OV5640 #1        │
             └──────────┬───────────┘
                        │
             ┌──────────▼───────────┐
             │   ov_camera_adapter   │
             └──────────┬───────────┘

             ┌──────────────────────┐
             │      OV5640 #2        │
             └──────────┬───────────┘
                        │
             ┌──────────▼───────────┐
             │   ov_camera_adapter   │
             └──────────┬───────────┘

                        │
                        ▼
              camera select / mux
                        │
                        ▼
              RGB565 1280x720 stream
                        │
                        ▼
             ┌──────────────────────┐
             │  pre_image_process    │
             │  - downscale 1/2      │
             │  - RGB565 → YUV       │
             │  - Gaussian filter    │
             └──────────┬───────────┘
                        │
        ┌───────────────┴────────────────┐
        │                                │
        ▼                                ▼
 RGB565 640x360                    YUV888 640x360
        │                                │
        ▼                                ▼
┌────────────────┐              ┌────────────────────┐
│ img_ddr_writer │              │ post_image_process  │
└───────┬────────┘              │ - color filter x3   │
        │                       │ - erosion/dilation  │
        ▼                       │ - bounding box x3   │
┌────────────────┐              └─────────┬──────────┘
│ ddr3_axi4_adpt │                        │
└───────┬────────┘                        │
        │                                 │
        ▼                                 │
      DDR3                                │
        │                                 │
        ▼                                 │
┌────────────────┐                       │
│  hdmi_wrapper  │◄──────────────────────┘
│ - read DDR     │
│ - overlay algo │
│ - output HDMI  │
└───────┬────────┘
        ▼
      HDMI
```

## 3. 目录结构说明

```text
source/
├── Top.v                         顶层模块，连接所有子系统
├── Top.fdc                       工程约束文件
├── KeyDebounce.v                 按键消抖
├── pulse_sync.v                  跨时钟域脉冲同步
├── gray_ptr_sync.v               异步 FIFO 灰码指针同步
├── img_ddr_writer.v              图像写 DDR 控制
│
├── camera_ov5640/                OV5640 摄像头采集与初始化
├── image_algo/                   图像预处理、颜色识别、形态学、目标框
├── ddr3_axi4/                    DDR3 / AXI4 读写适配
├── hdmi/                         HDMI 时序、配置与视频输出
└── sim/                          仿真图像源与图像保存模块
```

## 4. 顶层模块 Top.v

`Top.v` 是整个工程的系统顶层，主要职责是模块例化和信号连接。它本身不承担复杂算法逻辑，更多是把不同子系统连接成完整图像链路。

主要功能包括：

1. 生成和分发时钟。
2. 管理系统复位、DDR 复位、摄像头复位、HDMI 复位。
3. 例化两个 OV5640 摄像头适配器。
4. 根据 `cmos_sel` 选择当前摄像头。
5. 例化 `pre_image_process` 做前端图像处理。
6. 例化 `img_ddr_writer` 将图像写入 DDR。
7. 例化 `post_image_process` 做颜色识别和目标框计算。
8. 例化 `hdmi_wrapper` 从 DDR 读图像并叠加算法结果。
9. 例化 `ddr3_axi4_adapter` 连接 DDR3 控制器。
10. 例化 `sparrow_soc`，通过 SoC 控制摄像头选择、显示模式和算法输出模式。

## 5. 时钟域划分

工程中存在多个时钟域：

| 时钟信号 | 作用 |
|---|---|
| `sys_clk` | 系统主时钟，由 50 MHz 输入时钟缓冲得到 |
| `cmos_25m_clk` | 摄像头 I2C 配置时钟 |
| `cmos_img_pclk` | 摄像头像素时钟，图像采集和部分图像处理使用 |
| `hdmi_pix_clk` | HDMI 像素时钟，720p60 下约为 74.25 MHz |
| `hdmi_cfg_clk` | HDMI 芯片 I2C 配置时钟 |
| `ddr_clk` | DDR3 控制器内部时钟 |
| `soc_clk` | SoC 相关时钟 |

主要跨时钟域场景：

```text
摄像头像素时钟域 → DDR 时钟域
DDR 时钟域 → HDMI 像素时钟域
摄像头像素时钟域 → HDMI 像素时钟域
```

相关辅助模块：

- `pulse_sync.v`
- `gray_ptr_sync.v`
- DDR 图像读写 FIFO

## 6. 摄像头子系统 camera_ov5640

目录：

```text
camera_ov5640/
├── cmos_8_16bit.v
├── ov5640_reg_config.v
├── ov_camera_adapter.v
├── ov_camera_i2c_ctrl.v
└── ov_camera_init.v
```

该目录负责 OV5640 摄像头初始化和图像数据采集。

### 6.1 ov_camera_adapter.v

摄像头适配器，是摄像头子系统对外的统一接口。它负责：

- 调用摄像头初始化模块。
- 通过 I2C 配置 OV5640 寄存器。
- 接收摄像头 8-bit 数据。
- 拼接成 16-bit RGB565 像素。
- 输出统一的视频流接口。

典型输出：

```verilog
o_img_pclk
o_img_vs
o_img_de
o_img_rgb565
o_img_width
o_img_height
```

### 6.2 ov_camera_init.v

负责摄像头上电初始化流程，通常配合寄存器配置表使用。

### 6.3 ov_camera_i2c_ctrl.v

I2C 控制器，用于给 OV5640 写寄存器。

### 6.4 ov5640_reg_config.v

OV5640 寄存器配置表，决定摄像头输出分辨率、格式和工作模式。

### 6.5 cmos_8_16bit.v

将摄像头连续输出的 8-bit 数据拼接成 16-bit RGB565 像素。

## 7. 图像预处理 image_algo/pre_image_process.v

`pre_image_process.v` 是图像前处理流水线。输入为摄像头 RGB565 图像，默认输入分辨率为 1280x720。

处理步骤：

```text
RGB565 1280x720
    ↓
image_reduce_d2
    2 倍降采样，得到 640x360
    ↓
rgb565_to_yuv
    RGB565 → YUV888 / Gray
    ↓
image_gaussian_filter_3x3
    3x3 高斯滤波
```

该模块有两路输出：

### 7.1 DDR 写入路径

输出信号：

```verilog
o_hsync
o_vsync
o_de
o_pixels
```

默认宏为：

```verilog
`define OUTPUT_NORMAL
```

因此 DDR 写入路径默认输出降采样后的 RGB565 图像。

### 7.2 后处理算法路径

输出信号：

```verilog
e_hsync
e_vsync
e_de
e_pixels
```

默认宏为：

```verilog
`define ENDIAN_GF_YUV
```

因此后处理模块拿到的是高斯滤波后的 YUV888 图像。

## 8. 图像后处理 image_algo/post_image_process.v

`post_image_process.v` 是识别算法的汇总模块。输入为 640x360 的 YUV888 图像流。

内部有三路颜色过滤：

| 通道 | 目标颜色 |
|---|---|
| CF0 | 黑色 BLACK |
| CF1 | 红色 RED |
| CF2 | 绿色 GREEN |

每路处理链路为：

```text
YUV888
    ↓
image_color_filter
    ↓
image_erosion_dilation
    ↓
image_bounding_box
```

### 8.1 颜色过滤

`image_color_filter.v` 根据 YUV 阈值判断当前像素是否属于目标颜色，输出二值图。

颜色阈值在 `post_image_process.v` 顶部通过宏定义配置，例如：

```verilog
`define CF0_Y_MIN 8'd0
`define CF0_Y_MAX 8'd100
`define CF0_U_MIN 8'd114
`define CF0_U_MAX 8'd130
`define CF0_V_MIN 8'd114
`define CF0_V_MAX 8'd130
```

### 8.2 腐蚀和膨胀

`image_erosion_dilation.v` 对二值图做形态学处理，用于去除噪点、填补目标区域。

工程中默认开启：

```verilog
`define ENABLE_EROSION
`define ENABLE_DILATION
```

### 8.3 Bounding Box

`image_bounding_box.v` 根据二值目标区域计算目标外接框。

输出格式：

```text
{x_min[9:0], x_max[9:0], y_min[9:0], y_max[9:0]}
```

对应信号：

```verilog
o_bb0_xxyy
o_bb1_xxyy
o_bb2_xxyy
```

### 8.4 后处理结果读取

后处理结果不是直接输出完整 RGB 图像，而是以 32 像素为单位输出二值 mask：

```verilog
o_rt0_32pix
o_rt1_32pix
o_rt2_32pix
```

`i_rt_mode` 控制 HDMI 读取哪一级处理结果：

| `i_rt_mode` | 输出内容 |
|---|---|
| `0` | 仅颜色过滤结果 |
| `1` | 颜色过滤 + 腐蚀 |
| `2` | 颜色过滤 + 腐蚀 + 膨胀 |

## 9. DDR3 / AXI4 子系统 ddr3_axi4

目录：

```text
ddr3_axi4/
├── dbg_wr_rd_fifo.v
├── ddr3_axi4_adapter.v
├── frame_buffer.v
├── img_wr_rd_fifo.v
├── rd_ctrl.v
├── wr_cmd_trans.v
├── wr_ctrl.v
└── wr_rd_ctrl_top.v
```

该目录负责 DDR3 访问和 AXI4 读写控制。

### 9.1 ddr3_axi4_adapter.v

DDR 子系统顶层适配器，连接三类接口：

1. 图像写入接口：来自摄像头 / 预处理路径。
2. 图像读出接口：给 HDMI 显示使用。
3. 调试读写接口：给 SoC 或调试逻辑使用。

在顶层中，图像写入路径接摄像头时钟域：

```verilog
img_in_clk
img_in_req
img_in_addr
img_in_valid
img_in_data
img_in_ready
img_in_full
```

图像读出路径接 HDMI 时钟域：

```verilog
img_out_clk
img_out_req
img_out_addr
img_out_offset
img_out_data
img_out_valid
```

### 9.2 wr_rd_ctrl_top.v

DDR 读写控制顶层，负责连接 AXI4 写地址、写数据、读地址、读数据等通道。

### 9.3 wr_ctrl.v / rd_ctrl.v

- `wr_ctrl.v`：AXI 写通道控制。
- `rd_ctrl.v`：AXI 读通道控制。

### 9.4 wr_cmd_trans.v

写命令转换模块，将上层写请求转换为 DDR / AXI 可接受的写命令。

### 9.5 img_wr_rd_fifo.v

图像读写 FIFO，用于缓冲图像数据，并处理摄像头、DDR、HDMI 之间的时钟域差异。

### 9.6 dbg_wr_rd_fifo.v

调试读写 FIFO，用于 SoC 或外部调试访问 DDR。

### 9.7 frame_buffer.v

该模块看起来是通用帧缓存封装，但当前内部写 FIFO 和读 FIFO 部分仍有 TODO 标记，可能不是当前主链路使用的完整实现。

## 10. 图像写 DDR：img_ddr_writer.v

`img_ddr_writer.v` 位于 `source/` 根目录，用于将预处理后的 RGB565 图像流转换为 DDR 写接口。

输入图像流：

```verilog
in_hs
in_vs
in_de
in_pixels
```

输出 DDR 写请求：

```verilog
ddr_req
ddr_addr
ddr_valid
ddr_data
ddr_ready
ddr_full
```

其他重要输出：

```verilog
wr_addr_head
error_cnt
```

其中 `wr_addr_head` 会传给 HDMI 子系统，帮助 HDMI 判断当前可以读取的图像帧位置，避免读写冲突。

## 11. HDMI 子系统 hdmi

目录：

```text
hdmi/
├── hdmi_output_adapter.v
├── hdmi_wrapper.v
├── iic_dri.v
├── ms7200_ctl.v
├── ms7210_ctl.v
├── ms72xx_ctl.v
└── vesa_timer.v
```

该目录负责 HDMI 输出相关功能。

### 11.1 hdmi_wrapper.v

HDMI 子系统上层封装。主要功能：

1. 产生或使用 HDMI 显示坐标。
2. 从 DDR3 读取图像数据。
3. 向 `post_image_process` 发起识别结果读取请求。
4. 根据 `play_mode` 选择显示模式。
5. 将识别 mask 和 bounding box 叠加到图像上。
6. 输出 HDMI RGB888 和同步信号。

主要输出：

```verilog
vs_out
hs_out
de_out
r_out
g_out
b_out
```

### 11.2 vesa_timer.v

产生 VESA / HDMI 显示时序，包括行同步、场同步、显示有效区域和像素坐标。

### 11.3 iic_dri.v

I2C 驱动模块，用于配置 HDMI 芯片。

### 11.4 ms7200_ctl.v / ms7210_ctl.v / ms72xx_ctl.v

HDMI 相关芯片配置控制模块。

### 11.5 hdmi_output_adapter.v

将内部图像数据和显示时序转换为 HDMI 输出格式。

## 12. RISC-V SoC 控制

顶层中例化了：

```verilog
sparrow_soc inst_sparrow_soc
```

该模块不在当前 `source/` 列表中，可能来自 IP、其他目录或综合工程文件。

它在系统中主要负责控制和调试，而不是处理主图像数据流。

主要控制信号：

| 信号 | 作用 |
|---|---|
| `cmos_sel` | 选择当前摄像头 |
| `cmos_stop_capture` | 停止摄像头采集 |
| `hdmi_play_mode` | 控制 HDMI 显示模式 |
| `postp_i_rt_mode` | 控制后处理输出模式 |

此外，SoC 还连接 UART、LED、按键、JTAG 和 DDR 调试接口。

## 13. 仿真模块 sim

目录：

```text
sim/
├── image_src.v
├── image_store.v
└── image_store_v2.v
```

用于仿真环境：

- `image_src.v`：提供仿真图像源。
- `image_store.v` / `image_store_v2.v`：保存仿真输出图像，方便对比处理结果。

`Top.v` 通过宏区分 FPGA 实机和仿真：

```verilog
`ifdef SIMULATION
    `define CAMERA_SIMULATE
`else
    `define CAMERA_ENABLE
    `define CAMERA_BOTH
`endif
```

仿真模式下可以不依赖真实摄像头。

## 14. 辅助模块

### 14.1 KeyDebounce.v

按键消抖模块。顶层中 `KEY[7]` 作为硬复位，`KEY[6:0]` 经过消抖后送入 SoC。

### 14.2 pulse_sync.v

跨时钟域脉冲同步模块，用于将一个时钟域中的单周期脉冲可靠同步到另一个时钟域。

### 14.3 gray_ptr_sync.v

灰码指针同步模块，常用于异步 FIFO 的读写指针跨时钟域同步。

## 15. 关键模块关系表

| 层级 | 模块 | 职责 |
|---|---|---|
| 顶层 | `Top.v` | 系统集成、时钟复位、模块互连 |
| 摄像头 | `ov_camera_adapter.v` | 摄像头初始化和 RGB565 图像输出 |
| 摄像头 | `ov_camera_i2c_ctrl.v` | OV5640 I2C 配置 |
| 预处理 | `pre_image_process.v` | 降采样、RGB 转 YUV、高斯滤波 |
| 预处理 | `image_reduce_d2.v` | 图像缩小一半 |
| 预处理 | `rgb565_to_yuv.v` | 色彩空间转换 |
| 预处理 | `image_gaussian_filter_3x3.v` | 高斯滤波 |
| DDR 写入 | `img_ddr_writer.v` | 图像流转 DDR 写请求 |
| DDR 适配 | `ddr3_axi4_adapter.v` | 连接图像读写、调试接口和 DDR3 |
| DDR 控制 | `wr_rd_ctrl_top.v` | AXI4 读写控制顶层 |
| 后处理 | `post_image_process.v` | 三路颜色识别、形态学、目标框 |
| 后处理 | `image_color_filter.v` | YUV 阈值二值化 |
| 后处理 | `image_erosion_dilation.v` | 腐蚀和膨胀 |
| 后处理 | `image_bounding_box.v` | 目标外接框计算 |
| HDMI | `hdmi_wrapper.v` | DDR 读图、算法结果叠加、HDMI 输出 |
| HDMI | `vesa_timer.v` | HDMI/VESA 时序生成 |
| 控制 | `sparrow_soc` | 摄像头选择、显示模式、算法模式控制 |

## 16. 架构特点

### 16.1 流水线式图像处理

图像算法模块大多采用标准视频流接口：

```verilog
hsync / vsync / de / pixels
```

这种形式适合 FPGA 实时处理，一边输入一边计算，不需要整帧等待。

### 16.2 显示路径和算法路径分离

预处理之后分成两路：

```text
RGB565 → DDR → HDMI 显示
YUV888 → 后处理算法 → mask / bbox → HDMI 叠加
```

这样既能显示原始或预处理图像，又能叠加算法识别结果。

### 16.3 DDR3 作为跨时钟域帧缓存

摄像头和 HDMI 使用不同像素时钟，二者不能直接同步传输整帧图像，因此中间使用 DDR3 做帧缓存。

### 16.4 后处理输出轻量化

后处理模块并不输出完整 RGB 图像，而是输出：

- 每 32 像素一组的二值 mask。
- 每个颜色目标的 bounding box。

这种设计节省存储和带宽，更适合实时叠加显示。

### 16.5 SoC 负责控制，硬件流水线负责数据

RISC-V SoC 不参与主图像像素级处理，而是负责：

- 模式切换。
- 摄像头选择。
- HDMI 显示模式控制。
- 后处理输出模式控制。
- DDR 调试。

主图像链路全部由硬件流水线完成。

## 17. 阅读代码建议

建议按如下顺序理解代码：

1. 先读 `Top.v`，理解系统如何连线。
2. 再读 `pre_image_process.v`，理解前处理链路。
3. 再读 `post_image_process.v`，理解颜色识别和目标框输出。
4. 再读 `hdmi_wrapper.v`，理解显示和叠加方式。
5. 再读 `img_ddr_writer.v` 和 `ddr3_axi4_adapter.v`，理解 DDR 缓存路径。
6. 最后读 `camera_ov5640/`，理解摄像头初始化和采集。

## 18. 总结

当前 `source/` 目录的代码是一套较完整的 FPGA 视频处理系统。其核心思想是：

```text
摄像头采集 → 图像预处理 → DDR 帧缓存 → HDMI 显示
                       └→ 颜色识别 / 形态学 / 目标框 → HDMI 叠加
```

系统通过硬件流水线完成实时像素处理，通过 DDR3 解决摄像头与 HDMI 之间的帧缓存问题，通过 SoC 提供模式控制和调试能力。整体架构清晰，适合用于数字电路课程中的图像采集、图像处理、DDR 访问和 HDMI 显示综合实践。
