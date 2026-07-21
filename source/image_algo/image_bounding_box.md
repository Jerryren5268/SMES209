# Bounding Box 数据路径与平滑策略

本文说明当前工程中 bounding box 的生成、跨时钟域传递和 HDMI 叠加方式。相关实现位于：

- `image_bounding_box.v`：逐帧统计目标区域的最小/最大坐标。
- `post_image_process.v`：为三路目标生成 bounding box，并打包输出。
- `hdmi_wrapper.v`：完成跨时钟域采样、逐帧平滑、坐标放大和绿框叠加。

## 1. Bounding box 生成

`image_bounding_box` 在一帧内扫描 1-bit 的 `i_match` 数据，仅统计 ROI 内的目标像素，并维护：

```text
cur_x_min, cur_x_max, cur_y_min, cur_y_max
```

有效像素和目标像素的判定为：

```verilog
pixel_valid = i_hsync && i_de;
target_pixel = pixel_valid && i_match && pixel_in_roi;
```

ROI 起止坐标会先限制在 `IW × IH` 的图像边界内。第一个目标像素初始化四个边界，后续目标像素继续扩展 min/max。

### 行列计数

- `h_count` 只在 `pixel_valid` 有效时递增。
- `row_had_data` 记录当前 HSYNC 行是否真正出现过有效像素。
- HSYNC 下降沿到来时，只有 `row_had_data == 1` 才递增 `v_count`。
- VSYNC 上升沿到来时，行列计数和 `row_had_data` 清零。

`row_had_data` 可避免把没有 `i_de` 数据的空 HSYNC 行计入 Y 坐标，尤其适用于半分辨率或稀疏有效数据流。

### 帧边界行为

当前帧统计结果在下一次 `vsync_pos` 到来时锁存到输出：

- 若上一帧找到目标，更新 `o_Xmin/o_Xmax/o_Ymin/o_Ymax`。
- 若上一帧未找到目标，输出清为 `{0, 0, 0, 0}`，向 HDMI 侧明确传递无效框。
- 锁存后清空帧内统计状态，开始统计新一帧。

HDMI 侧把零框识别为无效输入，并通过连续无效帧超时机制过滤短暂丢检，见第 4 节。

## 2. 三路处理与输出格式

`post_image_process` 实例化了三个 `image_bounding_box`，分别对应三路颜色目标。当前编译同时定义：

```verilog
`define ENABLE_EROSION
`define ENABLE_DILATION
```

所以三个实例均使用膨胀输出 `dl_o0_*`、`dl_o1_*`、`dl_o2_*` 作为统计输入，而不是直接使用颜色阈值或腐蚀输出。

每个 bounding box 打包为 40 bit：

```text
{x_min[9:0], x_max[9:0], y_min[9:0], y_max[9:0]}
```

算法处理尺寸为半分辨率 320×180。HDMI 绘制前使用 `{coord, 1'b0}` 将每个坐标乘 2，映射到 640×360 显示坐标：

```text
x_display = x_bbox × 2
y_display = y_bbox × 2
```

平滑运算在 320×180 坐标域完成，最后才放大到显示坐标域。

## 3. HDMI 跨时钟域接收

bounding box 从图像处理时钟域进入 `hdmi_pix_clk` 域时，依次经过：

```text
postp_bb*_meta -> postp_bb*_sync -> postp_bb*_hdmi
```

只有连续两个同步样本相等，才把该值接收到 `postp_bb*_hdmi`。这样可以避免把正在变化的 40-bit 总线当成完整新框使用。

接收到的目标框不会在扫描途中直接参与绘制。`postp_bb*_draw` 只在 HDMI 帧起点 `vs_start` 更新，因此一整帧内四条边坐标保持不变，不会出现同一显示帧内框线撕裂或局部漂移。

## 4. Bounding box 平滑

`hdmi_wrapper.v` 对四个坐标分别执行死区和限速跟随。当前参数为：

```verilog
BB_DEADBAND       = 1;   // 半分辨率像素
BB_MAX_STEP       = 4;   // 每个 HDMI 帧最多移动 4 个半分辨率像素
BB_HOLD_FRAMES    = 10;  // 连续无效时保持的帧数
BB_LINE_THICKNESS = 4;   // HDMI 显示像素
```

单个坐标的更新规则为：

```text
delta = abs(target - current)

若 delta <= 1：
    current 保持不变
否则：
    step = clamp(delta / 4, 1, 4)
    current 向 target 移动 step 个像素
```

这里的像素单位是 320×180 的半分辨率坐标。因此每帧最大步长 4 对应 HDMI 上最多移动 8 个显示像素。

显示状态的具体行为如下：

- 第一次收到有效框时立即初始化，不从零点缓慢移动。
- 后续有效框在每个 `vs_start` 按上述规则逐步跟随。
- 1 像素以内的坐标抖动被死区滤除。
- 大幅位置变化被限制为每帧最多 4 个半分辨率像素。
- 输入框无效时，当前显示框最多保持 `BB_HOLD_FRAMES = 10` 帧；持续无目标后清零并停止绘制。

相较于旧的“连续两帧确认后直接跳到新坐标”，当前方案既避免确认完成时的瞬间跳变，也抑制检测边界的单像素抖动，同时保持整帧坐标稳定，因此 bounding box 的运动更连续。

## 5. HDMI 叠加优先级

bounding box 使用绿色线条绘制，线宽为 4 个 640×360 显示像素。叠加优先级中，绿色 bounding box 高于蓝色二值 mask；当两者覆盖同一像素时显示绿色框线。

这保证框线不会被蓝色高亮遮挡。蓝色 mask 的帧同步和 DDR 图像帧选择属于独立的数据对齐路径，不由 bounding box 坐标平滑逻辑改变。

## 6. 调参建议

| 参数 | 减小后的效果 | 增大后的效果 |
| --- | --- | --- |
| `BB_DEADBAND` | 更灵敏，但更容易抖动 | 更稳定，但小幅移动可能不跟随 |
| `BB_MAX_STEP` | 移动更平缓，跟随更慢 | 跟随更快，大变化时更容易显得跳跃 |
| `BB_HOLD_FRAMES` | 丢检后更快消失 | 更能容忍短暂丢检，但残留更久 |
| `BB_LINE_THICKNESS` | 框线更细 | 框线更醒目 |

当前参数优先保证 640×360 HDMI 显示中的稳定性和连续性。

## 7. 验证状态

已通过以下仿真/检查：

- binary mask bank 与 DDR frame ID 在 HDMI 帧边界原子切换。
- HDMI bounding-box overlay 测试。
- 空帧输出零框及 HDMI 连续无目标超时清除测试。
- DDR frame ID 和环形缓冲区回绕断言。
- bounding box 死区和最大移动步长断言。
- 完整 640×360 目标处理回归。
- 顶层 Icarus Verilog elaboration。

尚未重新运行 PDS 全量综合与布局布线；当前命令行环境缺少 `fabric_shell` license。RTL 合入后仍需在具备许可证的环境中完成时序收敛确认。
