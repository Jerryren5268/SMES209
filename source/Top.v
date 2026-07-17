// =============================================================================
// Top — FPGA Real-Time Image Processing System Top-Level Module
//
// Pure module instantiation and pin routing — no FSM or CDC logic resides here.
// All control logic is encapsulated in child modules.
//
// Video Pipeline (camera pixel domain):
//   OV5640 x2 → ov_camera_adapter → GTP_CLKBUFGMUX → pre_image_process
//     → img_ddr_writer → ddr3_axi4_adapter → DDR3 SDRAM
//     → hdmi_wrapper (read FSM + CDC) → HDMI 1280x720@60Hz
//
// Algorithm Pipeline (parallel in post_image_process):
//   rgb565_to_yuv → image_gaussian_filter_3x3
//     → image_color_filter (x3) → image_erosion_dilation (x3)
//     → image_bounding_box (x3) → overlay on video
//
// Clock Domains:
//   sys_clk (50M)  — Reset, SoC, Camera I2C, DDR ref
//   cmos_img_pclk  — Camera, Pre-process, Writer, Post-process
//   hdmi_pix_clk   — HDMI VESA, DDR Read FSM
//   hdmi_cfg_clk   — HDMI I2C config
//   cmos_25m_clk   — Camera I2C config
//   ddr_clk         — DDR3 PHY internal
//
// Build Config:
//   FPGA:  CAMERA_ENABLE + HDMI_ENABLE + HDMI_60HZ (720p60)
//   SIM:   CAMERA_SIMULATE + HDMI disabled
// =============================================================================

`define PLL_ENABLE

`define DDR_WR_RD_IMG
`define DDR_RD_IMG_BY_POSITION_ENABLE
`define DDR_IMG1_WIDTH         11'd1280
`define DDR_IMG1_HEIGHT        11'd720
`define DDR_IMG2_WIDTH         11'd640
`define DDR_IMG2_HEIGHT        11'd360
`define DDR_ADDR_AREA_IMG      5'b00000

`define HDMI_ENABLE

`ifdef SIMULATION
// `define HDMI_60HZ
`define CAMERA_SIMULATE
`else
`define HDMI_60HZ
`define CAMERA_ENABLE
`define CAMERA_BOTH
`endif


module Top #(
    parameter CLK_FREQ             = 50_000_000 ,
    parameter MEM_ROW_ADDR_WIDTH   = 15         ,
    parameter MEM_COL_ADDR_WIDTH   = 10         ,
    parameter MEM_BADDR_WIDTH      = 3          ,
    parameter MEM_DQ_WIDTH         = 32         ,
    parameter MEM_DQS_WIDTH        = 32/8       ,
    parameter UART_BAUD_RATE       = 115200     ,
    parameter UART_BPS_NUM         = 16'd434 // clk = 50M, bps=115200
    // parameter UART_BPS_NUM         = 16'd5208 // clk = 50M, bps=9600
)
(
    input                                clk                       , // 50MHz
//OV5647
`ifdef CAMERA_ENABLE
    //coms1
    inout                                cmos1_scl                 ,
    inout                                cmos1_sda                 ,
    input                                cmos1_vsync               ,
    input                                cmos1_href                ,
    input                                cmos1_pclk                ,
    input   [7:0]                        cmos1_data                ,
    output                               cmos1_reset               ,
    //coms2
    inout                                cmos2_scl                 ,
    inout                                cmos2_sda                 ,
    input                                cmos2_vsync               ,
    input                                cmos2_href                ,
    input                                cmos2_pclk                ,
    input   [7:0]                        cmos2_data                ,
    output                               cmos2_reset               ,
`endif // CAMERA_ENABLE
`ifdef CAMERA_SIMULATE
    input                                cmos_img_pclk             ,
    input                                cmos_img_vs               ,
    input                                cmos_img_de               ,
    input   [15:0]                       cmos_img_rgb565           ,
    input                                cmos_img_ready            ,
    input   [15:0]                       cmos_img_width            ,
    input   [15:0]                       cmos_img_height           ,
`endif // CAMERA_SIMULATE
// DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
// HDMI
`ifndef SIMULATION
    output            iic_tx_scl    ,
    inout             iic_tx_sda    ,
    output            hdmi_rstn_out ,
`endif // SIMULATION
    output            hdmi_pix_clk  ,
    output            vs_out        ,
    output            hs_out        ,
    output            de_out        ,
    output     [7:0]  r_out         ,
    output     [7:0]  g_out         ,
    output     [7:0]  b_out         ,
// Others - UART / LED / ...
    inout            uart_tx,
    inout            uart_rx,
    input  [7:0]     keys,
    output [7:0]     led,

    //JTAG
    input  wire JTAG_TMS,
    input  wire JTAG_TDI,
    output wire JTAG_TDO,
    input  wire JTAG_TCK
);


// ============================== Clocks ==============================

wire sys_clk;

wire hdmi_cfg_clk; //10MHz, HDMI I2C config
wire cmos_25m_clk; //25M, camera config clock inside
wire pll_locked;
wire soc_clk; // 40MHz

GTP_CLKBUFG g_clkbuf
(
 .CLKOUT(sys_clk  ),
 .CLKIN (clk      )
);

`ifdef PLL_ENABLE

pll u_pll (
    .clkin1   (  clk          ),//50MHz
`ifdef HDMI_60HZ
    .clkout3  (  hdmi_pix_clk ),//74.25M, HDMI output 720P60
`else
    .clkout0  (  hdmi_pix_clk ),//37.125M, HDMI output 720P30
`endif //HDMI_60HZ
    .clkout1  (  hdmi_cfg_clk ),//10MHz, HDMI I2C config
    .clkout2  (  cmos_25m_clk ),//25M
    .clkout4  (  soc_clk      ),
    .pll_lock (  pll_locked   )
);

`endif // PLL_ENABLE


// ============================== Resets ==============================

// KEY[7]: hardware reset (active low), used directly without debounce
wire       key_rstn = keys[7];

// KEY[6:0]: user keys through debounce for RISC-V SoC
wire [6:0] keys_stable;

KeyDebounce #(
   .CLK_FREQ(CLK_FREQ),
   .KEY_CNT(7)
) u_key (
   .clk          (sys_clk),
   .keys         (keys[6:0]),
   .keys_stable  (keys_stable)
);

// ---- 1Hz tick for delayed DDR reset release ----
reg [25:0] sys_1s_cnt = 26'd0;
wire       trigger_1s = (sys_1s_cnt == CLK_FREQ - 1);

always @(posedge sys_clk) begin
    if (~pll_locked)
        sys_1s_cnt <= 26'd0;
    else if (sys_1s_cnt >= CLK_FREQ - 1)
        sys_1s_cnt <= 26'd0;
    else
        sys_1s_cnt <= sys_1s_cnt + 26'd1;
end

// ---- DDR reset timer: release after delay ----
reg [1:0] ddr_rstn_timer = 2'd0;

always @(posedge sys_clk) begin
    // if (~key_rstn) begin
    if (~pll_locked) begin
        ddr_rstn_timer <= 2'd0;
    end
`ifdef SIMULATION
    else if (ddr_rstn_timer < 2'd1) begin
        ddr_rstn_timer <= ddr_rstn_timer + 2'd1;    // sim: release immediately
    end
`else
    else if (trigger_1s && ddr_rstn_timer < 2'd1) begin
        ddr_rstn_timer <= ddr_rstn_timer + 2'd1;    // fpga: release after 1 second
    end
`endif
end

// ---- Derived reset signals ----
wire ddr_rstn;
wire ddr_init_done;
wire cam_rstn;
wire top_rstn;
wire hdmi_rstn;

assign ddr_rstn  = (ddr_rstn_timer >= 2'd1);
assign cam_rstn = ddr_init_done & ddr_rstn;
assign top_rstn  = ddr_init_done & ddr_rstn & key_rstn;
assign hdmi_rstn = ddr_init_done & ddr_rstn;


// ============================== Camera Adapters ==============================

wire        cmos_stop_capture; // from sparrow_soc
wire        cmos_sel;          // from sparrow_soc

`ifdef CAMERA_ENABLE

wire        cmos_img_pclk;
wire        cmos_img_vs;
wire        cmos_img_de;
wire [15:0] cmos_img_rgb565;
wire [1:0]  cmos_init_done;
wire        cmos_img_ready; // = cmos_init_done[cmos_sel]
wire [15:0] cmos_img_width;
wire [15:0] cmos_img_height;

wire        cmos1_img_pclk;
wire        cmos1_img_vs;
wire        cmos1_img_de;
wire [15:0] cmos1_img_rgb565;
wire [15:0] cmos1_img_width;
wire [15:0] cmos1_img_height;

wire        cmos2_img_pclk;
wire        cmos2_img_vs;
wire        cmos2_img_de;
wire [15:0] cmos2_img_rgb565;
wire [15:0] cmos2_img_width;
wire [15:0] cmos2_img_height;

ov_camera_adapter u_cmos1_adpter(
    .clk_50M        (sys_clk            ),
    .clk_25M        (cmos_25m_clk       ),
    .rstn           (cam_rstn           ),
    .cmos_scl       (cmos1_scl          ),
    .cmos_sda       (cmos1_sda          ),
    .cmos_vsync     (cmos1_vsync        ),
    .cmos_href      (cmos1_href         ),
    .cmos_pclk      (cmos1_pclk         ),
    .cmos_data      (cmos1_data         ),
    .cmos_reset     (cmos1_reset        ),
    .cmos_init_done (cmos_init_done[0]  ),
    .o_img_pclk     (cmos1_img_pclk     ),
    .o_img_vs       (cmos1_img_vs       ),
    .o_img_de       (cmos1_img_de       ),
    .o_img_rgb565   (cmos1_img_rgb565   ),
    .o_img_width    (cmos1_img_width    ),
    .o_img_height   (cmos1_img_height   )
);

ov_camera_adapter u_cmos2_adpter(
    .clk_50M        (sys_clk            ),
    .clk_25M        (cmos_25m_clk       ),
    .rstn           (cam_rstn           ),
    .cmos_scl       (cmos2_scl          ),
    .cmos_sda       (cmos2_sda          ),
    .cmos_vsync     (cmos2_vsync        ),
    .cmos_href      (cmos2_href         ),
    .cmos_pclk      (cmos2_pclk         ),
    .cmos_data      (cmos2_data         ),
    .cmos_reset     (cmos2_reset        ),
    .cmos_init_done (cmos_init_done[1]  ),
    .o_img_pclk     (cmos2_img_pclk     ),
    .o_img_vs       (cmos2_img_vs       ),
    .o_img_de       (cmos2_img_de       ),
    .o_img_rgb565   (cmos2_img_rgb565   ),
    .o_img_width    (cmos2_img_width    ),
    .o_img_height   (cmos2_img_height   )
);

// Camera selection: CAMERA_BOTH only
GTP_CLKBUFGMUX #(
    .SIM_DEVICE   ("LOGOS"),
    .TRIGGER_MODE ("NEGEDGE")
) u_GTP_CLKBUFGMUX (
    .CLKIN0       (cmos1_img_pclk   ),
    .CLKIN1       (cmos2_img_pclk   ),
    .SEL          (cmos_sel         ),
    .CLKOUT       (cmos_img_pclk    )
);

assign cmos_img_vs     = cmos_sel ? cmos1_img_vs     : cmos2_img_vs;
assign cmos_img_de     = cmos_sel ? cmos1_img_de     : cmos2_img_de;
assign cmos_img_rgb565 = cmos_sel ? cmos1_img_rgb565 : cmos2_img_rgb565;
// assign cmos_img_ready  = cmos_sel ? cmos_init_done[0]: cmos_init_done[1];
assign cmos_img_ready  = cmos_init_done[0] & cmos_init_done[1];
assign cmos_img_width  = cmos_sel ? cmos1_img_width  : cmos2_img_width;
assign cmos_img_height = cmos_sel ? cmos1_img_height : cmos2_img_height;

`endif // CAMERA_ENABLE


// ============================== Image Pre-Process ==============================

wire        prep_img2ddr_hs;
wire        prep_img2ddr_vs;
wire        prep_img2ddr_de;
wire [15:0] prep_img2ddr_pixels;

wire        postp_i_hs;
wire        postp_i_vs;
wire        postp_i_de;
wire [23:0] postp_i_pixels;

pre_image_process #(
   .IW          (`DDR_IMG1_WIDTH    ),
   .IH          (`DDR_IMG1_HEIGHT   )
)
u_pre_image_process (
   .rst_n      (cam_rstn           ),
   .pclk       (cmos_img_pclk      ),
   .i_hsync    (cmos_img_de        ),
   .i_vsync    (cmos_img_vs        ),
   .i_de       (1'b1               ),
   .i_pixels   (cmos_img_rgb565    ),
   // → DDR write path
   .o_hsync    (prep_img2ddr_hs    ),
   .o_vsync    (prep_img2ddr_vs    ),
   .o_de       (prep_img2ddr_de    ),
   .o_pixels   (prep_img2ddr_pixels),
   // → post_image_process
   .e_hsync    (postp_i_hs         ),
   .e_vsync    (postp_i_vs         ),
   .e_de       (postp_i_de         ),
   .e_pixels   (postp_i_pixels     )
);


// ============================== Camera → DDR Writer ==============================

wire [2:0]  cam_wr_addr_head;
wire [15:0] cam_wr_error_cnt;

wire        cam_ddr_req;
wire [27:0] cam_ddr_addr;
wire        cam_ddr_valid;
wire [15:0] cam_ddr_data;
wire        cam_ddr_ready;
wire        cam_ddr_full;

img_ddr_writer #(
    .DDR_ADDR_AREA_IMG (`DDR_ADDR_AREA_IMG),
    .IMG_HEIGHT        (`DDR_IMG2_HEIGHT)
) u_img_ddr_writer (
    .pclk           (cmos_img_pclk      ),
    .rstn           (cam_rstn           ),
    .img_ready      (cmos_img_ready     ),
    .ddr_init_done  (ddr_init_done      ),
    .stop_capture   (cmos_stop_capture  ),
    .in_hs          (prep_img2ddr_hs    ),
    .in_vs          (prep_img2ddr_vs    ),
    .in_de          (prep_img2ddr_de    ),
    .in_pixels      (prep_img2ddr_pixels),
    .ddr_req        (cam_ddr_req        ),
    .ddr_addr       (cam_ddr_addr       ),
    .ddr_valid      (cam_ddr_valid      ),
    .ddr_data       (cam_ddr_data       ),
    .ddr_ready      (cam_ddr_ready      ),
    .ddr_full       (cam_ddr_full       ),
    .wr_addr_head   (cam_wr_addr_head   ),
    .error_cnt      (cam_wr_error_cnt   )
);


// ============================== Image Post-Process ==============================

wire [9:0]  postp_i_rd_row;
wire [9:0]  postp_i_rd_col;
wire        postp_i_rd_valid;
wire        postp_o_rd_ready;
wire [1:0]  postp_i_rt_mode;
wire [31:0] postp_o_rt0_32pix;
wire [31:0] postp_o_rt1_32pix;
wire [31:0] postp_o_rt2_32pix;
wire [39:0] postp_o_bb0_xxyy;
wire [39:0] postp_o_bb1_xxyy;
wire [39:0] postp_o_bb2_xxyy;

post_image_process #(
   .IW          (`DDR_IMG2_WIDTH    ),
   .IH          (`DDR_IMG2_HEIGHT   )
)
u_post_image_process (
    .rst_n          (cam_rstn           ),
    .pclk           (cmos_img_pclk      ),
    .i_hsync        (postp_i_hs         ),
    .i_vsync        (postp_i_vs         ),
    .i_de           (postp_i_de         ),
    .i_pixels       (postp_i_pixels     ),
    .rd_clk         (hdmi_pix_clk       ),
    .i_rd_row       (postp_i_rd_row     ),
    .i_rd_col       (postp_i_rd_col     ),
    .i_rd_valid     (postp_i_rd_valid   ),
    .o_rd_ready     (postp_o_rd_ready   ),
    .i_rt_mode      (postp_i_rt_mode    ),
    .o_rt0_32pix    (postp_o_rt0_32pix  ),
    .o_rt1_32pix    (postp_o_rt1_32pix  ),
    .o_rt2_32pix    (postp_o_rt2_32pix  ),
    .o_bb0_xxyy     (postp_o_bb0_xxyy   ),
    .o_bb1_xxyy     (postp_o_bb1_xxyy   ),
    .o_bb2_xxyy     (postp_o_bb2_xxyy   )
);


// ============================== HDMI Wrapper (with DDR read FSM + CDC) ==========

wire [1:0]  hdmi_play_mode;  // from sparrow_soc
wire [10:0] hdmi_src_x;
wire [10:0] hdmi_src_y;
wire        hdmi_postp_rd_valid;
wire        hdmi_init_over;

// DDR read interface between hdmi_wrapper and ddr3_axi4_adapter
wire        hdmi_ddr_rd_req;
wire [27:0] hdmi_ddr_rd_addr;
wire [15:0] hdmi_ddr_rd_data;
wire        hdmi_ddr_rd_valid;

`ifdef HDMI_ENABLE

hdmi_wrapper #(
    .SRC_IMG_WIDTH  ( `DDR_IMG2_WIDTH  ),
    .SRC_IMG_HEIGHT ( `DDR_IMG2_HEIGHT )
)
u_hdmi_wrapper (
    .pix_clk         (hdmi_pix_clk       ),
    .rstn            (hdmi_rstn          ),
`ifndef SIMULATION
    .cfg_clk         (hdmi_cfg_clk       ),
    .iic_tx_scl      (iic_tx_scl         ),
    .iic_tx_sda      (iic_tx_sda         ),
    .init_over       (hdmi_init_over     ),
    .rstn_out        (hdmi_rstn_out      ),
`endif // SIMULATION
    .vs_out          (vs_out             ),
    .hs_out          (hs_out             ),
    .de_out          (de_out             ),
    .r_out           (r_out              ),
    .g_out           (g_out              ),
    .b_out           (b_out              ),
    .play_mode       (hdmi_play_mode     ),
    .src_x           (hdmi_src_x         ),
    .src_y           (hdmi_src_y         ),
    // DDR read interface
    .ddr_rd_req      (hdmi_ddr_rd_req    ),
    .ddr_rd_addr     (hdmi_ddr_rd_addr   ),
    .ddr_rd_data     (hdmi_ddr_rd_data   ),
    .ddr_rd_valid    (hdmi_ddr_rd_valid  ),
    // CDC
    .cmos_pclk       (cmos_img_pclk      ),
    .cmos_rstn       (cam_rstn           ),
    .cmos_wr_addr_head(cam_wr_addr_head  ),
    // post_image_process
    .postp_rd_valid  (hdmi_postp_rd_valid),
    .postp_rd_ready  (postp_o_rd_ready   ),
    .postp_rt0_32pix (postp_o_rt0_32pix  ),
    .postp_rt1_32pix (postp_o_rt1_32pix  ),
    .postp_rt2_32pix (postp_o_rt2_32pix  ),
    .postp_bb0_xxyy  (postp_o_bb0_xxyy   ),
    .postp_bb1_xxyy  (postp_o_bb1_xxyy   ),
    .postp_bb2_xxyy  (postp_o_bb2_xxyy   )
);

`endif // HDMI_ENABLE

// hdmi_wrapper → post_image_process read trigger
assign postp_i_rd_col   = hdmi_src_x[9:0];
assign postp_i_rd_row   = hdmi_src_y[9:0];
assign postp_i_rd_valid = hdmi_postp_rd_valid;


// ============================== DDR3 Adapter ====================================

parameter DDR_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;

wire        ddr_pll_lock;
wire        ddr_clk;

// DDR debug interface — from sparrow_soc.img_ctrl
wire                ddr_in_req;
wire                ddr_in_valid;
wire [7:0]          ddr_in_data;
wire                ddr_in_ready;
wire                ddr_out_req;
wire [9:0]          ddr_out_size;
wire                ddr_out_ready;
wire [7:0]          ddr_out_data;
wire                ddr_out_valid;
wire [DDR_ADDR_WIDTH-1:0] ddr_in_addr;
wire [DDR_ADDR_WIDTH-1:0] ddr_out_addr;

ddr3_axi4_adapter #(
  .IMG_WIDTH            (`DDR_IMG2_WIDTH        ),
  .IMG_HEIGHT           (`DDR_IMG2_HEIGHT       ),
  .MEM_ROW_ADDR_WIDTH   (MEM_ROW_ADDR_WIDTH     ),
  .MEM_COL_ADDR_WIDTH   (MEM_COL_ADDR_WIDTH     ),
  .MEM_BADDR_WIDTH      (MEM_BADDR_WIDTH        ),
  .MEM_DQ_WIDTH         (MEM_DQ_WIDTH           )
) u_ddr_adapter (
  .ref_clk              (sys_clk                ),
  .rst_board            (ddr_rstn               ),
  .pll_lock             (ddr_pll_lock           ),
  .ddr_init_done        (ddr_init_done          ),
  .ddr_clk              (ddr_clk                ),

  .in_req               (ddr_in_req             ),
  .in_addr              (ddr_in_addr            ),
  .in_valid             (ddr_in_valid           ),
  .in_data              (ddr_in_data            ),
  .in_ready             (ddr_in_ready           ),
  .out_req              (ddr_out_req            ),
  .out_addr             (ddr_out_addr           ),
  .out_size             (ddr_out_size           ),
  .out_ready            (ddr_out_ready          ),
  .out_data             (ddr_out_data           ),
  .out_valid            (ddr_out_valid          ),

`ifdef DDR_WR_RD_IMG
  .img_in_clk           (cmos_img_pclk          ),
  .img_in_req           (cam_ddr_req            ),
  .img_in_addr          (cam_ddr_addr           ),
  .img_in_valid         (cam_ddr_valid          ),
  .img_in_data          (cam_ddr_data           ),
  .img_in_ready         (cam_ddr_ready          ),
  .img_in_full          (cam_ddr_full           ),

  .img_out_clk          (hdmi_pix_clk           ),
  .img_out_req          (hdmi_ddr_rd_req        ),
  .img_out_addr         (hdmi_ddr_rd_addr       ),
`ifdef DDR_RD_IMG_BY_POSITION_ENABLE
  .img_out_offset       (hdmi_src_x             ),
`else
  .img_out_ready        (1'b0                   ),
`endif
  .img_out_data         (hdmi_ddr_rd_data       ),
  .img_out_valid        (hdmi_ddr_rd_valid      ),
`endif // DDR_WR_RD_IMG

  .mem_rst_n             (mem_rst_n             ),
  .mem_ck                (mem_ck                ),
  .mem_ck_n              (mem_ck_n              ),
  .mem_cke               (mem_cke               ),
  .mem_cs_n              (mem_cs_n              ),
  .mem_ras_n             (mem_ras_n             ),
  .mem_cas_n             (mem_cas_n             ),
  .mem_we_n              (mem_we_n              ),
  .mem_odt               (mem_odt               ),
  .mem_a                 (mem_a                 ),
  .mem_ba                (mem_ba                ),
  .mem_dqs               (mem_dqs               ),
  .mem_dqs_n             (mem_dqs_n             ),
  .mem_dq                (mem_dq                ),
  .mem_dm                (mem_dm                )
);


// ============================== RISC-V SoC ======================================

// FPIOA signals
wire [15:0] fpioa_ot;
wire [15:0] fpioa_oe;
wire [15:0] fpioa_in;
wire core_active_w;

sparrow_soc inst_sparrow_soc (
    .clk(sys_clk),
    .hard_rst_n(top_rstn),
    .core_active(core_active_w),

    .JTAG_TMS(JTAG_TMS),
    .JTAG_TDI(JTAG_TDI),
    .JTAG_TDO(JTAG_TDO),
    .JTAG_TCK(JTAG_TCK),

    .fpioa_ot(fpioa_ot),
    .fpioa_oe(fpioa_oe),
    .fpioa_in(fpioa_in),

    .ddr_in_req(ddr_in_req),
    .ddr_in_addr(ddr_in_addr),
    .ddr_in_valid(ddr_in_valid),
    .ddr_in_data(ddr_in_data),
    .ddr_in_ready(ddr_in_ready),
    .ddr_out_req(ddr_out_req),
    .ddr_out_addr(ddr_out_addr),
    .ddr_out_size(ddr_out_size),
    .ddr_out_ready(ddr_out_ready),
    .ddr_out_data(ddr_out_data),
    .ddr_out_valid(ddr_out_valid),

    .cmos_sel(cmos_sel),
    .cmos_stop_capture(cmos_stop_capture),
    .hdmi_play_mode(hdmi_play_mode),
    .postp_i_rt_mode(postp_i_rt_mode)
);


// ============================== FPIOA Pin Assembly ==============================

// LED4/LED6 mode indication.  A "0.5 s blink" toggles every 0.5 s;
// a "0.25 s blink" toggles every 0.25 s.
reg [25:0] led_slow_cnt = 26'd0;
reg [24:0] led_fast_cnt = 25'd0;
reg        led_blink_0p5s = 1'b0;
reg        led_blink_0p25s = 1'b0;

always @(posedge sys_clk) begin
    if (~pll_locked || ~key_rstn) begin
        led_slow_cnt     <= 26'd0;
        led_fast_cnt     <= 25'd0;
        led_blink_0p5s   <= 1'b0;
        led_blink_0p25s  <= 1'b0;
    end
    else begin
        if (led_slow_cnt == CLK_FREQ / 2 - 1) begin
            led_slow_cnt   <= 26'd0;
            led_blink_0p5s <= ~led_blink_0p5s;
        end
        else begin
            led_slow_cnt <= led_slow_cnt + 26'd1;
        end

        if (led_fast_cnt == CLK_FREQ / 4 - 1) begin
            led_fast_cnt    <= 25'd0;
            led_blink_0p25s <= ~led_blink_0p25s;
        end
        else begin
            led_fast_cnt <= led_fast_cnt + 25'd1;
        end
    end
end

reg led4_mode;
reg led6_mode;

always @(*) begin
    case (postp_i_rt_mode)
        2'b01:  led4_mode = led_blink_0p5s;  // color filter + erosion
        2'b10:  led4_mode = led_blink_0p25s; // color filter + erosion + dilation
        default: led4_mode = 1'b0;            // color filter only
    endcase

    case (hdmi_play_mode)
        2'b01:  led6_mode = led_blink_0p5s;  // target overlay
        2'b10:  led6_mode = led_blink_0p25s; // target + bounding box
        2'b11:  led6_mode = 1'b1;            // static color bars
        default: led6_mode = 1'b0;            // no target overlay
    endcase
end

assign uart_tx = fpioa_oe[0] ? fpioa_ot[0] : 1'bz;
assign fpioa_in[0] = uart_tx;

assign uart_rx = fpioa_oe[1] ? fpioa_ot[1] : 1'bz;
assign fpioa_in[1] = uart_rx;

assign led[0] = fpioa_ot[2]; // LED1: debug mode, controlled by SoC firmware
assign led[1] = fpioa_ot[3];
assign led[2] = fpioa_ot[4];
assign led[3] = led4_mode;
assign led[4] = fpioa_ot[6];
assign led[5] = led6_mode;
assign led[6] = fpioa_ot[8];
assign fpioa_in[8:2] = 7'b0;

assign led[7] = core_active_w;

assign fpioa_in[15:9] = keys_stable[6:0];


endmodule
