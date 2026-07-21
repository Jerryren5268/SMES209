
// HDMI output wrapper — VESA timer + DDR read FSM + CDC exercise + algorithm overlay
// Manages DDR-to-HDMI reading: row-major addressing, frame-pointer sync via Gray-code CDC,
// post_image_process result overlay (color filter, bounding box)

// `define ENABLE_SRC_CLIPED
`define ENABLE_COLOR_FILTER
`define ENABLE_BOUNDING_BOX

module hdmi_wrapper #(
    parameter   SRC_IMG_WIDTH            = 11'd1280   ,
    parameter   SRC_IMG_HEIGHT           = 11'd720    ,
    parameter   X_WIDTH = 4'd12,
    parameter   Y_WIDTH = 4'd12
) (
   // Common signals
   input             pix_clk       , // 148.5Mhz (60Hz), 37.125M (30Hz)
   input             rstn          , // hdmi_rstn = ddr_init_done & pll_locked & key_rstn
   // Chip ms72xx, to configuration
`ifndef SIMULATION
   input             cfg_clk       , // 10Mhz
   output            iic_tx_scl    ,
   inout             iic_tx_sda    ,
   output            init_over     ,
   output            rstn_out      ,
`endif // SIMULATION
   // HDMI output
   output            vs_out        ,
   output            hs_out        ,
   output            de_out        ,
   output     [7:0]  r_out         ,
   output     [7:0]  g_out         ,
   output     [7:0]  b_out         ,
   // DDR source position
   input      [1:0]  play_mode     ,
   output     [10:0] src_x         ,
   output     [10:0] src_y         ,
   // DDR read interface (new — FSM now inside hdmi_wrapper)
   output            ddr_rd_req    ,
   output     [27:0] ddr_rd_addr   ,
   input      [15:0] ddr_rd_data   ,
   input             ddr_rd_valid  ,
   // camera write-pointer sync
   input             cmos_pclk         , // camera pixel clock
   input             cmos_rstn         , // reset in cmos_pclk domain
   input      [2:0]  cmos_wr_addr_head , // Gray-coded write frame index from img_ddr_writer
   // post_image_process read trigger
   output            postp_rd_valid    , // hs_start pulse for post_image_process row read
   output            postp_frame_start , // frame boundary for atomic mask-bank switch
   // post image process connection
   input            postp_rd_ready,
   input      [2:0] postp_rd_frame_id,
   input            postp_rd_frame_valid,
   input     [31:0] postp_rt0_32pix,
   input     [31:0] postp_rt1_32pix,
   input     [31:0] postp_rt2_32pix,
   input     [39:0] postp_bb0_xxyy ,
   input     [39:0] postp_bb1_xxyy ,
   input     [39:0] postp_bb2_xxyy
);

    parameter   COLOR_DEPTH = 8;

    // play mode
    localparam PLAY_MODE_NORMAL  = 2'b00;
    localparam PLAY_MODE_RT      = 2'b01;
    localparam PLAY_MODE_RT_N_BB = 2'b10;
    localparam PLAY_MODE_STOP    = 2'b11;

    /***************************** Reset *****************************/
    `ifdef SIMULATION

    wire rstn_out = rstn                   ;

    `else

    reg  [15:0]                 rstn_1ms   ;

    always @(posedge cfg_clk)
    begin
        if(!rstn)
            rstn_1ms <= 16'd0;
        else
        begin
            if(rstn_1ms == 16'h2710)
                rstn_1ms <= rstn_1ms;
            else
                rstn_1ms <= rstn_1ms + 1'b1;
        end
    end

    assign rstn_out = (rstn_1ms == 16'h2710);

    `endif // SIMULATION

    /****************** Chip ms72xx / Configuration *******************/
    `ifndef SIMULATION

    ms72xx_ctl ms72xx_ctl(
        .clk         (  cfg_clk    ),
        .rst_n       (  rstn_out   ),
        .init_over   (  init_over  ),
        .iic_tx_scl  (  iic_tx_scl ),
        .iic_tx_sda  (  iic_tx_sda ),
        .iic_scl     (  iic_scl    ),
        .iic_sda     (  iic_sda    )
    );

    `endif //SIMULATION


    /********************** Generate VESA Timing *********************/

    wire [X_WIDTH-1:0]          x_act      ;
    wire [Y_WIDTH-1:0]          y_act      ;
    wire                        x_overflow ;
    wire                        y_overflow ;
    wire [X_WIDTH-1:0]          x_extra    ;
    wire [Y_WIDTH-1:0]          y_extra    ;

    wire                        x_valid    ;
    wire                        y_valid    ;
    wire                        hs         ;
    wire                        vs         ;
    wire                        de         ;

    vesa_timer # (
        .X_BITS               (  X_WIDTH              ),
        .Y_BITS               (  Y_WIDTH              )
    )
    u_vesa_timer
    (
        .clk                  (  pix_clk              ),
        .rstn                 (  rstn_out             ),
        .vs_out               (  vs                   ),
        .hs_out               (  hs                   ),
        .de_out               (  de                   ),
        .x_act                (  x_act                ),
        .y_act                (  y_act                ),
        .x_valid              (  x_valid              ),
        .y_valid              (  y_valid              )
    );

    // src timing signals (internal, was previously exported to Top.v for FSM)
    wire src_vs = vs;
    wire src_hs = hs & y_valid;
    wire src_de = de;

    assign x_overflow = x_act >= SRC_IMG_WIDTH   ;
    assign y_overflow = y_act >= SRC_IMG_HEIGHT  ;
    assign x_extra    = x_act - SRC_IMG_WIDTH   ;
    assign y_extra    = y_act - SRC_IMG_HEIGHT  ;

    `ifdef ENABLE_SRC_CLIPED
    assign src_x  = x_act[10:0];
    assign src_y  = y_act[10:0];
    `else
    assign src_x  = x_overflow ? x_extra[10:0] : x_act[10:0];
    assign src_y  = y_overflow ? y_extra[10:0] : y_act[10:0];
    `endif


    // =================================================================
    // DDR Read FSM (moved from Top.v)
    // =================================================================

    wire        hdmi_stop_play  = (play_mode == PLAY_MODE_STOP) ? 1'b1 : 1'b0;

    // ---- sync camera write-pointer into hdmi_pix_clk domain ----
    reg [2:0] cmos_head_gray_d1;
    reg [2:0] cmos_head_gray_d2;
    wire [2:0] cmos_head_sync;

    // cmos_wr_addr_head is Gray-coded in cmos_pclk and decoded after sync.
    always @(posedge pix_clk) begin
        if (~rstn) begin
            cmos_head_gray_d1 <= 3'd0;
            cmos_head_gray_d2 <= 3'd0;
        end
        else begin
            cmos_head_gray_d1 <= cmos_wr_addr_head;
            cmos_head_gray_d2 <= cmos_head_gray_d1;
        end
    end

    assign cmos_head_sync[2] = cmos_head_gray_d2[2];
    assign cmos_head_sync[1] = cmos_head_gray_d2[2] ^ cmos_head_gray_d2[1];
    assign cmos_head_sync[0] = cmos_head_gray_d2[2] ^ cmos_head_gray_d2[1]
                                                   ^ cmos_head_gray_d2[0];

    // ---- vs/hs/de delayed for edge detection ----
    reg         src_vs_d1, src_hs_d1, src_de_d1;

    wire        vs_start = (~src_vs_d1) & src_vs;
    wire        hs_start = (~src_hs_d1) & src_hs;
    wire        hs_end   = (~src_hs) & src_hs_d1;
    wire        de_start = (~src_de_d1) & src_de;
    wire        de_end   = (~src_de) & src_de_d1;

    always @(posedge pix_clk) begin
        if (~rstn) begin
            src_vs_d1 <= 1'b0;
            src_hs_d1 <= 1'b0;
            src_de_d1 <= 1'b0;
        end else begin
            src_vs_d1 <= src_vs;
            src_hs_d1 <= src_hs;
            src_de_d1 <= src_de;
        end
    end

    // ---- read FSM registers ----
    reg         rd_req;
    reg  [2:0]  rd_addr_tail = 3'b0;
    reg  [9:0]  rd_addr_row  = 10'b0;
    reg  [15:0] rd_error     = 16'b0;
    reg  [15:0] rd_error_latch = 16'b0;

    wire        rd_addr_row_end = (({1'b0, rd_addr_row} + 11'b1) == SRC_IMG_HEIGHT[10:0]);
    wire        rd_read_last    = (rd_addr_tail + 3'b1) == cmos_head_sync;
    wire        rd_read_empty   = cmos_head_sync == rd_addr_tail;

    assign ddr_rd_addr = { 5'b00000, rd_addr_tail, rd_addr_row, 10'b0 };

    always @(posedge pix_clk) begin
        if (~rstn) begin
            rd_req       <= 1'b0;
            rd_addr_tail <= 3'b0;
            rd_addr_row  <= 10'b0;
        end
        else if (hdmi_stop_play & (~rd_req)) begin
            rd_req       <= 1'b0;
            rd_addr_row  <= 10'b0;
        end
`ifndef SIMULATION
        else if (rd_read_empty && ~postp_rd_frame_valid) begin
            rd_req       <= 1'b0;
            rd_addr_row  <= 10'b0;
        end
    `endif
        else if (vs_start) begin
            rd_req       <= 1'b0;
            rd_addr_row  <= 10'b0;
            rd_error     <= 16'b0;
            rd_error_latch <= rd_error;
        end
        else if (hs_start) begin
            rd_req    <= 1'b1;
            if ((src_y == 11'd0) && postp_rd_frame_valid)
                rd_addr_tail <= postp_rd_frame_id;
    `ifdef DDR_RD_IMG_BY_POSITION_ENABLE
            rd_addr_row <= src_y[9:0];
    `endif
        end
        else if (de_start) begin
            rd_error <= ddr_rd_valid ? rd_error : (rd_error + 16'b1);
        end
        else if (de_end) begin
            rd_req <= 1'b0;
    `ifndef DDR_RD_IMG_BY_POSITION_ENABLE
            rd_addr_row <= rd_addr_row_end ? 10'b0 : (rd_addr_row + 10'b1);
    `endif
            rd_addr_tail <= postp_rd_frame_valid
                                ? rd_addr_tail
                                : (((~rd_addr_row_end) | rd_read_last)
                                    ? rd_addr_tail : (rd_addr_tail + 3'b1));
        end
    end

    assign ddr_rd_req = rd_req;

    // post_image_process read trigger
    assign postp_rd_valid = hs_start;
    assign postp_frame_start = vs_start;

    // =================================================================
    // Post-Image-Process (unchanged)
    // =================================================================

    reg rt0_pix_match;
    reg rt1_pix_match;
    reg rt2_pix_match;

    reg bb0_pix_match;
    reg bb1_pix_match;
    reg bb2_pix_match;

    localparam [10:0] SRC_X_LAST = SRC_IMG_WIDTH - 11'd1;
    localparam [10:0] SRC_Y_LAST = SRC_IMG_HEIGHT - 11'd1;
    localparam [10:0] BB_LINE_THICKNESS = 11'd4;
    localparam [5:0]  BB_HOLD_FRAMES = 6'd10;
    localparam [9:0]  BB_DEADBAND = 10'd1;
    localparam [9:0]  BB_MAX_STEP = 10'd4;

    reg [39:0] postp_bb0_meta;
    reg [39:0] postp_bb0_sync;
    reg [39:0] postp_bb0_hdmi;
    reg [39:0] postp_bb0_draw;
    reg [39:0] postp_bb1_meta;
    reg [39:0] postp_bb1_sync;
    reg [39:0] postp_bb1_hdmi;
    reg [39:0] postp_bb1_draw;
    reg [39:0] postp_bb2_meta;
    reg [39:0] postp_bb2_sync;
    reg [39:0] postp_bb2_hdmi;
    reg [39:0] postp_bb2_draw;

    function [9:0] approach_coord;
        input [9:0] current;
        input [9:0] target;
        reg [9:0] delta;
        reg [9:0] step;
        begin
            delta = (current >= target) ? (current - target)
                                        : (target - current);
            step = delta >> 2;
            if (step == 10'd0)
                step = 10'd1;
            else if (step > BB_MAX_STEP)
                step = BB_MAX_STEP;
            if (delta <= BB_DEADBAND)
                approach_coord = current;
            else if (current < target)
                approach_coord = current + step;
            else
                approach_coord = current - step;
        end
    endfunction

    function [39:0] smooth_box;
        input [39:0] current;
        input [39:0] target;
        begin
            smooth_box = {
                approach_coord(current[39:30], target[39:30]),
                approach_coord(current[29:20], target[29:20]),
                approach_coord(current[19:10], target[19:10]),
                approach_coord(current[9:0],   target[9:0])
            };
        end
    endfunction

    wire postp_bb0_candidate_valid = (postp_bb0_hdmi[29:20] > postp_bb0_hdmi[39:30])
                                  && (postp_bb0_hdmi[ 9: 0] > postp_bb0_hdmi[19:10]);
    wire postp_bb1_candidate_valid = (postp_bb1_hdmi[29:20] > postp_bb1_hdmi[39:30])
                                  && (postp_bb1_hdmi[ 9: 0] > postp_bb1_hdmi[19:10]);
    wire postp_bb2_candidate_valid = (postp_bb2_hdmi[29:20] > postp_bb2_hdmi[39:30])
                                  && (postp_bb2_hdmi[ 9: 0] > postp_bb2_hdmi[19:10]);
    wire postp_bb0_draw_valid = (postp_bb0_draw[29:20] > postp_bb0_draw[39:30])
                              && (postp_bb0_draw[ 9: 0] > postp_bb0_draw[19:10]);
    wire postp_bb1_draw_valid = (postp_bb1_draw[29:20] > postp_bb1_draw[39:30])
                              && (postp_bb1_draw[ 9: 0] > postp_bb1_draw[19:10]);
    wire postp_bb2_draw_valid = (postp_bb2_draw[29:20] > postp_bb2_draw[39:30])
                              && (postp_bb2_draw[ 9: 0] > postp_bb2_draw[19:10]);

    reg [5:0] bb0_hold_cnt;
    reg [5:0] bb1_hold_cnt;
    reg [5:0] bb2_hold_cnt;

    always @(posedge pix_clk) begin
        if (~rstn) begin
            postp_bb0_meta <= 40'd0;
            postp_bb0_sync <= 40'd0;
            postp_bb0_hdmi <= 40'd0;
            postp_bb0_draw <= 40'd0;
            postp_bb1_meta <= 40'd0;
            postp_bb1_sync <= 40'd0;
            postp_bb1_hdmi <= 40'd0;
            postp_bb1_draw <= 40'd0;
            postp_bb2_meta <= 40'd0;
            postp_bb2_sync <= 40'd0;
            postp_bb2_hdmi <= 40'd0;
            postp_bb2_draw <= 40'd0;
            bb0_hold_cnt <= 6'd0;
            bb1_hold_cnt <= 6'd0;
            bb2_hold_cnt <= 6'd0;
        end
        else begin
            postp_bb0_meta <= postp_bb0_xxyy;
            postp_bb0_sync <= postp_bb0_meta;
            if (postp_bb0_sync == postp_bb0_meta)
                postp_bb0_hdmi <= postp_bb0_sync;

            postp_bb1_meta <= postp_bb1_xxyy;
            postp_bb1_sync <= postp_bb1_meta;
            if (postp_bb1_sync == postp_bb1_meta)
                postp_bb1_hdmi <= postp_bb1_sync;

            postp_bb2_meta <= postp_bb2_xxyy;
            postp_bb2_sync <= postp_bb2_meta;
            if (postp_bb2_sync == postp_bb2_meta)
                postp_bb2_hdmi <= postp_bb2_sync;

            if (vs_start) begin
                if (postp_bb0_candidate_valid) begin
                    postp_bb0_draw <= postp_bb0_draw_valid
                                        ? smooth_box(postp_bb0_draw, postp_bb0_hdmi)
                                        : postp_bb0_hdmi;
                    bb0_hold_cnt <= BB_HOLD_FRAMES;
                end
                else if (bb0_hold_cnt != 6'd0) begin
                    bb0_hold_cnt <= bb0_hold_cnt - 6'd1;
                end
                else begin
                    postp_bb0_draw <= 40'd0;
                end

                if (postp_bb1_candidate_valid) begin
                    postp_bb1_draw <= postp_bb1_draw_valid
                                        ? smooth_box(postp_bb1_draw, postp_bb1_hdmi)
                                        : postp_bb1_hdmi;
                    bb1_hold_cnt <= BB_HOLD_FRAMES;
                end
                else if (bb1_hold_cnt != 6'd0) begin
                    bb1_hold_cnt <= bb1_hold_cnt - 6'd1;
                end
                else begin
                    postp_bb1_draw <= 40'd0;
                end

                if (postp_bb2_candidate_valid) begin
                    postp_bb2_draw <= postp_bb2_draw_valid
                                        ? smooth_box(postp_bb2_draw, postp_bb2_hdmi)
                                        : postp_bb2_hdmi;
                    bb2_hold_cnt <= BB_HOLD_FRAMES;
                end
                else if (bb2_hold_cnt != 6'd0) begin
                    bb2_hold_cnt <= bb2_hold_cnt - 6'd1;
                end
                else begin
                    postp_bb2_draw <= 40'd0;
                end
            end
        end
    end

    wire [10:0] bb0_xmin_raw = {postp_bb0_draw[39:30], 1'b0};
    wire [10:0] bb0_xmax_raw = {postp_bb0_draw[29:20], 1'b0};
    wire [10:0] bb0_ymin_raw = {postp_bb0_draw[19:10], 1'b0};
    wire [10:0] bb0_ymax_raw = {postp_bb0_draw[ 9: 0], 1'b0};

    wire [10:0] bb1_xmin_raw = {postp_bb1_draw[39:30], 1'b0};
    wire [10:0] bb1_xmax_raw = {postp_bb1_draw[29:20], 1'b0};
    wire [10:0] bb1_ymin_raw = {postp_bb1_draw[19:10], 1'b0};
    wire [10:0] bb1_ymax_raw = {postp_bb1_draw[ 9: 0], 1'b0};

    wire [10:0] bb2_xmin_raw = {postp_bb2_draw[39:30], 1'b0};
    wire [10:0] bb2_xmax_raw = {postp_bb2_draw[29:20], 1'b0};
    wire [10:0] bb2_ymin_raw = {postp_bb2_draw[19:10], 1'b0};
    wire [10:0] bb2_ymax_raw = {postp_bb2_draw[ 9: 0], 1'b0};

    wire [10:0] bb0_xmin = (bb0_xmin_raw > SRC_X_LAST) ? SRC_X_LAST : bb0_xmin_raw;
    wire [10:0] bb0_xmax = (bb0_xmax_raw > SRC_X_LAST) ? SRC_X_LAST : bb0_xmax_raw;
    wire [10:0] bb0_ymin = (bb0_ymin_raw > SRC_Y_LAST) ? SRC_Y_LAST : bb0_ymin_raw;
    wire [10:0] bb0_ymax = (bb0_ymax_raw > SRC_Y_LAST) ? SRC_Y_LAST : bb0_ymax_raw;

    wire [10:0] bb1_xmin = (bb1_xmin_raw > SRC_X_LAST) ? SRC_X_LAST : bb1_xmin_raw;
    wire [10:0] bb1_xmax = (bb1_xmax_raw > SRC_X_LAST) ? SRC_X_LAST : bb1_xmax_raw;
    wire [10:0] bb1_ymin = (bb1_ymin_raw > SRC_Y_LAST) ? SRC_Y_LAST : bb1_ymin_raw;
    wire [10:0] bb1_ymax = (bb1_ymax_raw > SRC_Y_LAST) ? SRC_Y_LAST : bb1_ymax_raw;

    wire [10:0] bb2_xmin = (bb2_xmin_raw > SRC_X_LAST) ? SRC_X_LAST : bb2_xmin_raw;
    wire [10:0] bb2_xmax = (bb2_xmax_raw > SRC_X_LAST) ? SRC_X_LAST : bb2_xmax_raw;
    wire [10:0] bb2_ymin = (bb2_ymin_raw > SRC_Y_LAST) ? SRC_Y_LAST : bb2_ymin_raw;
    wire [10:0] bb2_ymax = (bb2_ymax_raw > SRC_Y_LAST) ? SRC_Y_LAST : bb2_ymax_raw;

    wire bb0_valid = (bb0_xmax > bb0_xmin) && (bb0_ymax > bb0_ymin);
    wire bb1_valid = (bb1_xmax > bb1_xmin) && (bb1_ymax > bb1_ymin);
    wire bb2_valid = (bb2_xmax > bb2_xmin) && (bb2_ymax > bb2_ymin);

    wire play_with_rt_pix = postp_rd_ready
                          && (play_mode == PLAY_MODE_RT || play_mode == PLAY_MODE_RT_N_BB);
    wire play_with_bb_pix = (play_mode == PLAY_MODE_RT_N_BB);

    always @(posedge pix_clk) begin
        if (~rstn) begin
            rt0_pix_match = 1'b0;
        end
        else if (~play_with_rt_pix) begin
            rt0_pix_match = 1'b0;
        end
        else case (src_x[5:1])
                5'd0: rt0_pix_match = postp_rt0_32pix[0];
                5'd1: rt0_pix_match = postp_rt0_32pix[1];
                5'd2: rt0_pix_match = postp_rt0_32pix[2];
                5'd3: rt0_pix_match = postp_rt0_32pix[3];
                5'd4: rt0_pix_match = postp_rt0_32pix[4];
                5'd5: rt0_pix_match = postp_rt0_32pix[5];
                5'd6: rt0_pix_match = postp_rt0_32pix[6];
                5'd7: rt0_pix_match = postp_rt0_32pix[7];
                5'd8: rt0_pix_match = postp_rt0_32pix[8];
                5'd9: rt0_pix_match = postp_rt0_32pix[9];
                5'd10: rt0_pix_match = postp_rt0_32pix[10];
                5'd11: rt0_pix_match = postp_rt0_32pix[11];
                5'd12: rt0_pix_match = postp_rt0_32pix[12];
                5'd13: rt0_pix_match = postp_rt0_32pix[13];
                5'd14: rt0_pix_match = postp_rt0_32pix[14];
                5'd15: rt0_pix_match = postp_rt0_32pix[15];
                5'd16: rt0_pix_match = postp_rt0_32pix[16];
                5'd17: rt0_pix_match = postp_rt0_32pix[17];
                5'd18: rt0_pix_match = postp_rt0_32pix[18];
                5'd19: rt0_pix_match = postp_rt0_32pix[19];
                5'd20: rt0_pix_match = postp_rt0_32pix[20];
                5'd21: rt0_pix_match = postp_rt0_32pix[21];
                5'd22: rt0_pix_match = postp_rt0_32pix[22];
                5'd23: rt0_pix_match = postp_rt0_32pix[23];
                5'd24: rt0_pix_match = postp_rt0_32pix[24];
                5'd25: rt0_pix_match = postp_rt0_32pix[25];
                5'd26: rt0_pix_match = postp_rt0_32pix[26];
                5'd27: rt0_pix_match = postp_rt0_32pix[27];
                5'd28: rt0_pix_match = postp_rt0_32pix[28];
                5'd29: rt0_pix_match = postp_rt0_32pix[29];
                5'd30: rt0_pix_match = postp_rt0_32pix[30];
                5'd31: rt0_pix_match = postp_rt0_32pix[31];
                default: rt0_pix_match = 1'b0;
            endcase
    end

    always @(posedge pix_clk) begin
        if (~rstn) begin
            rt1_pix_match = 1'b0;
        end
        else if (~play_with_rt_pix) begin
            rt1_pix_match = 1'b0;
        end
        else case (src_x[5:1])
                5'd0: rt1_pix_match = postp_rt1_32pix[0];
                5'd1: rt1_pix_match = postp_rt1_32pix[1];
                5'd2: rt1_pix_match = postp_rt1_32pix[2];
                5'd3: rt1_pix_match = postp_rt1_32pix[3];
                5'd4: rt1_pix_match = postp_rt1_32pix[4];
                5'd5: rt1_pix_match = postp_rt1_32pix[5];
                5'd6: rt1_pix_match = postp_rt1_32pix[6];
                5'd7: rt1_pix_match = postp_rt1_32pix[7];
                5'd8: rt1_pix_match = postp_rt1_32pix[8];
                5'd9: rt1_pix_match = postp_rt1_32pix[9];
                5'd10: rt1_pix_match = postp_rt1_32pix[10];
                5'd11: rt1_pix_match = postp_rt1_32pix[11];
                5'd12: rt1_pix_match = postp_rt1_32pix[12];
                5'd13: rt1_pix_match = postp_rt1_32pix[13];
                5'd14: rt1_pix_match = postp_rt1_32pix[14];
                5'd15: rt1_pix_match = postp_rt1_32pix[15];
                5'd16: rt1_pix_match = postp_rt1_32pix[16];
                5'd17: rt1_pix_match = postp_rt1_32pix[17];
                5'd18: rt1_pix_match = postp_rt1_32pix[18];
                5'd19: rt1_pix_match = postp_rt1_32pix[19];
                5'd20: rt1_pix_match = postp_rt1_32pix[20];
                5'd21: rt1_pix_match = postp_rt1_32pix[21];
                5'd22: rt1_pix_match = postp_rt1_32pix[22];
                5'd23: rt1_pix_match = postp_rt1_32pix[23];
                5'd24: rt1_pix_match = postp_rt1_32pix[24];
                5'd25: rt1_pix_match = postp_rt1_32pix[25];
                5'd26: rt1_pix_match = postp_rt1_32pix[26];
                5'd27: rt1_pix_match = postp_rt1_32pix[27];
                5'd28: rt1_pix_match = postp_rt1_32pix[28];
                5'd29: rt1_pix_match = postp_rt1_32pix[29];
                5'd30: rt1_pix_match = postp_rt1_32pix[30];
                5'd31: rt1_pix_match = postp_rt1_32pix[31];
                default: rt1_pix_match = 1'b0;
            endcase
    end

    always @(posedge pix_clk) begin
        if (~rstn) begin
            rt2_pix_match = 1'b0;
        end
        else if (~play_with_rt_pix) begin
            rt2_pix_match = 1'b0;
        end
        else case (src_x[5:1])
                5'd0: rt2_pix_match = postp_rt2_32pix[0];
                5'd1: rt2_pix_match = postp_rt2_32pix[1];
                5'd2: rt2_pix_match = postp_rt2_32pix[2];
                5'd3: rt2_pix_match = postp_rt2_32pix[3];
                5'd4: rt2_pix_match = postp_rt2_32pix[4];
                5'd5: rt2_pix_match = postp_rt2_32pix[5];
                5'd6: rt2_pix_match = postp_rt2_32pix[6];
                5'd7: rt2_pix_match = postp_rt2_32pix[7];
                5'd8: rt2_pix_match = postp_rt2_32pix[8];
                5'd9: rt2_pix_match = postp_rt2_32pix[9];
                5'd10: rt2_pix_match = postp_rt2_32pix[10];
                5'd11: rt2_pix_match = postp_rt2_32pix[11];
                5'd12: rt2_pix_match = postp_rt2_32pix[12];
                5'd13: rt2_pix_match = postp_rt2_32pix[13];
                5'd14: rt2_pix_match = postp_rt2_32pix[14];
                5'd15: rt2_pix_match = postp_rt2_32pix[15];
                5'd16: rt2_pix_match = postp_rt2_32pix[16];
                5'd17: rt2_pix_match = postp_rt2_32pix[17];
                5'd18: rt2_pix_match = postp_rt2_32pix[18];
                5'd19: rt2_pix_match = postp_rt2_32pix[19];
                5'd20: rt2_pix_match = postp_rt2_32pix[20];
                5'd21: rt2_pix_match = postp_rt2_32pix[21];
                5'd22: rt2_pix_match = postp_rt2_32pix[22];
                5'd23: rt2_pix_match = postp_rt2_32pix[23];
                5'd24: rt2_pix_match = postp_rt2_32pix[24];
                5'd25: rt2_pix_match = postp_rt2_32pix[25];
                5'd26: rt2_pix_match = postp_rt2_32pix[26];
                5'd27: rt2_pix_match = postp_rt2_32pix[27];
                5'd28: rt2_pix_match = postp_rt2_32pix[28];
                5'd29: rt2_pix_match = postp_rt2_32pix[29];
                5'd30: rt2_pix_match = postp_rt2_32pix[30];
                5'd31: rt2_pix_match = postp_rt2_32pix[31];
                default: rt2_pix_match = 1'b0;
            endcase
    end


    always @(posedge pix_clk) begin
        if (~rstn) begin
            bb0_pix_match = 1'b0;
        end
        else if (~play_with_bb_pix) begin
            bb0_pix_match = 1'b0;
        end
    `ifdef ENABLE_BOUNDING_BOX
        else if (bb0_valid
                 && (((src_x >= bb0_xmin) && (src_x < bb0_xmin + BB_LINE_THICKNESS))
                     || ((src_x >= bb0_xmax) && (src_x < bb0_xmax + BB_LINE_THICKNESS)))
                    && src_y >= bb0_ymin && src_y <= bb0_ymax) begin
            bb0_pix_match = 1'b1;
        end
        else if (bb0_valid
                 && (((src_y >= bb0_ymin) && (src_y < bb0_ymin + BB_LINE_THICKNESS))
                     || ((src_y >= bb0_ymax) && (src_y < bb0_ymax + BB_LINE_THICKNESS)))
                    && src_x >= bb0_xmin && src_x <= bb0_xmax) begin
            bb0_pix_match = 1'b1;
        end
    `endif // ENABLE_BOUNDING_BOX
        else begin
            bb0_pix_match = 1'b0;
        end
    end

    always @(posedge pix_clk) begin
        if (~rstn) begin
            bb1_pix_match = 1'b0;
        end
        else if (~play_with_bb_pix) begin
            bb1_pix_match = 1'b0;
        end
    `ifdef ENABLE_BOUNDING_BOX
        else if (bb1_valid
                 && (((src_x >= bb1_xmin) && (src_x < bb1_xmin + BB_LINE_THICKNESS))
                     || ((src_x >= bb1_xmax) && (src_x < bb1_xmax + BB_LINE_THICKNESS)))
                    && src_y >= bb1_ymin && src_y <= bb1_ymax) begin
            bb1_pix_match = 1'b1;
        end
        else if (bb1_valid
                 && (((src_y >= bb1_ymin) && (src_y < bb1_ymin + BB_LINE_THICKNESS))
                     || ((src_y >= bb1_ymax) && (src_y < bb1_ymax + BB_LINE_THICKNESS)))
                    && src_x >= bb1_xmin && src_x <= bb1_xmax) begin
            bb1_pix_match = 1'b1;
        end
    `endif // ENABLE_BOUNDING_BOX
        else begin
            bb1_pix_match = 1'b0;
        end
    end

    always @(posedge pix_clk) begin
        if (~rstn) begin
            bb2_pix_match = 1'b0;
        end
        else if (~play_with_bb_pix) begin
            bb2_pix_match = 1'b0;
        end
    `ifdef ENABLE_BOUNDING_BOX
        else if (bb2_valid
                 && (((src_x >= bb2_xmin) && (src_x < bb2_xmin + BB_LINE_THICKNESS))
                     || ((src_x >= bb2_xmax) && (src_x < bb2_xmax + BB_LINE_THICKNESS)))
                    && src_y >= bb2_ymin && src_y <= bb2_ymax) begin
            bb2_pix_match = 1'b1;
        end
        else if (bb2_valid
                 && (((src_y >= bb2_ymin) && (src_y < bb2_ymin + BB_LINE_THICKNESS))
                     || ((src_y >= bb2_ymax) && (src_y < bb2_ymax + BB_LINE_THICKNESS)))
                    && src_x >= bb2_xmin && src_x <= bb2_xmax) begin
            bb2_pix_match = 1'b1;
        end
    `endif // ENABLE_BOUNDING_BOX
        else begin
            bb2_pix_match = 1'b0;
        end
    end

    /********************** HDMI output prepare *********************/

    // vs, hs, de delayed by 1 to align with ddr_rd_data latency
    reg vs_d1;
    reg hs_d1;
    reg de_d1;

    always @(posedge pix_clk) begin
        vs_d1 <= vs;
        hs_d1 <= hs;
        de_d1 <= de;
    end


    reg [15:0] src_rgb565_cliped;

    `ifdef ENABLE_SRC_CLIPED
        always @(*) begin
            if (src_x >= SRC_IMG_WIDTH) begin
                src_rgb565_cliped = 16'b0;
            end
            else if (src_y >= SRC_IMG_HEIGHT) begin
                src_rgb565_cliped = 16'b0;
            end
            else begin
                src_rgb565_cliped = ddr_rd_data;
            end
        end

    `else

    `ifdef ENABLE_COLOR_FILTER
        always @(*) begin
            if (x_overflow & y_overflow) // Bottom-Right Area: color filter 2, green
            begin
                src_rgb565_cliped = bb2_pix_match ? 16'h7e0
                                    : rt2_pix_match ? 16'h1f : ddr_rd_data;
            end
            else if (x_overflow)         // Top-Right Area: color filter 0, black
            begin
                src_rgb565_cliped = bb0_pix_match ? 16'h7e0
                                    : rt0_pix_match ? 16'h1f : ddr_rd_data;
            end
            else if (y_overflow)         // Bottom-Left Area: color filter 1, red
            begin
                src_rgb565_cliped = bb1_pix_match ? 16'h7e0
                                    : rt1_pix_match ? 16'h1f : ddr_rd_data;
            end
            else                         // Top-Left Area: no filter
            begin
                src_rgb565_cliped = ddr_rd_data;
            end
        end

    `else
        // ~ENABLE_SRC_CLIPED, ~ENABLE_COLOR_FILTER
        always @(*) begin
            src_rgb565_cliped = ddr_rd_data;
        end

    `endif // ENABLE_COLOR_FILTER
    `endif // ENABLE_SRC_CLIPED

    /********************** HDMI output Adapter *********************/

    wire src_valid = (play_mode == PLAY_MODE_STOP) ? 1'b0 : 1'b1;

    hdmi_output_adapter #(
        .X_WIDTH              (  X_WIDTH              ),
        .Y_WIDTH              (  Y_WIDTH              ),
        .COLOR_DEPTH          (  COLOR_DEPTH          )
    )
    u_output_adapter (
        .rstn                 (  rstn_out             ),
        .pix_clk              (  pix_clk              ),
        .pix_data             (  src_rgb565_cliped    ),
        .pix_valid            (  src_valid            ),
        .vs_in                (  vs_d1                ),
        .hs_in                (  hs_d1                ),
        .de_in                (  de_d1                ),
        .x_act                (  x_act                ),
        .y_act                (  y_act                ),
        .vs_out               (  vs_out               ),
        .hs_out               (  hs_out               ),
        .de_out               (  de_out               ),
        .r_out                (  r_out                ),
        .g_out                (  g_out                ),
        .b_out                (  b_out                )
    );


endmodule
