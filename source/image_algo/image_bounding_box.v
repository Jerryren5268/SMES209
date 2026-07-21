// Bounding box detector for a binary target stream.
// Tracks per-frame min/max coordinates inside ROI and latches the result at
// the next frame start. If a frame has no target, a zero/invalid box is output.

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

localparam [9:0] IMG_X_MAX = IW - 1;
localparam [9:0] IMG_Y_MAX = IH - 1;

wire [9:0] roi_x_start = (i_Xstart > IMG_X_MAX) ? IMG_X_MAX : i_Xstart;
wire [9:0] roi_x_end   = (i_Xend   > IMG_X_MAX) ? IMG_X_MAX : i_Xend;
wire [9:0] roi_y_start = (i_Ystart > IMG_Y_MAX) ? IMG_Y_MAX : i_Ystart;
wire [9:0] roi_y_end   = (i_Yend   > IMG_Y_MAX) ? IMG_Y_MAX : i_Yend;
wire       roi_valid   = (i_Xstart <= i_Xend) && (i_Ystart <= i_Yend);

reg vsync_d1;
reg hsync_d1;
reg row_had_data;

wire vsync_pos = i_vsync && !vsync_d1;
wire hsync_neg = !i_hsync && hsync_d1;

reg [9:0] h_count;
reg [9:0] v_count;

reg       match_found;
reg [9:0] cur_x_min;
reg [9:0] cur_x_max;
reg [9:0] cur_y_min;
reg [9:0] cur_y_max;

wire pixel_valid = i_hsync && i_de;
wire pixel_in_roi = roi_valid
                  && (h_count >= roi_x_start) && (h_count <= roi_x_end)
                  && (v_count >= roi_y_start) && (v_count <= roi_y_end);
wire target_pixel = pixel_valid && i_match && pixel_in_roi;

always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        vsync_d1     <= 1'b0;
        hsync_d1     <= 1'b0;
        h_count      <= 10'd0;
        v_count      <= 10'd0;
        row_had_data <= 1'b0;
    end
    else begin
        vsync_d1 <= i_vsync;
        hsync_d1 <= i_hsync;

        if (vsync_pos) begin
            h_count      <= 10'd0;
            v_count      <= 10'd0;
            row_had_data <= 1'b0;
        end
        else begin
            if (pixel_valid) begin
                h_count      <= (h_count == IMG_X_MAX) ? 10'd0 : (h_count + 10'd1);
                row_had_data <= 1'b1;
            end

            if (hsync_neg) begin
                h_count <= 10'd0;
                if (row_had_data && v_count != IMG_Y_MAX)
                    v_count <= v_count + 10'd1;
                row_had_data <= 1'b0;
            end
        end
    end
end

always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        match_found <= 1'b0;
        cur_x_min   <= IMG_X_MAX;
        cur_x_max   <= 10'd0;
        cur_y_min   <= IMG_Y_MAX;
        cur_y_max   <= 10'd0;

        o_Xmin <= 10'd0;
        o_Xmax <= 10'd0;
        o_Ymin <= 10'd0;
        o_Ymax <= 10'd0;
    end
    else if (vsync_pos) begin
        if (match_found) begin
            o_Xmin <= cur_x_min;
            o_Xmax <= cur_x_max;
            o_Ymin <= cur_y_min;
            o_Ymax <= cur_y_max;
        end
        else begin
            o_Xmin <= 10'd0;
            o_Xmax <= 10'd0;
            o_Ymin <= 10'd0;
            o_Ymax <= 10'd0;
        end

        match_found <= 1'b0;
        cur_x_min   <= IMG_X_MAX;
        cur_x_max   <= 10'd0;
        cur_y_min   <= IMG_Y_MAX;
        cur_y_max   <= 10'd0;
    end
    else if (target_pixel) begin
        if (!match_found) begin
            match_found <= 1'b1;
            cur_x_min   <= h_count;
            cur_x_max   <= h_count;
            cur_y_min   <= v_count;
            cur_y_max   <= v_count;
        end
        else begin
            if (h_count < cur_x_min)
                cur_x_min <= h_count;
            if (h_count > cur_x_max)
                cur_x_max <= h_count;
            if (v_count < cur_y_min)
                cur_y_min <= v_count;
            if (v_count > cur_y_max)
                cur_y_max <= v_count;
        end
    end
end

endmodule
