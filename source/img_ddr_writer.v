// =============================================================================
// img_ddr_writer — Camera-to-DDR write controller
//
// Manages the write path from the camera pixel stream into DDR3 frame buffer.
// Receives pixel data from pre_image_process and generates DDR write requests
// through the ddr3_axi4_adapter interface. Operates in the camera pixel clock
// domain (cmos_img_pclk).
//
// Parameters:
//   DDR_ADDR_AREA_IMG — upper address bits for image area in DDR (default: 5'b00000)
//   IMG_HEIGHT        — image height in rows (default: 360 for half resolution)
//
// Interface:
//   pclk, rstn           — camera pixel clock and async reset (active low)
//   img_ready             — camera initialization complete flag
//   ddr_init_done         — DDR3 controller initialization complete
//   stop_capture          — software-controlled capture pause (from SoC)
//   in_hs, in_vs, in_de, in_pixels — camera pixel stream input (from pre_image_process)
//   ddr_req, ddr_addr      — DDR write request and address (to ddr3_axi4_adapter)
//   ddr_valid, ddr_data   — DDR write data valid and pixel data
//   ddr_ready, ddr_full    — DDR write FIFO status (from ddr3_axi4_adapter)
//   wr_addr_head           — current write frame index (for CDC to HDMI read side)
//   error_cnt              — latched write error count from previous frame
//
// DDR address format (28 bits):
//   {DDR_ADDR_AREA_IMG[4:0], addr_head[2:0], addr_row[9:0], 10'd0}
//   Each row occupies 1024 bytes (512 pixels x 2 bytes each), leaving
//   space for IMG_WIDTH=640 pixels (1280 bytes) per row.
//
// FSM states (implicit in req/frame_started):
//   IDLE:          waiting for vs_start, all counters reset
//   FRAME_STARTED: vs_start detected, waiting for first hs_start
//   WRITING_ROW:   hs_start detected, req asserted, pixels flowing
//   ROW_DONE:      hs_end detected, addr_row incremented or wrapped
//
// Error counting: when req=1, if ddr_ready=0 or ddr_full=1, increment err_cnt.
// At each vs_start, the current err_cnt is latched to error_cnt output.
// =============================================================================

module img_ddr_writer #(
    parameter DDR_ADDR_AREA_IMG = 5'b00000,  // DDR address area prefix for image data
    parameter IMG_HEIGHT        = 11'd360    // image height in pixel rows
)(
    input              pclk,              // camera pixel clock
    input              rstn,              // async reset (active low, = top_rstn)
    input              img_ready,         // camera sensor initialized and streaming
    input              ddr_init_done,     // DDR3 controller initialization complete
    input              stop_capture,      // software pause capture (from SoC)

    input              in_hs,             // input horizontal sync (line valid)
    input              in_vs,             // input vertical sync (frame valid)
    input              in_de,             // input data enable (pixel valid)
    input  [15:0]      in_pixels,         // input pixel data (RGB565)

    output             ddr_req,           // DDR write request (to ddr3_axi4_adapter)
    output [27:0]      ddr_addr,          // DDR write address (28-bit row-major)
    output             ddr_valid,         // DDR write data valid
    output [15:0]      ddr_data,          // DDR write pixel data (RGB565)
    input              ddr_ready,         // DDR write FIFO ready to accept data
    input              ddr_full,          // DDR write FIFO full

    output [2:0]       wr_addr_head,      // Gray-coded write frame index (CDC to HDMI side)
    output [15:0]      error_cnt          // latched write error count from previous frame
);

    // ------------------------------------------------------------------
    // Pipeline registers: delay input signals by 1 clock cycle
    // This aligns pixel data with the edge-detection outputs
    // ------------------------------------------------------------------
    reg [15:0] in_pixels_d1;
    reg        in_vs_d1 = 1'b0;
    reg        in_hs_d1 = 1'b0;
    reg        in_de_d1 = 1'b0;

    always @(posedge pclk) begin
        if (~rstn || ~img_ready) begin
            // Hold pipeline in reset while camera is not ready
            in_pixels_d1 <= 16'b0;
            in_vs_d1 <= 1'b0;
            in_hs_d1 <= 1'b0;
            in_de_d1 <= 1'b0;
        end else begin
            in_pixels_d1 <= in_pixels;
            in_vs_d1 <= in_vs;
            in_hs_d1 <= in_hs;
            in_de_d1 <= in_de;
        end
    end

    // ------------------------------------------------------------------
    // Edge detection on VSYNC and HSYNC
    // vs_start: rising edge of vsync — new frame begins
    // hs_start: rising edge of hsync — new row within a frame
    // hs_end:   falling edge of hsync — current row ends
    // ------------------------------------------------------------------
    wire vs_start = (~in_vs_d1) & in_vs;
    wire hs_start = (~in_hs_d1) & in_hs;
    wire hs_end   = (~in_hs) & in_hs_d1;

    // ------------------------------------------------------------------
    // Write FSM registers
    // frame_started: high when inside a valid frame
    // req:           DDR write request, asserted during each pixel row
    // addr_head:     frame index (wraps 0..7, provides 8-frame ring buffer)
    // addr_row:      current row index within the frame (0..IMG_HEIGHT-1)
    // ------------------------------------------------------------------
    reg        frame_started = 1'b0;
    reg        req           = 1'b0;
    reg  [2:0] addr_head     = 3'b0;
    reg  [9:0] addr_row      = 10'b0;

    reg [15:0] err_cnt       = 16'b0;     // per-frame error accumulator
    reg [15:0] err_cnt_latch = 16'b0;     // latched at each vs_start

    // Address end-of-row detection: true when next row equals IMG_HEIGHT
    wire            addr_row_end = (({1'b0, addr_row} + 11'b1) == IMG_HEIGHT[10:0]);

    // DDR address format: area_prefix[4:0] + frame[2:0] + row[9:0] + 10'h000
    wire [27:0]     addr         = { DDR_ADDR_AREA_IMG, addr_head, addr_row, 10'b0 };

    // ------------------------------------------------------------------
    // Write FSM main logic
    // Priority: reset > stop_capture > vs_start > hs_start | hs_end
    // ------------------------------------------------------------------
    always @(posedge pclk) begin
        if (~rstn || ~img_ready || ~ddr_init_done) begin
            // Global reset: hold FSM idle until camera and DDR are ready
            frame_started <= 1'b0;
            req           <= 1'b0;
            err_cnt       <= 16'b0;
            addr_head     <= 3'b0;
            addr_row      <= 10'b0;
        end
        else if (stop_capture & (~req)) begin
            // Software pause: stop at end of current row, hold counters
            frame_started <= 1'b0;
            req           <= 1'b0;
            addr_row      <= 10'b0;
        end
        else if (vs_start) begin
            // New frame: latch previous frame's error count, reset per-frame state
            frame_started <= 1'b1;
            req           <= 1'b0;
            err_cnt       <= 16'b0;
            err_cnt_latch <= err_cnt;
            addr_row      <= 10'b0;
        end
        else if (hs_start & frame_started) begin
            // Start of a new row: assert DDR write request
            req  <= 1'b1;
            // Check DDR FIFO readiness — error if full or not ready
            err_cnt <= (ddr_ready & (~ddr_full))
                        ? err_cnt : (err_cnt + 16'b1);
        end
        else if (hs_end & frame_started) begin
            // End of current row: deassert request, advance to next row
            req      <= 1'b0;
            // If last row: wrap row to 0, advance frame index
            // Otherwise: increment row within current frame
            addr_row <= addr_row_end ? 10'b0 : (addr_row + 10'b1);
            addr_head <= addr_row_end ? (addr_head + 3'b1) : addr_head;
        end
    end

    // Output assignments — delayed by 1 clock for timing alignment
    assign ddr_req   = req;
    assign ddr_addr  = addr;
    assign ddr_valid = in_de_d1;    // pixel valid delayed 1 cycle
    assign ddr_data  = in_pixels_d1;

    assign wr_addr_head = {addr_head[2],
                           addr_head[2] ^ addr_head[1],
                           addr_head[1] ^ addr_head[0]};
    assign error_cnt    = err_cnt_latch;

endmodule
