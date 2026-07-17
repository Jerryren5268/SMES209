// Image write/read FIFO: DDR_WR_Cache + DDR_RD_Cache for frame buffer pixel access
// Write side (camera domain): buffers one row then triggers AXI4 burst write
// Read side (HDMI domain): requests row from DDR, buffers and streams pixels

`define RD_BY_COL_ADDR_ENABLE

module img_wr_rd_fifo # (
    parameter                    CTRL_ADDR_WIDTH      = 28    ,
    parameter                    DDR_DATA_WIDTH       = 32 * 8,
    parameter                    LEN_WIDTH            = 32
) (
// common signals
    input                                  wr_clk         , // top layer, write clock (from camera)
    input                                  rd_clk         , // top layer, read clock (from hdmi)
    input                                  ddr_clk        , // ddr inside, write / read clock
    input                                  rstn           , // reset
// I/O between top layer
    input                                  in_req          ,
    input     [CTRL_ADDR_WIDTH-1:0]        in_addr         ,
    input                                  in_valid        ,
    input     [15:0]                       in_data         , // 1 pixel
    output    reg                          in_ready        ,
    output                                 in_full         ,
    input                                  out_req         ,
    input     [CTRL_ADDR_WIDTH-1:0]        out_addr        ,
    input     [10:0]                       out_size        , // count of pixels (16 bits per pixel), max to 2047
`ifdef RD_BY_COL_ADDR_ENABLE
    input     [10:0]                       out_offset      , // index of columns in current row
`else
    input                                  out_ready       ,
`endif
    output    [15:0]                       out_data        , // 1 pixel
    output    reg                          out_valid       ,
// interaction between ddr/axi
    // write channel
    output    reg                          wr_cmd_req =1'b0, // request DDR to write, keep high when in_req done (high -> low)
    output    reg [LEN_WIDTH-1 : 0]        wr_cmd_pix_cnt ,  // when reading from ddr3, next 16 piexel (256 bits) each clock when rd_cmd_rdata_valid
    input                                  wr_cmd_ready    , // when ddr write done, output from AXI master, keep high when write done
    input                                  wr_cmd_wdone    , // when ddr write done, output from AXI master, have 1 clock high when write done
    output    reg [CTRL_ADDR_WIDTH-1:0]    wr_cmd_waddr    ,
    output    reg [DDR_DATA_WIDTH-1:0]     wr_cmd_wdata    ,
    input                                  wr_cmd_wdata_req, // keep high when write start (1st to last pkt), output from AXI master
    input                                  wr_cmd_wdata_req_end, // negedge
    // read channel
    output    reg                          rd_cmd_req =1'b0, // request to AXI master to read data from DDR3, keep high until rd_cmd_done happen
    output    reg [LEN_WIDTH-1 : 0]        rd_cmd_pix_cnt  , // pixel size = cmd_len (brust count) * (2^4)
    input                                  rd_cmd_done     , // output from AXI master, have 1 clock high when read done
    output    reg [CTRL_ADDR_WIDTH-1:0]    rd_cmd_raddr    ,
    input     [DDR_DATA_WIDTH-1:0]         rd_cmd_rdata    ,
    input                                  rd_cmd_rdata_valid // source from AXI slave (DDR3), keep high when reading
);


/****************************** internal signals ******************************/

reg in_req_d1 = 1'b0;                      // delay 1 clock, aligned with wr_clk
always @(posedge wr_clk) begin
  in_req_d1 <= in_req;
end

wire in_req_start = (~in_req_d1) & in_req; // posedge
wire in_req_end   = (~in_req) & in_req_d1; // negedge


reg out_req_d1 = 1'b0;
always @(posedge rd_clk) begin
  out_req_d1 <= out_req;
end
wire out_req_start   = (~out_req_d1) & out_req; // posedge
wire out_req_end     = (~out_req) & out_req_d1; // negedge

reg [CTRL_ADDR_WIDTH-1:0] out_addr_hold;
reg [10:0]                out_size_hold;
reg                       out_req_start_toggle;
reg                       out_req_end_toggle;

// CDC: request edge is synchronized into ddr_clk with toggle pulses.
always @(posedge rd_clk) begin
  if (~rstn) begin
    out_addr_hold <= {CTRL_ADDR_WIDTH{1'b0}};
    out_size_hold <= 11'd0;
    out_req_start_toggle <= 1'b0;
    out_req_end_toggle <= 1'b0;
  end
  else begin
    if (out_req_start) begin
      out_addr_hold <= out_addr;
      out_size_hold <= out_size;
      out_req_start_toggle <= ~out_req_start_toggle;
    end

    if (out_req_end) begin
      out_req_end_toggle <= ~out_req_end_toggle;
    end
  end
end

reg out_req_start_toggle_d1;
reg out_req_start_toggle_d2;
reg out_req_end_toggle_d1;
reg out_req_end_toggle_d2;
reg [CTRL_ADDR_WIDTH-1:0] out_addr_ddr;
reg [10:0]                out_size_ddr;

always @(posedge ddr_clk) begin
  if (~rstn) begin
    out_req_start_toggle_d1 <= 1'b0;
    out_req_start_toggle_d2 <= 1'b0;
    out_req_end_toggle_d1 <= 1'b0;
    out_req_end_toggle_d2 <= 1'b0;
  end
  else begin
    out_req_start_toggle_d1 <= out_req_start_toggle;
    out_req_start_toggle_d2 <= out_req_start_toggle_d1;
    out_req_end_toggle_d1 <= out_req_end_toggle;
    out_req_end_toggle_d2 <= out_req_end_toggle_d1;
  end
end

wire out_req_start_ddr = out_req_start_toggle_d1 ^ out_req_start_toggle_d2;
wire out_req_end_ddr   = out_req_end_toggle_d1 ^ out_req_end_toggle_d2;

/******************************FIFO: Write channel ******************************/
// write address fifo, index aligned with wr_buf_wr_head & wr_buf_rd_tail
reg [CTRL_ADDR_WIDTH-1:0] wr_addr_buf[7:0];

reg [12:0] wr_buf_wr_offset = 12'b0;
reg [2:0]  wr_buf_wr_head = 3'b0;

reg [8:0]  wr_buf_rd_offset = 9'b0;
reg [2:0]  wr_buf_rd_tail = 3'b0;

// wire wr_buf_wr_full  = ((wr_buf_wr_head + 3'b1) == wr_buf_rd_tail) ? 1'b1 : 1'b0;
assign in_full = ((wr_buf_wr_head + 3'b1) == wr_buf_rd_tail_sync) ? 1'b1 : 1'b0;

wire wr_buf_rd_empty = (wr_buf_rd_tail == wr_buf_wr_head_sync) ? 1'b1 : 1'b0;

reg wr_cmd_wdata_req_d1 = 1'b0;
reg [1:0] wr_cmd_req_d1 = 2'b0;
always @(posedge ddr_clk) begin
  wr_cmd_wdata_req_d1 <= wr_cmd_wdata_req;
  wr_cmd_req_d1 <= wr_cmd_req;
end

reg  [255:0] wr_buf_rd_data_d1;
wire [255:0] wr_buf_rd_data;

wire [2:0] wr_buf_rd_tail_gray = {wr_buf_rd_tail[2],
                                  wr_buf_rd_tail[2] ^ wr_buf_rd_tail[1],
                                  wr_buf_rd_tail[1] ^ wr_buf_rd_tail[0]};
wire [2:0] wr_buf_wr_head_gray = {wr_buf_wr_head[2],
                                  wr_buf_wr_head[2] ^ wr_buf_wr_head[1],
                                  wr_buf_wr_head[1] ^ wr_buf_wr_head[0]};

reg [2:0] wr_buf_rd_tail_gray_w1;
reg [2:0] wr_buf_rd_tail_gray_w2;
reg [2:0] wr_buf_wr_head_gray_d1;
reg [2:0] wr_buf_wr_head_gray_d2;

// CDC: FIFO pointers are Gray-coded and synchronized across clock domains.
always @(posedge wr_clk) begin
  if (~rstn) begin
    wr_buf_rd_tail_gray_w1 <= 3'd0;
    wr_buf_rd_tail_gray_w2 <= 3'd0;
  end
  else begin
    wr_buf_rd_tail_gray_w1 <= wr_buf_rd_tail_gray;
    wr_buf_rd_tail_gray_w2 <= wr_buf_rd_tail_gray_w1;
  end
end

always @(posedge ddr_clk) begin
  if (~rstn) begin
    wr_buf_wr_head_gray_d1 <= 3'd0;
    wr_buf_wr_head_gray_d2 <= 3'd0;
  end
  else begin
    wr_buf_wr_head_gray_d1 <= wr_buf_wr_head_gray;
    wr_buf_wr_head_gray_d2 <= wr_buf_wr_head_gray_d1;
  end
end

wire [2:0] wr_buf_rd_tail_sync;
wire [2:0] wr_buf_wr_head_sync;

assign wr_buf_rd_tail_sync[2] = wr_buf_rd_tail_gray_w2[2];
assign wr_buf_rd_tail_sync[1] = wr_buf_rd_tail_gray_w2[2] ^ wr_buf_rd_tail_gray_w2[1];
assign wr_buf_rd_tail_sync[0] = wr_buf_rd_tail_gray_w2[2] ^ wr_buf_rd_tail_gray_w2[1]
                                                           ^ wr_buf_rd_tail_gray_w2[0];

assign wr_buf_wr_head_sync[2] = wr_buf_wr_head_gray_d2[2];
assign wr_buf_wr_head_sync[1] = wr_buf_wr_head_gray_d2[2] ^ wr_buf_wr_head_gray_d2[1];
assign wr_buf_wr_head_sync[0] = wr_buf_wr_head_gray_d2[2] ^ wr_buf_wr_head_gray_d2[1]
                                                           ^ wr_buf_wr_head_gray_d2[0];

// assign wr_cmd_wdata = wr_cmd_wdata_req_d1 ? wr_buf_rd_data : wr_buf_rd_data_d1;
always @(*) begin
    if (wr_cmd_wdata_req_d1) begin
        wr_cmd_wdata = wr_buf_rd_data;
    end
    else begin
        wr_cmd_wdata = wr_buf_rd_data_d1;
    end
end

always @(posedge wr_clk) begin
  if (~rstn) begin
      wr_buf_wr_offset <= 13'b0;
  end
  else if (in_req & in_valid) begin // ddr3 write request on-going, with pixels sync[ing]
      wr_buf_wr_offset <= wr_buf_wr_offset + 13'b1;
  end
end

always @(posedge wr_clk) begin
  if (~rstn) begin
      wr_cmd_pix_cnt <= 32'b0;
      wr_buf_wr_head <= 3'b0;
  end
  else if (in_req_start) begin // ddr3 write request start
      wr_cmd_pix_cnt <= in_valid ? 32'b1 : 32'b0; // when in_valid also, that 1st pixel sync[ed]
      wr_addr_buf[wr_buf_wr_head] <= in_addr;
  end
  else if (in_req_end) begin // ddr3 write request end, with last pixel sync[ed]
      // wr_buf_wr_head <= (in_full) ? wr_buf_wr_head : (wr_buf_wr_head + 3'b1);
      wr_buf_wr_head <= wr_buf_wr_head + 3'b1;
  end
  else if (in_req & in_valid) begin // ddr3 write request on-going, with pixels sync[ing]
      wr_cmd_pix_cnt <= wr_cmd_pix_cnt + 32'b1;
  end
end


always @(posedge ddr_clk) begin
  // reset when reset from top, or request finish
  if (~rstn) begin
    wr_buf_rd_offset <= 9'b0;
    wr_buf_rd_tail <= 3'b0;
    wr_cmd_req <= 1'b0;
    in_ready <= 1'b0;
  end
  else if ((~wr_cmd_req) & (~wr_buf_rd_empty)) begin // ddr3 write official start
    wr_cmd_req <= 1'b1;
    wr_cmd_waddr <= wr_addr_buf[wr_buf_rd_tail];
    in_ready <= 1'b0;
  end
  else if (wr_cmd_req & (~wr_cmd_req_d1)) begin // delay 1 clock
    // last 1 clock - ddr3 ddr3 write official started last clock,
    // ddr write on-wating, cache 0th brust data, wr_buf_rd_data_d1 as 0th brust data, wr_buf_rd_data as t+1
    wr_buf_rd_offset <= wr_buf_rd_offset + 9'b1;
    wr_buf_rd_data_d1 <= wr_buf_rd_data;
  end
  else if (wr_cmd_req && wr_cmd_wdone) begin // ddr3 write official end
    wr_cmd_req <= 1'b0;
    in_ready <= 1'b1;
    wr_buf_rd_tail <= wr_buf_rd_tail + 3'b1;
    wr_buf_rd_offset <= wr_buf_rd_offset - 9'b1;   // on-purpose, wr_buf_rd_offset ahead 1
  end
  else if (wr_cmd_req && wr_cmd_wdata_req) begin // ddr3 write on-going
    wr_buf_rd_offset <= wr_buf_rd_offset + 9'b1;
    wr_buf_rd_data_d1 <= wr_buf_rd_data;
  end
  else if (wr_cmd_req && wr_cmd_wdata_req_end) begin // on-wait: ddr3 write next
    // wr_buf_rd_offset <= wr_buf_rd_offset + 7'b1; // on-purpose, wr_buf_rd_data still no write into ddr, so current wr_buf_rd_offset don't update
    wr_buf_rd_data_d1 <= wr_buf_rd_data;
  end
end


////////////////////////////// wr_buf, support by IP CORE / DRM ///////////////////
wire wr_buf_wr_en = in_valid & in_req;

wire [12:0] wr_buf_wr_addr;
assign wr_buf_wr_addr = wr_buf_wr_offset;

wire [8:0]  wr_buf_rd_addr;
assign wr_buf_rd_addr = wr_buf_rd_offset;

DDR_WR_Cache wr_buf (
    .wr_data        ( in_data                       ), // input [15:0]
    .wr_addr        ( wr_buf_wr_addr                ), // input [12:0]
    .wr_en          ( wr_buf_wr_en                  ), // input, 1 - write enable, 0 - read enable
    .wr_clk         ( wr_clk                        ),
    .wr_rst         ( ~rstn                         ),

    .rd_data        ( wr_buf_rd_data                ), // output [255:0]
    .rd_addr        ( wr_buf_rd_addr                ), // input [8:0]
    .rd_clk         ( ddr_clk                       ),
    .rd_rst         ( ~rstn                         )
) ;

//////////////////////////////////////////////////////////////////////////////////


/******************************FIFO: Read channel ******************************/
// wire rd_buf_wr_en = rd_cmd_req;   // when reqest read ddr3 offical, enable write rd_buf

// reg [1:0]  rd_buf_wr_head = 2'b0;
reg        rd_buf_wr_head = 1'b0;
reg [6:0]  rd_buf_wr_offset = 7'b0;  // when reading from ddr3, +1 (while rd_buf_rd_offset need to +16)

// reg [1:0]  rd_buf_rd_tail = 2'b0;
reg        rd_buf_rd_tail = 1'b0;

`ifndef RD_BY_COL_ADDR_ENABLE
reg [10:0] rd_buf_rd_offset = 11'b0; // when reading from upper module, +1 (meaning 1 pixel)
wire rd_buf_rd_empty = ( (rd_buf_rd_offset + 11'd1) == {rd_buf_wr_offset, 4'b0} ) ? 1'b1 : 1'b0;
`endif

always @(*) begin
    rd_cmd_pix_cnt = {21'b0, out_size_ddr};
end

always @(posedge rd_clk) begin
  if (~rstn) begin
    // rd_buf_rd_tail <= 2'b0;
    rd_buf_rd_tail <= 1'b0;
`ifndef RD_BY_COL_ADDR_ENABLE
    rd_buf_rd_offset <= 11'b0;
`endif
  end
`ifndef RD_BY_COL_ADDR_ENABLE
  else if (out_req_start) begin
    // when top layer start to read rd_buf
    rd_buf_rd_offset <= 11'b0;
  end
`endif
  else if (out_req_end) begin
    // when top layer finish read rd_buf
    // rd_buf_rd_tail <= rd_buf_rd_tail + 2'b1;
    rd_buf_rd_tail <= rd_buf_rd_tail + 1'b1;
`ifndef RD_BY_COL_ADDR_ENABLE
    rd_buf_rd_offset <= 11'b0;
`endif
  end
`ifndef RD_BY_COL_ADDR_ENABLE
  else if (out_valid && out_ready) begin
    // when top layer is reading rd_buf
    rd_buf_rd_offset <= rd_buf_rd_offset + 11'd1;
  end
`endif
end

always @(posedge ddr_clk) begin
  if (~rstn) begin
    // rd_buf_wr_head <= 2'b0;
    rd_buf_wr_head <= 1'b0;
    rd_buf_wr_offset <= 7'b0;
    rd_cmd_req <= 1'b0;
    out_valid <= 1'b0;
    out_addr_ddr <= {CTRL_ADDR_WIDTH{1'b0}};
    out_size_ddr <= 11'd0;
  end
  else if (rd_cmd_req && rd_cmd_done) begin
    // when ddr3 read offical end, with last 256bits sync[ed] to rd_buf
    rd_cmd_req <= 1'b0; // need to stop ddr3 read request
    out_valid <= 1'b1;  // callback to top layer, rd_buf is ready to read
  end
  else if (rd_cmd_req && rd_cmd_rdata_valid) begin
    // case 1: ddr3 read offical started, with 0st 256bits sync[ed]
    // case 2: ddr3 read on-going, with {rd_buf_wr_offset} th 256bits sync[ed]
    rd_buf_wr_offset <= rd_buf_wr_offset + 7'd1;
  end
  else if ((~rd_cmd_req) & out_req_start_ddr) begin
    // when top layer request to read DDR3, rise ddr3 read request (rd_cmd_req)
    rd_buf_wr_offset <= 7'b0;
    rd_cmd_req <= 1'b1;
    out_addr_ddr <= out_addr_hold;
    out_size_ddr <= out_size_hold;
    rd_cmd_raddr <= out_addr_hold;
    out_valid <= 1'b0;
  end
  else if (out_valid & out_req_end_ddr) begin
    // when top layer finish read rd_buf, also need to ensure only (rd_buf_wr_head ++) once
    // rd_buf_wr_head <= rd_buf_wr_head + 2'b1;
    rd_buf_wr_head <= rd_buf_wr_head + 1'b1;
    rd_buf_wr_offset <= 7'b0;
    out_valid <= 1'b0;
  end
end


////////////////////////////// rd_buf, support by IP CORE / DRM ///////////////////

wire [11:0] rd_buf_rd_addr; // 10:0 bit -- offset of row, lefting bits -- index of row
`ifdef RD_BY_COL_ADDR_ENABLE
assign rd_buf_rd_addr = {rd_buf_rd_tail, out_offset};
`else
assign rd_buf_rd_addr = {rd_buf_rd_tail, rd_buf_rd_offset};
`endif

wire [7:0]  rd_buf_wr_addr; // 6:0 bit -- ofset of row, lefting bit -- index of row, aligned with rd_buf_rd_addr[end:11]
assign rd_buf_wr_addr = {rd_buf_wr_head, rd_buf_wr_offset};

DDR_RD_Cache rd_buf (
    .wr_data        ( rd_cmd_rdata                  ), // input [255:0]
    .wr_addr        ( rd_buf_wr_addr                ), // input [7:0]
    .wr_en          ( rd_cmd_req                    ), // input, 1 - write enable, 0 - read enable
    .wr_clk         ( ddr_clk                       ),
    .wr_rst         ( ~rstn                         ),

    .rd_data        ( out_data                      ), // output [15:0]
    .rd_addr        ( rd_buf_rd_addr                ), // input [11:0]
    .rd_clk         ( rd_clk                        ),
    .rd_rst         ( ~rstn                         )
) ;


//////////////////////////////////////////////////////////////////////////////////


endmodule
