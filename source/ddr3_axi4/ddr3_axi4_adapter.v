// DDR3 AXI4 adapter — write/read router, image FIFO, debug FIFO, AXI4 master controller
// Bridges camera write and HDMI read to DDR3 via AXI4. Internal ctrl_rstn = rst_board & ddr_init_done.

`timescale 1ps/1ps

`define DDR3
`define DDR_WR_RD_IMG
`define RD_IMG_BY_COL_ADDR_ENABLE

module ddr3_axi4_adapter #(
  parameter IMG_WIDTH            = 11'd1280   , // count of pixels (16 bits per pixel) each row, max to 2047
  parameter IMG_HEIGHT           = 11'd720    , 
  parameter MEM_ROW_ADDR_WIDTH   = 15         ,
  parameter MEM_COL_ADDR_WIDTH   = 10         ,
  parameter MEM_BADDR_WIDTH      = 3          ,
  parameter MEM_DQ_WIDTH         = 32         ,
  parameter MEM_DM_WIDTH         = MEM_DQ_WIDTH/8,
  parameter MEM_DQS_WIDTH        = MEM_DQ_WIDTH/8,
  parameter CTRL_ADDR_WIDTH      = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH
)(
  input                                  ref_clk         ,
  input                                  rst_board       ,
  output                                 pll_lock        ,           
  output                                 ddr_init_done   ,
  output                                 ddr_clk         ,
  
  // debug interface
  // each sync will have 16 x 2^5 (2^9=512) bytes fixed
  input                                  in_req          ,
  input     [CTRL_ADDR_WIDTH-1:0]        in_addr         ,
  input                                  in_valid        ,
  input     [7:0]                        in_data         ,
  output                                 in_ready        ,
  input                                  out_req         ,
  input     [CTRL_ADDR_WIDTH-1:0]        out_addr        ,
  input     [9:0]                        out_size        , // count of bytes, max to 1023
  input                                  out_ready       ,
  output    [7:0]                        out_data        ,
  output                                 out_valid       ,

  // image W/R interface
  // each sync will have 1 row pixels, max to 2047 pixels, adjustable
`ifdef DDR_WR_RD_IMG
  input                                  img_in_clk       ,
  input                                  img_in_req       ,
  input     [CTRL_ADDR_WIDTH-1:0]        img_in_addr      ,
  input                                  img_in_valid     ,
  input     [15:0]                       img_in_data      ,
  output                                 img_in_ready     ,
  output                                 img_in_full      ,
  input                                  img_out_clk      ,
  input                                  img_out_req      ,
  input     [CTRL_ADDR_WIDTH-1:0]        img_out_addr     ,
`ifdef RD_IMG_BY_COL_ADDR_ENABLE
  input     [10:0]                       img_out_offset   , // index of columns in current row, max to 2047
`else
  input                                  img_out_ready    ,
`endif
  output    [15:0]                       img_out_data     ,
  output                                 img_out_valid    ,
`endif 

  // DDR interface
  output                                 mem_rst_n       ,                       
  output                                 mem_ck          ,
  output                                 mem_ck_n        ,
  output                                 mem_cke         ,
  output                                 mem_cs_n        ,
  output                                 mem_ras_n       ,
  output                                 mem_cas_n       ,
  output                                 mem_we_n        ,  
  output                                 mem_odt         ,
  output     [MEM_ROW_ADDR_WIDTH-1:0]    mem_a           ,   
  output     [MEM_BADDR_WIDTH-1:0]       mem_ba          ,   
  inout      [MEM_DQS_WIDTH-1:0]         mem_dqs         ,
  inout      [MEM_DQS_WIDTH-1:0]         mem_dqs_n       ,
  inout      [MEM_DQ_WIDTH-1:0]          mem_dq          ,
  output     [MEM_DM_WIDTH-1:0]          mem_dm          
);

assign ctrl_rstn = rst_board & ddr_init_done;

//***********************************************************************************

localparam LEN_WIDTH       = 32;
localparam DDR_DATA_WIDTH  = MEM_DQ_WIDTH * 8;

reg                         wr_cmd_req;     
reg  [CTRL_ADDR_WIDTH-1:0]  wr_cmd_addr;            // write addr address
reg  [LEN_WIDTH-1 : 0]      wr_cmd_len;             // cmd_len (brust count) = byte size / (2^5)
wire                        wr_cmd_ready;           // output from AXI master, keep high when write done
wire                        rd_cmd_ready;           // output from AXI master, high when read command path is ready
wire                        wr_cmd_wdone;           // output from AXI master, have 1 clock high when write done
reg [DDR_DATA_WIDTH-1:0]    wr_cmd_wdata;    
wire                        wr_cmd_wdata_req;       // output from AXI master, keep high when write start (1st to last pkt)

reg                         rd_cmd_req;             // request to AXI master to read data from DDR3, keep high until rd_cmd_done happen
reg [CTRL_ADDR_WIDTH-1:0]   rd_cmd_addr;            // read ddr address
reg [LEN_WIDTH-1 : 0]       rd_cmd_len  ;           // cmd_len (brust count) = byte size / (2^5)
wire                        rd_cmd_done;            // output from AXI master, have 1 clock high when read done
                            
wire                        rd_cmd_rdata_ready = 1'b1;
wire [DDR_DATA_WIDTH-1:0]   rd_cmd_rdata  ;
wire                        rd_cmd_rdata_valid;     // source from AXI slave (DDR3), keep high when reading

//////////////////////////////////////////////////////////////////////////////////


/****************************** Router: Write Request ******************************/

reg wr_cmd_wdata_req_d1;
always @(posedge ddr_clk) begin
  wr_cmd_wdata_req_d1 <= wr_cmd_wdata_req;
end 

// wire wr_cmd_wdata_req_posedge = (~wr_cmd_wdata_req_d1) & wr_cmd_wdata_req;
wire wr_cmd_wdata_req_negedge = (~wr_cmd_wdata_req) & wr_cmd_wdata_req_d1;


///////////////////////////// signals related to Debug FIFO 
reg                               dbg_wr_enable = 1'b0  ;
wire                              dbg_wr_cmd_req        ;
wire [LEN_WIDTH-1:0]              dbg_wr_cmd_byte_cnt   ;
wire [DDR_DATA_WIDTH-1:0]         dbg_wr_cmd_wdata      ;


///////////////////////////// signals related to Debug FIFO 
`ifdef DDR_WR_RD_IMG
reg                               img_wr_enable = 1'b0  ;
wire                              img_wr_cmd_req        ;
wire [LEN_WIDTH-1:0]              img_wr_cmd_pix_cnt    ;
wire [DDR_DATA_WIDTH-1:0]         img_wr_cmd_wdata      ;
wire [CTRL_ADDR_WIDTH-1:0]        img_wr_cmd_waddr      ;
`endif 

///////////////////////////// wr_enable

always @(posedge ddr_clk) begin
  if (~ctrl_rstn) begin
    dbg_wr_enable <= 1'b0;
  end 
  // when have 1 enable writer 
  else if (dbg_wr_enable) begin
    dbg_wr_enable <= wr_cmd_wdone ? 1'b0 : 1'b1;
  end 
`ifdef DDR_WR_RD_IMG
  else if (img_wr_enable) begin
    img_wr_enable <= wr_cmd_wdone ? 1'b0 : 1'b1;
  end 
`endif 
  // when have no write, wait for request
  // priority: imgage > debug
`ifdef DDR_WR_RD_IMG
  else if (img_wr_cmd_req) begin
    img_wr_enable <= 1'b1;
  end 
`endif 
  else if (dbg_wr_cmd_req) begin
    dbg_wr_enable <= 1'b1;
  end 
end 


///////////////////////////// wr_cmd_req & wr_cmd_len

always @(*) begin
  // case by case
  if (dbg_wr_enable) begin
    wr_cmd_req <= dbg_wr_cmd_req;
    wr_cmd_len <= (dbg_wr_cmd_byte_cnt >> 5); // 256 bits = 32 x 8 bits (1 byte)
    wr_cmd_wdata <= dbg_wr_cmd_wdata;
    wr_cmd_addr <= in_addr;
  end 
`ifdef DDR_WR_RD_IMG
  else if (img_wr_enable) begin
    wr_cmd_req <= img_wr_cmd_req;
    // wr_cmd_len = (img_wr_cmd_pix_cnt >> 4); // 256 bits = 16 x 16 bits (1 pixel)
    wr_cmd_len <= {21'b0, IMG_WIDTH >> 4};
    wr_cmd_wdata <= img_wr_cmd_wdata;
    wr_cmd_addr <= img_wr_cmd_waddr;
  end 
`endif 
  // no writer
  else begin
    wr_cmd_req <= 1'b0;
    wr_cmd_len <= 32'b0;
    wr_cmd_wdata <= 512'b0;
    wr_cmd_addr <= 28'b0;
  end 
end 


//////////////////////////////////////////////////////////////////////////////////


/******************************Router: Read Request ******************************/

reg rd_cmd_done_d1 = 1'b0;
always @(posedge ddr_clk) begin
  rd_cmd_done_d1 <= rd_cmd_done;
end 

wire rd_cmd_done_posedge = (~rd_cmd_done_d1) & rd_cmd_done;
// wire rd_cmd_done_negedge = (~rd_cmd_done) & rd_cmd_done_d1;


///////////////////////////// signals related to Debug FIFO 
reg                               dbg_rd_enable = 1'b0  ;
wire                              dbg_rd_cmd_req        ;
wire [LEN_WIDTH-1:0]              dbg_rd_cmd_byte_cnt   ;


///////////////////////////// signals related to Debug FIFO 
`ifdef DDR_WR_RD_IMG
reg                               img_rd_enable = 1'b0  ;
wire                              img_rd_cmd_req        ;
wire [CTRL_ADDR_WIDTH-1:0]        img_rd_cmd_raddr      ;
wire [LEN_WIDTH-1:0]              img_rd_cmd_pix_cnt    ;
`endif 


///////////////////////////// rd_enable

always @(posedge ddr_clk) begin
  if (~ctrl_rstn) begin
    dbg_rd_enable <= 1'b0;
  end 
  // when have 1 enable reader 
  else if (dbg_rd_enable) begin
    dbg_rd_enable <= rd_cmd_done_posedge ? 1'b0 : 1'b1;
  end 
`ifdef DDR_WR_RD_IMG
  else if (img_rd_enable) begin
    img_rd_enable <= rd_cmd_done_posedge ? 1'b0 : 1'b1;
  end 
`endif 
  // when have no reader, wait for request
  // priority: image > debug
`ifdef DDR_WR_RD_IMG
  else if (img_rd_cmd_req) begin
    img_rd_enable <= 1'b1;
  end 
`endif 
  else if (dbg_rd_cmd_req) begin
    dbg_rd_enable <= 1'b1;
  end 
end 


///////////////////////////// rd_cmd_req & rd_cmd_len

always @(*) begin
  // case by case
  if (dbg_rd_enable) begin
    rd_cmd_req <= dbg_rd_cmd_req;
    rd_cmd_len <= (dbg_rd_cmd_byte_cnt >> 5); // 256 bits = 32 x 8 bits (1 byte)
    rd_cmd_addr <= out_addr;
  end 
`ifdef DDR_WR_RD_IMG
  else if (img_rd_enable) begin
    rd_cmd_req <= img_rd_cmd_req;
    rd_cmd_len <= (img_rd_cmd_pix_cnt >> 4);  // 256 bits = 16 x 16 bits (1 pixel)
    rd_cmd_addr <= img_rd_cmd_raddr;
  end
`endif 
  // no writer
  else begin
    rd_cmd_req <= 1'b0;
    rd_cmd_len <= 32'b0;
    rd_cmd_addr <= 28'b0;
  end 
end 


//////////////////////////////////////////////////////////////////////////////////


/****************************** Debug WR/RD FIFO ********************************/

wire dbg_wr_cmd_ready         = dbg_wr_enable ? wr_cmd_ready         : 1'b0;
wire dbg_wr_cmd_wdone         = dbg_wr_enable ? wr_cmd_wdone         : 1'b0;
wire dbg_wr_cmd_wdata_req     = dbg_wr_enable ? wr_cmd_wdata_req     : 1'b0;
wire dbg_wr_cmd_wdata_req_end = dbg_wr_enable ? wr_cmd_wdata_req_negedge : 1'b0;

wire dbg_rd_cmd_done          = dbg_rd_enable ? rd_cmd_done_posedge  : 1'b0;
wire dbg_rd_cmd_rdata_valid   = dbg_rd_enable ? rd_cmd_rdata_valid   : 1'b0;

dbg_wr_rd_fifo #(
    .CTRL_ADDR_WIDTH             (CTRL_ADDR_WIDTH             ),
    .DDR_DATA_WIDTH              (DDR_DATA_WIDTH              ),
    .LEN_WIDTH                   (LEN_WIDTH                   )
) u_dbg_wr_rd_fifo (
// common signals
    .ref_clk                     (ref_clk                     ), // input, top layer, write / read clock
    .ddr_clk                     (ddr_clk                     ), // input, ddr inside, write / read clock
    .rstn                        (ctrl_rstn                   ), // input, reset 
// I/O between top layer
    .in_req                      (in_req                      ), // input
    .in_valid                    (in_valid                    ), // input
    .in_data                     (in_data                     ), // input [7:0]
    .in_ready                    (in_ready                    ), // output
    .out_req                     (out_req                     ), // input 
    .out_size                    (out_size                    ), // input 
    .out_ready                   (out_ready                   ), // input 
    .out_data                    (out_data                    ), // output reg [7:0]
    .out_valid                   (out_valid                   ), // output
// interaction between ddr/axi
    // write channel
    .wr_cmd_req                  (dbg_wr_cmd_req              ), // output, request DDR to write, keep high when in_req done (high -> low)
    .wr_cmd_byte_cnt             (dbg_wr_cmd_byte_cnt         ), // output, when reading from ddr3, +32 each clock when rd_cmd_rdata_valid
    .wr_cmd_ready                (dbg_wr_cmd_ready            ), // input, when ddr write done, output from AXI master, keep high when write done
    .wr_cmd_wdone                (dbg_wr_cmd_wdone            ), // input, when ddr write done, output from AXI master, have 1 clock high when write done
    .wr_cmd_wdata                (dbg_wr_cmd_wdata            ), // output [DDR_DATA_WIDTH-1:0]
    .wr_cmd_wdata_req            (dbg_wr_cmd_wdata_req        ), // input, keep high when write start (1st to last pkt), output from AXI master
    .wr_cmd_wdata_req_end        (dbg_wr_cmd_wdata_req_end    ), // input, negedge
    // read channel 
    .rd_cmd_req                  (dbg_rd_cmd_req              ), // output, request to AXI master to read data from DDR3, keep high until rd_cmd_done happen
    .rd_cmd_byte_cnt             (dbg_rd_cmd_byte_cnt         ), // output [LEN_WIDTH-1 : 0], cmd_len (brust count) = byte size / (2^5)
    .rd_cmd_done                 (dbg_rd_cmd_done             ), // input, output from AXI master, have 1 clock high when read done
    .rd_cmd_rdata                (rd_cmd_rdata                ), // input [DDR_DATA_WIDTH-1:0]
    .rd_cmd_rdata_valid          (dbg_rd_cmd_rdata_valid      )  // input, source from AXI slave (DDR3), keep high when reading
);

//////////////////////////////////////////////////////////////////////////////////


/****************************** Image WR/RD FIFO ********************************/
`ifdef DDR_WR_RD_IMG

wire img_wr_cmd_ready         = img_wr_enable ? wr_cmd_ready         : 1'b0;
wire img_wr_cmd_wdone         = img_wr_enable ? wr_cmd_wdone         : 1'b0;
wire img_wr_cmd_wdata_req     = img_wr_enable ? wr_cmd_wdata_req     : 1'b0;
wire img_wr_cmd_wdata_req_end = img_wr_enable ? wr_cmd_wdata_req_negedge : 1'b0;

wire img_rd_cmd_done          = img_rd_enable ? rd_cmd_done_posedge  : 1'b0;
wire img_rd_cmd_rdata_valid   = img_rd_enable ? rd_cmd_rdata_valid   : 1'b0;

img_wr_rd_fifo #(
    .CTRL_ADDR_WIDTH             (CTRL_ADDR_WIDTH             ),
    .DDR_DATA_WIDTH              (DDR_DATA_WIDTH              ),
    .LEN_WIDTH                   (LEN_WIDTH                   )
) u_img_wr_rd_fifo (
// common signals
    .wr_clk                      (img_in_clk                  ), // input, top layer, image write clock
    .rd_clk                      (img_out_clk                 ), // input, top layer, image read clock
    .ddr_clk                     (ddr_clk                     ), // input, ddr inside, write / read clock
    .rstn                        (ctrl_rstn                   ), // input, reset 
// I/O between top layer
    .in_req                      (img_in_req                  ), // input
    .in_addr                     (img_in_addr                 ), // input [27:0]
    .in_valid                    (img_in_valid                ), // input
    .in_data                     (img_in_data                 ), // input [15:0]
    .in_ready                    (img_in_ready                ), // output
    .in_full                     (img_in_full                 ), // output
    .out_req                     (img_out_req                 ), // input 
    .out_addr                    (img_out_addr                ), // input [27:0]
    .out_size                    (IMG_WIDTH                   ), // input [10:0] 
`ifdef RD_IMG_BY_COL_ADDR_ENABLE
    .out_offset                  (img_out_offset              ), // input [10:0]
`else 
    .out_ready                   (img_out_ready               ), // input 
`endif 
    .out_data                    (img_out_data                ), // output [15:0]
    .out_valid                   (img_out_valid               ), // output
// interaction between ddr/axi
    // write channel
    .wr_cmd_req                  (img_wr_cmd_req              ), // output, request DDR to write, keep high when in_req done (high -> low)
    .wr_cmd_pix_cnt              (img_wr_cmd_pix_cnt          ), // output, when reading from ddr3, next 16 piexel (256 bits) each clock when rd_cmd_rdata_valid
    .wr_cmd_ready                (img_wr_cmd_ready            ), // input, when ddr write done, output from AXI master, keep high when write done
    .wr_cmd_wdone                (img_wr_cmd_wdone            ), // input, when ddr write done, output from AXI master, have 1 clock high when write done
    .wr_cmd_wdata                (img_wr_cmd_wdata            ), // output [DDR_DATA_WIDTH-1:0]
    .wr_cmd_waddr                (img_wr_cmd_waddr            ), // output [27:0]
    .wr_cmd_wdata_req            (img_wr_cmd_wdata_req        ), // input, keep high when write start (1st to last pkt), output from AXI master
    .wr_cmd_wdata_req_end        (img_wr_cmd_wdata_req_end    ), // input, negedge
    // read channel 
    .rd_cmd_req                  (img_rd_cmd_req              ), // output, request to AXI master to read data from DDR3, keep high until rd_cmd_done happen
    .rd_cmd_pix_cnt              (img_rd_cmd_pix_cnt          ), // output [LEN_WIDTH-1 : 0], pixel size = cmd_len (brust count) * (2^4)
    .rd_cmd_done                 (img_rd_cmd_done             ), // input, output from AXI master, have 1 clock high when read done
    .rd_cmd_rdata                (rd_cmd_rdata                ), // input [DDR_DATA_WIDTH-1:0]
    .rd_cmd_raddr                (img_rd_cmd_raddr            ), // output [27:0]
    .rd_cmd_rdata_valid          (img_rd_cmd_rdata_valid      )  // input, source from AXI slave (DDR3), keep high when reading
);

`endif 

//////////////////////////////////////////////////////////////////////////////////


/************************* AXI Master as Controller ***************************/

wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr     ;
wire [3:0]                  axi_awlen      ;
wire [2:0]                  axi_awsize     ;
wire [1:0]                  axi_awburst    ;
wire                        axi_awready    ;
wire                        axi_awvalid    ;
                                            
wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata      ;
wire [MEM_DQ_WIDTH -1 :0]   axi_wstrb      ;
wire                        axi_wlast      ;
wire                        axi_wvalid     ;
wire                        axi_wready     ;                                
wire [3:0]                  axi_wusero_id  ;

wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr     ;
wire [3:0]                  axi_arlen      ;
wire [2:0]                  axi_arsize     ;
wire [1:0]                  axi_arburst    ;
wire                        axi_arvalid    ;
wire                        axi_arready    ;
                                            
wire                        axi_rready     ;
wire  [MEM_DQ_WIDTH*8-1:0]  axi_rdata      ;
wire                        axi_rvalid     ;
wire                        axi_rlast      ;
wire  [3:0]                 axi_rid        ;


wr_rd_ctrl_top#(
    .CTRL_ADDR_WIDTH  (  CTRL_ADDR_WIDTH  ), //parameter                   CTRL_ADDR_WIDTH      = 28,
    .MEM_DQ_WIDTH     (  MEM_DQ_WIDTH     ) //parameter                    MEM_DQ_WIDTH         = 16
)wr_rd_ctrl_top (                         
    .clk              (  ddr_clk          ),//input                        clk            ,            
    .rstn             (  ctrl_rstn        ),//input                        rstn           ,            
                                          
    .wr_cmd_en        (  wr_cmd_req       ),//input                        wr_cmd_en   ,
    .wr_cmd_addr      (  wr_cmd_addr      ),//input  [CTRL_ADDR_WIDTH-1:0] wr_cmd_addr ,
    .wr_cmd_len       (  wr_cmd_len       ),//input  [31：0]               wr_cmd_len  ,
    .wr_cmd_ready     (  wr_cmd_ready     ),//output                       wr_cmd_ready,
    .wr_cmd_done      (  wr_cmd_wdone     ),//output                       wr_cmd_done,                                   
    .wr_cmd_data      (  wr_cmd_wdata     ),//input  [MEM_DQ_WIDTH*8-1:0]  wr_ctrl_data,
    .wr_cmd_data_req  (  wr_cmd_wdata_req ),//output                       wr_data_req  ,
                                          
    .rd_cmd_en        (  rd_cmd_req       ),//input                        rd_cmd_en   ,
    .rd_cmd_addr      (  rd_cmd_addr      ),//input  [CTRL_ADDR_WIDTH-1:0] rd_cmd_addr ,
    .rd_cmd_len       (  rd_cmd_len       ),//input  [31：0]               rd_cmd_len  ,
    .rd_cmd_ready     (  rd_cmd_ready     ),//output                       rd_cmd_ready,
    .rd_cmd_done      (  rd_cmd_done      ),//output                       rd_cmd_done,
                                          
    .read_ready       (  rd_cmd_rdata_ready),//input                        read_ready  ,    
    .read_rdata       (  rd_cmd_rdata      ),//output [MEM_DQ_WIDTH*8-1:0]  read_rdata  ,    
    .read_en          (  rd_cmd_rdata_valid),//output                       read_rdata_en  ,

    // write channel                        
    .axi_awaddr       (  axi_awaddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_awaddr     ,  
    .axi_awlen        (  axi_awlen        ),//output [3:0]                 axi_awlen      , // input, <= 4'd15, which tx burst count <= 16
    .axi_awsize       (  axi_awsize       ),//output [2:0]                 axi_awsize     , // input, Fixed at 3'b101, which tx 32 bytes per burst (that is 256 bits)
    .axi_awburst      (  axi_awburst      ),//output [1:0]                 axi_awburst    , //only support 2'b01: INCR
    .axi_awready      (  axi_awready      ),//input                        axi_awready    ,
    .axi_awvalid      (  axi_awvalid      ),//output                       axi_awvalid    ,
                                          
    .axi_wdata        (  axi_wdata        ),//output [MEM_DQ_WIDTH*8-1:0]  axi_wdata      ,
    .axi_wstrb        (  axi_wstrb        ),//output [MEM_DQ_WIDTH -1 :0]  axi_wstrb      ,
    .axi_wlast        (  axi_wlast        ),//input                        axi_wlast      ,
    .axi_wvalid       (  axi_wvalid       ),//output                       axi_wvalid     ,
    .axi_wready       (  axi_wready       ),//input                        axi_wready     ,.
    .axi_bresp        (  2'd0             ),//input  [1 : 0]               axi_bresp      , // Write response. This signal indicates the status of the write transaction.
    .axi_bvalid       (  1'b0             ),//input                        axi_bvalid     , // Write response valid. This signal indicates that the channel is signaling a valid write response.
    .axi_bready       (                   ),//output                       axi_bready     ,
                                          
    // read channel                          
    .axi_araddr       (  axi_araddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_araddr     ,    
    .axi_arlen        (  axi_arlen        ),//output [3:0]                 axi_arlen      , // input, <= 4'd15, which rx burst count <= 16
    .axi_arsize       (  axi_arsize       ),//output [2:0]                 axi_arsize     , // input, Fixed at 3'b101, which rx 32 bytes per burst (that is 256 bits)
    .axi_arburst      (  axi_arburst      ),//output [1:0]                 axi_arburst    ,
    .axi_arvalid      (  axi_arvalid      ),//output                       axi_arvalid    , 
    .axi_arready      (  axi_arready      ),//input                        axi_arready    , //only support 2'b01: INCR
                                          
    .axi_rready       (  axi_rready       ),//output                       axi_rready     ,
    .axi_rdata        (  axi_rdata        ),//input  [MEM_DQ_WIDTH*8-1:0]  axi_rdata      ,
    .axi_rvalid       (  axi_rvalid       ),//input                        axi_rvalid     ,
    .axi_rlast        (  axi_rlast        ),//input                        axi_rlast      ,
    .axi_rresp        (  2'd0             ) //input  [1:0]                 axi_rresp      
);

//////////////////////////////////////////////////////////////////////////////////


//*********************************** DDR3 as AXI Slave *******************************

assign ddr_rstn = rst_board ;

wire axi_awuser_ap = 1'b1;
wire [3:0] axi_awuser_id = 4'b0;

wire axi_aruser_ap = 1'b1;
wire [3:0] axi_aruser_id = 4'b0;

DDR3_50H  #
  (
   //***************************************************************************
   // The following parameters are Memory Feature
   //***************************************************************************
   .MEM_ROW_WIDTH          (MEM_ROW_ADDR_WIDTH),     
   .MEM_COLUMN_WIDTH       (MEM_COL_ADDR_WIDTH),     
   .MEM_BANK_WIDTH         (MEM_BADDR_WIDTH   ),     
   .MEM_DQ_WIDTH           (MEM_DQ_WIDTH      ),     
   .MEM_DM_WIDTH           (MEM_DM_WIDTH      ),     
   .MEM_DQS_WIDTH          (MEM_DQS_WIDTH     ),     
   .CTRL_ADDR_WIDTH        (CTRL_ADDR_WIDTH   )     
  )

  u_DDR3_50H (
    
   .ref_clk                (ref_clk                ),
   .resetn                 (ddr_rstn               ),
   .ddr_init_done          (ddr_init_done          ),
   .ddrphy_clkin           (ddr_clk                ),
   .pll_lock               (pll_lock               ), 

   .axi_awaddr             (axi_awaddr             ),
   .axi_awuser_ap          (axi_awuser_ap          ),
   .axi_awuser_id          (axi_awuser_id          ),
   .axi_awlen              (axi_awlen              ),
   .axi_awready            (axi_awready            ),
   .axi_awvalid            (axi_awvalid            ),
   .axi_wdata              (axi_wdata              ),
   .axi_wstrb              (axi_wstrb              ),
   .axi_wready             (axi_wready             ),
   .axi_wusero_id          (axi_wusero_id          ), // output from DDR / slave, identify upper write
   .axi_wusero_last        (axi_wlast              ),

   .axi_araddr             (axi_araddr             ),
   .axi_aruser_ap          (axi_aruser_ap          ),
   .axi_aruser_id          (axi_aruser_id          ),
   .axi_arlen              (axi_arlen              ),
   .axi_arready            (axi_arready            ),
   .axi_arvalid            (axi_arvalid            ),
   .axi_rdata              (axi_rdata              ),
   .axi_rid                (axi_rid                ), // output from DDR / slave, identify upper receiver
   .axi_rlast              (axi_rlast              ),
   .axi_rvalid             (axi_rvalid             ),

  .apb_clk                   (1'b0               ),// input
  .apb_rst_n                 (1'b1               ),// input
  .apb_sel                   (1'b0               ),// input
  .apb_enable                (1'b0               ),// input
  .apb_addr                  (8'b0               ),// input [7:0]
  .apb_write                 (1'b0               ),// input
  .apb_ready                 (                   ),// output
  .apb_wdata                 (16'b0              ),// input [15:0]
  .apb_rdata                 (                   ),// output [15:0]
  .apb_int                   (                   ),// output

   .mem_rst_n              (mem_rst_n              ),
   .mem_ck                 (mem_ck                 ),
   .mem_ck_n               (mem_ck_n               ),
   .mem_cke                (mem_cke                ),
   .mem_cs_n               (mem_cs_n               ),
   .mem_ras_n              (mem_ras_n              ),
   .mem_cas_n              (mem_cas_n              ),
   .mem_we_n               (mem_we_n               ),
   .mem_odt                (mem_odt                ),
   .mem_a                  (mem_a                  ),
   .mem_ba                 (mem_ba                 ),
   .mem_dqs                (mem_dqs                ),
   .mem_dqs_n              (mem_dqs_n              ),
   .mem_dq                 (mem_dq                 ),
   .mem_dm                 (mem_dm                 ),

   //debug
  .debug_data                (                   ),// output [135:0]
  .debug_slice_state         (                   ),// output [51:0]
  .debug_calib_ctrl          (                   ),// output [21:0]
  .ck_dly_set_bin            (                   ),// output [7:0]
  .force_ck_dly_en           (1'b0               ),// input
  .force_ck_dly_set_bin      (8'h05              ),// input [7:0]
  .dll_step                  (                   ),// output [7:0]
  .dll_lock                  (                   ),// output
  .init_read_clk_ctrl        (2'b0               ),// input [1:0]
  .init_slip_step            (4'b0               ),// input [3:0]
  .force_read_clk_ctrl       (1'b0               ),// input
  .ddrphy_gate_update_en     (1'b0               ),// input
  .update_com_val_err_flag   (                   ),// output [3:0]
  .rd_fake_stop              (1'b0               ) // input
);

//////////////////////////////////////////////////////////////////////////////////


endmodule
