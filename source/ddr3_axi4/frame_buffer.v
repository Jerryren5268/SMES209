// Simple dual-port RAM frame buffer for pixel line storage

`timescale 1ns / 1ps
`define UD #1

module frame_buffer #(
    parameter                     MEM_ROW_WIDTH        = 15    ,
    parameter                     MEM_COLUMN_WIDTH     = 10    ,
    parameter                     MEM_BANK_WIDTH       = 3     ,
    parameter                     MEM_ADDR_WIDTH       = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH,
    parameter                     MEM_DQ_WIDTH         = 32    ,
    parameter                     IO_DATA_WIDTH        = 16    ,
    parameter                     IO_FRAME_WIDTH       = 9     ,
    parameter                     IO_ADDR_WIDTH        = MEM_ADDR_WIDTH - IO_FRAME_WIDTH + 1 // MEM_DQ_WIDTH / IO_DATA_WIDTH
)(
    input                         ddr_clk,
    input                         ddr_rstn,

    input                         wr_clk,
    input                         wr_fsync,
    input                         wr_en,
    input  [IO_ADDR_WIDTH-1 : 0]  wr_faddr,  // address of data frame
    input  [IO_DATA_WIDTH-1 : 0]  wr_data,
    output [IO_FRAME_WIDTH-1: 0]  wr_fidx,   // index of current data in current frame 
    
    input                         rd_clk,
    input                         rd_fsync,
    input                         rd_en,
    input  [IO_ADDR_WIDTH-1 : 0]  rd_faddr,  // address of data frame
    output [IO_DATA_WIDTH-1 : 0]  rd_data,
    output [IO_FRAME_WIDTH-1: 0]  rd_fidx,   // index of current data in current frame 
    
    output [MEM_ADDR_WIDTH-1:0]   axi_awaddr     ,
    output [3:0]                  axi_awid       ,
    output [3:0]                  axi_awlen      ,
    output [2:0]                  axi_awsize     ,
    output [1:0]                  axi_awburst    ,
    input                         axi_awready    ,
    output                        axi_awvalid    ,
                                                  
    output [MEM_DQ_WIDTH*8-1:0]   axi_wdata      ,
    output [MEM_DQ_WIDTH -1 :0]   axi_wstrb      ,
    input                         axi_wlast      ,
    output                        axi_wvalid     ,
    input                         axi_wready     ,
    input  [3 : 0]                axi_bid        ,                                      
                                                  
    output [MEM_ADDR_WIDTH-1:0]   axi_araddr     ,
    output [3:0]                  axi_arid       ,
    output [3:0]                  axi_arlen      ,
    output [2:0]                  axi_arsize     ,
    output [1:0]                  axi_arburst    ,
    output                        axi_arvalid    ,
    input                         axi_arready    ,
                                                  
    output                        axi_rready     ,
    input  [MEM_DQ_WIDTH*8-1:0]   axi_rdata      ,
    input                         axi_rvalid     ,
    input                         axi_rlast      ,
    input  [3:0]                  axi_rid            
);

    parameter LEN_WIDTH       = 32;
    
    wire                        ddr_wreq;     
    wire [MEM_ADDR_WIDTH-1:0]   ddr_waddr;    
    wire [LEN_WIDTH-1 : 0]      ddr_wr_len;   
    wire                        ddr_wrdy;     
    wire                        ddr_wdone;    
    wire [8*MEM_DQ_WIDTH-1:0]   ddr_wdata;    
    wire                        ddr_wdata_req;
    
    wire                        rd_cmd_en   ;
    wire [MEM_ADDR_WIDTH-1:0]   rd_cmd_addr ;
    wire [LEN_WIDTH-1 : 0]      rd_cmd_len  ;
    wire                        rd_cmd_ready;
    wire                        rd_cmd_done;
                                
    wire                        read_ready  = 1'b1;
    wire [MEM_DQ_WIDTH*8-1:0]   read_rdata  ;
    wire                        read_en     ;
    wire                        ddr_wr_bac;

    /******************************FIFO: Write channel ******************************/
    // TODO 
    //////////////////////////////////////////////////////////////////////////////////


    /******************************FIFO: Read channel ******************************/
    // TODO
    //////////////////////////////////////////////////////////////////////////////////
    

    wr_rd_ctrl_top#(
        .CTRL_ADDR_WIDTH  (  MEM_ADDR_WIDTH  ), //parameter                    CTRL_ADDR_WIDTH      = 28,
        .MEM_DQ_WIDTH     (  MEM_DQ_WIDTH     ) //parameter                    MEM_DQ_WIDTH         = 16
    )wr_rd_ctrl_top (                         
        .clk              (  ddr_clk          ),//input                        clk            ,            
        .rstn             (  ddr_rstn         ),//input                        rstn           ,            
                                              
        .wr_cmd_en        (  ddr_wreq         ),//input                        wr_cmd_en   ,
        .wr_cmd_addr      (  ddr_waddr        ),//input  [CTRL_ADDR_WIDTH-1:0] wr_cmd_addr ,
        .wr_cmd_len       (  ddr_wr_len       ),//input  [31：0]               wr_cmd_len  ,
        .wr_cmd_ready     (  ddr_wrdy         ),//output                       wr_cmd_ready,
        .wr_cmd_done      (  ddr_wdone        ),//output                       wr_cmd_done,
        .wr_bac           (  ddr_wr_bac       ),//output                       wr_bac,                                     
        .wr_ctrl_data     (  ddr_wdata        ),//input  [MEM_DQ_WIDTH*8-1:0]  wr_ctrl_data,
        .wr_data_re       (  ddr_wdata_req    ),//output                       wr_data_re  ,
                                              
        .rd_cmd_en        (  rd_cmd_en        ),//input                        rd_cmd_en   ,
        .rd_cmd_addr      (  rd_cmd_addr      ),//input  [CTRL_ADDR_WIDTH-1:0] rd_cmd_addr ,
        .rd_cmd_len       (  rd_cmd_len       ),//input  [31：0]               rd_cmd_len  ,
        .rd_cmd_ready     (  rd_cmd_ready     ),//output                       rd_cmd_ready, 
        .rd_cmd_done      (  rd_cmd_done      ),//output                       rd_cmd_done,
                                              
        .read_ready       (  read_ready       ),//input                        read_ready  ,    
        .read_rdata       (  read_rdata       ),//output [MEM_DQ_WIDTH*8-1:0]  read_rdata  ,    
        .read_en          (  read_en          ),//output                       read_en     ,

        // write channel                        
        .axi_awaddr       (  axi_awaddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_awaddr     ,  
        .axi_awid         (  axi_awid         ),//output [3:0]                 axi_awid       ,
        .axi_awlen        (  axi_awlen        ),//output [3:0]                 axi_awlen      ,
        .axi_awsize       (  axi_awsize       ),//output [2:0]                 axi_awsize     ,
        .axi_awburst      (  axi_awburst      ),//output [1:0]                 axi_awburst    , //only support 2'b01: INCR
        .axi_awready      (  axi_awready      ),//input                        axi_awready    ,
        .axi_awvalid      (  axi_awvalid      ),//output                       axi_awvalid    ,
                                              
        .axi_wdata        (  axi_wdata        ),//output [MEM_DQ_WIDTH*8-1:0]  axi_wdata      ,
        .axi_wstrb        (  axi_wstrb        ),//output [MEM_DQ_WIDTH -1 :0]  axi_wstrb      ,
        .axi_wlast        (  axi_wlast        ),//output                       axi_wlast      ,
        .axi_wvalid       (  axi_wvalid       ),//output                       axi_wvalid     ,
        .axi_wready       (  axi_wready       ),//input                        axi_wready     ,
        .axi_bid          (  4'd0             ),//input  [3 : 0]               axi_bid        , // Master Interface Write Response.
        .axi_bresp        (  2'd0             ),//input  [1 : 0]               axi_bresp      , // Write response. This signal indicates the status of the write transaction.
        .axi_bvalid       (  1'b0             ),//input                        axi_bvalid     , // Write response valid. This signal indicates that the channel is signaling a valid write response.
        .axi_bready       (                   ),//output                       axi_bready     ,
                                              
        // read channel                          
        .axi_araddr       (  axi_araddr       ),//output [CTRL_ADDR_WIDTH-1:0] axi_araddr     ,    
        .axi_arid         (  axi_arid         ),//output [3:0]                 axi_arid       ,
        .axi_arlen        (  axi_arlen        ),//output [3:0]                 axi_arlen      ,
        .axi_arsize       (  axi_arsize       ),//output [2:0]                 axi_arsize     ,
        .axi_arburst      (  axi_arburst      ),//output [1:0]                 axi_arburst    ,
        .axi_arvalid      (  axi_arvalid      ),//output                       axi_arvalid    , 
        .axi_arready      (  axi_arready      ),//input                        axi_arready    , //only support 2'b01: INCR
                                              
        .axi_rready       (  axi_rready       ),//output                       axi_rready     ,
        .axi_rdata        (  axi_rdata        ),//input  [MEM_DQ_WIDTH*8-1:0]  axi_rdata      ,
        .axi_rvalid       (  axi_rvalid       ),//input                        axi_rvalid     ,
        .axi_rlast        (  axi_rlast        ),//input                        axi_rlast      ,
        .axi_rid          (  axi_rid          ),//input  [3:0]                 axi_rid        ,
        .axi_rresp        (  2'd0             ) //input  [1:0]                 axi_rresp      
    );


endmodule
