// OV5640 camera adapter — init delay, I2C config, 8-to-16bit conversion, frame size detect

module ov_camera_adapter(
    input                                clk_50M        ,
    input                                clk_25M        ,
    input                                rstn           ,
    // From/TO Top modules I/O
    inout                                cmos_scl            ,//cmos1 i2c 
    inout                                cmos_sda            ,//cmos1 i2c 
    input                                cmos_vsync          ,//cmos1 vsync
    input                                cmos_href           ,//cmos1 hsync refrence,data valid
    input                                cmos_pclk           ,//cmos1 pxiel clock
    input   [7:0]                        cmos_data           ,//cmos1 data
    output                               cmos_reset          ,//cmos1 reset
    output                               cmos_init_done      ,//OV5640寄存器初始化完成
    // pixels outputs
    output                               o_img_pclk          ,
    output                               o_img_vs            ,
    output                               o_img_de            ,
    output  [15:0]                       o_img_rgb565        ,
    output  [15:0]                       o_img_width         ,
    output  [15:0]                       o_img_height        
);

/////////////////////////////// interal signals ///////////////////////////////////
wire                        cmos_en;

// sync by cmos_pclk
reg                         cmos_vsync_d0      ;
reg                         cmos_href_d0       ;
reg [7:0]                   cmos_d_d0          ;

// sync by cmos_pclk_16bit = cmos_pclk / 2
wire                        cmos_pclk_16bit    ;
wire                        cmos_href_16bit    ;
wire [15:0]                 cmos_d_16bit       ;


/////////////////////////////////// CMOS setup ///////////////////////////////////
//OV5640 register configure enable    
ov_camera_init	u_init_on_delay(
    .clk_50M                 (clk_50M        ),//input
    .reset_n                 (rstn           ),//input	
    .camera_rstn             (cmos_reset     ),//output
    .camera_pwnd             (               ),//output
    .initial_en              (cmos_en        ) //output		
);

//CMOS1 Camera 
OV5640_reg_config	u_coms_reg_config(
    .clk_25M                 (clk_25M            ),//input
    .camera_rstn             (cmos_reset         ),//input
    .initial_en              (cmos_en            ),//input		
    .i2c_sclk                (cmos_scl           ),//output
    .i2c_sdat                (cmos_sda           ),//inout
    .reg_conf_done           (cmos_init_done     ),//output config_finished
    .reg_index               (                   ),//output reg [8:0]
    .clock_20k               (                   ) //output reg
);


///////////////////////////////// 8bit to 16bit ///////////////////////////////////

always@(posedge cmos_pclk)
begin
    cmos_d_d0        <= cmos_data    ;
    cmos_href_d0     <= cmos_href    ;
    cmos_vsync_d0    <= cmos_vsync   ;
end

cmos_8_16bit u_cmos_8_16bit(
    .pclk           (cmos_pclk       ),//input
    .rst_n          (cmos_init_done  ),//input
    .pdata_i        (cmos_d_d0       ),//input[7:0]
    .de_i           (cmos_href_d0    ),//input
    .vs_i           (cmos_vsync_d0   ),//input
    
    .pixel_clk      (cmos_pclk_16bit ),//output
    .pdata_o        (cmos_d_16bit    ),//output[15:0]
    .de_o           (cmos_href_16bit ) //output
);

/////////////////////////////// monitor width & heigh ///////////////////////////////////

// latest completed frame 
reg [15:0] o_img_w_l = 16'b0;
reg [15:0] o_img_h_l = 16'b0;

// current on-going frame 
reg [15:0] o_img_w_c = 16'b0;
reg [15:0] o_img_h_c = 16'b0;

reg cmos_vsync_d1;
reg cmos_href_16bit_d1;

always @(posedge cmos_pclk_16bit) begin
    cmos_vsync_d1      <= cmos_vsync_d0;
    cmos_href_16bit_d1 <= cmos_href_16bit;
end 

always @(posedge cmos_pclk_16bit) begin
    // negedge vsync, new frame comming
    if ((~cmos_vsync_d1) & cmos_vsync_d0) begin 
        o_img_h_l <= o_img_h_c;
    end 
    // negedge hsync, current line ending
    if (cmos_href_16bit_d1 & (~cmos_href_16bit)) begin 
        o_img_w_l <= o_img_w_c;
    end 
end 

always @(posedge cmos_pclk_16bit) begin
    // posedge vsync, new frame comming
    if ((~cmos_vsync_d1) & cmos_vsync_d0) begin 
        o_img_h_c <= 16'b0;
    end 
    // posedge hsync, new line comming
    else if ((~cmos_href_16bit_d1) & cmos_href_16bit) begin 
        o_img_w_c <= 16'b1;
        o_img_h_c <= o_img_h_c + 16'b1;
    end 
    else if (cmos_href_16bit) begin
        o_img_w_c <= o_img_w_c + 16'b1;
    end 
end 


///////////////////////////////// final output /////////////////////////////////

assign     o_img_pclk          =    cmos_pclk_16bit    ;
assign     o_img_vs            =    cmos_vsync_d0      ;
assign     o_img_de            =    cmos_href_16bit    ;
assign     o_img_rgb565        =    {cmos_d_16bit[4:0], cmos_d_16bit[10:5], cmos_d_16bit[15:11]}; //{r,g,b}

assign     o_img_width         =    o_img_w_l           ;
assign     o_img_height        =    o_img_h_l           ;

endmodule
