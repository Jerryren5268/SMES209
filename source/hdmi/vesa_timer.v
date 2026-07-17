// =============================================================================
// vesa_timer — VESA-compliant video timing generator for HDMI output
//
// Generates HSYNC, VSYNC, DE (data enable), and active pixel coordinates
// (x_act, y_act) based on VESA monitor timing standard parameters.
// Default configuration: 1280x720@60Hz (74.25 MHz pixel clock).
//
// Timing diagram (horizontal line):
//   HS:   |--h_sync--|----h_bp----|------h_act------|----h_fp----|
//   DE:                            |---active pixels---|
//
// Parameters (VESA 1280x720@60Hz defaults):
//   H_TOTAL=1650, H_FP=110, H_BP=220, H_SYNC=40, H_ACT=1280
//   V_TOTAL=750,  V_FP=5,   V_BP=20,  V_SYNC=5,  V_ACT=720
//
// Interface:
//   clk, rstn     — pixel clock and async reset (active low, = rstn_out from hdmi_wrapper)
//   vs_out, hs_out, de_out — VESA sync and data-enable outputs
//   x_act, y_act  — active pixel coordinates (0-based within visible frame)
//   x_valid, y_valid — high when counter is within active region
// =============================================================================

`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////      
// Dependencies: 
//    VS     ______                                       ______
//HS  __    |      |_______________##____________________|      |
//   |       h_sync  h_bp       h_act                h_fp
//   |__                    _______________________
//DE    |   _______________|                       |_____________
//      |
//      .
//      |
//    __|
//   |  
//   |__ 
//      |
// 
//////////////////////////////////////////////////////////////////////////////////

`define UD #1

module vesa_timer # (
    // default as: 1280 x 720
    parameter               X_BITS = 4'd12,
    parameter               Y_BITS = 4'd12,
    parameter               V_TOTAL = 12'd750,
    parameter               V_FP = 12'd5,
    parameter               V_BP = 12'd20,
    parameter               V_SYNC = 12'd5,
    parameter               V_ACT = 12'd720,
    parameter               H_TOTAL = 12'd1650,
    parameter               H_FP = 12'd110,
    parameter               H_BP = 12'd220,
    parameter               H_SYNC = 12'd40,
    parameter               H_ACT = 12'd1280
)(
    input                   clk,
    input                   rstn,
    output reg              vs_out,
    output reg              hs_out,
    output reg              de_out,
    // output reg              de_ahead, // ahead 2 pix clock
    output reg [X_BITS-1:0] x_act,
    output reg [Y_BITS-1:0] y_act,
    output reg              x_valid, // when scan to active columns
    output reg              y_valid  // when scan to active rows
);
    // parameter               HV_OFFSET = 12'd0 ;
    reg [X_BITS-1:0]        h_count = 'd0;
    reg [Y_BITS-1:0]        v_count = 'd0;
    
    /* horizontal counter */
    always @(posedge clk)
    begin
        if (!rstn)
            h_count <= `UD 0;
        else
        begin
            if (h_count < H_TOTAL - 1)
                h_count <= `UD h_count + 1;
            else
                h_count <= `UD 0;
        end
    end
    
    /* vertical counter */
    always @(posedge clk)
    begin
        if (!rstn)
            v_count <= `UD 0;
        else
        if (h_count == H_TOTAL - 1)
        begin
            if (v_count == V_TOTAL - 1)
                v_count <= `UD 0;
            else
                v_count <= `UD v_count + 1;
        end
    end
    
    always @(posedge clk)
    begin
        if (!rstn)
            hs_out <= `UD 4'b0;
        else 
            hs_out <= `UD ((h_count < H_SYNC));
    end
    
    always @(posedge clk)
    begin
        if (!rstn)
            vs_out <= `UD 4'b0;
        else 
        begin
            if (v_count == 0)
                vs_out <= `UD 1'b1;
            else if (v_count == V_SYNC)
                vs_out <= `UD 1'b0;
            else
                vs_out <= `UD vs_out;
        end
    end
    
    always @(posedge clk)
    begin
        if (!rstn)
            de_out <= `UD 4'b0;
        else
            de_out <= (((v_count >= V_SYNC + V_BP) && (v_count <= V_TOTAL - V_FP - 1)) && 
                      ((h_count >= H_SYNC + H_BP) && (h_count <= H_TOTAL - H_FP - 1)));
    end
    
/*
    always @(posedge clk)
    begin
        if (!rstn)
            de_ahead <= `UD 4'b0;
        else
            de_ahead <= (((v_count >= V_SYNC + V_BP) && (v_count <= V_TOTAL - V_FP - 1)) && 
                      ((h_count >= H_SYNC + H_BP - 2'd2) && (h_count <= H_TOTAL - H_FP - 3)));
    end
*/
    // active pixels counter output
    always @(posedge clk)
    begin
        if (!rstn)
            x_act <= `UD 'd0;
        else 
        begin
        /* X coords - for a backend pattern generator */
            // if(h_count > (H_SYNC + H_BP - 1'b1))
            if (x_valid)
                // when h_count == H_SYNC + H_BP, x_act = 0
                x_act <= `UD (h_count - (H_SYNC + H_BP));
            else
                x_act <= `UD 'd0;
        end
    end
    
    always @(posedge clk)
    begin
        if (!rstn)
            y_act <= `UD 'd0;
        else 
        begin
            /* Y coords - for a backend pattern generator */
            // if(v_count > (V_SYNC + V_BP - 1'b1))
            if (y_valid)
                // when v_count == V_SYNC + V_BP, y_act = 0
                y_act <= `UD (v_count - (V_SYNC + V_BP));
            else
                y_act <= `UD 'd0;
        end
    end

    always @(*) begin
        x_valid = (h_count >= H_SYNC + H_BP) && (h_count <= H_TOTAL - H_FP - 1);
    end

    always @(*) begin
        y_valid = (v_count >= V_SYNC + V_BP) && (v_count <= V_TOTAL - V_FP - 1);
    end 
    
endmodule
