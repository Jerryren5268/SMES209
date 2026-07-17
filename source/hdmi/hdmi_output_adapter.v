// RGB565 to 8-bit RGB888 output adapter with debug pattern fallback
// Converts 5-6-5 bits per channel to 8-8-8 by bit replication

`timescale 1ns / 1ps

`define UD #1

module hdmi_output_adapter # (
    parameter                            X_WIDTH     = 4'd12,
    parameter                            Y_WIDTH     = 4'd12,
    parameter                            COLOR_DEPTH = 8 // number of bits per channel
)(
    input                                rstn,
    // image input
    input                                pix_clk,
    input [15:0]                         pix_data,  // 16 bit, rgb565
    input                                pix_valid,
    // input video timing
    input                                vs_in,
    input                                hs_in,
    input                                de_in,
    input [X_WIDTH-1:0]                  x_act,
    input [Y_WIDTH-1:0]                  y_act,
    // image output, w/ sync signals
    output reg                           vs_out,
    output reg                           hs_out,
    output reg                           de_out,
    output reg [COLOR_DEPTH-1:0]         r_out,
    output reg [COLOR_DEPTH-1:0]         g_out,
    output reg [COLOR_DEPTH-1:0]         b_out
);

    always @(posedge pix_clk)
    begin
        vs_out <= `UD vs_in;
        hs_out <= `UD hs_in;
        de_out <= `UD de_in;
    end

    always @(posedge pix_clk)
    begin
        if (de_in & pix_valid)
        begin
            // rgb565 -> rgb888: replicate upper bits to fill lower bits
            r_out <= {pix_data[15:11],pix_data[15:13]}; // 5 bits + 3 bits
            g_out <= {pix_data[10:5], pix_data[10:9] }; // 6 bits + 2 bits
            b_out <= {pix_data[4:0],  pix_data[2:0]  }; // 5 bits + 3 bits
        end
        else if (de_in & (~pix_valid))
        begin
            // debug pattern based on pixel position when play stopped
            r_out <= x_act[11:4];
            g_out <= x_act[7 :0];
            b_out <= {y_act[11:8], 4'b0};
        end
        else
        begin
            r_out <= 8'h00;
            g_out <= 8'h00;
            b_out <= 8'h00;
        end
    end

endmodule
