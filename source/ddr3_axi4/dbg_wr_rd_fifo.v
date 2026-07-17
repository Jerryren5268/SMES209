// Debug write/read FIFO — UART OCD debug access to DDR3 through AXI4

module dbg_wr_rd_fifo # (
    parameter                    CTRL_ADDR_WIDTH      = 28    ,
    parameter                    DDR_DATA_WIDTH       = 32 * 8,
    parameter                    LEN_WIDTH            = 32    
) (
// common signals
    input                                  ref_clk        , // top layer, write / read clock
    input                                  ddr_clk        , // ddr inside, write / read clock
    input                                  rstn           , // reset 
// I/O between top layer
    input                                  in_req          ,
    input                                  in_valid        ,
    (* MAX_FANOUT = 64 *) input [7:0]     in_data         ,
    output                                 in_ready        ,
    input                                  out_req         ,
    input     [9:0]                        out_size        ,
    input                                  out_ready       ,
    output    reg [7:0]                    out_data        ,
    output                                 out_valid       ,
// interaction between ddr/axi
    // write channel
    output    reg                          wr_cmd_req =1'b0, // request DDR to write, keep high when in_req done (high -> low)
    output    reg [LEN_WIDTH-1 : 0]        wr_cmd_byte_cnt , // when reading from ddr3, next 32 bytes (256 bits) each clock when rd_cmd_rdata_valid
    input                                  wr_cmd_ready    , // when ddr write done, output from AXI master, keep high when write done
    input                                  wr_cmd_wdone    , // when ddr write done, output from AXI master, have 1 clock high when write done
    output    [DDR_DATA_WIDTH-1:0]         wr_cmd_wdata    ,
    input                                  wr_cmd_wdata_req, // keep high when write start (1st to last pkt), output from AXI master
    input                                  wr_cmd_wdata_req_end, // negedge
    // read channel 
    output    reg                          rd_cmd_req =1'b0, // request to AXI master to read data from DDR3, keep high until rd_cmd_done happen
    output    reg [LEN_WIDTH-1 : 0]        rd_cmd_byte_cnt , // byte size = cmd_len (brust count) * (2^5)
    input                                  rd_cmd_done     , // output from AXI master, have 1 clock high when read done
    input     [DDR_DATA_WIDTH-1:0]         rd_cmd_rdata    ,
    input                                  rd_cmd_rdata_valid // source from AXI slave (DDR3), keep high when reading
);


/****************************** internal signals ******************************/

reg in_req_d1 = 1'b0;
always @(posedge ref_clk) begin
  in_req_d1 <= in_req;
end

wire in_req_start = (~in_req_d1) & in_req; // posedge
wire in_req_end   = (~in_req) & in_req_d1; // negedge

// ------------------------------------------------------------------
// out_valid CDC: level signal from ddr_clk, reconstructed in ref_clk
// ------------------------------------------------------------------
reg  out_valid_ddr;          // internal: out_valid in ddr_clk domain
reg  out_valid_ddr_d1;       // edge detection
wire out_valid_posedge_ddr;
wire out_valid_negedge_ddr;
wire out_valid_posedge_ref;
wire out_valid_negedge_ref;
reg  out_valid_ref;          // reconstructed out_valid in ref_clk domain

assign out_valid = out_valid_ref;

always @(posedge ddr_clk) begin
  if (~rstn) begin
    out_valid_ddr_d1 <= 1'b0;
  end else begin
    out_valid_ddr_d1 <= out_valid_ddr;
  end
end

assign out_valid_posedge_ddr = (~out_valid_ddr_d1) & out_valid_ddr;
assign out_valid_negedge_ddr = out_valid_ddr_d1 & (~out_valid_ddr);

// ------------------------------------------------------------------
// in_ready CDC: level signal from ddr_clk, reconstructed in ref_clk
// ------------------------------------------------------------------
reg  in_ready_ddr;           // internal: in_ready in ddr_clk domain
reg  in_ready_ddr_d1;        // edge detection
wire in_ready_posedge_ddr;
wire in_ready_negedge_ddr;
wire in_ready_posedge_ref;
wire in_ready_negedge_ref;
reg  in_ready_ref;           // reconstructed in_ready in ref_clk domain

assign in_ready = in_ready_ref;

always @(posedge ddr_clk) begin
  if (~rstn) begin
    in_ready_ddr_d1 <= 1'b0;
  end else begin
    in_ready_ddr_d1 <= in_ready_ddr;
  end
end

assign in_ready_posedge_ddr = (~in_ready_ddr_d1) & in_ready_ddr;
assign in_ready_negedge_ddr = in_ready_ddr_d1 & (~in_ready_ddr);

// ------------------------------------------------------------------
// CDC Sync: in_req edge detection from ref_clk to ddr_clk
// Replace 1-tap delay with pulse_sync for reliable edge detection
// ------------------------------------------------------------------
wire in_req_start_ddr;
wire in_req_end_ddr;

pulse_sync u_ps_in_req_start (
    .clk_a   (ref_clk),
    .rst_n_a (rstn),
    .pulse_a (in_req_start),
    .clk_b   (ddr_clk),
    .rst_n_b (rstn),
    .pulse_b (in_req_start_ddr)
);

pulse_sync u_ps_in_req_end (
    .clk_a   (ref_clk),
    .rst_n_a (rstn),
    .pulse_a (in_req_end),
    .clk_b   (ddr_clk),
    .rst_n_b (rstn),
    .pulse_b (in_req_end_ddr)
);


reg out_req_d1 = 1'b0;
always @(posedge ref_clk) begin
  out_req_d1 <= out_req;
end

wire out_req_start   = (~out_req_d1) & out_req; // posedge
// wire out_req_end = (~out_req) & out_req_d1;  // negedge

// ------------------------------------------------------------------
// CDC Sync: out_req_start from ref_clk to ddr_clk
// ------------------------------------------------------------------
wire out_req_start_ddr;

pulse_sync u_ps_out_req_start (
    .clk_a   (ref_clk),
    .rst_n_a (rstn),
    .pulse_a (out_req_start),
    .clk_b   (ddr_clk),
    .rst_n_b (rstn),
    .pulse_b (out_req_start_ddr)
);

pulse_sync u_ps_out_valid_posedge (
    .clk_a   (ddr_clk),
    .rst_n_a (rstn),
    .pulse_a (out_valid_posedge_ddr),
    .clk_b   (ref_clk),
    .rst_n_b (rstn),
    .pulse_b (out_valid_posedge_ref)
);

pulse_sync u_ps_out_valid_negedge (
    .clk_a   (ddr_clk),
    .rst_n_a (rstn),
    .pulse_a (out_valid_negedge_ddr),
    .clk_b   (ref_clk),
    .rst_n_b (rstn),
    .pulse_b (out_valid_negedge_ref)
);

pulse_sync u_ps_in_ready_posedge (
    .clk_a   (ddr_clk),
    .rst_n_a (rstn),
    .pulse_a (in_ready_posedge_ddr),
    .clk_b   (ref_clk),
    .rst_n_b (rstn),
    .pulse_b (in_ready_posedge_ref)
);

pulse_sync u_ps_in_ready_negedge (
    .clk_a   (ddr_clk),
    .rst_n_a (rstn),
    .pulse_a (in_ready_negedge_ddr),
    .clk_b   (ref_clk),
    .rst_n_b (rstn),
    .pulse_b (in_ready_negedge_ref)
);


/******************************FIFO: Write channel ******************************/

reg [8:0] wr_cmd_pos_r = 9'b0;

(* ram_style = "block" *) reg [7:0] wr_buf [511:0];

// Pipeline register: latch in_data locally to place driver near wr_buf loads
reg [7:0] in_data_r;
reg       in_valid_r;
reg       in_req_start_r;

always @(posedge ref_clk) begin
    if (~rstn) begin
        in_valid_r     <= 1'b0;
        in_req_start_r <= 1'b0;
    end else begin
        in_valid_r     <= in_req & in_valid;
        in_req_start_r <= in_req_start;
    end
    in_data_r <= in_data;
end


always @(posedge ref_clk) begin
  if (~rstn || in_req_start_r) begin
      wr_cmd_byte_cnt <= 10'b1;
      wr_buf[0] <= in_data_r;
  end 
  else if (in_valid_r) begin
      wr_buf[wr_cmd_byte_cnt[8:0]] <= in_data_r;
      wr_cmd_byte_cnt <= wr_cmd_byte_cnt + 32'b1;
  end 
end 

always @(posedge ddr_clk) begin
  // reset when reset from top, or request finish
  if (~rstn) begin
    wr_cmd_pos_r <= 9'b0;
    wr_cmd_req <= 1'b0;
    in_ready_ddr <= 1'b0;
  end
  else if (in_req_start_ddr) begin
    wr_cmd_pos_r <= 9'b0;
    wr_cmd_req <= 1'b0;
    in_ready_ddr <= 1'b0;
  end
  else if (in_req_end_ddr) begin
    wr_cmd_pos_r <= 9'b0;
    wr_cmd_req <= 1'b1;
    in_ready_ddr <= 1'b0;
  end
  else if (wr_cmd_req && (wr_cmd_wdata_req_end | wr_cmd_wdone)) begin
  // else if (wr_cmd_req && wr_cmd_wdone) begin
    wr_cmd_req <= 1'b0;
    in_ready_ddr <= 1'b1;
  end
  else if (wr_cmd_req && wr_cmd_wdata_req) begin
    wr_cmd_pos_r <= wr_cmd_pos_r + 9'd32;
  end
end 

reg [DDR_DATA_WIDTH-1:0]   wr_cmd_wdata_r;
assign  wr_cmd_wdata = wr_cmd_wdata_r;

generate
  genvar gen_wr_i;
  for (gen_wr_i = 0; gen_wr_i < 32; gen_wr_i = gen_wr_i + 1) begin
    always @(*) begin
      wr_cmd_wdata_r[8*(gen_wr_i+1)-1 : 8*gen_wr_i] = wr_buf[wr_cmd_pos_r + gen_wr_i];
    end 
  end 
endgenerate

//////////////////////////////////////////////////////////////////////////////////


/******************************FIFO: Read channel ******************************/

reg [8:0] rd_cmd_len_r = 9'b0; // when reading from ddr3, +32
reg [8:0] rd_cmd_pos_r = 9'b0; // when reading from upper module, +1

// ------------------------------------------------------------------
// CDC Sync: rd_cmd_pos_r from ref_clk to ddr_clk
// Fix: mixed clock domain comparison between rd_cmd_pos_r (ref_clk)
//      and rd_cmd_len_r (ddr_clk)
// ------------------------------------------------------------------
wire [8:0] rd_cmd_pos_r_sync;

gray_ptr_sync #( .WIDTH(9) ) u_sync_rd_pos (
    .clk_a     (ref_clk),
    .rst_n_a   (rstn),
    .ptr_bin_a (rd_cmd_pos_r),
    .clk_b     (ddr_clk),
    .rst_n_b   (rstn),
    .ptr_bin_b (rd_cmd_pos_r_sync)
);

reg [255:0] rd_buf [15:0];

wire [255:0] out_data_256b;
assign out_data_256b = rd_buf[rd_cmd_pos_r[8:5]];

always @(*) begin
  case (rd_cmd_pos_r[4:0])
    5'd0: out_data = out_data_256b[8'h07 : 8'h00];
    5'd1: out_data = out_data_256b[8'h0f : 8'h08];
    5'd2: out_data = out_data_256b[8'h17 : 8'h10];
    5'd3: out_data = out_data_256b[8'h1f : 8'h18];
    5'd4: out_data = out_data_256b[8'h27 : 8'h20];
    5'd5: out_data = out_data_256b[8'h2f : 8'h28];
    5'd6: out_data = out_data_256b[8'h37 : 8'h30];
    5'd7: out_data = out_data_256b[8'h3f : 8'h38];
    5'd8: out_data = out_data_256b[8'h47 : 8'h40];
    5'd9: out_data = out_data_256b[8'h4f : 8'h48];
    5'd10: out_data = out_data_256b[8'h57 : 8'h50];
    5'd11: out_data = out_data_256b[8'h5f : 8'h58];
    5'd12: out_data = out_data_256b[8'h67 : 8'h60];
    5'd13: out_data = out_data_256b[8'h6f : 8'h68];
    5'd14: out_data = out_data_256b[8'h77 : 8'h70];
    5'd15: out_data = out_data_256b[8'h7f : 8'h78];
    5'd16: out_data = out_data_256b[8'h87 : 8'h80];
    5'd17: out_data = out_data_256b[8'h8f : 8'h88];
    5'd18: out_data = out_data_256b[8'h97 : 8'h90];
    5'd19: out_data = out_data_256b[8'h9f : 8'h98];
    5'd20: out_data = out_data_256b[8'ha7 : 8'ha0];
    5'd21: out_data = out_data_256b[8'haf : 8'ha8];
    5'd22: out_data = out_data_256b[8'hb7 : 8'hb0];
    5'd23: out_data = out_data_256b[8'hbf : 8'hb8];
    5'd24: out_data = out_data_256b[8'hc7 : 8'hc0];
    5'd25: out_data = out_data_256b[8'hcf : 8'hc8];
    5'd26: out_data = out_data_256b[8'hd7 : 8'hd0];
    5'd27: out_data = out_data_256b[8'hdf : 8'hd8];
    5'd28: out_data = out_data_256b[8'he7 : 8'he0];
    5'd29: out_data = out_data_256b[8'hef : 8'he8];
    5'd30: out_data = out_data_256b[8'hf7 : 8'hf0];
    5'd31: out_data = out_data_256b[8'hff : 8'hf8];
  endcase
end 

always @(posedge ref_clk) begin
  if (~rstn) begin
    rd_cmd_pos_r <= 9'b0;
  end 
  else if (out_req_start) begin
    rd_cmd_pos_r <= 9'b0;
  end 
  else if (out_valid_ref && out_ready) begin
    rd_cmd_pos_r <= rd_cmd_pos_r + 9'd1;
  end
end

// ------------------------------------------------------------------
// Reconstruct out_valid in ref_clk from synced posedge/negedge pulses
// ------------------------------------------------------------------
always @(posedge ref_clk) begin
  if (~rstn || out_req_start) begin
    out_valid_ref <= 1'b0;
  end
  else if (out_valid_posedge_ref) begin
    out_valid_ref <= 1'b1;
  end
  else if (out_valid_negedge_ref) begin
    out_valid_ref <= 1'b0;
  end
end

// ------------------------------------------------------------------
// Reconstruct in_ready in ref_clk from synced posedge/negedge pulses
// ------------------------------------------------------------------
always @(posedge ref_clk) begin
  if (~rstn || in_req_start) begin
    in_ready_ref <= 1'b0;
  end
  else if (in_ready_posedge_ref) begin
    in_ready_ref <= 1'b1;
  end
  else if (in_ready_negedge_ref) begin
    in_ready_ref <= 1'b0;
  end
end

always @(posedge ddr_clk) begin
  if (~rstn) begin
    rd_cmd_len_r <= 9'b0;
    rd_cmd_req <= 1'b0;
    out_valid_ddr <= 1'b0;
  end
  else if (out_req_start_ddr) begin
    rd_cmd_len_r <= 9'b0;
    rd_cmd_req <= 1'b1;
    out_valid_ddr <= 1'b0;
  end
  else if (rd_cmd_req && rd_cmd_done) begin
    rd_cmd_req <= 1'b0;
    out_valid_ddr <= 1'b1;
  end
  else if (rd_cmd_req && rd_cmd_rdata_valid) begin
    rd_cmd_len_r <= rd_cmd_len_r + 9'd32;
  end
  else if (out_valid_ddr && ((rd_cmd_pos_r_sync + 9'd1) == rd_cmd_len_r[8:0])) begin
    // Use synchronized rd_cmd_pos_r to avoid mixed clock domain comparison
    out_valid_ddr <= 1'b0;
  end
end 

always @(posedge ddr_clk) begin
  if (rd_cmd_rdata_valid) begin
    rd_buf[rd_cmd_len_r[8:5]] = rd_cmd_rdata;
  end 
end 

always @(*) begin
  if (out_size < 10'd512) begin
    rd_cmd_byte_cnt = 32'd512;
  end 
  else begin
    rd_cmd_byte_cnt = {22'b0, out_size};
  end 
end 

//////////////////////////////////////////////////////////////////////////////////


endmodule
