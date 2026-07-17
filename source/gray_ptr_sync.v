// =============================================================================
// gray_ptr_sync — Gray-coded multi-bit pointer CDC synchronizer
//
// Safely transfers a multi-bit binary pointer between two asynchronous
// clock domains. Uses Gray encoding to ensure only one bit changes per
// pointer increment, so the destination domain always samples either the
// old or new value, never an intermediate glitch.
//
// Parameters:
//   WIDTH — width of the pointer in bits (default: 3, supports 0..7)
//
// Interface:
//   clk_a, rst_n_a, ptr_bin_a — source domain: clock, async reset (active low),
//                                binary pointer input
//   clk_b, rst_n_b, ptr_bin_b — destination domain: clock, async reset (active low),
//                                synchronized binary pointer output
//
// Three-stage pipeline:
//   Stage 1 (clk_a): Binary → Gray conversion
//     Gray = Binary ^ (Binary >> 1)
//   Stage 2 (clk_b): Gray code 2-stage flip-flop synchronizer
//     Eliminates metastability by double-registering
//   Stage 3 (clk_b): Gray → Binary conversion
//     MSB-to-LSB: Bin[MSB] = Gray[MSB], Bin[i] = Bin[i+1] ^ Gray[i]
//
// Latency: output lags input by ~2 clk_b cycles (plus CDC delay)
// =============================================================================

module gray_ptr_sync #(
    parameter WIDTH = 3  // pointer width in bits
)(
    input  wire                  clk_a,
    input  wire                  rst_n_a,
    input  wire [WIDTH-1:0]     ptr_bin_a,  // binary pointer in source clock domain

    input  wire                  clk_b,
    input  wire                  rst_n_b,
    output reg  [WIDTH-1:0]     ptr_bin_b   // synchronized binary pointer in dest clock domain
);

    // ------------------------------------------------------------------
    // Stage 1 (clk_a domain): Binary to Gray code conversion
    // Gray = Bin ^ (Bin >> 1) — adjacent values differ by exactly 1 bit
    // ------------------------------------------------------------------
    reg [WIDTH-1:0] ptr_gray_a;
    always @(posedge clk_a or negedge rst_n_a) begin
        if (!rst_n_a) begin
            ptr_gray_a <= {WIDTH{1'b0}};
        end else begin
            // Gray encode: each bit = current binary bit XOR next higher bit
            ptr_gray_a <= ptr_bin_a ^ (ptr_bin_a >> 1);
        end
    end

    // ------------------------------------------------------------------
    // Stage 2 (clk_b domain): 2-stage flip-flop synchronizer
    // Metastability is resolved after the second register capture
    // ------------------------------------------------------------------
    reg [WIDTH-1:0] ptr_gray_b_d1;
    reg [WIDTH-1:0] ptr_gray_b_d2;
    always @(posedge clk_b or negedge rst_n_b) begin
        if (!rst_n_b) begin
            ptr_gray_b_d1 <= {WIDTH{1'b0}};
            ptr_gray_b_d2 <= {WIDTH{1'b0}};
        end else begin
            ptr_gray_b_d1 <= ptr_gray_a;      // first register: sample from source domain
            ptr_gray_b_d2 <= ptr_gray_b_d1;   // second register: stable synchronized Gray code
        end
    end

    // ------------------------------------------------------------------
    // Stage 3 (clk_b domain): Gray to Binary conversion
    // Algorithm: Bin[MSB] = Gray[MSB]; Bin[i] = Bin[i+1] XOR Gray[i]
    // This is a combinational loop-free chain from MSB down to LSB
    // ------------------------------------------------------------------
    reg [WIDTH-1:0] next_ptr_bin_b;
    integer i;

    always @(*) begin
        next_ptr_bin_b[WIDTH-1] = ptr_gray_b_d2[WIDTH-1];  // MSB: same as Gray
        for (i = WIDTH-2; i >= 0; i = i - 1) begin
            next_ptr_bin_b[i] = next_ptr_bin_b[i+1] ^ ptr_gray_b_d2[i];
        end
    end

    // Register the final binary pointer output
    always @(posedge clk_b or negedge rst_n_b) begin
        if (!rst_n_b) begin
            ptr_bin_b <= {WIDTH{1'b0}};
        end else begin
            ptr_bin_b <= next_ptr_bin_b;
        end
    end

endmodule
