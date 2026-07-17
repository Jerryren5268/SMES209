// =============================================================================
// pulse_sync — Single-pulse cross-clock-domain (CDC) synchronizer
//
// Converts a single-cycle pulse from source clock domain (clk_a) to a
// single-cycle pulse in destination clock domain (clk_b) using the
// toggle-level synchronizer method (pulse-to-level, 2-FF sync, edge detect).
//
// Method: Pulse (clk_a) → Toggle level → 2-stage FF sync (clk_b) → Edge detect (clk_b)
//
// Parameters: none (pure behavioral)
//
// Interface:
//   clk_a, rst_n_a, pulse_a  — source domain: clock, async reset (active low), input pulse
//   clk_b, rst_n_b, pulse_b  — destination domain: clock, async reset (active low), output pulse
//
// Timing: output pulse appears in clk_b 2-3 cycles after input pulse in clk_a
// =============================================================================

module pulse_sync (
    input  wire clk_a,
    input  wire rst_n_a,
    input  wire pulse_a,    // single-cycle input pulse in clk_a domain

    input  wire clk_b,
    input  wire rst_n_b,
    output wire pulse_b     // single-cycle output pulse in clk_b domain
);

    // ------------------------------------------------------------------
    // Stage 1 (clk_a domain): pulse-to-level conversion (toggle flip-flop)
    // Each input pulse toggles the level, converting a narrow pulse
    // into a wide level signal safe for cross-domain transfer.
    // ------------------------------------------------------------------
    reg toggle_a;
    always @(posedge clk_a or negedge rst_n_a) begin
        if (!rst_n_a) begin
            toggle_a <= 1'b0;
        end else if (pulse_a) begin
            toggle_a <= ~toggle_a;
        end
    end

    // ------------------------------------------------------------------
    // Stage 2 (clk_b domain): 2-stage flip-flop synchronizer
    // Eliminates metastability by double-registering the level signal
    // from clk_a domain into clk_b domain.
    // ------------------------------------------------------------------
    reg toggle_b_d1;
    reg toggle_b_d2;
    always @(posedge clk_b or negedge rst_n_b) begin
        if (!rst_n_b) begin
            toggle_b_d1 <= 1'b0;
            toggle_b_d2 <= 1'b0;
        end else begin
            toggle_b_d1 <= toggle_a;      // first register: sample the level
            toggle_b_d2 <= toggle_b_d1;   // second register: stable output
        end
    end

    // ------------------------------------------------------------------
    // Stage 3 (clk_b domain): edge detection — toggle to pulse recovery
    // A third register stores the previous synchronized value.
    // XOR of current and previous values produces a 1-cycle pulse
    // on every detected edge (rise or fall of the level signal).
    // ------------------------------------------------------------------
    reg toggle_b_d3;
    always @(posedge clk_b or negedge rst_n_b) begin
        if (!rst_n_b) begin
            toggle_b_d3 <= 1'b0;
        end else begin
            toggle_b_d3 <= toggle_b_d2;
        end
    end

    // Edge detect: any change in the synchronized toggle level
    // indicates an input pulse was received in source domain
    assign pulse_b = toggle_b_d2 ^ toggle_b_d3;

endmodule
