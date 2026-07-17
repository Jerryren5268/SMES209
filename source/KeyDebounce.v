// =============================================================================
// KeyDebounce — hardware key debounce with 20ms stable sampling window
//
// Filters out mechanical switch bounce from physical push-buttons.
// When a key change is detected, the module resets its internal counter
// and waits 20ms. Only after the input remains stable (no further changes)
// for the full 20ms period does the output update to match the input.
// This prevents multiple spurious edges from reaching downstream logic.
//
// Parameters:
//   CLK_FREQ — system clock frequency in Hz (default: 50,000,000)
//   KEY_CNT  — number of key channels to debounce (default: 8)
//
// Interface:
//   clk          — system clock
//   keys         — raw key input pins (KEY_CNT bits wide)
//   keys_stable  — debounced key status (KEY_CNT bits wide), 0 = pressed
//
// Algorithm: two-phase detection
//   Phase 1 (key_change_now): detect any change in raw input from previous
//     latched value. If detected, latch the new raw input into key_prev
//     and reset the 20ms stability counter.
//   Phase 2 (key_change_stably): counter reaches KEY_CLK_MAX (20ms) AND
//     the latched key_prev differs from current stable output → update
//     keys_stable to key_prev, confirming the change.
// =============================================================================

module KeyDebounce #(
    parameter CLK_FREQ = 50_000_000,  // clock frequency in Hz
    parameter KEY_CNT  = 8            // number of key channels
)
(
    input                   clk,           // system clock
    input  [KEY_CNT-1:0]    keys,          // raw key input pins, 0 = pressed
    output [KEY_CNT-1:0]    keys_stable    // debounced output, 0 = pressed
);

    // 20ms counter threshold: CLK_FREQ (Hz) * 0.020s = CLK_FREQ / 50
    localparam KEY_CLK_MAX = CLK_FREQ * 20 / 1000 - 1;

    reg [31:0]        key_clk_cnt = 32'b0;     // 20ms stability counter
    reg [KEY_CNT-1:0] key_status  = 16'hffff;  // current stable output (initial: all released)
    reg [KEY_CNT-1:0] key_prev    = 16'hffff;  // latched raw input at last change

    assign keys_stable = key_status;

    // key_change_now: true when raw input differs from latched value
    wire [KEY_CNT-1:0] key_change_now;
    assign key_change_now = keys ^ key_prev;

    // key_change_stably: true when counter reached 20ms AND latched value
    // differs from current stable status — the change is confirmed
    wire key_change_stably;
    assign key_change_stably = (key_clk_cnt == KEY_CLK_MAX) && (keys_stable ^ key_prev) > 0;

    // ------------------------------------------------------------------
    // Stability counter and key_prev latch
    // On any input change: latch current raw value, restart 20ms counter
    // On stability: advance counter toward KEY_CLK_MAX
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (key_change_now > 0) begin
            // Raw input changed — latch and restart 20ms window
            key_prev    <= keys;
            key_clk_cnt <= 32'b0;
        end else begin
            // No change — hold latched value, count toward 20ms
            key_prev    <= key_prev;
            if (key_clk_cnt == KEY_CLK_MAX) begin
                key_clk_cnt <= 32'b0;  // wrap at max
            end else begin
                key_clk_cnt <= key_clk_cnt + 32'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Stable output update
    // Only commits to keys_stable after 20ms of stable input
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (key_change_stably > 0) begin
            key_status <= key_prev;     // confirmed: update stable output
        end else begin
            key_status <= keys_stable;  // hold current value
        end
    end

endmodule
