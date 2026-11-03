// =============================================================================
// Module  : ov2640_capture  (rev 3)
//
// BUG FIXES vs rev 2:
//
// BUG 1 - Captures frame without 0x02 trigger (clock-domain crossing):
//   capture_trigger is a 1-cycle pulse on sys_clk (cmd_decoder domain).
//   ov2640_capture runs on cam_pclk (different clock, different frequency).
//   A raw 1-cycle sys_clk pulse routed directly to cam_pclk logic causes:
//     (a) Metastability / spurious arming at startup when rst_n deasserts
//     (b) The pulse can be sampled as 0, 1, or X by cam_pclk
//   FIX: A 2-flop synchroniser converts capture_trigger into cam_pclk domain.
//   Additionally the armed latch is explicitly held in reset (ST_IDLE only
//   leaves on a clean synchronised trigger, never on a reset glitch).
//
// BUG 2 - Gibberish / garbled frame (broken downsampler):
//   The previous fixed-point step was STEP = SRC*1024/DST = 8533.
//   Since STEP > 1024 its integer part [19:10] is ALWAYS non-zero, so
//   (acc_nxt[19:10] != acc[19:10]) is TRUE for EVERY source pixel.
//   Result: ALL 800 source columns are selected, dst_x clamps at 96,
//   and every pixel on the last dst column is overwritten 700+ times.
//   FIX: Use a Bresenham / DDA error accumulator:
//     acc += FRAME_W each source pixel; fire (select) when acc >= SRC_W;
//     then subtract SRC_W and increment dst counter.
//   This selects exactly FRAME_W pixels from SRC_W with even spacing.
//   Same principle applied to rows (FRAME_H from SRC_H).
// =============================================================================

module ov2640_capture #(
    parameter SRC_W   = 800,
    parameter SRC_H   = 600,
    parameter FRAME_W = 96,
    parameter FRAME_H = 96
)(
    input  wire        cam_pclk,
    input  wire        cam_vsync,
    input  wire        cam_href,
    input  wire [7:0]  cam_data,

    // capture_trigger is a 1-clk pulse from cmd_decoder (sys_clk domain).
    // It is synchronised internally to cam_pclk.
    input  wire        capture_trigger,
    input  wire        rst_n,           // PLL locked (async reset)

    output reg         wr_en,
    output reg  [16:0] wr_addr,
    output reg  [15:0] wr_data,
    output reg         frame_done
);

// ---------------------------------------------------------------------------
// Widths for accumulators
//   Bresenham X: acc_x counts up by FRAME_W per src pixel, resets at SRC_W
//                needs ceil(log2(SRC_W))  = 10 bits
//   Bresenham Y: acc_y counts up by FRAME_H per src line,  resets at SRC_H
//                needs ceil(log2(SRC_H))  = 10 bits
// ---------------------------------------------------------------------------
localparam [6:0] DST_W_MAX = FRAME_W;   // 96
localparam [6:0] DST_H_MAX = FRAME_H;   // 96
localparam [10:0] BRES_SW = SRC_W;    // 800
localparam [10:0] BRES_SH = SRC_H;    // 600
localparam [10:0] BRES_FW = FRAME_W;  //  96
localparam [10:0] BRES_FH = FRAME_H;  //  96

// ---------------------------------------------------------------------------
// FSM states
// ---------------------------------------------------------------------------
localparam ST_IDLE    = 2'd0;
localparam ST_ARMED   = 2'd1;
localparam ST_CAPTURE = 2'd2;
localparam ST_DONE    = 2'd3;

reg [1:0] state;

// ===========================================================================
// BUG 1 FIX: 2-flop synchroniser for capture_trigger (sys_clk -> cam_pclk)
// ===========================================================================
reg trig_s1, trig_s2, trig_s3;

// trig_s2 is the synchronised level in cam_pclk domain.
// trig_s3 lets us detect the rising edge (single-cycle pulse).
wire trig_sync_rise = trig_s2 & ~trig_s3;

always @(posedge cam_pclk or negedge rst_n) begin
    if (!rst_n) begin
        trig_s1 <= 1'b0;
        trig_s2 <= 1'b0;
        trig_s3 <= 1'b0;
    end else begin
        trig_s1 <= capture_trigger;
        trig_s2 <= trig_s1;
        trig_s3 <= trig_s2;
    end
end

// ---------------------------------------------------------------------------
// VSYNC / HREF edge detection (all in cam_pclk domain)
// ---------------------------------------------------------------------------
reg vsync_d;
reg href_d;

wire vsync_rise = cam_vsync & ~vsync_d;
wire href_rise  = cam_href  & ~href_d;
wire href_fall  = href_d    & ~cam_href;

always @(posedge cam_pclk or negedge rst_n) begin
    if (!rst_n) begin
        vsync_d <= 1'b0;
        href_d  <= 1'b0;
    end else begin
        vsync_d <= cam_vsync;
        href_d  <= cam_href;
    end
end

// ---------------------------------------------------------------------------
// Byte assembly (RGB565 = 2 bytes per pixel on 8-bit bus)
// ---------------------------------------------------------------------------
reg       byte_sel;
reg [7:0] hi_byte;

// ---------------------------------------------------------------------------
// Source pixel counters
// ---------------------------------------------------------------------------
reg [9:0] src_x;   // 0..SRC_W-1  (800 max -> 10 bits)
reg [9:0] src_y;   // 0..SRC_H-1  (600 max -> 10 bits)

// ---------------------------------------------------------------------------
// BUG 2 FIX: Bresenham / DDA error accumulators
//
//   X axis (per source pixel within a line):
//     acc_x += FRAME_W each source pixel
//     select pixel and acc_x -= SRC_W when acc_x >= SRC_W
//
//   Y axis (per source line within a frame):
//     acc_y += FRAME_H each source line
//     select line and acc_y -= SRC_H when acc_y >= SRC_H
//
// Accumulator width: needs to hold SRC_W-1 max (800-1=799) -> 10 bits
// We use 11 bits to safely accommodate the addition before the compare.
// ---------------------------------------------------------------------------
reg [10:0] acc_x;    // 11 bits: 0 .. SRC_W + FRAME_W - 1 (max ~895)
reg [10:0] acc_y;    // 11 bits: 0 .. SRC_H + FRAME_H - 1 (max ~695)

// Combinatorial next values
wire [10:0] acc_x_nxt = acc_x + BRES_FW;
wire [10:0] acc_y_nxt = acc_y + BRES_FH;

// Select flags
wire col_select = (acc_x_nxt >= BRES_SW);   // Bresenham X fire
wire row_tick   = (acc_y_nxt >= BRES_SH);   // Bresenham Y fire

// ---------------------------------------------------------------------------
// Destination counters
// ---------------------------------------------------------------------------
reg [6:0] dst_x;   // 0..FRAME_W-1
reg [6:0] dst_y;   // 0..FRAME_H-1

// Row-keep flag: latched at href_rise for the whole line duration
reg row_keep;

// ---------------------------------------------------------------------------
// Address: flat_addr = dst_y * 96 + dst_x
//   96 = 64 + 32 = (1<<6) + (1<<5) -> no multiplier primitive needed
// ---------------------------------------------------------------------------
wire [16:0] dst_y96   = ({10'd0, dst_y} << 6) + ({10'd0, dst_y} << 5);
wire [16:0] flat_addr = dst_y96 + {10'd0, dst_x};

// ===========================================================================
// 1. FSM  (cam_pclk domain)
// ===========================================================================
always @(posedge cam_pclk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= ST_IDLE;
        frame_done <= 1'b0;
    end else begin
        frame_done <= 1'b0;
        case (state)

            // Wait for synchronised trigger pulse from cmd_decoder (0x02)
            ST_IDLE: begin
                if (trig_sync_rise)
                    state <= ST_ARMED;
            end

            // Trigger received - wait for the NEXT clean VSYNC rising edge
            // so we always capture a complete, aligned frame.
            ST_ARMED: begin
                if (vsync_rise)
                    state <= ST_CAPTURE;
            end

            // Actively capturing: exit once all dst rows are filled.
            // dst_y reaches DST_H_MAX only after the last href_fall,
            // guaranteeing all pixels for row 95 are written first.
            ST_CAPTURE: begin
                if (dst_y == DST_H_MAX)
                    state <= ST_DONE;
            end

            ST_DONE: begin
                frame_done <= 1'b1;
                state      <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

// ===========================================================================
// 2. Row accumulator (Bresenham Y)
//    row_keep latched at href_rise; counters advance at href_fall.
// ===========================================================================
always @(posedge cam_pclk or negedge rst_n) begin
    if (!rst_n) begin
        acc_y    <= 11'd0;
        dst_y    <= 7'd0;
        src_y    <= 10'd0;
        row_keep <= 1'b0;
    end else begin
        if (state != ST_CAPTURE) begin
            // Hold reset while idle/armed/done
            acc_y    <= 11'd0;
            dst_y    <= 7'd0;
            src_y    <= 10'd0;
            row_keep <= 1'b0;
        end else begin
            // At the START of each href burst: decide if this src row is kept
            if (href_rise)
                row_keep <= row_tick;

            // At the END of each href burst: advance Bresenham Y state
            if (href_fall) begin
                if (row_tick) begin
                    acc_y <= acc_y_nxt - BRES_SH;  // subtract SRC_H
                    dst_y <= dst_y + 7'd1;
                end else begin
                    acc_y <= acc_y_nxt;
                end
                src_y <= src_y + 10'd1;
            end
        end
    end
end

// ===========================================================================
// 3. Column accumulator (Bresenham X) + byte assembly + frame-buffer write
//
//    href_rise: RESET ONLY - do NOT consume cam_data this cycle.
//    cam_href (not href_rise): DATA phase - consume bytes, write buffer.
//    This mutual exclusion prevents the byte-phase inversion bug.
// ===========================================================================
always @(posedge cam_pclk or negedge rst_n) begin
    if (!rst_n) begin
        acc_x    <= 11'd0;
        dst_x    <= 7'd0;
        src_x    <= 10'd0;
        byte_sel <= 1'b0;
        hi_byte  <= 8'd0;
        wr_en    <= 1'b0;
        wr_addr  <= 17'd0;
        wr_data  <= 16'd0;
    end else begin
        wr_en <= 1'b0;  // default: no write

        if (state != ST_CAPTURE) begin
            // Hold in reset while not capturing
            acc_x    <= 11'd0;
            dst_x    <= 7'd0;
            src_x    <= 10'd0;
            byte_sel <= 1'b0;

        end else if (href_rise) begin
            // ----------------------------------------------------------------
            // RESET PHASE (href just went high): reset column state only.
            // cam_data is NOT consumed this cycle - the camera holds the
            // first real byte until the following pclk edge.
            // ----------------------------------------------------------------
            acc_x    <= 11'd0;
            dst_x    <= 7'd0;
            src_x    <= 10'd0;
            byte_sel <= 1'b0;

        end else if (cam_href) begin
            // ----------------------------------------------------------------
            // DATA PHASE: consume bytes, run Bresenham X, write buffer
            // ----------------------------------------------------------------
            if (!byte_sel) begin
                // First byte of pixel pair (high byte: R[4:0] G[5:3])
                hi_byte  <= cam_data;
                byte_sel <= 1'b1;

            end else begin
                // Second byte of pixel pair (low byte: G[2:0] B[4:0])
                // Full RGB565 word is now available.
                byte_sel <= 1'b0;

                // Write to frame buffer only when:
                //   - Bresenham X selects this column (col_select)
                //   - Row was selected at start of this line (row_keep)
                //   - Still within destination bounds (guard against overshoot)
                if (col_select && row_keep &&
                    (dst_x < DST_W_MAX) &&
                    (dst_y < DST_H_MAX))
                begin
                    wr_en   <= 1'b1;
                   wr_data <= {hi_byte, cam_data};   //wr_data <= 16'hF800; 
                    wr_addr <= flat_addr;
                end

                // Advance Bresenham X state
                if (col_select) begin
                    acc_x <= acc_x_nxt - BRES_SW;  // subtract SRC_W
                    if (dst_x < DST_W_MAX)
                        dst_x <= dst_x + 7'd1;
                end else begin
                    acc_x <= acc_x_nxt;
                end
                src_x <= src_x + 10'd1;
            end
        end
    end
end

endmodule
// =============================================================================
// End of ov2640_capture  rev 3
// =============================================================================