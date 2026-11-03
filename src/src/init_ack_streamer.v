`timescale 1ns / 1ps

// ============================================================
// init_ack_streamer.v   (corrected, resource-reduced)
//
// 1. Normal initialization ACK:
//
//       BB 66 00 0F
//       CAM INIT DONE\r\n
//
// 2. DVP debug packet, one per 0x03 trigger:
//
//       BB 66 01 7D
//       381 bytes of ASCII DVP information
//
// DECIMAL CONVERSION:
//
//   The six numeric counter fields (Pixel Clock Edges, Href
//   Lines Counted, Vsync Frames Counted, Last/Max Line PCLK,
//   Last Frame Href Lines) are ASCII DECIMAL, not hex.
//
//   A binary value cannot be turned into decimal digits by
//   re-slicing nibbles and relabeling the ASCII function - that
//   only works for hex, because hex digits ARE nibbles. Decimal
//   digits are not powers of the underlying binary radix, so a
//   real binary -> BCD conversion (double dabble) has to run
//   first.
//
//   The five 0/1 status flags (*_Seen, *_Overflow) are still
//   single ASCII characters and needed no conversion; they were
//   already valid decimal.
//
// RESOURCE FIX vs previous version:
//
//   The previous version called a 32-iteration bin2bcd32()
//   FUNCTION six times (once per counter). Verilog functions
//   are unrolled into pure combinational logic at synthesis -
//   there is no hardware reuse across calls or across loop
//   iterations. Six independent 32-stage cascades of ten 4-bit
//   comparator/adders each is a large amount of combinational
//   logic that only ever needs to toggle once per debug packet
//   (a rare, non-performance-critical event).
//
//   This version replaces that with ONE shared iterative
//   double-dabble datapath (bcd_adjust() is now a single-stage
//   combinational function, not a loop) driven by three extra
//   DVP-FSM states (DVP_BCD_LOAD / DVP_BCD_ITER / DVP_BCD_STORE)
//   that walk through the six values sequentially, one dabble
//   step per clock (32 clocks per value, ~192 clocks total).
//   That is negligible latency for a manually-triggered debug
//   packet, and should remove roughly 5/6 of the logic that
//   bin2bcd32 was contributing.
//
// FIXES carried over from the previous version:
//
//   - dvp_trigger is edge detected and LATCHED into dvp_req,
//     so a trigger that arrives while the UART is busy is
//     remembered instead of being thrown away.
//
//   - req_toggle is driven ONLY by the DVP state machine,
//     exactly once per packet.
//
//   - dvp_snapshot_pending is cleared when the request is
//     issued, so a stale pending flag from the previous packet
//     can never be consumed.
//
//   - Snapshot wait has a timeout. If cam_pclk stops, the
//     packet still goes out instead of locking dvp_active high.
//
//   - Byte send waits for the UART to actually raise tx_busy
//     before waiting for it to fall, and busy is not released
//     until the final byte has finished shifting out.
// ============================================================

module init_ack_streamer (

    input  wire       clk,
    input  wire       rst_n,

    // Normal initialization ACK request (rising edge)
    input  wire       init_done,

    // DVP debug request (from cmd_decoder, UART command 0x03)
    input  wire       dvp_trigger,

    // UART
    output reg        busy,
    output reg [7:0]  tx_data,
    output reg        tx_start,
    input  wire       tx_busy,

    // Camera DVP
    input  wire       cam_pclk,
    input  wire       cam_href,
    input  wire       cam_vsync,

    // DVP active status (single driver)
    output wire       dvp_active
);

    // =========================================================
    // HEX TO ASCII
    // =========================================================
    function [7:0] h2a;
        input [3:0] val;
        begin
            if (val < 10)
                h2a = val + 8'h30;
            else
                h2a = val + 8'h37;
        end
    endfunction

    // =========================================================
    // ONE HEX DIGIT OF A 32 BIT VALUE
    // d = 0 -> most significant nibble
    // =========================================================
    function [7:0] hexd;
        input [31:0] v;
        input [2:0]  d;
        reg   [3:0]  n;
        begin
            case (d)
                3'd0:    n = v[31:28];
                3'd1:    n = v[27:24];
                3'd2:    n = v[23:20];
                3'd3:    n = v[19:16];
                3'd4:    n = v[15:12];
                3'd5:    n = v[11:8];
                3'd6:    n = v[7:4];
                default: n = v[3:0];
            endcase
            hexd = h2a(n);
        end
    endfunction


    // =========================================================
    // DECIMAL DIGIT (0-9) TO ASCII
    //
    // A true BCD digit is always 0-9, so unlike h2a there is no
    // >=10 branch. IMPORTANT: this must only ever be fed a digit
    // that came out of the sequential BCD converter below.
    // =========================================================
    function [7:0] d2a;
        input [3:0] digit;
        begin
            d2a = digit + 8'h30;
        end
    endfunction

    // =========================================================
    // ONE DOUBLE-DABBLE ADJUST STEP (combinational, single stage)
    //
    // Given the current 40-bit BCD shift register, add 3 to any
    // nibble that is >= 5. This is ONE hardware instance reused
    // every clock by the DVP FSM below (see DVP_BCD_ITER) -
    // unlike the old bin2bcd32() function, this is NOT a loop,
    // so it synthesizes to a single small combinational block
    // instead of 32 cascaded copies of itself.
    // =========================================================
    function [39:0] bcd_adjust;
        input [39:0] bcd;
        begin
            bcd_adjust = bcd;
            if (bcd_adjust[3:0]   >= 5) bcd_adjust[3:0]   = bcd_adjust[3:0]   + 3;
            if (bcd_adjust[7:4]   >= 5) bcd_adjust[7:4]   = bcd_adjust[7:4]   + 3;
            if (bcd_adjust[11:8]  >= 5) bcd_adjust[11:8]  = bcd_adjust[11:8]  + 3;
            if (bcd_adjust[15:12] >= 5) bcd_adjust[15:12] = bcd_adjust[15:12] + 3;
            if (bcd_adjust[19:16] >= 5) bcd_adjust[19:16] = bcd_adjust[19:16] + 3;
            if (bcd_adjust[23:20] >= 5) bcd_adjust[23:20] = bcd_adjust[23:20] + 3;
            if (bcd_adjust[27:24] >= 5) bcd_adjust[27:24] = bcd_adjust[27:24] + 3;
            if (bcd_adjust[31:28] >= 5) bcd_adjust[31:28] = bcd_adjust[31:28] + 3;
            if (bcd_adjust[35:32] >= 5) bcd_adjust[35:32] = bcd_adjust[35:32] + 3;
            if (bcd_adjust[39:36] >= 5) bcd_adjust[39:36] = bcd_adjust[39:36] + 3;
        end
    endfunction

    // =========================================================
    // ONE DECIMAL DIGIT OF A PRE-CONVERTED 40-BIT BCD WORD
    // d = 0 -> most significant digit (10 digits total, each 0-9)
    //
    // Cheap - just a slice, same role hexd() plays for hex - but
    // it must be fed an already-converted BCD register, never a
    // raw binary value.
    // =========================================================
    function [7:0] decd;
        input [39:0] bcd;
        input [3:0]  d;
        reg   [3:0]  n;
        begin
            case (d)
                4'd0:    n = bcd[39:36];
                4'd1:    n = bcd[35:32];
                4'd2:    n = bcd[31:28];
                4'd3:    n = bcd[27:24];
                4'd4:    n = bcd[23:20];
                4'd5:    n = bcd[19:16];
                4'd6:    n = bcd[15:12];
                4'd7:    n = bcd[11:8];
                4'd8:    n = bcd[7:4];
                default: n = bcd[3:0];
            endcase
            decd = d2a(n);
        end
    endfunction


    // =========================================================
    // =========================================================
    // DVP LIVE COUNTERS  (camera clock domain)
    // =========================================================
    // =========================================================

    reg [31:0] pclk_edges;
    reg [31:0] href_lines;
    reg [31:0] vsync_frames;

    reg [31:0] current_line_pclk;
    reg [31:0] last_line_pclk;
    reg [31:0] max_line_pclk;

    reg [31:0] current_frame_lines;
    reg [31:0] last_frame_lines;

    reg pclk_seen;
    reg href_seen;
    reg vsync_seen;

    reg line_overflow;
    reg frame_overflow;

    reg href_d;
    reg vsync_d;

    localparam [31:0] MAX_ALLOWED_LINE_PCLK   = 32'd2048;
    localparam [31:0] MAX_ALLOWED_FRAME_LINES = 32'd1024;


    always @(posedge cam_pclk or negedge rst_n) begin

        if (!rst_n) begin

            pclk_edges          <= 32'd0;
            href_lines          <= 32'd0;
            vsync_frames        <= 32'd0;

            current_line_pclk   <= 32'd0;
            last_line_pclk      <= 32'd0;
            max_line_pclk       <= 32'd0;

            current_frame_lines <= 32'd0;
            last_frame_lines    <= 32'd0;

            pclk_seen           <= 1'b0;
            href_seen           <= 1'b0;
            vsync_seen          <= 1'b0;

            line_overflow       <= 1'b0;
            frame_overflow      <= 1'b0;

            href_d              <= 1'b0;
            vsync_d             <= 1'b0;

        end

        else begin

            pclk_edges <= pclk_edges + 32'd1;
            pclk_seen  <= 1'b1;

            href_d  <= cam_href;
            vsync_d <= cam_vsync;

            // ---------------- HREF RISING EDGE ----------------
            if (cam_href && !href_d) begin

                href_lines          <= href_lines + 32'd1;
                current_line_pclk   <= 32'd1;
                current_frame_lines <= current_frame_lines + 32'd1;
                href_seen           <= 1'b1;

                if ((current_frame_lines + 32'd1) >
                    MAX_ALLOWED_FRAME_LINES)
                    frame_overflow <= 1'b1;

            end

            // ---------------- HREF HIGH -----------------------
            else if (cam_href) begin

                current_line_pclk <= current_line_pclk + 32'd1;

                if ((current_line_pclk + 32'd1) >
                    MAX_ALLOWED_LINE_PCLK)
                    line_overflow <= 1'b1;

            end

            // ---------------- HREF FALLING EDGE ---------------
            else if (!cam_href && href_d) begin

                last_line_pclk <= current_line_pclk;

                if (current_line_pclk > max_line_pclk)
                    max_line_pclk <= current_line_pclk;

            end

            // ---------------- VSYNC RISING EDGE ---------------
            if (cam_vsync && !vsync_d) begin

                vsync_frames        <= vsync_frames + 32'd1;
                last_frame_lines    <= current_frame_lines;
                current_frame_lines <= 32'd0;
                vsync_seen          <= 1'b1;

            end

        end

    end

    // =========================================================
    // SNAPSHOT REGISTERS
    // =========================================================
    reg [31:0] snap_pclk_edges;
    reg [31:0] snap_href_lines;
    reg [31:0] snap_vsync_frames;

    reg [31:0] snap_last_line_pclk;
    reg [31:0] snap_max_line_pclk;
    reg [31:0] snap_last_frame_lines;

    reg snap_pclk_seen;
    reg snap_href_seen;
    reg snap_vsync_seen;

    reg snap_line_overflow;
    reg snap_frame_overflow;

    reg ack_toggle_pclk;

    // =========================================================
    // BCD (decimal) VERSIONS OF THE SIX COUNTER FIELDS
    //
    // Filled sequentially by the shared converter driven from
    // the DVP FSM (DVP_BCD_LOAD / DVP_BCD_ITER / DVP_BCD_STORE),
    // NOT computed all-at-once by six parallel functions anymore.
    // =========================================================
    reg [39:0] snap_pclk_edges_bcd;
    reg [39:0] snap_href_lines_bcd;
    reg [39:0] snap_vsync_frames_bcd;
    reg [39:0] snap_last_line_pclk_bcd;
    reg [39:0] snap_max_line_pclk_bcd;
    reg [39:0] snap_last_frame_lines_bcd;


    // =========================================================
    // SNAPSHOT REQUEST TOGGLE
    //
    // Driven ONLY by the DVP state machine (see below).
    // =========================================================
    reg req_toggle;


    // =========================================================
    // REQUEST INTO CAMERA DOMAIN
    // =========================================================
    reg [2:0] req_sync_pclk;

    always @(posedge cam_pclk or negedge rst_n) begin

        if (!rst_n)
            req_sync_pclk <= 3'b000;
        else
            req_sync_pclk <= {req_sync_pclk[1:0], req_toggle};

    end

    wire take_snapshot = req_sync_pclk[2] ^ req_sync_pclk[1];


    // =========================================================
    // TAKE DVP SNAPSHOT
    // =========================================================
    always @(posedge cam_pclk or negedge rst_n) begin

        if (!rst_n) begin

            snap_pclk_edges       <= 32'd0;
            snap_href_lines       <= 32'd0;
            snap_vsync_frames     <= 32'd0;

            snap_last_line_pclk   <= 32'd0;
            snap_max_line_pclk    <= 32'd0;
            snap_last_frame_lines <= 32'd0;

            snap_pclk_seen        <= 1'b0;
            snap_href_seen        <= 1'b0;
            snap_vsync_seen       <= 1'b0;

            snap_line_overflow    <= 1'b0;
            snap_frame_overflow   <= 1'b0;

            ack_toggle_pclk       <= 1'b0;

        end

        else if (take_snapshot) begin

            snap_pclk_edges       <= pclk_edges;
            snap_href_lines       <= href_lines;
            snap_vsync_frames     <= vsync_frames;

            snap_last_line_pclk   <= last_line_pclk;
            snap_max_line_pclk    <= max_line_pclk;
            snap_last_frame_lines <= last_frame_lines;

            snap_pclk_seen        <= pclk_seen;
            snap_href_seen        <= href_seen;
            snap_vsync_seen       <= vsync_seen;

            snap_line_overflow    <= line_overflow;
            snap_frame_overflow   <= frame_overflow;

            ack_toggle_pclk       <= ~ack_toggle_pclk;

        end

    end


    // =========================================================
    // SNAPSHOT ACK BACK INTO SYSTEM DOMAIN
    // =========================================================
    reg [2:0] ack_sync;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n)
            ack_sync <= 3'b000;
        else
            ack_sync <= {ack_sync[1:0], ack_toggle_pclk};

    end

    wire snapshot_ready = ack_sync[2] ^ ack_sync[1];


    // =========================================================
    // =========================================================
    // NORMAL INIT ACK FSM
    // =========================================================
    // =========================================================

    localparam MSG_LEN = 15;

    localparam [15*8-1:0] S_INIT = {"CAM INIT DONE", 8'h0D, 8'h0A};

    localparam INIT_IDLE   = 3'd0;
    localparam INIT_LOAD   = 3'd1;
    localparam INIT_SEND   = 3'd2;
    localparam INIT_SETTLE = 3'd3;
    localparam INIT_WAIT   = 3'd4;
    localparam INIT_FIN    = 3'd5;

    reg [2:0] init_state;
    reg [3:0] init_idx;
    reg [1:0] init_settle;

    reg init_done_d1;

    reg       init_busy;
    reg [7:0] init_tx_data;
    reg       init_tx_start;


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            init_state    <= INIT_IDLE;
            init_busy     <= 1'b0;
            init_tx_start <= 1'b0;
            init_tx_data  <= 8'd0;
            init_idx      <= 4'd0;
            init_settle   <= 2'd0;
            init_done_d1  <= 1'b0;

        end

        else begin

            init_tx_start <= 1'b0;
            init_done_d1  <= init_done;

            case (init_state)

                INIT_IDLE: begin

                    init_busy <= 1'b0;

                    if (init_done && !init_done_d1) begin
                        init_busy  <= 1'b1;
                        init_idx   <= 4'd0;
                        init_state <= INIT_LOAD;
                    end

                end

                INIT_LOAD: begin

                    init_tx_data <= S_INIT[(14 - init_idx)*8 +: 8];
                    init_state   <= INIT_SEND;

                end

                INIT_SEND: begin

                    if (!tx_busy) begin
                        init_tx_start <= 1'b1;
                        init_settle   <= 2'd0;
                        init_state    <= INIT_SETTLE;
                    end

                end

                // Give the UART time to raise tx_busy before
                // we start waiting for it to fall.
                INIT_SETTLE: begin

                    if (init_settle == 2'd3)
                        init_state <= INIT_WAIT;
                    else
                        init_settle <= init_settle + 1'b1;

                end

                INIT_WAIT: begin

                    if (!tx_busy) begin

                        if (init_idx == MSG_LEN - 1)
                            init_state <= INIT_FIN;
                        else begin
                            init_idx   <= init_idx + 1'b1;
                            init_state <= INIT_LOAD;
                        end

                    end

                end

                INIT_FIN: begin

                    init_busy  <= 1'b0;
                    init_state <= INIT_IDLE;

                end

                default: begin

                    init_state <= INIT_IDLE;
                    init_busy  <= 1'b0;

                end

            endcase

        end

    end


    // =========================================================
    // =========================================================
    // DVP ASCII PAYLOAD
    // =========================================================
    // =========================================================
    //
    // Byte map. The six counters are now DECIMAL (10 digits,
    // from the *_bcd registers), the five 0/1 flags are
    // unchanged single ASCII digits - a flag is already exactly
    // one decimal digit, so it never needed hex/BCD conversion.
    //
    //   0   .. 17    "Pixel Clock Edges:"
    //   18  .. 27    10 decimal digits
    //   28  .. 29    CR LF
    //   30  .. 64    "Horizontal Reference Lines Counted:"
    //   65  .. 74    10 decimal digits
    //   75  .. 76    CR LF
    //   77  ..116    "Vertical Synchronization Frames Counted:"
    //   117 ..126    10 decimal digits
    //   127 ..128    CR LF
    //   129 ..157    "Last Line Pixel Clock Cycles:"
    //   158 ..167    10 decimal digits
    //   168 ..169    CR LF
    //   170 ..201    "Maximum Line Pixel Clock Cycles:"
    //   202 ..211    10 decimal digits
    //   212 ..213    CR LF
    //   214 ..251    "Last Frame Horizontal Reference Lines:"
    //   252 ..261    10 decimal digits
    //   262 ..263    CR LF
    //   264 ..280    "Pixel Clock Seen:"
    //   281          flag
    //   282 ..283    CR LF
    //   284 ..309    "Horizontal Reference Seen:"
    //   310          flag
    //   311 ..312    CR LF
    //   313 ..342    "Vertical Synchronization Seen:"
    //   343          flag
    //   344 ..345    CR LF
    //   346 ..359    "Line Overflow:"
    //   360          flag
    //   361 ..362    CR LF
    //   363 ..377    "Frame Overflow:"
    //   378          flag
    //   379 ..380    CR LF
    //
    //   TOTAL = 381 = 0x017D
    // =========================================================

    localparam [16:0] DVP_TEXT_LEN = 17'd381;

    localparam [18*8-1:0] S_PCLK   = "Pixel Clock Edges:";
    localparam [35*8-1:0] S_HREF   = "Horizontal Reference Lines Counted:";
    localparam [40*8-1:0] S_VSYNC  = "Vertical Synchronization Frames Counted:";
    localparam [29*8-1:0] S_LLINE  = "Last Line Pixel Clock Cycles:";
    localparam [32*8-1:0] S_MLINE  = "Maximum Line Pixel Clock Cycles:";
    localparam [38*8-1:0] S_LFRAME = "Last Frame Horizontal Reference Lines:";
    localparam [17*8-1:0] S_PSEEN  = "Pixel Clock Seen:";
    localparam [26*8-1:0] S_HSEEN  = "Horizontal Reference Seen:";
    localparam [30*8-1:0] S_VSEEN  = "Vertical Synchronization Seen:";
    localparam [14*8-1:0] S_LOVF   = "Line Overflow:";
    localparam [15*8-1:0] S_FOVF   = "Frame Overflow:";


    function [7:0] dvp_byte;
        input [8:0] i;
        begin

            dvp_byte = 8'h20;

            // ------------------- PIXEL CLOCK EDGES (decimal)
            if      (i <   18) dvp_byte = S_PCLK[(17 - i)*8 +: 8];
            else if (i <   28) dvp_byte = decd(snap_pclk_edges_bcd, i - 18);
            else if (i ==  28) dvp_byte = 8'h0D;
            else if (i ==  29) dvp_byte = 8'h0A;

            // ------------------- HREF LINES (decimal)
            else if (i <   65) dvp_byte = S_HREF[(64 - i)*8 +: 8];
            else if (i <   75) dvp_byte = decd(snap_href_lines_bcd, i - 65);
            else if (i ==  75) dvp_byte = 8'h0D;
            else if (i ==  76) dvp_byte = 8'h0A;

            // ------------------- VSYNC FRAMES (decimal)
            else if (i <  117) dvp_byte = S_VSYNC[(116 - i)*8 +: 8];
            else if (i <  127) dvp_byte = decd(snap_vsync_frames_bcd, i - 117);
            else if (i == 127) dvp_byte = 8'h0D;
            else if (i == 128) dvp_byte = 8'h0A;

            // ------------------- LAST LINE PCLK (decimal)
            else if (i <  158) dvp_byte = S_LLINE[(157 - i)*8 +: 8];
            else if (i <  168) dvp_byte = decd(snap_last_line_pclk_bcd, i - 158);
            else if (i == 168) dvp_byte = 8'h0D;
            else if (i == 169) dvp_byte = 8'h0A;

            // ------------------- MAX LINE PCLK (decimal)
            else if (i <  202) dvp_byte = S_MLINE[(201 - i)*8 +: 8];
            else if (i <  212) dvp_byte = decd(snap_max_line_pclk_bcd, i - 202);
            else if (i == 212) dvp_byte = 8'h0D;
            else if (i == 213) dvp_byte = 8'h0A;

            // ------------------- LAST FRAME LINES (decimal)
            else if (i <  252) dvp_byte = S_LFRAME[(251 - i)*8 +: 8];
            else if (i <  262) dvp_byte = decd(snap_last_frame_lines_bcd, i - 252);
            else if (i == 262) dvp_byte = 8'h0D;
            else if (i == 263) dvp_byte = 8'h0A;

            // ------------------- PCLK SEEN
            else if (i <  281) dvp_byte = S_PSEEN[(280 - i)*8 +: 8];
            else if (i == 281) dvp_byte = snap_pclk_seen  ? "1" : "0";
            else if (i == 282) dvp_byte = 8'h0D;
            else if (i == 283) dvp_byte = 8'h0A;

            // ------------------- HREF SEEN
            else if (i <  310) dvp_byte = S_HSEEN[(309 - i)*8 +: 8];
            else if (i == 310) dvp_byte = snap_href_seen  ? "1" : "0";
            else if (i == 311) dvp_byte = 8'h0D;
            else if (i == 312) dvp_byte = 8'h0A;

            // ------------------- VSYNC SEEN
            else if (i <  343) dvp_byte = S_VSEEN[(342 - i)*8 +: 8];
            else if (i == 343) dvp_byte = snap_vsync_seen ? "1" : "0";
            else if (i == 344) dvp_byte = 8'h0D;
            else if (i == 345) dvp_byte = 8'h0A;

            // ------------------- LINE OVERFLOW
            else if (i <  360) dvp_byte = S_LOVF[(359 - i)*8 +: 8];
            else if (i == 360) dvp_byte = snap_line_overflow  ? "1" : "0";
            else if (i == 361) dvp_byte = 8'h0D;
            else if (i == 362) dvp_byte = 8'h0A;

            // ------------------- FRAME OVERFLOW
            else if (i <  378) dvp_byte = S_FOVF[(377 - i)*8 +: 8];
            else if (i == 378) dvp_byte = snap_frame_overflow ? "1" : "0";
            else if (i == 379) dvp_byte = 8'h0D;
            else               dvp_byte = 8'h0A;   // i == 380

        end
    endfunction


    // =========================================================
    // =========================================================
    // DVP PACKET FSM
    //
    // Stream index 0..384:
    //   0 = BB
    //   1 = 66
    //   2 = length high (0x01)
    //   3 = length low  (0x7D)
    //   4..384 = payload byte (index - 4), 381 bytes total
    //
    // Three extra states (DVP_BCD_LOAD / DVP_BCD_ITER /
    // DVP_BCD_STORE) sit between DVP_WAIT_SNAP and DVP_LOAD and
    // drive the ONE shared double-dabble datapath through all
    // six counter values, one dabble step per clock, before any
    // bytes are sent. See header comment for why.
    // =========================================================
    // =========================================================

    localparam [8:0] DVP_TOTAL = 9'd385;

    localparam DVP_IDLE      = 4'd0;
    localparam DVP_WAIT_INIT = 4'd1;
    localparam DVP_REQ_SNAP  = 4'd2;
    localparam DVP_WAIT_SNAP = 4'd3;
    localparam DVP_LOAD      = 4'd4;
    localparam DVP_SEND      = 4'd5;
    localparam DVP_SETTLE    = 4'd6;
    localparam DVP_WAIT      = 4'd7;
    localparam DVP_FIN       = 4'd8;
    localparam DVP_BCD_LOAD  = 4'd9;
    localparam DVP_BCD_ITER  = 4'd10;
    localparam DVP_BCD_STORE = 4'd11;

    reg [3:0] dvp_state;
    reg [8:0] dvp_idx;
    reg [1:0] dvp_settle;

    reg       dvp_busy;
    reg [7:0] dvp_tx_data;
    reg       dvp_tx_start;

    // Latched request. A 0x03 that arrives while the UART is
    // busy is remembered, not lost.
    reg dvp_req;
    reg dvp_trig_d;

    // Latched snapshot completion.
    reg dvp_snapshot_pending;

    // Snapshot timeout: about 39 ms at 27 MHz.
    reg [19:0] snap_timeout;

    // Shared sequential BCD converter state.
    reg [2:0]  bcd_sel;       // which of the 6 values (0..5)
    reg [5:0]  bcd_iter;      // dabble iteration (0..31)
    reg [39:0] bcd_shreg;     // BCD working register
    reg [31:0] bcd_bin_shreg; // binary value being shifted in


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            dvp_state            <= DVP_IDLE;
            dvp_idx              <= 9'd0;
            dvp_settle           <= 2'd0;

            dvp_busy             <= 1'b0;
            dvp_tx_data          <= 8'd0;
            dvp_tx_start         <= 1'b0;

            dvp_req              <= 1'b0;
            dvp_trig_d           <= 1'b0;

            dvp_snapshot_pending <= 1'b0;
            snap_timeout         <= 20'd0;

            req_toggle           <= 1'b0;

            bcd_sel              <= 3'd0;
            bcd_iter             <= 6'd0;
            bcd_shreg            <= 40'd0;
            bcd_bin_shreg        <= 32'd0;

            snap_pclk_edges_bcd       <= 40'd0;
            snap_href_lines_bcd       <= 40'd0;
            snap_vsync_frames_bcd     <= 40'd0;
            snap_last_line_pclk_bcd   <= 40'd0;
            snap_max_line_pclk_bcd    <= 40'd0;
            snap_last_frame_lines_bcd <= 40'd0;

        end

        else begin

            dvp_tx_start <= 1'b0;
            dvp_trig_d   <= dvp_trigger;

            // Set before the case so that a clear inside
            // DVP_REQ_SNAP always wins over a stale set.
            if (snapshot_ready)
                dvp_snapshot_pending <= 1'b1;

            case (dvp_state)

                // -------------------------------------------------
                // IDLE
                // -------------------------------------------------
                DVP_IDLE: begin

                    dvp_busy <= 1'b0;

                    if (dvp_req) begin
                        dvp_req    <= 1'b0;
                        dvp_busy   <= 1'b1;
                        dvp_state  <= DVP_WAIT_INIT;
                    end

                end

                // -------------------------------------------------
                // LET THE INIT ACK FINISH FIRST
                // -------------------------------------------------
                DVP_WAIT_INIT: begin

                    dvp_busy <= 1'b1;

                    if (!init_busy)
                        dvp_state <= DVP_REQ_SNAP;

                end

                // -------------------------------------------------
                // ASK THE CAMERA DOMAIN FOR A SNAPSHOT
                //
                // This is the ONLY place req_toggle is flipped.
                // -------------------------------------------------
                DVP_REQ_SNAP: begin

                    req_toggle           <= ~req_toggle;
                    dvp_snapshot_pending <= 1'b0;
                    snap_timeout         <= 20'd0;

                    dvp_state <= DVP_WAIT_SNAP;

                end

                // -------------------------------------------------
                // WAIT FOR THE SNAPSHOT (WITH TIMEOUT)
                //
                // The timeout stops a stopped cam_pclk from
                // hanging dvp_active high forever, which would
                // also freeze frame_streamer.
                // -------------------------------------------------
                DVP_WAIT_SNAP: begin

                    snap_timeout <= snap_timeout + 20'd1;

                    if (dvp_snapshot_pending ||
                        (snap_timeout == {20{1'b1}})) begin

                        dvp_snapshot_pending <= 1'b0;
                        bcd_sel              <= 3'd0;
                        dvp_state            <= DVP_BCD_LOAD;

                    end

                end

                // -------------------------------------------------
                // LOAD THE NEXT BINARY VALUE INTO THE SHARED
                // CONVERTER (one value per pass through this
                // state, selected by bcd_sel).
                // -------------------------------------------------
                DVP_BCD_LOAD: begin

                    case (bcd_sel)
                        3'd0: bcd_bin_shreg <= snap_pclk_edges;
                        3'd1: bcd_bin_shreg <= snap_href_lines;
                        3'd2: bcd_bin_shreg <= snap_vsync_frames;
                        3'd3: bcd_bin_shreg <= snap_last_line_pclk;
                        3'd4: bcd_bin_shreg <= snap_max_line_pclk;
                        default: bcd_bin_shreg <= snap_last_frame_lines;
                    endcase

                    bcd_shreg <= 40'd0;
                    bcd_iter  <= 6'd0;
                    dvp_state <= DVP_BCD_ITER;

                end

                // -------------------------------------------------
                // ONE DABBLE STEP PER CLOCK, REUSING THE SAME
                // bcd_adjust() HARDWARE FOR ALL SIX VALUES.
                // -------------------------------------------------
                DVP_BCD_ITER: begin

                    bcd_shreg     <= {bcd_adjust(bcd_shreg)[38:0], bcd_bin_shreg[31]};
                    bcd_bin_shreg <= {bcd_bin_shreg[30:0], 1'b0};

                    if (bcd_iter == 6'd31)
                        dvp_state <= DVP_BCD_STORE;
                    else
                        bcd_iter <= bcd_iter + 6'd1;

                end

                // -------------------------------------------------
                // STORE THE FINISHED BCD WORD, THEN MOVE ON TO
                // THE NEXT VALUE OR START SENDING THE PACKET.
                // -------------------------------------------------
                DVP_BCD_STORE: begin

                    case (bcd_sel)
                        3'd0: snap_pclk_edges_bcd       <= bcd_shreg;
                        3'd1: snap_href_lines_bcd       <= bcd_shreg;
                        3'd2: snap_vsync_frames_bcd     <= bcd_shreg;
                        3'd3: snap_last_line_pclk_bcd    <= bcd_shreg;
                        3'd4: snap_max_line_pclk_bcd     <= bcd_shreg;
                        default: snap_last_frame_lines_bcd <= bcd_shreg;
                    endcase

                    if (bcd_sel == 3'd5) begin
                        dvp_idx   <= 9'd0;
                        dvp_state <= DVP_LOAD;
                    end
                    else begin
                        bcd_sel   <= bcd_sel + 3'd1;
                        dvp_state <= DVP_BCD_LOAD;
                    end

                end

                // -------------------------------------------------
                // LOAD ONE BYTE
                // -------------------------------------------------
                DVP_LOAD: begin

                    case (dvp_idx)
                        9'd0: dvp_tx_data <= 8'hBB;
                        9'd1: dvp_tx_data <= 8'h66;
                        9'd2: dvp_tx_data <= DVP_TEXT_LEN[15:8];
                        9'd3: dvp_tx_data <= DVP_TEXT_LEN[7:0];
                        default: dvp_tx_data <= dvp_byte(dvp_idx - 9'd4);
                    endcase

                    dvp_state <= DVP_SEND;

                end

                // -------------------------------------------------
                // SEND WHEN THE UART IS FREE
                // -------------------------------------------------
                DVP_SEND: begin

                    if (!tx_busy) begin
                        dvp_tx_start <= 1'b1;
                        dvp_settle   <= 2'd0;
                        dvp_state    <= DVP_SETTLE;
                    end

                end

                // -------------------------------------------------
                // LET tx_busy RISE
                // -------------------------------------------------
                DVP_SETTLE: begin

                    if (dvp_settle == 2'd3)
                        dvp_state <= DVP_WAIT;
                    else
                        dvp_settle <= dvp_settle + 1'b1;

                end

                // -------------------------------------------------
                // WAIT FOR THE BYTE TO FINISH
                // -------------------------------------------------
                DVP_WAIT: begin

                    if (!tx_busy) begin

                        if (dvp_idx == DVP_TOTAL - 9'd1)
                            dvp_state <= DVP_FIN;
                        else begin
                            dvp_idx   <= dvp_idx + 9'd1;
                            dvp_state <= DVP_LOAD;
                        end

                    end

                end

                // -------------------------------------------------
                // DONE
                // -------------------------------------------------
                DVP_FIN: begin

                    dvp_busy  <= 1'b0;
                    dvp_state <= DVP_IDLE;

                end

                default: begin

                    dvp_busy  <= 1'b0;
                    dvp_state <= DVP_IDLE;

                end

            endcase

            // Set after the case so a trigger arriving on the
            // same clock that a packet starts is still honoured
            // and produces a second packet.
            if (dvp_trigger && !dvp_trig_d)
                dvp_req <= 1'b1;

        end

    end


    // =========================================================
    // DVP ACTIVE (single driver)
    //
    // High from the moment the request is accepted until the
    // last byte has left the UART, so frame_streamer will not
    // start a new image in the middle of the packet.
    // =========================================================
    assign dvp_active = dvp_busy;


    // =========================================================
    // UART OUTPUT MULTIPLEXER
    // INIT ACK has priority over DVP.
    // =========================================================
    always @(*) begin

        if (init_busy) begin
            busy     = 1'b1;
            tx_data  = init_tx_data;
            tx_start = init_tx_start;
        end
        else if (dvp_busy) begin
            busy     = 1'b1;
            tx_data  = dvp_tx_data;
            tx_start = dvp_tx_start;
        end
        else begin
            busy     = 1'b0;
            tx_data  = 8'd0;
            tx_start = 1'b0;
        end

    end

endmodule