`timescale 1ns / 1ps

module frame_streamer
#(
    parameter FRAME_W = 96,
    parameter FRAME_H = 96,
    parameter AW      = 14
)
(
    input wire clk,
    input wire rst_n,

    // ============================================================
    // FRAME CONTROL
    // ============================================================
    input wire start,
    output reg busy,

    // ============================================================
    // FRAMEBUFFER READ INTERFACE
    // ============================================================
    output reg [AW-1:0] fb_raddr,
    input wire [15:0]   fb_rdata,

    // ============================================================
    // UART TX INTERFACE
    // ============================================================
    output reg [7:0] tx_data,
    output reg       tx_start,
    input wire       tx_busy,

    // ============================================================
    // INIT ACK / DVP STREAMER INTERFACE
    // ============================================================
    output reg        ack_trigger,   // FIXED: Declared safely as output reg
    input wire        dvp_active,
    input wire        ack_busy,
    input wire [7:0]  ack_tx_data,
    input wire        ack_tx_start,
    output reg        ack_tx_busy,

    // ============================================================
    // DVP DEBUG REQUEST (Kept for port compatibility with top level)
    // ============================================================
    input wire        dvp_start,

    // ============================================================
    // I2C DEBUG INTERFACE
    // ============================================================
    input wire        debug_valid,
    output reg        debug_ready,
    input wire [2:0]  debug_type,
    input wire [7:0]  debug_addr,
    input wire [7:0]  debug_data,
    input wire        debug_ack
);

    localparam TOTAL_PIXELS = FRAME_W * FRAME_H;
    localparam [15:0] FRAME_WIDTH_VALUE  = FRAME_W;
    localparam [15:0] FRAME_HEIGHT_VALUE = FRAME_H;

    // ============================================================
    // STATE MACHINE
    // ============================================================
    localparam IDLE       = 5'd0;
    localparam BB0        = 5'd1;
    localparam BB1        = 5'd2;
    localparam LEN0       = 5'd3;
    localparam LEN1       = 5'd4;
    localparam TEXT_PASS  = 5'd5;
    localparam HDR0       = 5'd6;
    localparam HDR1       = 5'd7;
    localparam HDR2       = 5'd8;
    localparam HDR3       = 5'd9;
    localparam HDR4       = 5'd10;
    localparam HDR5       = 5'd11;
    
    // Image transmission (Binary Bytes)
    localparam RD_SETUP   = 5'd12;
    localparam SEND_BYTE0 = 5'd13;
    localparam SEND_BYTE1 = 5'd14;
    localparam NEXT_PIXEL = 5'd15;
    
    localparam WAIT_TX    = 5'd16;
    localparam DBG_LOAD   = 5'd17;
    localparam DBG_SEND   = 5'd18;
    localparam DBG_WAIT   = 5'd19;
    localparam DVP_PASS   = 5'd20;

    reg [4:0] state;
    reg [4:0] ret_state;

    reg [AW:0] px_cnt;
    reg [15:0] px_data_latched;

    reg [7:0] tx_data_reg;
    reg       tx_start_reg;

    reg start_d;
    wire start_pulse = start & ~start_d;

    reg [7:0] packet [0:5];
    reg [2:0] byte_index;
    reg       pending_frame;

    // ============================================================
    // UART MULTIPLEXER
    // ============================================================
    always @(*) begin
        if ((state == TEXT_PASS) ||
            (state == DVP_PASS)  ||
            ((state == IDLE) && ack_busy)) begin

            tx_data     = ack_tx_data;
            tx_start    = ack_tx_start;
            ack_tx_busy = tx_busy;
        end
        else begin
            tx_data     = tx_data_reg;
            tx_start    = tx_start_reg;
            ack_tx_busy = 1'b1;
        end
    end

    // ============================================================
    // MAIN STATE MACHINE
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= IDLE;
            ret_state       <= IDLE;
            busy            <= 1'b0;
            tx_data_reg     <= 8'd0;
            tx_start_reg    <= 1'b0;
            fb_raddr        <= {AW{1'b0}};
            px_cnt          <= {(AW+1){1'b0}};
            px_data_latched <= 16'd0;
            start_d         <= 1'b0;
            ack_trigger     <= 1'b0;
            pending_frame   <= 1'b0;
            debug_ready     <= 1'b1;
            byte_index      <= 3'd0;
            packet[0]       <= 8'h00;
            packet[1]       <= 8'h00;
            packet[2]       <= 8'h00;
            packet[3]       <= 8'h00;
            packet[4]       <= 8'h00;
            packet[5]       <= 8'h00;
        end
        else begin
            tx_start_reg <= 1'b0;
            ack_trigger  <= 1'b0;
            start_d      <= start;

            case (state)

                IDLE: begin
                    busy        <= 1'b0;
                    debug_ready <= 1'b1;

                    if (start_pulse && dvp_active)
                        pending_frame <= 1'b1;

                    if (dvp_active) begin
                        debug_ready <= 1'b0;
                        state       <= DVP_PASS;
                    end
                    else if (debug_valid) begin
                        packet[0] <= 8'hBB;
                        packet[1] <= 8'h66;
                        packet[2] <= {5'd0, debug_type};
                        packet[3] <= debug_addr;
                        packet[4] <= debug_data;
                        packet[5] <= {7'd0, debug_ack};
                        byte_index  <= 3'd0;
                        debug_ready <= 1'b0;
                        state       <= DBG_LOAD;
                    end
                    else if (pending_frame) begin
                        pending_frame <= 1'b0;
                        busy          <= 1'b1;
                        fb_raddr      <= {AW{1'b0}};
                        px_cnt        <= {(AW+1){1'b0}};
                        ack_trigger   <= 1'b1;
                        state         <= BB0;
                    end
                    else if (start_pulse) begin
                        busy          <= 1'b1;
                        fb_raddr      <= {AW{1'b0}};
                        px_cnt        <= {(AW+1){1'b0}};
                        ack_trigger   <= 1'b1;
                        state         <= BB0;
                    end
                end

                DVP_PASS: begin
                    busy        <= 1'b0;
                    debug_ready <= 1'b0;

                    if (start_pulse)
                        pending_frame <= 1'b1;

                    if (!dvp_active) begin
                        debug_ready <= 1'b1;
                        state       <= IDLE;
                    end
                end

                DBG_LOAD: begin
                    tx_data_reg <= packet[byte_index];
                    state       <= DBG_SEND;
                end

                DBG_SEND: begin
                    if (!tx_busy) begin
                        tx_start_reg <= 1'b1;
                        state        <= DBG_WAIT;
                    end
                end

                DBG_WAIT: begin
                    if (!tx_busy) begin
                        if (byte_index == 3'd5) begin
                            byte_index  <= 3'd0;
                            debug_ready <= 1'b1;
                            state       <= IDLE;
                        end
                        else begin
                            byte_index <= byte_index + 1'b1;
                            state      <= DBG_LOAD;
                        end
                    end
                end

                BB0: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= 8'hBB;
                        tx_start_reg <= 1'b1;
                        ret_state    <= BB1;
                        state        <= WAIT_TX;
                    end
                end

                BB1: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= 8'h66;
                        tx_start_reg <= 1'b1;
                        ret_state    <= LEN0;
                        state        <= WAIT_TX;
                    end
                end

                LEN0: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= 8'h00;
                        tx_start_reg <= 1'b1;
                        ret_state    <= LEN1;
                        state        <= WAIT_TX;
                    end
                end

                LEN1: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= 8'h0F;
                        tx_start_reg <= 1'b1;
                        ret_state    <= TEXT_PASS;
                        state        <= WAIT_TX;
                    end
                end

                TEXT_PASS: begin
                    if (!ack_busy)
                        state <= HDR0;
                end

                HDR0: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= 8'hAA;
                        tx_start_reg <= 1'b1;
                        ret_state    <= HDR1;
                        state        <= WAIT_TX;
                    end
                end

                HDR1: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= 8'h55;
                        tx_start_reg <= 1'b1;
                        ret_state    <= HDR2;
                        state        <= WAIT_TX;
                    end
                end

                HDR2: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= FRAME_WIDTH_VALUE[15:8];
                        tx_start_reg <= 1'b1;
                        ret_state    <= HDR3;
                        state        <= WAIT_TX;
                    end
                end

                HDR3: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= FRAME_WIDTH_VALUE[7:0];
                        tx_start_reg <= 1'b1;
                        ret_state    <= HDR4;
                        state        <= WAIT_TX;
                    end
                end

                HDR4: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= FRAME_HEIGHT_VALUE[15:8];
                        tx_start_reg <= 1'b1;
                        ret_state    <= HDR5;
                        state        <= WAIT_TX;
                    end
                end

                HDR5: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= FRAME_HEIGHT_VALUE[7:0];
                        tx_start_reg <= 1'b1;
                        ret_state    <= RD_SETUP;
                        state        <= WAIT_TX;
                    end
                end

                // =================================================
                // FRAMEBUFFER DATA (RAW BINARY)
                // =================================================
                RD_SETUP: begin
                    state <= SEND_BYTE0;
                end

                SEND_BYTE0: begin
                    px_data_latched <= fb_rdata;
                    if (!tx_busy) begin
                        tx_data_reg  <= fb_rdata[15:8]; // Send raw High Byte
                        tx_start_reg <= 1'b1;
                        ret_state    <= SEND_BYTE1;
                        state        <= WAIT_TX;
                    end
                end

                SEND_BYTE1: begin
                    if (!tx_busy) begin
                        tx_data_reg  <= px_data_latched[7:0]; // Send raw Low Byte
                        tx_start_reg <= 1'b1;
                        ret_state    <= NEXT_PIXEL;
                        state        <= WAIT_TX;
                    end
                end

                NEXT_PIXEL: begin
                    if (px_cnt == TOTAL_PIXELS - 1) begin
                        busy  <= 1'b0;
                        state <= IDLE;
                    end
                    else begin
                        px_cnt   <= px_cnt + 1'b1;
                        fb_raddr <= fb_raddr + 1'b1;
                        state    <= RD_SETUP;
                    end
                end

                WAIT_TX: begin
                    state <= ret_state;
                end

                default: begin
                    state       <= IDLE;
                    busy        <= 1'b0;
                    debug_ready <= 1'b1;
                end

            endcase
        end
    end

endmodule