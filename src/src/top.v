`timescale 1ns / 1ps

module top_1 #(
    parameter CLK_FREQ   = 27_000_000,
    parameter I2C_FREQ   = 100_000,
    parameter SLAVE_ADDR = 8'h60
)(
    input wire        clk_27mhz,

    input wire        cam_pclk,
    input wire        cam_vsync,
    input wire        cam_href,
    input wire [7:0]  cam_data,

    output wire       cam_xclk,
    inout wire        cam_scl,
    inout wire        cam_sda,
    output wire       cam_rst_n,
    output wire       cam_pwdn,

    output wire       cam_pwdn_flg,
    output wire       cam_rst_flg,

    output wire       led,

    input wire        uart_rxd,
    output wire       uart_txd
);

    // ============================================================
    // RESET / PLL
    // ============================================================
    wire clk_24mhz;
    wire pll_locked;
    wire rst_n;

    assign rst_n = pll_locked;

    Gowin_PLLVR_pxl u_pll (
        .clkin   (clk_27mhz),
        .clkout  (clk_24mhz),
        .clkoutd (),
        .lock    (pll_locked)
    );

    assign cam_xclk = clk_24mhz;

    // ============================================================
    // SYSTEM STATUS
    // ============================================================
    wire system_init_done;
    wire camera_error;
    wire camera_reset_done;

    wire i2c_read_done;
    wire i2c_written;
    wire i2c_read_back;
    wire i2c_write_done;

    wire camera_init_done;

    // ============================================================
    // CAMERA INITIALIZATION STATUS
    // ============================================================
    wire        init_busy;
    wire        init_done;
    wire        init_error;
    wire        init_written;

    // ============================================================
    // CAMERA POWER / RESET SEQUENCE
    // ============================================================
    reg [15:0] cam_reset_cnt;
    reg        cam_reset_done;
    reg        cam_rst_n_reg;

    always @(posedge clk_27mhz) begin

        if (!pll_locked) begin

            cam_reset_cnt  <= 16'd0;
            cam_reset_done <= 1'b0;
            cam_rst_n_reg  <= 1'b0;

        end

        else if (!cam_reset_done) begin

            // Hold camera reset for approximately 1.21 ms
            if (cam_reset_cnt == 16'd32767) begin

                cam_reset_cnt  <= 16'd0;
                cam_rst_n_reg  <= 1'b1;
                cam_reset_done <= 1'b1;

            end

            else begin

                cam_reset_cnt <= cam_reset_cnt + 1'b1;

            end

        end

    end

    assign cam_rst_n = cam_rst_n_reg;
    assign cam_pwdn  = 1'b0;

    assign cam_rst_flg  = cam_rst_n;
    assign cam_pwdn_flg = cam_pwdn;

    assign camera_reset_done = cam_reset_done;

    // ============================================================
    // CAMERA ID
    // ============================================================
    wire camera_id_ok;

    reg       id_i2c_start;
    reg       id_i2c_read;
    reg [7:0] id_i2c_addr;
    reg [7:0] id_i2c_data;

    wire       init_i2c_start;
    wire       init_i2c_read;
    wire [7:0] init_i2c_addr;
    wire [7:0] init_i2c_data;

    reg [7:0] pidh_reg;
    reg [7:0] pidl_reg;
    reg [7:0] midh_reg;
    reg [7:0] midl_reg;

    localparam [3:0]
        ID_START       = 4'd0,
        ID_BANK_WAIT   = 4'd1,
        ID_PIDH_START  = 4'd2,
        ID_PIDH_WAIT   = 4'd3,
        ID_PIDL_START  = 4'd4,
        ID_PIDL_WAIT   = 4'd5,
        ID_MIDH_START  = 4'd6,
        ID_MIDH_WAIT   = 4'd7,
        ID_MIDL_START  = 4'd8,
        ID_MIDL_WAIT   = 4'd9,
        ID_CHECK       = 4'd10,
        ID_DONE        = 4'd11,
        ID_ERROR       = 4'd12;

    reg [3:0] id_state;
    reg       id_ok_reg;
    reg       i2c_read_done_reg;

    assign camera_id_ok = id_ok_reg;

    // ============================================================
    // I2C SIGNALS
    // ============================================================
    wire       i2c_start;
    wire       i2c_read;
    wire [7:0] i2c_addr;
    wire [7:0] i2c_data;

    wire       i2c_busy;
    wire       i2c_done;
    wire [7:0] i2c_read_data;
    wire       i2c_error;

    // ============================================================
    // CAMERA ID FSM
    // ============================================================
    always @(posedge clk_27mhz or negedge rst_n) begin

        if (!rst_n) begin

            id_state <= ID_START;

            id_i2c_start <= 1'b0;
            id_i2c_read  <= 1'b0;
            id_i2c_addr  <= 8'h00;
            id_i2c_data  <= 8'h00;

            pidh_reg <= 8'h00;
            pidl_reg <= 8'h00;
            midh_reg <= 8'h00;
            midl_reg <= 8'h00;

            id_ok_reg <= 1'b0;
            i2c_read_done_reg <= 1'b0;

        end

        else begin

            id_i2c_start <= 1'b0;

            case (id_state)

                // =================================================
                // START
                // =================================================
                ID_START: begin

                   if (camera_reset_done && !i2c_busy) begin

                        id_i2c_read  <= 1'b0;
                        id_i2c_addr  <= 8'hFF;
                        id_i2c_data  <= 8'h01;

                        id_i2c_start <= 1'b1;

                        id_state <= ID_BANK_WAIT;

                    end

                end

                // =================================================
                // WAIT FOR BANK WRITE
                // =================================================
                ID_BANK_WAIT: begin

                    if (i2c_done) begin

                        if (i2c_error)
                            id_state <= ID_ERROR;
                        else
                            id_state <= ID_PIDH_START;

                    end

                end

                // =================================================
                // PIDH
                // =================================================
                ID_PIDH_START: begin

                    if (!i2c_busy) begin

                        id_i2c_read  <= 1'b1;
                        id_i2c_addr  <= 8'h0A;
                        id_i2c_data  <= 8'h00;

                        id_i2c_start <= 1'b1;

                        id_state <= ID_PIDH_WAIT;

                    end

                end

                ID_PIDH_WAIT: begin

                    if (i2c_done) begin

                        if (i2c_error) begin

                            id_state <= ID_ERROR;

                        end

                        else begin

                            pidh_reg <= i2c_read_data;
                            id_state <= ID_PIDL_START;

                        end

                    end

                end

                // =================================================
                // PIDL
                // =================================================
                ID_PIDL_START: begin

                    if (!i2c_busy) begin

                        id_i2c_read  <= 1'b1;
                        id_i2c_addr  <= 8'h0B;
                        id_i2c_data  <= 8'h00;

                        id_i2c_start <= 1'b1;

                        id_state <= ID_PIDL_WAIT;

                    end

                end

                ID_PIDL_WAIT: begin

                    if (i2c_done) begin

                        if (i2c_error) begin

                            id_state <= ID_ERROR;

                        end

                        else begin

                            pidl_reg <= i2c_read_data;
                            id_state <= ID_MIDH_START;

                        end

                    end

                end

                // =================================================
                // MIDH
                // =================================================
                ID_MIDH_START: begin

                    if (!i2c_busy) begin

                        id_i2c_read  <= 1'b1;
                        id_i2c_addr  <= 8'h1C;
                        id_i2c_data  <= 8'h00;

                        id_i2c_start <= 1'b1;

                        id_state <= ID_MIDH_WAIT;

                    end

                end

                ID_MIDH_WAIT: begin

                    if (i2c_done) begin

                        if (i2c_error) begin

                            id_state <= ID_ERROR;

                        end

                        else begin

                            midh_reg <= i2c_read_data;
                            id_state <= ID_MIDL_START;

                        end

                    end

                end

                // =================================================
                // MIDL
                // =================================================
                ID_MIDL_START: begin

                    if (!i2c_busy) begin

                        id_i2c_read  <= 1'b1;
                        id_i2c_addr  <= 8'h1D;
                        id_i2c_data  <= 8'h00;

                        id_i2c_start <= 1'b1;

                        id_state <= ID_MIDL_WAIT;

                    end

                end

                ID_MIDL_WAIT: begin

                    if (i2c_done) begin

                        if (i2c_error) begin

                            id_state <= ID_ERROR;

                        end

                        else begin

                            midl_reg <= i2c_read_data;
                            id_state <= ID_CHECK;

                        end

                    end

                end

                // =================================================
                // CHECK CAMERA ID
                //
                // Expected OV2640:
                // PIDH = 26
                // PIDL = 42
                // MIDH = 7F
                // MIDL = A2
                // =================================================
                ID_CHECK: begin

                    if ((pidh_reg == 8'h26) &&
                        (pidl_reg == 8'h42) &&
                        (midh_reg == 8'h7F) &&
                        (midl_reg == 8'hA2)) begin

                        id_ok_reg <= 1'b1;
                        i2c_read_done_reg <= 1'b1;

                        id_state <= ID_DONE;

                    end

                    else begin

                        id_ok_reg <= 1'b0;
                        i2c_read_done_reg <= 1'b0;

                        id_state <= ID_ERROR;

                    end

                end

                // =================================================
                // DONE
                // =================================================
                ID_DONE: begin

                    id_ok_reg <= 1'b1;

                end

                // =================================================
                // ERROR
                // =================================================
                ID_ERROR: begin

                    id_ok_reg <= 1'b0;

                end

                default: begin

                    id_state <= ID_START;

                end

            endcase

        end

    end

    // ============================================================
    // I2C COMMAND MUX
    // ============================================================
    assign i2c_start = camera_id_ok ? init_i2c_start : id_i2c_start;
    assign i2c_read  = camera_id_ok ? init_i2c_read  : id_i2c_read;
    assign i2c_addr  = camera_id_ok ? init_i2c_addr  : id_i2c_addr;
    assign i2c_data  = camera_id_ok ? init_i2c_data  : id_i2c_data;

    // ============================================================
    // OV2640 INITIALIZATION
    // ============================================================
    ov2640_init #(
        .CLK_FREQ(CLK_FREQ)
    ) ov2640_init_inst (

        .clk          (clk_27mhz),
        .rst_n        (pll_locked),

        .start_init   (camera_id_ok),

        .i2c_start    (init_i2c_start),
        .i2c_read     (init_i2c_read),
        .reg_addr     (init_i2c_addr),
        .reg_data     (init_i2c_data),

        .i2c_done     (i2c_done),
        .i2c_error    (i2c_error),

        .init_busy    (init_busy),
        .init_done    (init_done),
        .init_error   (init_error),
        .init_written (init_written)
    );

    // ============================================================
    // I2C DEBUG
    // ============================================================
    wire       debug_valid;
    wire       debug_ready;
    wire [2:0] debug_type;
    wire [7:0] debug_addr;
    wire [7:0] debug_data;
    wire       debug_ack;

    // ============================================================
    // I2C MASTER
    // ============================================================
    i2c_master #(
        .CLK_FREQ   (CLK_FREQ),
        .I2C_FREQ   (I2C_FREQ),
        .SLAVE_ADDR (SLAVE_ADDR)
    ) i2c_master_inst (

        .clk          (clk_27mhz),
        .rst_n        (pll_locked),

        .scl          (cam_scl),
        .sda          (cam_sda),

        .start        (i2c_start),
        .read         (i2c_read),
        .addr         (i2c_addr),
        .data         (i2c_data),

        .busy         (i2c_busy),
        .done         (i2c_done),
        .read_data    (i2c_read_data),

        .camera_error (i2c_error),

        .debug_valid  (debug_valid),
        .debug_ready  (debug_ready),
        .debug_type   (debug_type),
        .debug_addr   (debug_addr),
        .debug_data   (debug_data),
        .debug_ack    (debug_ack)
    );

    // ============================================================
    // DVP ACTIVITY DEBUG
    //
    // These are only local activity indicators.
    // They are NOT sent continuously over UART.
    // The detailed DVP values are generated by init_ack_streamer
    // only after UART command 0x03.
    // ============================================================
    reg        dvp_pclk_seen;
    reg        dvp_href_seen;
    reg        dvp_vsync_seen;
    reg [15:0] dvp_frame_count;

    always @(posedge cam_pclk or negedge pll_locked) begin

        if (!pll_locked) begin

            dvp_pclk_seen   <= 1'b0;
            dvp_href_seen   <= 1'b0;
            dvp_vsync_seen  <= 1'b0;
            dvp_frame_count <= 16'd0;

        end

        else begin

            dvp_pclk_seen <= 1'b1;

            if (cam_href)
                dvp_href_seen <= 1'b1;

            if (cam_vsync) begin

                dvp_vsync_seen <= 1'b1;
                dvp_frame_count <= dvp_frame_count + 1'b1;

            end

        end

    end

    // ============================================================
    // UART RX
    // ============================================================
    wire [7:0] rx_data;
    wire       rx_valid;

    wire       cam_init_trigger;
    wire       cam_capture_trigger;
    wire       all_status_read;

    uart_rx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (1_000_000)
    ) u_uart_rx (

        .clk        (clk_27mhz),
        .rst_n      (pll_locked),

        .rxd        (uart_rxd),

        .data       (rx_data),
        .data_valid (rx_valid)
        

    );

    // ============================================================
    // COMMAND DECODER
    // ============================================================
    cmd_decoder u_cmd (

        .clk                  (clk_27mhz),
        .rst_n                (pll_locked),

        .rx_data              (rx_data),
        .rx_valid             (rx_valid),

        .cam_init_trigger     (cam_init_trigger),
        .cam_capture_trigger  (cam_capture_trigger),
        .all_status_read      (all_status_read)

    );

    // ============================================================
    // CAMERA CAPTURE
    // ============================================================
    wire        cap_wr_en;
    wire [16:0] cap_wr_addr;
    wire [15:0] cap_wr_data;
    wire        cap_frame_done;

    // ============================================================
    // CONTINUOUS STREAMING TRIGGER LOGIC
    // ============================================================

    reg init_done_d;
    reg stream_busy_d;

    wire init_done_pulse;
    wire frame_sent_pulse;

    always @(posedge clk_27mhz or negedge pll_locked) begin

        if (!pll_locked) begin

            init_done_d  <= 1'b0;
            stream_busy_d <= 1'b0;

        end

        else begin

            init_done_d   <= init_done;
            stream_busy_d <= streaming_busy;

        end

    end

    assign init_done_pulse  = init_done & ~init_done_d;
    assign frame_sent_pulse = stream_busy_d & ~streaming_busy;
    wire combined_capture_trigger;

    // 0x03 IS NOT INCLUDED HERE.
    //
    // 0x03 is DVP debug only.

    // ============================================================
    // CAPTURE COMMAND CDC
    // clk_27mhz -> cam_pclk
    // ============================================================
    reg capture_cmd_toggle;

    always @(posedge clk_27mhz or negedge pll_locked) begin

        if (!pll_locked)
            capture_cmd_toggle <= 1'b0;

        else if (combined_capture_trigger)
            capture_cmd_toggle <= ~capture_cmd_toggle;

    end

    // ============================================================
    // OV2640 CAPTURE
    // ============================================================
    ov2640_capture #(
        .SRC_W   (800),
        .SRC_H   (600),
        .FRAME_W (96),
        .FRAME_H (96)
    ) u_capture (

        .cam_pclk        (cam_pclk),
        .cam_vsync       (cam_vsync),
        .cam_href        (cam_href),
        .cam_data        (cam_data),
        .rst_n           (pll_locked), 
        .wr_en           (cap_wr_en),
        .wr_addr         (cap_wr_addr),
        .wr_data         (cap_wr_data),

        .capture_trigger (cam_capture_trigger),
        .frame_done      (cap_frame_done)

    );

   // assign combined_capture_trigger = cam_capture_trigger | init_done_pulse | frame_sent_pulse;
    assign combined_capture_trigger = 1'b1;


    // ============================================================
    // FRAME DONE CDC
    // cam_pclk -> clk_27mhz
    // ============================================================
    reg frame_done_toggle;

    always @(posedge cam_pclk or negedge pll_locked) begin

        if (!pll_locked)
            frame_done_toggle <= 1'b0;

        else if (cap_frame_done)
            frame_done_toggle <= ~frame_done_toggle;

    end

    reg [2:0] sync_shift;

    always @(posedge clk_27mhz or negedge pll_locked) begin

        if (!pll_locked)
            sync_shift <= 3'b000;

        else
            sync_shift <= {
                sync_shift[1:0],
                frame_done_toggle
            };

    end

    wire sync_stream_start;

    assign sync_stream_start =
        sync_shift[2] ^ sync_shift[1];

    // ============================================================
    // STREAMING BUSY CDC
    // clk_27mhz -> cam_pclk
    // ============================================================
    reg busy_sync1;
    reg busy_sync2;

    always @(posedge cam_pclk or negedge pll_locked) begin

        if (!pll_locked) begin

            busy_sync1 <= 1'b0;
            busy_sync2 <= 1'b0;

        end

        else begin

            busy_sync1 <= streaming_busy;
            busy_sync2 <= busy_sync1;

        end

    end

    // ============================================================
    // FRAMEBUFFER
    // ============================================================
    wire [16:0] fb_raddr;
    wire [15:0] fb_rdata;

    Gowin_SDPB u_fb (

        // READ DATA
        .dout   (fb_rdata),

        // WRITE PORT
        .clka   (cam_pclk),

        .cea    (cap_wr_en & ~busy_sync2),

        .reseta (1'b0),

        .ada    (cap_wr_addr[16:0]),
        .din    (cap_wr_data),

        // READ PORT
        .clkb   (clk_27mhz),

        .ceb    (1'b1),

        .resetb (1'b0),

        .oce    (1'b1),

        .adb    (fb_raddr)

    );

    // ============================================================
    // UART SIGNALS
    // ============================================================
    wire [7:0] stream_tx_data;
    wire       stream_tx_start;

    wire [7:0] ack_tx_data_wire;
    wire       ack_tx_start_wire;

    wire       ack_busy_wire;
    wire       ack_trigger_wire;
    wire       ack_tx_busy_wire;

    wire       tx_busy;

    wire [7:0] tx_data;
    wire       tx_start;

    wire       streaming_busy;
    wire dvp_active_wire;
    // ============================================================
    // INIT ACK + DVP DEBUG STREAMER
    //
    // 0x01 / initialization flow:
    //     CAM INIT DONE\r\n
    //
    // 0x03:
    //     BB 66 TXT_LEN TXT_DATA
    //
    // The DVP packet is generated ONLY when
    // all_status_read pulses.
    // ============================================================
init_ack_streamer u_sys_dbg (

    .clk         (clk_27mhz),
    .rst_n       (pll_locked),

    .init_done   (ack_trigger_wire),
    .dvp_trigger (all_status_read),

    .busy        (ack_busy_wire),
    .dvp_active  (dvp_active_wire),

    .tx_data     (ack_tx_data_wire),
    .tx_start    (ack_tx_start_wire),
    .tx_busy     (ack_tx_busy_wire),

    .cam_pclk    (cam_pclk),
    .cam_href    (cam_href),
    .cam_vsync   (cam_vsync)

);

    // ============================================================
    // FRAME STREAMER
    //
    // NOTE:
    // There are NO DVP ports here anymore.
    //
    // DVP data is generated by init_ack_streamer and passed
    // through the UART multiplexer.
    // ============================================================

frame_streamer #(
    .FRAME_W(96),
    .FRAME_H(96),
    .AW(14)
) u_streamer (

    .clk          (clk_27mhz),
    .rst_n        (pll_locked),

    .start        (sync_stream_start),
    .busy         (streaming_busy),

    .fb_raddr     (fb_raddr),
    .fb_rdata     (fb_rdata),

    .tx_data      (stream_tx_data),
    .tx_start     (stream_tx_start),
    .tx_busy      (tx_busy),

    .ack_trigger  (ack_trigger_wire),
    .ack_busy     (ack_busy_wire),
    .ack_tx_data  (ack_tx_data_wire),
    .ack_tx_start (ack_tx_start_wire),
    .ack_tx_busy  (ack_tx_busy_wire),

    .dvp_active   (dvp_active_wire),

    .debug_valid  (debug_valid),
    .debug_ready  (debug_ready),
    .debug_type   (debug_type),
    .debug_addr   (debug_addr),
    .debug_data   (debug_data),
    .debug_ack    (debug_ack)

);

    // ============================================================
    // UART DATA SELECTION
    // ============================================================
    assign tx_data  = stream_tx_data;
    assign tx_start = stream_tx_start;

    // ============================================================
    // UART TX
    // ============================================================

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (1_000_000)
    ) u_uart_tx (

        .clk      (clk_27mhz),
        .rst_n    (pll_locked),

        .tx_data  (tx_data),
        .tx_start (tx_start),

        .tx_busy  (tx_busy),
        .txd      (uart_txd)

    );


    // ============================================================
    // CAMERA VSYNC LED DEBUG
    // ============================================================
    reg vsync_d;
    reg vsync_led;

    always @(posedge cam_pclk or negedge pll_locked) begin

        if (!pll_locked) begin

            vsync_d   <= 1'b0;
            vsync_led <= 1'b0;

        end

        else begin

            vsync_d <= cam_vsync;

            if (cam_vsync && !vsync_d)
                vsync_led <= ~vsync_led;

        end

    end


reg [7:0] dbg_status_count = 0;
always @(posedge clk_27mhz or negedge pll_locked) begin
    if (!pll_locked) dbg_status_count <= 8'd0;
    else if (all_status_read) dbg_status_count <= dbg_status_count + 1'b1;
end

reg [7:0] dbg_any_rx_count;
always @(posedge clk_27mhz or negedge pll_locked) begin
    if (!pll_locked) dbg_any_rx_count <= 8'd0;
    else if (rx_valid) dbg_any_rx_count <= dbg_any_rx_count + 1'b1;
end

    // ============================================================
    // STATUS FLAG ASSIGNMENTS
    // ============================================================
    assign i2c_written    = init_done;
    assign i2c_read_back  = init_done;
    assign i2c_read_done  = i2c_read_done_reg;
    assign i2c_write_done = i2c_written && i2c_read_back;

    assign camera_init_done =
        camera_reset_done &&
        i2c_read_done &&
        i2c_write_done;

    assign system_init_done = camera_init_done;

    assign camera_error =
        i2c_error |
        init_error |
        (id_state == ID_ERROR);

    assign led =system_init_done;

endmodule