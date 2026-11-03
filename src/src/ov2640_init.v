`timescale 1ns / 1ps

// ============================================================
// OV2640 init - 800 x 600, RGB565
//
// Pipeline: sensor UXGA 1600x1200 -> DSP window (full frame)
//           -> DSP scaler -> 800x600 output
//   C0/C1 (HSIZE8/VSIZE8)   = 1600 x 1200  (value * 8)
//   51/52+55 (H_SIZE/V_SIZE)= 1600 x 1200  window, (value * 4)
//   5A/5B (OUTW/OUTH)       =  800 x  600  output, (value * 4)
//
// Checked against OV2640 datasheet v1.6, Table 12 (0xFF=00)
// and Table 13 (0xFF=01).
// ============================================================

module ov2640_init #(
    parameter CLK_FREQ = 27_000_000
)(
    input  wire        clk,
    input  wire        rst_n,

    // START
    input  wire        start_init,

    // I2C MASTER INTERFACE
    output reg         i2c_start,
    output reg         i2c_read,
    output reg [7:0]   reg_addr,
    output reg [7:0]   reg_data,

    input  wire        i2c_done,
    input  wire        i2c_error,

    // STATUS
    output reg         init_busy,
    output reg         init_done,
    output reg         init_error,
    output reg         init_written
);

    // ============================================================
    // 10 ms DELAY (27 MHz * 10 ms = 270000 clocks)
    // Datasheet: software reset settling < 1 ms,
    //            register setting settling < 300 ms (Table 7)
    // ============================================================

    localparam integer WAIT_10MS = CLK_FREQ / 100;

    reg [31:0] wait_cnt;

    // ============================================================
    // FSM
    // ============================================================

    localparam [3:0]
        S_IDLE     = 4'd0,
        S_WRITE    = 4'd1,
        S_START    = 4'd2,
        S_I2C_WAIT = 4'd3,
        S_DELAY    = 4'd4,
        S_FINISH   = 4'd5,
        S_ERROR    = 4'd6;

    reg [3:0] state;

    // ============================================================
    // REGISTER TABLE  { register, value }
    //
    // 0xFF selects bank:  00 = DSP (Table 12), 01 = SENSOR (Table 13)
    // 16'hFFFF = END OF TABLE
    // ============================================================

    localparam integer ROM_LAST = 66;

    reg [15:0] init_rom [0:66];

    reg [7:0] rom_addr;

    reg delay_after_write;

    initial begin

        // ========================================================
        // SENSOR BANK + SOFTWARE RESET
        // ========================================================

        init_rom[0]  = {8'hFF, 8'h01};   // RA_DLMT: sensor bank
        init_rom[1]  = {8'h12, 8'h80};   // COM7[7] SRST: all regs -> default (10 ms delay)
        init_rom[2]  = {8'hFF, 8'h01};   // re-select sensor bank after reset

        // ========================================================
        // SENSOR CONFIGURATION
        // ========================================================

        init_rom[3]  = {8'h11, 8'h01};   // CLKRC: doubler OFF, CLK = XVCLK/(1+1)
        init_rom[4]  = {8'h09, 8'h02};   // COM2[1:0]=10: 2x output drive
        init_rom[5]  = {8'h04, 8'hE8};   // REG04: bit7 H-MIRROR ON, bit6 V-FLIP ON, bit3 HREF[0]

        // GAIN / AGC / AEC
        //   COM8 = E5: bit5 banding filter ON, bit2 = 1 AGC AUTO,
        //              bit0 = 1 AEC AUTO
        //   GAIN = 00: initial gain 1x (AGC adjusts from here)
        //   COM9 = 28: AGC ceiling 4x (bits 7:5 = 001)
        init_rom[6]  = {8'h13, 8'hE5};   // COM8
        init_rom[7]  = {8'h00, 8'h00};   // GAIN: start at 1x
        init_rom[8]  = {8'h14, 8'h28};   // COM9: AGC ceiling 4x
        init_rom[9]  = {8'h2C, 8'h0C};   // reserved (vendor value)
        init_rom[10] = {8'h33, 8'h78};   // reserved (vendor value)
        init_rom[11] = {8'h3A, 8'h33};   // reserved (vendor value)
        init_rom[12] = {8'h3B, 8'hFB};   // reserved (vendor value)
        init_rom[13] = {8'h3E, 8'h00};   // reserved (vendor value)
        init_rom[14] = {8'h43, 8'h11};   // reserved (vendor value)
        init_rom[15] = {8'h16, 8'h10};   // reserved (vendor value)
        init_rom[16] = {8'h39, 8'h02};   // reserved (vendor value)
        init_rom[17] = {8'h35, 8'h88};   // reserved (vendor value)
        init_rom[18] = {8'h22, 8'h0A};   // reserved (vendor value)
        init_rom[19] = {8'h37, 8'h40};   // reserved (vendor value)
        init_rom[20] = {8'h23, 8'h00};   // reserved (vendor value)
        init_rom[21] = {8'h34, 8'hA0};   // ARCOM2 (only bit2 documented)
        init_rom[22] = {8'h36, 8'h1A};   // reserved (vendor value)
        init_rom[23] = {8'h06, 8'h02};   // reserved (vendor value)
        init_rom[24] = {8'h07, 8'hC0};   // reserved (vendor value)
        init_rom[25] = {8'h0D, 8'hB7};   // COM4 (only bit2 documented)
        init_rom[26] = {8'h0E, 8'h01};   // reserved (vendor value)
        init_rom[27] = {8'h4C, 8'h00};   // reserved (vendor value)
        init_rom[28] = {8'h4A, 8'h81};   // reserved (vendor value)
        init_rom[29] = {8'h21, 8'h99};   // reserved (vendor value)

        // AEC / AGC operating thresholds
        init_rom[30] = {8'h24, 8'h78};   // AEW: upper luminance (lower = darker)
        init_rom[31] = {8'h25, 8'h68};   // AEB: lower luminance
        init_rom[32] = {8'h26, 8'h82};   // VV: fast-mode thresholds

        init_rom[33] = {8'h5C, 8'h00};   // reserved (sensor bank)
        init_rom[34] = {8'h63, 8'h00};   // reserved
        init_rom[35] = {8'h46, 8'h3F};   // FLL: +63 lines per frame
        init_rom[36] = {8'h0C, 8'h3C};   // COM3: bit2=1 -> 50 Hz banding
        init_rom[37] = {8'h61, 8'h70};   // HISTO_LOW
        init_rom[38] = {8'h62, 8'h80};   // HISTO_HIGH

        // COM10 (0x15) - datasheet: "if Bypass DSP is selected".
        // DSP is active here, so this is kept at default polarity.
        init_rom[39] = {8'h15, 8'h00};

        // ========================================================
        // DSP BANK
        // ========================================================

        init_rom[40] = {8'hFF, 8'h00};   // RA_DLMT: DSP bank
        init_rom[41] = {8'h2C, 8'hFF};   // reserved (vendor value, after reset)
        init_rom[42] = {8'h2E, 8'hDF};   // reserved (vendor value, after reset)

        // E0 RESET: bit2 = DVP held in reset (10 ms delay)
        init_rom[43] = {8'hE0, 8'h04};

        init_rom[44] = {8'h05, 8'h00};   // R_BYPASS[0]=0: DSP enabled

        // --------------------------------------------------------
        // DSP input window = UXGA 1600 x 1200
        // HSIZE8 = 0xC8 -> 200*8 = 1600, VSIZE8 = 0x96 -> 150*8 = 1200
        // --------------------------------------------------------
        init_rom[45] = {8'hC0, 8'hC8};   // HSIZE8
        init_rom[46] = {8'hC1, 8'h96};   // VSIZE8
        init_rom[47] = {8'h8C, 8'h00};   // SIZEL: upper/lower size bits = 0

        // CTRL2 = 2D: DCW=1, SDE=0, UV_ADJ=1, UV_AVG=1, CMX=1
        init_rom[48] = {8'h86, 8'h2D};

        // --------------------------------------------------------
        // DSP input window = FULL UXGA frame 1600 x 1200
        // H_SIZE = 0x190 * 4 = 1600 , V_SIZE = 0x12C * 4 = 1200
        // (H_SIZE[8] is in 0x55 bit3, V_SIZE[8] is in 0x55 bit7)
        // --------------------------------------------------------
        init_rom[49] = {8'h50, 8'h00};   // CTRL1: H/V dividers = 0
        init_rom[50] = {8'h51, 8'h90};   // H_SIZE[7:0] (real/4)
        init_rom[51] = {8'h52, 8'h2C};   // V_SIZE[7:0] (real/4)
        init_rom[52] = {8'h53, 8'h00};   // OFFSET_X[7:0]
        init_rom[53] = {8'h54, 8'h00};   // OFFSET_Y[7:0]
        init_rom[54] = {8'h55, 8'h88};   // VHYX: V_SIZE[8]=1, H_SIZE[8]=1, offsets[10:8]=0
        init_rom[55] = {8'h57, 8'h00};   // H_SIZE[9]=0

        // Scaler OUTPUT size (OUTW/OUTH) = 800 x 600
        init_rom[56] = {8'h5A, 8'hC8};   // OUTW[7:0] (real/4) = 800
        init_rom[57] = {8'h5B, 8'h96};   // OUTH[7:0] (real/4) = 600
        init_rom[58] = {8'h5C, 8'h00};   // ZMSPD=0, OUTH[8]=0, OUTW[9:8]=0

        init_rom[59] = {8'hC2, 8'h04};  // CTRL0: RGB_EN=1, YUV_EN=0

        // R_DVP_SP: bit7 auto mode, [6:0] = 2 -> PCLK = 48 MHz / 2
        // (datasheet default 0x82; 0x80 would leave divider = 0)
        init_rom[60] = {8'hD3, 8'h82};

        // IMAGE_MODE: [3:2]=10 -> RGB565, JPEG off, bit0=1 byte swap (low byte first)
        // Use 08 instead if your capture logic already swaps the bytes.
        init_rom[61] = {8'hDA, 8'h09};

        // E0 RESET = 00: release DVP reset (10 ms delay)
        init_rom[62] = {8'hE0, 8'h00};

        // ========================================================
        // NORMAL IMAGE MODE
        // ========================================================

        init_rom[63] = {8'hFF, 8'h01};   // sensor bank
        init_rom[64] = {8'h12, 8'h00};   // COM7: UXGA, color bar OFF, zoom OFF

        // END OF TABLE
        init_rom[65] = 16'hFFFF;
        init_rom[66] = 16'hFFFF;

    end

    // ============================================================
    // FSM
    // ============================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            state    <= S_IDLE;
            rom_addr <= 8'd0;

            i2c_start <= 1'b0;
            i2c_read  <= 1'b0;

            reg_addr <= 8'h00;
            reg_data <= 8'h00;

            init_busy    <= 1'b0;
            init_done    <= 1'b0;
            init_error   <= 1'b0;
            init_written <= 1'b0;

            wait_cnt          <= 32'd0;
            delay_after_write <= 1'b0;

        end

        else begin

            // Default: one-clock I2C start pulse
            i2c_start <= 1'b0;

            case (state)

                // ---------------------------------------------
                S_IDLE: begin

                    init_busy  <= 1'b0;
                    init_done  <= 1'b0;
                    init_error <= 1'b0;

                    if (start_init) begin
                        rom_addr     <= 8'd0;
                        init_busy    <= 1'b1;
                        init_written <= 1'b0;
                        state        <= S_WRITE;
                    end

                end

                // ---------------------------------------------
                // LOAD REGISTER
                // ---------------------------------------------
                S_WRITE: begin

                    if (init_rom[rom_addr] == 16'hFFFF) begin

                        state <= S_FINISH;

                    end

                    else begin

                        reg_addr <= init_rom[rom_addr][15:8];
                        reg_data <= init_rom[rom_addr][7:0];

                        // Table is write-only
                        i2c_read <= 1'b0;

                        // 10 ms delay after: sensor reset, DVP reset, DVP release
                        if ((init_rom[rom_addr][15:8] == 8'h12) &&
                            (init_rom[rom_addr][7:0]  == 8'h80)) begin
                            delay_after_write <= 1'b1;   // COM7 SRST
                        end
                        else if ((init_rom[rom_addr][15:8] == 8'hE0) &&
                                 (init_rom[rom_addr][7:0]  == 8'h04)) begin
                            delay_after_write <= 1'b1;   // RESET: DVP held
                        end
                        else if ((init_rom[rom_addr][15:8] == 8'hE0) &&
                                 (init_rom[rom_addr][7:0]  == 8'h00)) begin
                            delay_after_write <= 1'b1;   // RESET: DVP released
                        end
                        else begin
                            delay_after_write <= 1'b0;
                        end

                        // One clock for register parameters to settle
                        state <= S_START;

                    end

                end

                // ---------------------------------------------
                S_START: begin

                    i2c_start <= 1'b1;
                    state     <= S_I2C_WAIT;

                end

                // ---------------------------------------------
                S_I2C_WAIT: begin

                    if (i2c_error) begin

                        init_error <= 1'b1;
                        init_busy  <= 1'b0;
                        state      <= S_ERROR;

                    end

                    else if (i2c_done) begin

                        if (delay_after_write) begin

                            wait_cnt <= 32'd0;
                            state    <= S_DELAY;

                        end

                        else begin

                            if (rom_addr < ROM_LAST)
                                rom_addr <= rom_addr + 1'b1;

                            state <= S_WRITE;

                        end

                    end

                end

                // ---------------------------------------------
                // 10 ms DELAY
                // ---------------------------------------------
                S_DELAY: begin

                    if (wait_cnt >= WAIT_10MS - 1) begin

                        wait_cnt          <= 32'd0;
                        delay_after_write <= 1'b0;

                        if (rom_addr < ROM_LAST)
                            rom_addr <= rom_addr + 1'b1;

                        state <= S_WRITE;

                    end

                    else begin

                        wait_cnt <= wait_cnt + 1'b1;

                    end

                end

                // ---------------------------------------------
                S_FINISH: begin

                    init_busy    <= 1'b0;
                    init_done    <= 1'b1;
                    init_written <= 1'b1;

                    state <= S_FINISH;

                end

                // ---------------------------------------------
                S_ERROR: begin

                    init_busy  <= 1'b0;
                    init_done  <= 1'b0;
                    init_error <= 1'b1;

                    state <= S_ERROR;

                end

                // ---------------------------------------------
                default: begin

                    state <= S_IDLE;

                end

            endcase

        end

    end

endmodule