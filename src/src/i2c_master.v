module i2c_master #(
    parameter CLK_FREQ   = 27_000_000,
    parameter I2C_FREQ   = 100_000,
    parameter SLAVE_ADDR = 8'h60
)(
    input  wire clk,
    input  wire rst_n,

    output reg        scl,
    inout  wire       sda,

    // Transaction command
    input  wire       start,
    input  wire       read,       // 0 = WRITE, 1 = READ
    input  wire [7:0] addr,
    input  wire [7:0] data,

    output reg        busy,
    output reg        done,
    output reg [7:0]  read_data,

    // Error
    output reg        camera_error,

    // Debug interface
    output reg        debug_valid,
    input  wire        debug_ready,

    output reg [2:0]  debug_type,
    output reg [7:0]  debug_addr,
    output reg [7:0]  debug_data,
    output reg        debug_ack
);

    // ============================================================
    // I2C CLOCK
    // ============================================================

    localparam integer DIVIDER =
        CLK_FREQ / (I2C_FREQ * 2);

    reg [31:0] clk_count;

    wire tick = (clk_count >= DIVIDER - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            clk_count <= 32'd0;
        else if (tick)
            clk_count <= 32'd0;
        else
            clk_count <= clk_count + 1'b1;
    end

    // ============================================================
    // OPEN DRAIN SDA
    // ============================================================

    reg sda_oe;
    reg sda_out;

    assign sda = sda_oe ? sda_out : 1'bz;

    wire sda_in = sda;

    // ============================================================
    // STATES
    // ============================================================

    localparam [5:0]
        ST_IDLE          = 6'd0,

        ST_START_1       = 6'd1,
        ST_START_2       = 6'd2,

        ST_WR_ADDR_SETUP = 6'd3,
        ST_WR_ADDR_HIGH  = 6'd4,
        ST_WR_ADDR_LOW   = 6'd5,
        ST_WR_ADDR_ACK_H = 6'd6,
        ST_WR_ADDR_ACK_L = 6'd7,

        ST_REG_SETUP     = 6'd8,
        ST_REG_HIGH      = 6'd9,
        ST_REG_LOW       = 6'd10,
        ST_REG_ACK_H     = 6'd11,
        ST_REG_ACK_L     = 6'd12,

        ST_WR_DATA_SETUP = 6'd13,
        ST_WR_DATA_HIGH  = 6'd14,
        ST_WR_DATA_LOW   = 6'd15,
        ST_WR_DATA_ACK_H = 6'd16,
        ST_WR_DATA_ACK_L = 6'd17,

        ST_RESTART_1     = 6'd18,
        ST_RESTART_2     = 6'd19,

        ST_RD_ADDR_SETUP = 6'd20,
        ST_RD_ADDR_HIGH  = 6'd21,
        ST_RD_ADDR_LOW   = 6'd22,
        ST_RD_ADDR_ACK_H = 6'd23,
        ST_RD_ADDR_ACK_L = 6'd24,

        ST_READ_SETUP    = 6'd25,
        ST_READ_HIGH     = 6'd26,
        ST_READ_LOW      = 6'd27,

        ST_NACK_SETUP    = 6'd28,
        ST_NACK_HIGH     = 6'd29,
        ST_NACK_LOW      = 6'd30,

        ST_DEBUG         = 6'd31,

        ST_STOP_1        = 6'd32,
        ST_STOP_2        = 6'd33,
        ST_STOP_3        = 6'd34,

        ST_DONE          = 6'd35;

    reg [5:0] state;

    // ============================================================
    // REGISTERS
    // ============================================================

    reg [7:0] tx_byte;
    reg [7:0] rx_byte;

    reg [7:0] reg_addr_reg;
    reg [7:0] write_data_reg;

    reg [2:0] bit_index;

    reg slave_write_ack;
    reg register_ack;
    reg write_data_ack;
    reg slave_read_ack;

    reg transaction_read;

    // ============================================================
    // MAIN FSM
    // ============================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            state <= ST_IDLE;

            scl <= 1'b1;

            sda_oe  <= 1'b0;
            sda_out <= 1'b1;

            busy <= 1'b0;
            done <= 1'b0;

            read_data <= 8'h00;

            camera_error <= 1'b0;

            debug_valid <= 1'b0;
            debug_type  <= 3'd0;
            debug_addr  <= 8'h00;
            debug_data  <= 8'h00;
            debug_ack   <= 1'b0;

            tx_byte <= 8'h00;
            rx_byte <= 8'h00;

            reg_addr_reg  <= 8'h00;
            write_data_reg <= 8'h00;

            bit_index <= 3'd7;

            slave_write_ack <= 1'b0;
            register_ack    <= 1'b0;
            write_data_ack  <= 1'b0;
            slave_read_ack  <= 1'b0;

            transaction_read <= 1'b0;
        end

        else begin

            done <= 1'b0;

            // ====================================================
            // IDLE
            // ====================================================

            if ((state == ST_IDLE) && start) begin

                busy <= 1'b1;

                camera_error <= 1'b0;

                transaction_read <= read;

                reg_addr_reg   <= addr;
                write_data_reg <= data;

                rx_byte <= 8'h00;

                slave_write_ack <= 1'b0;
                register_ack    <= 1'b0;
                write_data_ack  <= 1'b0;
                slave_read_ack  <= 1'b0;

                scl <= 1'b1;

                sda_oe  <= 1'b0;
                sda_out <= 1'b1;

                state <= ST_START_1;
            end

            // ====================================================
            // START
            // ====================================================

            else if ((state == ST_START_1) && tick) begin

                sda_oe  <= 1'b1;
                sda_out <= 1'b0;

                state <= ST_START_2;
            end

            else if ((state == ST_START_2) && tick) begin

                scl <= 1'b0;

                tx_byte   <= SLAVE_ADDR;
                bit_index <= 3'd7;

                state <= ST_WR_ADDR_SETUP;
            end

            // ====================================================
            // WRITE SLAVE ADDRESS
            // ====================================================

            else if ((state == ST_WR_ADDR_SETUP) && tick) begin

                scl <= 1'b0;

                sda_oe  <= 1'b1;
                sda_out <= tx_byte[bit_index];

                state <= ST_WR_ADDR_HIGH;
            end

            else if ((state == ST_WR_ADDR_HIGH) && tick) begin

                scl <= 1'b1;

                state <= ST_WR_ADDR_LOW;
            end

            else if ((state == ST_WR_ADDR_LOW) && tick) begin

                scl <= 1'b0;

                if (bit_index != 0) begin
                    bit_index <= bit_index - 1'b1;
                    state <= ST_WR_ADDR_SETUP;
                end
                else begin
                    sda_oe <= 1'b0;
                    state <= ST_WR_ADDR_ACK_H;
                end
            end

            // ====================================================
            // WRITE ADDRESS ACK
            // ====================================================

            else if ((state == ST_WR_ADDR_ACK_H) && tick) begin

                scl <= 1'b1;
                state <= ST_WR_ADDR_ACK_L;
            end

            else if ((state == ST_WR_ADDR_ACK_L) && tick) begin

                scl <= 1'b0;

                if (sda_in == 1'b0) begin

                    slave_write_ack <= 1'b1;

                    tx_byte   <= reg_addr_reg;
                    bit_index <= 3'd7;

                    state <= ST_REG_SETUP;
                end
                else begin

                    slave_write_ack <= 1'b0;
                    camera_error <= 1'b1;

                    state <= ST_STOP_1;
                end
            end

            // ====================================================
            // REGISTER ADDRESS
            // ====================================================

            else if ((state == ST_REG_SETUP) && tick) begin

                scl <= 1'b0;

                sda_oe  <= 1'b1;
                sda_out <= tx_byte[bit_index];

                state <= ST_REG_HIGH;
            end

            else if ((state == ST_REG_HIGH) && tick) begin

                scl <= 1'b1;
                state <= ST_REG_LOW;
            end

            else if ((state == ST_REG_LOW) && tick) begin

                scl <= 1'b0;

                if (bit_index != 0) begin
                    bit_index <= bit_index - 1'b1;
                    state <= ST_REG_SETUP;
                end
                else begin
                    sda_oe <= 1'b0;
                    state <= ST_REG_ACK_H;
                end
            end

            // ====================================================
            // REGISTER ACK
            // ====================================================

            else if ((state == ST_REG_ACK_H) && tick) begin

                scl <= 1'b1;
                state <= ST_REG_ACK_L;
            end

            else if ((state == ST_REG_ACK_L) && tick) begin

                scl <= 1'b0;

                if (sda_in == 1'b0) begin

                    register_ack <= 1'b1;

                    if (transaction_read) begin

                        sda_oe <= 1'b0;
                        state <= ST_RESTART_1;
                    end
                    else begin

                        tx_byte   <= write_data_reg;
                        bit_index <= 3'd7;

                        state <= ST_WR_DATA_SETUP;
                    end
                end
                else begin

                    register_ack <= 1'b0;
                    camera_error <= 1'b1;

                    state <= ST_STOP_1;
                end
            end

            // ====================================================
            // WRITE DATA
            // ====================================================

            else if ((state == ST_WR_DATA_SETUP) && tick) begin

                scl <= 1'b0;

                sda_oe  <= 1'b1;
                sda_out <= tx_byte[bit_index];

                state <= ST_WR_DATA_HIGH;
            end

            else if ((state == ST_WR_DATA_HIGH) && tick) begin

                scl <= 1'b1;
                state <= ST_WR_DATA_LOW;
            end

            else if ((state == ST_WR_DATA_LOW) && tick) begin

                scl <= 1'b0;

                if (bit_index != 0) begin
                    bit_index <= bit_index - 1'b1;
                    state <= ST_WR_DATA_SETUP;
                end
                else begin
                    sda_oe <= 1'b0;
                    state <= ST_WR_DATA_ACK_H;
                end
            end

            // ====================================================
            // WRITE DATA ACK
            // ====================================================

            else if ((state == ST_WR_DATA_ACK_H) && tick) begin

                scl <= 1'b1;
                state <= ST_WR_DATA_ACK_L;
            end

            else if ((state == ST_WR_DATA_ACK_L) && tick) begin

                scl <= 1'b0;

                if (sda_in == 1'b0) begin
                    write_data_ack <= 1'b1;
                    state <= ST_DEBUG;
                end
                else begin
                    write_data_ack <= 1'b0;
                    camera_error <= 1'b1;
                    state <= ST_STOP_1;
                end
            end

            // ====================================================
            // REPEATED START
            // ====================================================

            else if ((state == ST_RESTART_1) && tick) begin

                sda_oe <= 1'b0;
                scl <= 1'b1;

                state <= ST_RESTART_2;
            end

            else if ((state == ST_RESTART_2) && tick) begin

                sda_oe  <= 1'b1;
                sda_out <= 1'b0;

                tx_byte   <= SLAVE_ADDR | 8'h01;
                bit_index <= 3'd7;

                state <= ST_RD_ADDR_SETUP;
            end

            // ====================================================
            // READ SLAVE ADDRESS
            // ====================================================

            else if ((state == ST_RD_ADDR_SETUP) && tick) begin

                scl <= 1'b0;

                sda_oe  <= 1'b1;
                sda_out <= tx_byte[bit_index];

                state <= ST_RD_ADDR_HIGH;
            end

            else if ((state == ST_RD_ADDR_HIGH) && tick) begin

                scl <= 1'b1;
                state <= ST_RD_ADDR_LOW;
            end

            else if ((state == ST_RD_ADDR_LOW) && tick) begin

                scl <= 1'b0;

                if (bit_index != 0) begin
                    bit_index <= bit_index - 1'b1;
                    state <= ST_RD_ADDR_SETUP;
                end
                else begin
                    sda_oe <= 1'b0;
                    state <= ST_RD_ADDR_ACK_H;
                end
            end

            // ====================================================
            // READ ADDRESS ACK
            // ====================================================

            else if ((state == ST_RD_ADDR_ACK_H) && tick) begin

                scl <= 1'b1;
                state <= ST_RD_ADDR_ACK_L;
            end

            else if ((state == ST_RD_ADDR_ACK_L) && tick) begin

                scl <= 1'b0;

                if (sda_in == 1'b0) begin

                    slave_read_ack <= 1'b1;

                    rx_byte   <= 8'h00;
                    bit_index <= 3'd7;

                    sda_oe <= 1'b0;

                    state <= ST_READ_SETUP;
                end
                else begin

                    slave_read_ack <= 1'b0;
                    camera_error <= 1'b1;

                    state <= ST_STOP_1;
                end
            end

            // ====================================================
            // READ DATA
            // ====================================================

            else if ((state == ST_READ_SETUP) && tick) begin

                scl <= 1'b0;
                sda_oe <= 1'b0;

                state <= ST_READ_HIGH;
            end

            else if ((state == ST_READ_HIGH) && tick) begin

                scl <= 1'b1;

                state <= ST_READ_LOW;
            end

            else if ((state == ST_READ_LOW) && tick) begin

                scl <= 1'b0;

                if (bit_index != 0) begin

                    rx_byte[bit_index] <= sda_in;

                    bit_index <= bit_index - 1'b1;

                    state <= ST_READ_SETUP;
                end
                else begin

                    // Capture final bit correctly
                    rx_byte[0] <= sda_in;
                    read_data <= {rx_byte[7:1], sda_in};

                    state <= ST_NACK_SETUP;
                end
            end

            // ====================================================
            // MASTER NACK
            // ====================================================

            else if ((state == ST_NACK_SETUP) && tick) begin

                scl <= 1'b0;

                // Release SDA = NACK
                sda_oe <= 1'b0;

                state <= ST_NACK_HIGH;
            end

            else if ((state == ST_NACK_HIGH) && tick) begin

                scl <= 1'b1;
                state <= ST_NACK_LOW;
            end

            else if ((state == ST_NACK_LOW) && tick) begin

                scl <= 1'b0;
                state <= ST_DEBUG;
            end

            // ====================================================
            // DEBUG
            // ====================================================

            else if (state == ST_DEBUG) begin

                if (!debug_valid) begin

                    debug_type <= transaction_read ? 3'd3 : 3'd1;
                    debug_addr <= reg_addr_reg;
                    debug_data <= transaction_read ? read_data : write_data_reg;

                    if (transaction_read)
                        debug_ack <= slave_read_ack;
                    else
                        debug_ack <= write_data_ack;

                    debug_valid <= 1'b1;
                end

                else if (debug_valid && debug_ready) begin

                    debug_valid <= 1'b0;

                    state <= ST_STOP_1;
                end
            end

            // ====================================================
            // STOP
            // ====================================================

            else if ((state == ST_STOP_1) && tick) begin

                scl <= 1'b0;

                sda_oe  <= 1'b1;
                sda_out <= 1'b0;

                state <= ST_STOP_2;
            end

            else if ((state == ST_STOP_2) && tick) begin

                scl <= 1'b1;

                state <= ST_STOP_3;
            end

            else if ((state == ST_STOP_3) && tick) begin

                sda_oe <= 1'b0;

                state <= ST_DONE;
            end

            // ====================================================
            // DONE
            // ====================================================

            else if ((state == ST_DONE) && tick) begin

                busy <= 1'b0;
                done <= 1'b1;

                state <= ST_IDLE;
            end

        end
    end

endmodule