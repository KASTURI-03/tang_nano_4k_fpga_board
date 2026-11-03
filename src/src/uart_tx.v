module uart_tx #(
    parameter CLK_FREQ = 27_000_000,
    parameter BAUD = 1_000_000 // start here; drop to 921600 if unstable
)(
    input wire clk,
    input wire rst_n,
    input wire [7:0] tx_data,
    input wire tx_start, // 1-clk pulse to send tx_data
    output reg tx_busy,
    output reg txd
);

localparam CYCLES_PER_BIT = CLK_FREQ / BAUD;
localparam IDLE = 0, START = 1, DATA = 2, STOP = 3;

reg [1:0] state;
reg [15:0] clk_cnt;
reg [2:0] bit_idx;
reg [7:0] shift;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        txd <= 1'b1;
        tx_busy <= 1'b0;
        clk_cnt <= 0;
        bit_idx <= 0;
    end else begin
        case (state)
            IDLE: begin
                txd <= 1'b1;
                if (tx_start) begin
                    shift <= tx_data;
                    tx_busy <= 1'b1;
                    clk_cnt <= 0;
                    state <= START;
                end else
                    tx_busy <= 1'b0;
            end
            START: begin
                txd <= 1'b0; // start bit
                if (clk_cnt == CYCLES_PER_BIT-1) begin
                    clk_cnt <= 0;
                    bit_idx <= 0;
                    state <= DATA;
                end else
                    clk_cnt <= clk_cnt + 1'b1;
            end
            DATA: begin
                txd <= shift[bit_idx];
                if (clk_cnt == CYCLES_PER_BIT-1) begin
                    clk_cnt <= 0;
                    if (bit_idx == 7)
                        state <= STOP;
                    else
                        bit_idx <= bit_idx + 1'b1;
                end else
                    clk_cnt <= clk_cnt + 1'b1;
            end
            STOP: begin
                txd <= 1'b1; // stop bit
                if (clk_cnt == CYCLES_PER_BIT-1) begin
                    clk_cnt <= 0;
                    tx_busy <= 1'b0;
                    state <= IDLE;
                end else
                    clk_cnt <= clk_cnt + 1'b1;
            end
            default: state <= IDLE;
        endcase
    end
end
endmodule