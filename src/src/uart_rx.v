//
// Module: uart_rx
//
// Notes:
// - UART receiver module.
// - Adapted from a reference design to match the port/parameter naming
//   used by top_1 (CLK_FREQ / BAUD, clk / rst_n / rxd / data / data_valid).
// - Core FSM and mid-bit sampling strategy are unchanged from the reference.
//

module uart_rx #(
    parameter CLK_FREQ = 27_000_000,
    parameter BAUD     = 1_000_000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rxd,
    output reg  [7:0] data,
    output wire       data_valid
);

// ---------------------------------------------------------------------------
// Internal parameters.
//

localparam PAYLOAD_BITS = 8;

// Clock cycles per UART bit period.
localparam CYCLES_PER_BIT = CLK_FREQ / BAUD;

// Size of the registers which store sample counts and bit durations.
localparam COUNT_REG_LEN = 1 + $clog2(CYCLES_PER_BIT);

// ---------------------------------------------------------------------------
// Internal registers.
//

// Internally latched value of rxd. Two-flop synchronizer to break long
// timing paths from the input pin into the logic.
reg rxd_reg;
reg rxd_reg_0;

// Storage for the received serial data (shift register).
reg [PAYLOAD_BITS-1:0] received_data;

// Counter for the number of cycles over a packet bit.
reg [COUNT_REG_LEN-1:0] cycle_counter;

// Counter for the number of received bits of the packet.
reg [3:0] bit_counter;

// Sample of the UART input line taken mid-bit-frame.
reg bit_sample;

// Current and next states of the internal FSM.
reg [2:0] fsm_state;
reg [2:0] n_fsm_state;

localparam FSM_IDLE  = 0;
localparam FSM_START = 1;
localparam FSM_RECV  = 2;
localparam FSM_STOP  = 3;

// ---------------------------------------------------------------------------
// Output assignment
//

assign data_valid = fsm_state == FSM_STOP && n_fsm_state == FSM_IDLE;

always @(posedge clk) begin
    if (!rst_n) begin
        data <= {PAYLOAD_BITS{1'b0}};
    end else if (fsm_state == FSM_STOP) begin
        data <= received_data;
    end
end

// ---------------------------------------------------------------------------
// FSM next state selection.
//

wire next_bit     = cycle_counter == CYCLES_PER_BIT ||
                        fsm_state       == FSM_STOP &&
                        cycle_counter   == CYCLES_PER_BIT/2;
wire payload_done = bit_counter == PAYLOAD_BITS;

//
// Handle picking the next state.
always @(*) begin : p_n_fsm_state
    case (fsm_state)
        FSM_IDLE : n_fsm_state = rxd_reg      ? FSM_IDLE : FSM_START;
        FSM_START: n_fsm_state = next_bit     ? FSM_RECV : FSM_START;
        FSM_RECV : n_fsm_state = payload_done ? FSM_STOP : FSM_RECV;
        FSM_STOP : n_fsm_state = next_bit     ? FSM_IDLE : FSM_STOP;
        default  : n_fsm_state = FSM_IDLE;
    endcase
end

// ---------------------------------------------------------------------------
// Internal register setting and re-setting.
//

//
// Handle updates to the received data register.
integer i;
always @(posedge clk) begin : p_received_data
    if (!rst_n) begin
        received_data <= {PAYLOAD_BITS{1'b0}};
    end else if (fsm_state == FSM_IDLE) begin
        received_data <= {PAYLOAD_BITS{1'b0}};
    end else if (fsm_state == FSM_RECV && next_bit) begin
        received_data[PAYLOAD_BITS-1] <= bit_sample;
        for (i = PAYLOAD_BITS-2; i >= 0; i = i - 1) begin
            received_data[i] <= received_data[i+1];
        end
    end
end

//
// Increments the bit counter when receiving.
always @(posedge clk) begin : p_bit_counter
    if (!rst_n) begin
        bit_counter <= 4'b0;
    end else if (fsm_state != FSM_RECV) begin
        bit_counter <= {COUNT_REG_LEN{1'b0}};
    end else if (fsm_state == FSM_RECV && next_bit) begin
        bit_counter <= bit_counter + 1'b1;
    end
end

//
// Sample the received bit when in the middle of a bit frame.
always @(posedge clk) begin : p_bit_sample
    if (!rst_n) begin
        bit_sample <= 1'b0;
    end else if (cycle_counter == CYCLES_PER_BIT/2) begin
        bit_sample <= rxd_reg;
    end
end

//
// Increments the cycle counter when receiving.
always @(posedge clk) begin : p_cycle_counter
    if (!rst_n) begin
        cycle_counter <= {COUNT_REG_LEN{1'b0}};
    end else if (next_bit) begin
        cycle_counter <= {COUNT_REG_LEN{1'b0}};
    end else if (fsm_state == FSM_START ||
                 fsm_state == FSM_RECV  ||
                 fsm_state == FSM_STOP) begin
        cycle_counter <= cycle_counter + 1'b1;
    end
end

//
// Progresses the next FSM state.
always @(posedge clk) begin : p_fsm_state
    if (!rst_n) begin
        fsm_state <= FSM_IDLE;
    end else begin
        fsm_state <= n_fsm_state;
    end
end

//
// Responsible for updating the internal value of rxd_reg.
// (uart_rx_en removed — top_1 doesn't drive an enable pin, so this
// always samples.)
always @(posedge clk) begin : p_rxd_reg
    if (!rst_n) begin
        rxd_reg   <= 1'b1;
        rxd_reg_0 <= 1'b1;
    end else begin
        rxd_reg   <= rxd_reg_0;
        rxd_reg_0 <= rxd;
    end
end

endmodule