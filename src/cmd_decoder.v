module cmd_decoder (
    input wire clk,
    input wire rst_n,
    input wire [7:0] rx_data,
    input wire rx_valid,
    output reg cam_init_trigger, // pulses on CMD_INIT_CAMERA
    output reg cam_capture_trigger, // pulses on CMD_CAPTURE_FRAME
    output reg all_status_read
);

localparam CMD_INIT_CAMERA = 8'h01;
localparam CMD_CAPTURE_FRAME = 8'h02;
localparam CMD_STATUS_REGISTRY_READ = 8'h03;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cam_init_trigger <= 1'b0;
        cam_capture_trigger <= 1'b0;
        all_status_read <= 1'b0;
    end else begin
        cam_init_trigger <= 1'b0; // default: 1-clk pulse
        cam_capture_trigger <= 1'b0;
        all_status_read <= 1'b0;
        if (rx_valid) begin
            case (rx_data)
                CMD_INIT_CAMERA: cam_init_trigger <= 1'b1;
                CMD_CAPTURE_FRAME: cam_capture_trigger <= 1'b1;
                CMD_STATUS_REGISTRY_READ: all_status_read <= 1'b1;
                default: ; // unknown command, ignore
            endcase
        end
    end
end
endmodule