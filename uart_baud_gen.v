// =============================================================================
// Module      : uart_baud_gen
// Description : Configurable Baud Rate Generator for UART
//               Generates a tick pulse at 16x oversampling rate
//               tick_rate = clk_freq / (baud_rate * 16)
// Parameters  :
//   CLK_FREQ  - System clock frequency in Hz (default: 50 MHz)
//   BAUD_RATE - UART baud rate (default: 115200)
// =============================================================================

module uart_baud_gen #(
    parameter CLK_FREQ  = 50_000_000,   // System clock: 50 MHz
    parameter BAUD_RATE = 115_200       // Default baud rate
)(
    input  wire clk,
    input  wire rst_n,          // Active-low synchronous reset
    output reg  baud_tick       // High for 1 clk cycle at 16x baud rate
);

    // Calculated divisor: how many clock cycles per baud tick
    localparam integer DIVISOR = CLK_FREQ / (BAUD_RATE * 16);

    // Counter width: ceil(log2(DIVISOR))
    localparam integer CTR_WIDTH = $clog2(DIVISOR);

    reg [CTR_WIDTH-1:0] counter;

    always @(posedge clk) begin
        if (!rst_n) begin
            counter   <= 0;
            baud_tick <= 1'b0;
        end else begin
            if (counter == DIVISOR - 1) begin
                counter   <= 0;
                baud_tick <= 1'b1;
            end else begin
                counter   <= counter + 1;
                baud_tick <= 1'b0;
            end
        end
    end

endmodule
