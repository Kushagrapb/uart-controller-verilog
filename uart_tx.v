// =============================================================================
// Module      : uart_tx
// Description : UART Transmitter
//               - Configurable data bits (5-9), parity (none/even/odd),
//                 and stop bits (1 or 2)
//               - 16x oversampling baud tick input
//               - Frame: [START | DATA | PARITY (opt) | STOP]
//
// Parameters:
//   DATA_BITS  - Number of data bits: 5, 6, 7, 8, or 9 (default: 8)
//   PARITY     - 0: None, 1: Odd, 2: Even             (default: 0)
//   STOP_BITS  - Number of stop bits: 1 or 2           (default: 1)
// =============================================================================

module uart_tx #(
    parameter DATA_BITS = 8,    // Data bits per frame
    parameter PARITY    = 0,    // 0=None, 1=Odd, 2=Even
    parameter STOP_BITS = 1     // 1 or 2 stop bits
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             baud_tick,      // 16x oversampled baud tick
    input  wire             tx_start,       // Pulse high to start transmission
    input  wire [DATA_BITS-1:0] tx_data,   // Data to transmit
    output reg              tx,             // UART TX line (idle = 1)
    output reg              tx_busy,        // High while transmitting
    output reg              tx_done         // Pulses high when frame complete
);

    // FSM States
    localparam [2:0]
        IDLE    = 3'd0,
        START   = 3'd1,
        DATA    = 3'd2,
        PARITY_BIT = 3'd3,
        STOP    = 3'd4;

    reg [2:0]           state, next_state;
    reg [3:0]           tick_cnt;       // Counts to 15 (one baud period)
    reg [3:0]           bit_cnt;        // Bit index counter
    reg [DATA_BITS-1:0] shift_reg;      // Shift register for TX
    reg                 parity_bit_val; // Computed parity
    reg [1:0]           stop_cnt;       // Stop bit counter

    // -----------------------------------------------------------------------
    // Parity computation (combinational)
    // -----------------------------------------------------------------------
    function automatic parity_calc;
        input [DATA_BITS-1:0] data;
        input [1:0]           parity_type; // 1=Odd, 2=Even
        integer i;
        reg p;
        begin
            p = 1'b0;
            for (i = 0; i < DATA_BITS; i = i + 1)
                p = p ^ data[i];
            if (parity_type == 1)       // Odd parity: invert
                parity_calc = ~p;
            else                        // Even parity
                parity_calc = p;
        end
    endfunction

    // -----------------------------------------------------------------------
    // Sequential state register
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // -----------------------------------------------------------------------
    // Datapath & control
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            tx           <= 1'b1;
            tx_busy      <= 1'b0;
            tx_done      <= 1'b0;
            tick_cnt     <= 4'd0;
            bit_cnt      <= 4'd0;
            shift_reg    <= 0;
            parity_bit_val <= 1'b0;
            stop_cnt     <= 2'd0;
            next_state   <= IDLE;
        end else begin
            tx_done <= 1'b0; // Default: de-assert

            case (state)
                // -----------------------------------------------------------
                IDLE: begin
                    tx      <= 1'b1;    // Line idle high
                    tx_busy <= 1'b0;
                    tick_cnt <= 4'd0;

                    if (tx_start) begin
                        shift_reg      <= tx_data;
                        parity_bit_val <= (PARITY != 0) ? parity_calc(tx_data, PARITY) : 1'b0;
                        tx_busy        <= 1'b1;
                        next_state     <= START;
                    end
                end

                // -----------------------------------------------------------
                START: begin
                    tx <= 1'b0;         // Start bit = logic 0
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;
                        if (tick_cnt == 4'd15) begin
                            tick_cnt   <= 4'd0;
                            bit_cnt    <= 4'd0;
                            next_state <= DATA;
                        end
                    end
                end

                // -----------------------------------------------------------
                DATA: begin
                    tx <= shift_reg[0]; // LSB first
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;
                        if (tick_cnt == 4'd15) begin
                            tick_cnt  <= 4'd0;
                            shift_reg <= shift_reg >> 1;
                            bit_cnt   <= bit_cnt + 1;

                            if (bit_cnt == DATA_BITS - 1) begin
                                if (PARITY != 0)
                                    next_state <= PARITY_BIT;
                                else begin
                                    stop_cnt   <= 2'd0;
                                    next_state <= STOP;
                                end
                            end
                        end
                    end
                end

                // -----------------------------------------------------------
                PARITY_BIT: begin
                    tx <= parity_bit_val;
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;
                        if (tick_cnt == 4'd15) begin
                            tick_cnt   <= 4'd0;
                            stop_cnt   <= 2'd0;
                            next_state <= STOP;
                        end
                    end
                end

                // -----------------------------------------------------------
                STOP: begin
                    tx <= 1'b1;         // Stop bit = logic 1
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            stop_cnt <= stop_cnt + 1;
                            if (stop_cnt == STOP_BITS - 1) begin
                                tx_done    <= 1'b1;
                                next_state <= IDLE;
                            end
                        end
                    end
                end

                default: next_state <= IDLE;
            endcase
        end
    end

endmodule
