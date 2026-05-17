// =============================================================================
// Module      : uart_rx
// Description : UART Receiver with 16x oversampling
//               - Configurable data bits, parity, stop bits
//               - Majority voting on received bits (noise immunity)
//               - Error detection: framing error & parity error flags
//
// Parameters:
//   DATA_BITS  - 5-9 data bits          (default: 8)
//   PARITY     - 0=None, 1=Odd, 2=Even  (default: 0)
//   STOP_BITS  - 1 or 2 stop bits       (default: 1)
// =============================================================================

module uart_rx #(
    parameter DATA_BITS = 8,
    parameter PARITY    = 0,
    parameter STOP_BITS = 1
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             baud_tick,          // 16x oversampled baud tick
    input  wire             rx,                 // UART RX line input
    output reg  [DATA_BITS-1:0] rx_data,        // Received data
    output reg              rx_done,            // Pulses when data is valid
    output reg              parity_error,       // Parity mismatch detected
    output reg              framing_error       // Stop bit not detected
);

    // FSM States
    localparam [2:0]
        IDLE       = 3'd0,
        START      = 3'd1,
        DATA       = 3'd2,
        PARITY_BIT = 3'd3,
        STOP       = 3'd4;

    reg [2:0]            state;
    reg [3:0]            tick_cnt;       // 0-15 tick counter (1 baud period)
    reg [3:0]            bit_cnt;        // Bit position counter
    reg [DATA_BITS-1:0]  shift_reg;      // Incoming bit shift register

    // Majority vote sampling: sample at ticks 7,8,9 (mid-bit)
    reg sample_7, sample_8, sample_9;
    reg voted_bit;

    // RX input synchronizer (2-FF metastability protection)
    reg rx_sync1, rx_sync2;
    always @(posedge clk) begin
        rx_sync1 <= rx;
        rx_sync2 <= rx_sync1;
    end

    // -----------------------------------------------------------------------
    // Majority vote on samples taken at tick 7, 8, 9 (center of bit)
    // -----------------------------------------------------------------------
    always @(*) begin
        voted_bit = (sample_7 & sample_8) |
                    (sample_8 & sample_9) |
                    (sample_7 & sample_9);
    end

    // -----------------------------------------------------------------------
    // Parity check (combinational)
    // -----------------------------------------------------------------------
    function automatic parity_check;
        input [DATA_BITS-1:0] data;
        input                 received_parity;
        input [1:0]           parity_type;
        integer i;
        reg p;
        begin
            p = 1'b0;
            for (i = 0; i < DATA_BITS; i = i + 1)
                p = p ^ data[i];
            if (parity_type == 1)       // Odd: XOR result ^ received should = 1
                parity_check = (p ^ received_parity) ? 1'b0 : 1'b1; // error if same
            else                        // Even: XOR result ^ received should = 0
                parity_check = (p ^ received_parity) ? 1'b1 : 1'b0; // error if diff
        end
    endfunction

    reg received_parity_bit;

    // -----------------------------------------------------------------------
    // Main FSM
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= IDLE;
            tick_cnt       <= 4'd0;
            bit_cnt        <= 4'd0;
            shift_reg      <= 0;
            rx_data        <= 0;
            rx_done        <= 1'b0;
            parity_error   <= 1'b0;
            framing_error  <= 1'b0;
            sample_7       <= 1'b0;
            sample_8       <= 1'b0;
            sample_9       <= 1'b0;
            received_parity_bit <= 1'b0;
        end else begin
            rx_done       <= 1'b0;
            parity_error  <= 1'b0;
            framing_error <= 1'b0;

            case (state)
                // -----------------------------------------------------------
                IDLE: begin
                    if (!rx_sync2) begin    // Falling edge = start bit detected
                        tick_cnt <= 4'd0;
                        state    <= START;
                    end
                end

                // -----------------------------------------------------------
                // Wait to sample at center of start bit (tick 7)
                START: begin
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;
                        if (tick_cnt == 4'd7) begin
                            if (!rx_sync2) begin    // Confirm start bit valid
                                tick_cnt <= 4'd0;
                                bit_cnt  <= 4'd0;
                                state    <= DATA;
                            end else begin
                                state    <= IDLE;   // False start, abort
                            end
                        end
                    end
                end

                // -----------------------------------------------------------
                DATA: begin
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;

                        // Capture samples at ticks 7, 8, 9 for majority vote
                        if (tick_cnt == 4'd7)  sample_7 <= rx_sync2;
                        if (tick_cnt == 4'd8)  sample_8 <= rx_sync2;
                        if (tick_cnt == 4'd9)  sample_9 <= rx_sync2;

                        if (tick_cnt == 4'd15) begin
                            tick_cnt  <= 4'd0;
                            shift_reg <= {voted_bit, shift_reg[DATA_BITS-1:1]}; // LSB first
                            bit_cnt   <= bit_cnt + 1;

                            if (bit_cnt == DATA_BITS - 1) begin
                                if (PARITY != 0)
                                    state <= PARITY_BIT;
                                else
                                    state <= STOP;
                            end
                        end
                    end
                end

                // -----------------------------------------------------------
                PARITY_BIT: begin
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;

                        if (tick_cnt == 4'd7)  sample_7 <= rx_sync2;
                        if (tick_cnt == 4'd8)  sample_8 <= rx_sync2;
                        if (tick_cnt == 4'd9)  sample_9 <= rx_sync2;

                        if (tick_cnt == 4'd15) begin
                            tick_cnt            <= 4'd0;
                            received_parity_bit <= voted_bit;
                            state               <= STOP;
                        end
                    end
                end

                // -----------------------------------------------------------
                STOP: begin
                    if (baud_tick) begin
                        tick_cnt <= tick_cnt + 1;

                        if (tick_cnt == 4'd7)  sample_7 <= rx_sync2;
                        if (tick_cnt == 4'd8)  sample_8 <= rx_sync2;
                        if (tick_cnt == 4'd9)  sample_9 <= rx_sync2;

                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;

                            if (!voted_bit) begin
                                framing_error <= 1'b1;  // Stop bit must be 1
                            end else begin
                                rx_data <= shift_reg;
                                rx_done <= 1'b1;

                                if (PARITY != 0) begin
                                    parity_error <= parity_check(
                                        shift_reg,
                                        received_parity_bit,
                                        PARITY
                                    );
                                end
                            end
                            state <= IDLE;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
