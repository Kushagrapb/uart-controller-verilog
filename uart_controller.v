// =============================================================================
// Module      : uart_controller
// Description : Top-level Configurable UART Communication Controller
//               Integrates: Baud Gen + TX + RX + TX/RX FIFOs
//
// Features:
//   - Full-duplex UART (simultaneous TX and RX)
//   - Configurable: 5-9 data bits, parity (none/odd/even), 1-2 stop bits
//   - 16-deep TX and RX FIFOs
//   - Configurable baud rate via parameter
//   - Status flags: tx_full, rx_empty, rx_error flags
// =============================================================================

module uart_controller #(
    parameter CLK_FREQ   = 50_000_000,
    parameter BAUD_RATE  = 115_200,
    parameter DATA_BITS  = 8,
    parameter PARITY     = 0,
    parameter STOP_BITS  = 1,
    parameter FIFO_DEPTH = 16
)(
    input  wire             clk,
    input  wire             rst_n,

    // Host interface (write side = TX, read side = RX)
    input  wire             tx_wr_en,           // Write data into TX FIFO
    input  wire [DATA_BITS-1:0] tx_wr_data,     // Data to transmit
    output wire             tx_full,            // TX FIFO full
    output wire             tx_empty,           // TX FIFO empty

    input  wire             rx_rd_en,            
    output wire [DATA_BITS-1:0] rx_rd_data,      
    output wire             rx_empty,            
    output wire             rx_full,            

    // Status & Error flags
    output wire             parity_error,       // Parity error from RX
    output wire             framing_error,      // Framing error from RX
    output wire             rx_overflow,        // RX FIFO overflowed

    // UART physical lines
    input  wire             uart_rx,
    output wire             uart_tx
);
    
    wire baud_tick;

    // TX FIFO <-> TX core
    wire [DATA_BITS-1:0] tx_fifo_dout;
    wire                 tx_fifo_empty;
    wire                 tx_rd_en;
    wire                 tx_busy;
    wire                 tx_done;

    // RX core -> RX FIFO
    wire [DATA_BITS-1:0] rx_data_raw;
    wire                 rx_done_raw;
    wire                 rx_parity_err;
    wire                 rx_frame_err;

    
    // Baud Rate Generator

    uart_baud_gen #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_baud_gen (
        .clk       (clk),
        .rst_n     (rst_n),
        .baud_tick (baud_tick)
    );

    // TX FIFO
    
    sync_fifo #(
        .DATA_WIDTH (DATA_BITS),
        .DEPTH      (FIFO_DEPTH)
    ) u_tx_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (tx_wr_en),
        .wr_data (tx_wr_data),
        .rd_en   (tx_rd_en),
        .rd_data (tx_fifo_dout),
        .full    (tx_full),
        .empty   (tx_fifo_empty)
    );

    // TX empty is exposed
    assign tx_empty = tx_fifo_empty;

    // Read from TX FIFO when TX core is free and FIFO has data
    assign tx_rd_en = (!tx_busy) && (!tx_fifo_empty);

    // UART TX Core
    
    uart_tx #(
        .DATA_BITS (DATA_BITS),
        .PARITY    (PARITY),
        .STOP_BITS (STOP_BITS)
    ) u_uart_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .baud_tick (baud_tick),
        .tx_start  (tx_rd_en),
        .tx_data   (tx_fifo_dout),
        .tx        (uart_tx),
        .tx_busy   (tx_busy),
        .tx_done   (tx_done)
    );

    // UART RX Core
    
    uart_rx #(
        .DATA_BITS (DATA_BITS),
        .PARITY    (PARITY),
        .STOP_BITS (STOP_BITS)
    ) u_uart_rx (
        .clk           (clk),
        .rst_n         (rst_n),
        .baud_tick     (baud_tick),
        .rx            (uart_rx),
        .rx_data       (rx_data_raw),
        .rx_done       (rx_done_raw),
        .parity_error  (rx_parity_err),
        .framing_error (rx_frame_err)
    );

    // RX FIFO
    
    wire rx_fifo_full;

    sync_fifo #(
        .DATA_WIDTH (DATA_BITS),
        .DEPTH      (FIFO_DEPTH)
    ) u_rx_fifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (rx_done_raw && !rx_fifo_full),    // Drop on overflow
        .wr_data (rx_data_raw),
        .rd_en   (rx_rd_en),
        .rd_data (rx_rd_data),
        .full    (rx_fifo_full),
        .empty   (rx_empty)
    );

    assign rx_full = rx_fifo_full;

    
    reg parity_err_r, framing_err_r, rx_overflow_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            parity_err_r   <= 1'b0;
            framing_err_r  <= 1'b0;
            rx_overflow_r  <= 1'b0;
        end else begin
            parity_err_r  <= rx_parity_err;
            framing_err_r <= rx_frame_err;
            rx_overflow_r <= rx_done_raw && rx_fifo_full; // Overflow: data lost
        end
    end

    assign parity_error  = parity_err_r;
    assign framing_error = framing_err_r;
    assign rx_overflow   = rx_overflow_r;

endmodule
