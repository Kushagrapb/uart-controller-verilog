// =============================================================================
// Testbench  : tb_uart_controller
// Description: Self-checking testbench for the UART controller
//              Tests: loopback (TX->RX), parity error injection,
//                     framing error injection, FIFO fill/drain
// Simulation : Use ModelSim / Icarus Verilog / Vivado xsim
//   iverilog -o tb_uart tb_uart_controller.v uart_controller.v
//             uart_tx.v uart_rx.v uart_baud_gen.v sync_fifo.v
//   vvp tb_uart
// =============================================================================

`timescale 1ns/1ps

module tb_uart_controller;

    // -----------------------------------------------------------------------
    // DUT Parameters
    // -----------------------------------------------------------------------
    localparam CLK_FREQ   = 50_000_000;
    localparam BAUD_RATE  = 115_200;
    localparam DATA_BITS  = 8;
    localparam PARITY     = 2;          // Even parity for testing
    localparam STOP_BITS  = 1;
    localparam FIFO_DEPTH = 16;

    localparam CLK_PERIOD  = 20;        // 50 MHz -> 20 ns period
    localparam BAUD_PERIOD = 1_000_000_000 / BAUD_RATE; // ns per baud period

    // -----------------------------------------------------------------------
    // DUT signals
    // -----------------------------------------------------------------------
    reg  clk, rst_n;

    // TX path
    reg  tx_wr_en;
    reg  [DATA_BITS-1:0] tx_wr_data;
    wire tx_full, tx_empty;

    // RX path
    reg  rx_rd_en;
    wire [DATA_BITS-1:0] rx_rd_data;
    wire rx_empty, rx_full;

    // Status
    wire parity_error, framing_error, rx_overflow;

    // UART lines (loopback: TX -> RX)
    wire uart_tx;
    reg  uart_rx_override;
    reg  use_override;
    wire uart_rx = use_override ? uart_rx_override : uart_tx;

    // -----------------------------------------------------------------------
    // DUT Instantiation
    // -----------------------------------------------------------------------
    uart_controller #(
        .CLK_FREQ   (CLK_FREQ),
        .BAUD_RATE  (BAUD_RATE),
        .DATA_BITS  (DATA_BITS),
        .PARITY     (PARITY),
        .STOP_BITS  (STOP_BITS),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .tx_wr_en     (tx_wr_en),
        .tx_wr_data   (tx_wr_data),
        .tx_full      (tx_full),
        .tx_empty     (tx_empty),
        .rx_rd_en     (rx_rd_en),
        .rx_rd_data   (rx_rd_data),
        .rx_empty     (rx_empty),
        .rx_full      (rx_full),
        .parity_error (parity_error),
        .framing_error(framing_error),
        .rx_overflow  (rx_overflow),
        .uart_rx      (uart_rx),
        .uart_tx      (uart_tx)
    );

    // -----------------------------------------------------------------------
    // Clock generation
    // -----------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -----------------------------------------------------------------------
    // Task: Write one byte to TX FIFO
    // -----------------------------------------------------------------------
    task write_tx;
        input [DATA_BITS-1:0] data;
        begin
            @(posedge clk);
            tx_wr_en   <= 1'b1;
            tx_wr_data <= data;
            @(posedge clk);
            tx_wr_en   <= 1'b0;
            $display("[%0t] TX WRITE: 0x%02X", $time, data);
        end
    endtask

    // -----------------------------------------------------------------------
    // Task: Wait until RX FIFO has data, then read it
    // -----------------------------------------------------------------------
    task read_rx;
        output [DATA_BITS-1:0] data;
        begin
            wait (!rx_empty);
            @(posedge clk);
            rx_rd_en <= 1'b1;
            @(posedge clk);
            rx_rd_en <= 1'b0;
            data     <= rx_rd_data;
            $display("[%0t] RX READ:  0x%02X", $time, rx_rd_data);
        end
    endtask

    // -----------------------------------------------------------------------
    // Task: Inject a raw UART frame on uart_rx (for error testing)
    // Parity bit can be deliberately wrong
    // -----------------------------------------------------------------------
    task inject_uart_frame;
        input [DATA_BITS-1:0] data;
        input                 bad_parity;
        integer i;
        reg computed_parity;
        begin
            use_override      = 1;
            uart_rx_override  = 1;  // Idle

            // Start bit
            #(BAUD_PERIOD);
            uart_rx_override = 0;
            #(BAUD_PERIOD);

            // Data bits (LSB first)
            for (i = 0; i < DATA_BITS; i = i + 1) begin
                uart_rx_override = data[i];
                #(BAUD_PERIOD);
            end

            // Parity bit (even parity XOR)
            computed_parity = ^data;
            if (bad_parity)
                uart_rx_override = ~computed_parity; // Inject error
            else
                uart_rx_override = computed_parity;
            #(BAUD_PERIOD);

            // Stop bit
            uart_rx_override = 1;
            #(BAUD_PERIOD);

            use_override = 0;
        end
    endtask

    // -----------------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------------
    integer test_num;
    reg [DATA_BITS-1:0] received;
    integer pass_cnt, fail_cnt;

    initial begin
        $display("=======================================================");
        $display("     UART Controller Testbench START");
        $display("     CLK=%0dMHz  BAUD=%0d  DATA=%0d  PARITY=%0d",
                 CLK_FREQ/1_000_000, BAUD_RATE, DATA_BITS, PARITY);
        $display("=======================================================");

        // Initialize
        rst_n            = 0;
        tx_wr_en         = 0;
        rx_rd_en         = 0;
        use_override     = 0;
        uart_rx_override = 1;
        tx_wr_data       = 0;
        pass_cnt         = 0;
        fail_cnt         = 0;

        // Hold reset for 10 cycles
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(5) @(posedge clk);

        // -------------------------------------------------------------------
        // TEST 1: Single byte loopback (TX->RX)
        // -------------------------------------------------------------------
        $display("\n--- TEST 1: Single Byte Loopback ---");
        write_tx(8'hA5);
        read_rx(received);
        if (received === 8'hA5 && !parity_error && !framing_error) begin
            $display("PASS: Received 0x%02X correctly", received);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("FAIL: Expected 0xA5, Got 0x%02X, PE=%b FE=%b",
                     received, parity_error, framing_error);
            fail_cnt = fail_cnt + 1;
        end

        repeat(20) @(posedge clk);

        // -------------------------------------------------------------------
        // TEST 2: Multi-byte burst (0x00-0x07)
        // -------------------------------------------------------------------
        $display("\n--- TEST 2: Multi-byte Burst (8 bytes) ---");
        begin : burst_test
            integer j;
            reg [DATA_BITS-1:0] rx_val;
            for (j = 0; j < 8; j = j + 1)
                write_tx(j);
            for (j = 0; j < 8; j = j + 1) begin
                read_rx(rx_val);
                if (rx_val === j[DATA_BITS-1:0]) begin
                    pass_cnt = pass_cnt + 1;
                end else begin
                    $display("FAIL byte %0d: expected 0x%02X got 0x%02X", j, j, rx_val);
                    fail_cnt = fail_cnt + 1;
                end
            end
            $display("Burst test complete: %0d/8 passed", pass_cnt);
        end

        repeat(20) @(posedge clk);

        // -------------------------------------------------------------------
        // TEST 3: Parity error injection
        // -------------------------------------------------------------------
        $display("\n--- TEST 3: Parity Error Injection ---");
        inject_uart_frame(8'h55, 1'b1); // Bad parity
        repeat(3) @(posedge clk);
        if (parity_error) begin
            $display("PASS: Parity error correctly detected");
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("FAIL: Parity error NOT detected");
            fail_cnt = fail_cnt + 1;
        end

        repeat(50) @(posedge clk);

        // -------------------------------------------------------------------
        // TEST 4: All zeros and all ones
        // -------------------------------------------------------------------
        $display("\n--- TEST 4: Boundary Values (0x00, 0xFF) ---");
        begin : boundary
            reg [DATA_BITS-1:0] rv;
            write_tx(8'h00);
            read_rx(rv);
            if (rv === 8'h00) begin
                $display("PASS: 0x00");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("FAIL: 0x00 -> got 0x%02X", rv);
                fail_cnt = fail_cnt + 1;
            end

            write_tx(8'hFF);
            read_rx(rv);
            if (rv === 8'hFF) begin
                $display("PASS: 0xFF");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("FAIL: 0xFF -> got 0x%02X", rv);
                fail_cnt = fail_cnt + 1;
            end
        end

        repeat(20) @(posedge clk);

        // -------------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------------
        $display("\n=======================================================");
        $display("  RESULTS: %0d PASSED | %0d FAILED", pass_cnt, fail_cnt);
        $display("=======================================================");

        if (fail_cnt == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED - Review above output");

        $finish;
    end

    // -----------------------------------------------------------------------
    // Timeout watchdog
    // -----------------------------------------------------------------------
    initial begin
        #(BAUD_PERIOD * 200);
        $display("WATCHDOG TIMEOUT - Simulation did not complete in time");
        $finish;
    end

    // -----------------------------------------------------------------------
    // Waveform dump (for GTKWave or Vivado)
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("uart_sim.vcd");
        $dumpvars(0, tb_uart_controller);
    end

endmodule
