// =============================================================================
// Module      : sync_fifo
// Description : Synchronous FIFO (single clock domain)
//               Used as TX and RX buffers in the UART controller
//
// Parameters:
//   DATA_WIDTH - Width of data bus       (default: 8)
//   DEPTH      - FIFO depth (power of 2) (default: 16)
// =============================================================================

module sync_fifo #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH      = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output wire                  full,
    output wire                  empty
);

    localparam ADDR_WIDTH = $clog2(DEPTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH:0]   wr_ptr; // Extra bit for full/empty detection
    reg [ADDR_WIDTH:0]   rd_ptr;

    wire [ADDR_WIDTH-1:0] wr_addr = wr_ptr[ADDR_WIDTH-1:0];
    wire [ADDR_WIDTH-1:0] rd_addr = rd_ptr[ADDR_WIDTH-1:0];

    // Full when MSBs differ but lower bits match
    assign full  = (wr_ptr == {~rd_ptr[ADDR_WIDTH], rd_ptr[ADDR_WIDTH-1:0]});
    assign empty = (wr_ptr == rd_ptr);

    // Write port
    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= 0;
        end else if (wr_en && !full) begin
            mem[wr_addr] <= wr_data;
            wr_ptr       <= wr_ptr + 1;
        end
    end

    // Read port
    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr  <= 0;
            rd_data <= 0;
        end else if (rd_en && !empty) begin
            rd_data <= mem[rd_addr];
            rd_ptr  <= rd_ptr + 1;
        end
    end

endmodule
