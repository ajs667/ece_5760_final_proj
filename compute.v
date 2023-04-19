wire signed     [7:0]   M10K_read_data_source;
reg signed      [7:0]   M10K_data_buffer_source;
reg signed      [7:0]   M10K_write_data_source;
reg             [7:0]   M10K_read_address_source; // 8 bits, 0 to 256
reg             [7:0]   M10K_write_address_source; // 8 bits, 0 to 256
reg             M10K_u_N_write_source; // write to M10K block [i]

wire signed     [7:0]   M10K_read_data_int;
reg signed      [7:0]   M10K_data_buffer_int;
reg signed      [7:0]   M10K_write_data_int;
reg             [7:0]   M10K_read_address_int; // 8 bits, 0 to 256
reg             [7:0]   M10K_write_address_int; // 8 bits, 0 to 256
reg             M10K_u_N_write_int; // write to M10K block [i]

M10K_1K_8 M10K_source( 
    .q              (M10K_read_data_source),
    .d              (M10K_write_data_source),
    .write_address  (M10K_write_address_source), 
    .read_address   (M10K_read_address_source),
    .we             (M10K_write_source), 
    .clk            (clk)
);

M10K_1K_8 M10K_int( 
    .q              (M10K_read_data_int),
    .d              (M10K_write_data_int),
    .write_address  (M10K_write_address_int), 
    .read_address   (M10K_read_address_int),
    .we             (M10K_write_int), 
    .clk            (clk)
);

localparam VID_IN_WIDTH = 4
localparam VID_IN_HEIGHT = 4

reg [1:0] row, col, col_to_add;

reg [1:0] state_reg ;
wire [1:0] state ;
assign state = state_reg;

reg [7:0] curr_row_source_data [0:VID_IN_WIDTH-1] ;
reg [7:0] sum ;

integer row_data_i;

always @ posedge(clk) begin
    if ( state == 0 ) begin // reset
        row <= 0;
        col <= 0;
        col_to_add <= 0;
        M10K_write_source <= 0;
        M10K_write_int <= 0;
        sum <= 0;
    end
    else if ( state == 1 ) begin // request values to read
        M10K_read_address_source <= row << 2 + COL ;
        M10K_read_address_int <= (row - 1) << 2 + COL ;

        M10K_write_source <= 0;
        M10K_write_int <= 0;

        state_reg <= state + 1;
    end
    else if ( state == 2 ) begin // wait one 
        state_reg <= state + 1;
    end
    else if ( state == 3 ) begin // receive the data from M10K block 
        curr_row_source_data[col] <= M10K_read_data_source;
        sum <= M10K_read_data_int;
        state_reg <= state + 1;
    end
    else if ( state == 4 ) begin // repeat in state until added all curr_row_source_data
        if (col_to_add <= col) begin // include this source 
            sum <= sum + curr_row_source_data[col_to_add];
            state <= 4;
        end
        else begin // write to M10K block 
            M10K_write_data_int <= sum; 
            M10K_write_address_int <= row << 2 + COL ;
            M10K_write_int <= 1;
            state <= 1; 
        end

        if ( col < VID_IN_WIDTH ) begin 
            col <= col + 1;
        end
        else begin 
            col <= 0;
            // check to see if reached last row 
            if ( row < VID_IN_HEIGHT ) begin 
                row <= row + 1;
            end
            else begin 
                row <= 0;
            end
        end
    end
end

module M10K_1K_8( 
    output reg [7:0] q,
    input [7:0] d,
    input [7:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
    reg [7:0] mem [255:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d ;
		  end
        q <= mem[read_address] ; // q doesn't get d in this clock cycle
    end
endmodule

