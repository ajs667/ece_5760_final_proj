module compute (
    input           clk, reset, START
    input  signed   [7:0]   M10K_read_data_source;
    output signed   [7:0]   M10K_write_data_source_wire;
    output          [7:0]   M10K_read_address_source_wire; // 8 bits, 0 to 256
    output          [7:0]   M10K_write_address_source_wire; // 8 bits, 0 to 256
    output          M10K_write_source_wire; // write to M10K block [i]

    input  signed   [7:0]   M10K_read_data_int;
    output signed   [7:0]   M10K_write_data_int_wire;
    output          [7:0]   M10K_read_address_int_wire; // 8 bits, 0 to 256
    output          [7:0]   M10K_write_address_int_wire; // 8 bits, 0 to 256
    output          M10K_write_int_wire; // write to M10K block [i]
);

reg signed      [7:0]   M10K_data_buffer_source;
reg signed      [7:0]   M10K_write_data_source;
reg             [7:0]   M10K_read_address_source; // 8 bits, 0 to 256
reg             [7:0]   M10K_write_address_source; // 8 bits, 0 to 256
reg             M10K_write_source; // write to M10K block [i]

assign M10K_write_data_source_wire    = M10K_write_data_source;
assign M10K_read_address_source_wire  = M10K_read_address_source;
assign M10K_write_address_source_wire = M10K_write_address_source;
assign M10K_write_source_wire         = M10K_write_source;

reg signed      [7:0]   M10K_data_buffer_int;
reg signed      [7:0]   M10K_write_data_int;
reg             [7:0]   M10K_read_address_int; // 8 bits, 0 to 256
reg             [7:0]   M10K_write_address_int; // 8 bits, 0 to 256
reg             M10K_write_int; // write to M10K block [i]

assign M10K_write_data_int_wire    = M10K_write_data_int;
assign M10K_read_address_int_wire  = M10K_read_address_int;
assign M10K_write_address_int_wire = M10K_write_address_int;
assign M10K_write_int_wire         = M10K_write_int;






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
    if ( reset ) begin // reset
        row <= 0;
        col <= 0;
        col_to_add <= 0;
        M10K_write_source <= 0;
        M10K_write_int <= 0;
        sum <= 0;
    end
    else begin
        if (state == 0) begin
            if(START) begin
                state_reg <= 1;
            end
            else begin state_reg <= state_reg; end
        end
        else if ( state == 1 ) begin // request values to read
        M10K_read_address_source <= row << 2 + col ;

        if ( row > 0 ) begin // if row is not 0, get integral data from row-1, col
            M10K_read_address_int <= (row - 1) << 2 + col ; // curr row * num col + curr col 
        end

        M10K_write_source <= 0;
        M10K_write_int <= 0;

        state_reg <= state + 1;
    end
    else if ( state == 2 ) begin // wait one 
        state_reg <= state + 1;
    end
    else if ( state == 3 ) begin // receive the data from M10K block 
        curr_row_source_data[col] <= M10K_read_data_source;
        if ( row > 0 ) begin
            sum <= M10K_read_data_int;
        end
        else begin 
            sum <= M10K_read_data_source;
        end
        state_reg <= state + 1;
    end
    else if ( state == 4 ) begin // repeat in state until added all curr_row_source_data
        if (col_to_add <= col) begin // include this source 
            sum <= sum + curr_row_source_data[col_to_add];
            state_reg <= 3;
        end
        else begin // write to M10K block 
            M10K_write_data_int <= sum; 
            M10K_write_address_int <= row << 2 + COL ;
            M10K_write_int <= 1;
            state_reg <= 0; 
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
end

endmodule

