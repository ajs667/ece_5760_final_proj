`timescale 1ns/1ns

module testbench();
	
	reg clk_50, clk_25, reset;
	
	//Initialize clocks and index
	initial begin
		clk_50 = 1'b0;
		clk_25 = 1'b0;
		//testbench_out = 15'd0 ;
	end
	
	//Toggle the clocks
	always begin
		#10
		clk_50  = !clk_50;
	end
	
	always begin
		#20
		clk_25  = !clk_25;
	end
	
	//Intialize and drive signals
	initial begin
		reset  = 1'b0;
		#10 
		reset  = 1'b1;
		#30
		reset  = 1'b0;
	end

reg START;

// connected to inputs and outputs of M10K blocks
wire signed     [7:0]   M10K_read_data_source;
reg signed      [7:0]   M10K_write_data_source;
reg             [7:0]   M10K_read_address_source; // 8 bits, 0 to 256
reg             [7:0]   M10K_write_address_source; // 8 bits, 0 to 256
reg             M10K_write_source; // write to M10K block [i]

wire signed     [7:0]   M10K_read_data_int;
reg signed      [7:0]   M10K_write_data_int;
reg             [7:0]   M10K_read_address_int; // 8 bits, 0 to 256
reg             [7:0]   M10K_write_address_int; // 8 bits, 0 to 256
reg             M10K_write_int; // write to M10K block [i]


wire signed     [7:0]   M10K_write_data_source_wire;
wire            [7:0]   M10K_read_address_source_wire; // 8 bits, 0 to 256
wire            [7:0]   M10K_write_address_source_wire; // 8 bits, 0 to 256
wire            M10K_write_source_wire; // write to M10K block [i]

wire signed     [7:0]   M10K_write_data_int_wire;
wire            [7:0]   M10K_read_address_int_wire; // 8 bits, 0 to 256
wire            [7:0]   M10K_write_address_int_wire; // 8 bits, 0 to 256
wire            M10K_write_int_wire; // write to M10K block [i]

// assign M10K_write_data_source_wire = M10K_write_data_source;
// assign M10K_read_address_source_wire = M10K_read_address_source;
// assign M10K_write_address_source_wire = M10K_write_address_source;
// assign M10K_write_source_wire = M10K_write_source;

// assign M10K_write_data_int_wire = M10K_write_data_int;
// assign M10K_read_address_int_wire = M10K_read_address_int;
// assign M10K_write_address_int_wire = M10K_write_address_int;
// assign M10K_write_int_wire = M10K_write_int;

M10K_1K_8 M10K_source( 
    .q              (M10K_read_data_source),
    .d              (M10K_write_data_source),
    .write_address  (M10K_write_address_source), 
    .read_address   (M10K_read_address_source),
    .we             (M10K_write_source), 
    .clk            (clk_50)
);

M10K_1K_8 M10K_int( 
    .q              (M10K_read_data_int),
    .d              (M10K_write_data_int),
    .write_address  (M10K_write_address_int), 
    .read_address   (M10K_read_address_int),
    .we             (M10K_write_int), 
    .clk            (clk_50)
);

reg  [4:0] idx;
reg  [4:0] state_reg;
wire [4:0] state;
assign state = state_reg;

always @(posedge clk_50) begin

    if(reset) begin
        idx <= 0;
        state_reg <= 0;
        START <= 0;
    end
    else begin
        if(state == 0) begin // write 1 to source 
            M10K_write_address_source <= idx;
            M10K_write_data_source    <= 32'b1;
            M10K_write_source <= 1;

            if (idx == 16) begin
                state_reg <= state_reg + 1;
                START <= 1;
                M10K_write_source <= 0;
            end
            else begin
                idx <= idx + 5'd1;
                state_reg <= 0;
            end
        end
        else begin
            M10K_write_address_source <= M10K_write_address_source_wire ;
            M10K_write_data_source <= M10K_write_data_source_wire ;
            M10K_write_source <= M10K_write_source_wire ;
            M10K_read_address_source <= M10K_read_address_source_wire ;

            M10K_write_address_int <= M10K_write_address_int_wire ;
            M10K_write_data_int <= M10K_write_data_int_wire ;
            M10K_write_int <= M10K_write_int_wire ;
            M10K_read_address_int <= M10K_read_address_int_wire ;
        end
    end

end

compute dut(
	.clk                            (clk_50),
	.reset                          (reset),
	.M10K_read_data_source          (M10K_read_data_source),
    .M10K_write_data_source_wire    (M10K_write_data_source_wire),
    .M10K_read_address_source_wire  (M10K_read_address_source_wire),
    .M10K_write_address_source_wire (M10K_write_address_source_wire),
    .M10K_write_source_wire         (M10K_write_source_wire),
    .M10K_read_data_int             (M10K_read_data_int),
    .M10K_write_data_int_wire       (M10K_write_data_int_wire),
    .M10K_read_address_int_wire     (M10K_read_address_int_wire),
    .M10K_write_address_int_wire    (M10K_write_address_int_wire),
    .M10K_write_int_wire            (M10K_write_int_wire),
    .START                          (START)
);

endmodule






/////////// MODULES /////////////

module M10K_1K_8( 
    output reg [7:0] q,
    input [7:0] d,
    input [7:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
    // reg [7:0] mem [255:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
    reg [7:0] mem [15:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d ;
		  end
        q <= mem[read_address] ; // q doesn't get d in this clock cycle
    end
endmodule







module compute (
    input           clk, reset, START,
    input  signed   [7:0]   M10K_read_data_source,
    output signed   [7:0]   M10K_write_data_source_wire,
    output          [7:0]   M10K_read_address_source_wire, // 8 bits, 0 to 256
    output          [7:0]   M10K_write_address_source_wire, // 8 bits, 0 to 256
    output          M10K_write_source_wire, // write to M10K block [i]

    input  signed   [7:0]   M10K_read_data_int,
    output signed   [7:0]   M10K_write_data_int_wire,
    output          [7:0]   M10K_read_address_int_wire, // 8 bits, 0 to 256
    output          [7:0]   M10K_write_address_int_wire, // 8 bits, 0 to 256
    output          M10K_write_int_wire // write to M10K block [i]
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

localparam VID_IN_WIDTH = 4;
localparam VID_IN_HEIGHT = 4;

reg [3:0] row, col, col_to_add;

reg [4:0] state_reg ;
wire [4:0] state ;
assign state = state_reg;

reg [7:0] curr_row_source_data [0:VID_IN_WIDTH-1] ;
reg [7:0] sum ;

wire [7:0] rowls2;
assign rowls2 = row << 2; 

integer row_data_i;

always @ (posedge clk) begin
    if ( reset ) begin // reset
        row <= 0;
        col <= 0;
        col_to_add <= 0;
        M10K_write_source <= 0;
        M10K_write_int <= 0;
        sum <= 0;
        state_reg <= 0;
    end
    else begin
        if (state == 0) begin // reset state
            if(START) begin
                state_reg <= 1;
            end
            else begin state_reg <= state_reg; end
        end
        else if ( state == 1 ) begin // request values to read
            M10K_read_address_source <= (row << 2) + col ;
            // M10K_read_address_source <= col ;

            if ( row > 0 ) begin // if row is not 0, get integral data from row-1, col
                M10K_read_address_int <= ((row - 1) << 2) + col ; // curr row * num col + curr col 
            end

            M10K_write_source <= 0;
            M10K_write_int <= 0;

            state_reg <= state + 1;
        end
        else if ( state == 2 ) begin // wait one 
            state_reg <= state + 1;
            // state_reg <= state;
        end
        else if ( state == 3 ) begin // wait one 
            state_reg <= state + 1;
            // state_reg <= state;
        end
        else if ( state == 4 ) begin // receive the data from M10K block 
            curr_row_source_data[col] <= M10K_read_data_source;
            if ( row > 0 ) begin
                sum <= M10K_read_data_int;
            end
            else begin 
                sum <= 0; //M10K_read_data_source;
            end
            state_reg <= state + 1;
        end
        else if ( state == 5 ) begin // repeat in state until added all curr_row_source_data
            if (col_to_add <= col) begin // include this source 
                sum <= sum + curr_row_source_data[col_to_add];
                col_to_add <= col_to_add + 1;
                state_reg <= 5;
            end
            else begin // write to M10K block and go to next index
                M10K_write_data_int <= sum; 
                M10K_write_address_int <= (row << 2) + col ;
                // M10K_write_address_int <= col ;
                M10K_write_int <= 1;
                state_reg <= 6; 
                col_to_add <= 0 ;

                // next index logic
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
        else if ( state == 6 ) begin // wait state
            state_reg <= 1; 
        end
    end 
end

endmodule

