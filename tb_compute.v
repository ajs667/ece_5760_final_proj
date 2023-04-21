`timescale 1ns / 1ps

module tb_comptute();

reg clk = 1'b0;
always #10 clk = ~clk;

reg START;

reg reset;
initial begin
	reset = 1'b0;
	#10
	reset = 1'b1;
	#30
	reset = 1'b0;
end

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

reg        idx;
reg  [3:0] state_reg;
wire [3:0] state;
assign state = state_reg;

always @(posedge clk) begin

    if(reset) begin
        idx <= 0;
        state_reg <= 0;
        START <= 0;
    end
    else begin
        if(state == 0) begin
            M10K_write_address_source <= idx;
            M10K_write_data_source    <= 32'b1;
            state_reg <= 1;
        end
        else if (state == 1) begin
            state_reg <= 2;
            idx <= idx + 1;
        end
        else if (state == 2) begin
            if (idx == 17) begin
                state_reg <= state_reg;
                START <= 1;
            end
            else begin
                state_reg <= 0;
            end
        end
    end

end

compute dut(
	.clk                            (clk),
	.reset                          (reset),
	.M10K_read_data_source          (M10K_read_data_source),
    .M10K_write_data_source_wire    (M10K_write_data_source),
    .M10K_read_address_source_wire  (M10K_read_address_source),
    .M10K_write_address_source_wire (M10K_write_address_source),
    .M10K_write_source_wire         (M10K_write_source),
    .M10K_read_data_int             (M10K_read_data_int),
    .M10K_write_data_int_wire       (M10K_write_data_int),
    .M10K_read_address_int_wire     (M10K_read_address_int),
    .M10K_write_address_int_wire    (M10K_write_address_int),
    .M10K_write_int_wire            (M10K_write_int),
    .START                          (START)
);

endmodule

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

