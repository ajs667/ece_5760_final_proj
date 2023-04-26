`timescale 1ns/1ns

module compute_dummy (
    input           clk, reset//, START,
    // input  signed   [7:0]   M10K_read_data_source,
    // output signed   [7:0]   M10K_write_data_source_wire,
    // output          [7:0]   M10K_read_address_source_wire, // 8 bits, 0 to 256
    // output          [7:0]   M10K_write_address_source_wire, // 8 bits, 0 to 256
    // output          M10K_write_source_wire, // write to M10K block [i]

    // input  signed   [7:0]   M10K_read_data_int,
    // output signed   [7:0]   M10K_write_data_int_wire,
    // output          [7:0]   M10K_read_address_int_wire, // 8 bits, 0 to 256
    // output          [7:0]   M10K_write_address_int_wire, // 8 bits, 0 to 256
    // output          M10K_write_int_wire // write to M10K block [i]
);

reg dummy_var;

always @ (posedge clk) begin
    if (reset) begin
        dummy_var <= 0;
    end
    dummy_var <= ~dummy_var;
end


// reg signed      [7:0]   M10K_data_buffer_source;
// reg signed      [7:0]   M10K_write_data_source;
// reg             [7:0]   M10K_read_address_source; // 8 bits, 0 to 256
// reg             [7:0]   M10K_write_address_source; // 8 bits, 0 to 256
// reg             M10K_write_source; // write to M10K block [i]

// assign M10K_write_data_source_wire    = M10K_write_data_source;
// assign M10K_read_address_source_wire  = M10K_read_address_source;
// assign M10K_write_address_source_wire = M10K_write_address_source;
// assign M10K_write_source_wire         = M10K_write_source;

// reg signed      [7:0]   M10K_data_buffer_int;
// reg signed      [7:0]   M10K_write_data_int;
// reg             [7:0]   M10K_read_address_int; // 8 bits, 0 to 256
// reg             [7:0]   M10K_write_address_int; // 8 bits, 0 to 256
// reg             M10K_write_int; // write to M10K block [i]

// assign M10K_write_data_int_wire    = M10K_write_data_int;
// assign M10K_read_address_int_wire  = M10K_read_address_int;
// assign M10K_write_address_int_wire = M10K_write_address_int;
// assign M10K_write_int_wire         = M10K_write_int;

// always @ (posedge clk) begin
//     if ( reset ) begin // reset
//         if (START) begin
//             M10K_write_source <= 0;
//             M10K_write_int <= 0;
//         end
//         else begin
//             M10K_write_source <= 1;
//             M10K_write_int <= 1;
//         end
//     end
//     else begin
//         if (START) begin
//             M10K_write_source <= 0;
//             M10K_write_int <= 0;
//         end
//         else begin
//             M10K_write_source <= 1;
//             M10K_write_int <= 1;
//         end
//     end
// end

endmodule