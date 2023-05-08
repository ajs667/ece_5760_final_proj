

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire			[15: 0]	hex3_hex0;
//wire			[15: 0]	hex5_hex4;

//assign HEX0 = ~hex3_hex0[ 6: 0]; // hex3_hex0[ 6: 0]; 
//assign HEX1 = ~hex3_hex0[14: 8];
//assign HEX2 = ~hex3_hex0[22:16];
//assign HEX3 = ~hex3_hex0[30:24];
assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;

HexDigit Digit0(HEX0, hex3_hex0[3:0]);
HexDigit Digit1(HEX1, hex3_hex0[7:4]);
HexDigit Digit2(HEX2, hex3_hex0[11:8]);
HexDigit Digit3(HEX3, hex3_hex0[15:12]);

// MAY need to cycle this switch on power-up to get video
assign TD_RESET_N = SW[1]; // assign to smth else 

// get some signals exposed
// connect bus master signals to i/o for probes
assign GPIO_0[0] = TD_HS ;
assign GPIO_0[1] = TD_VS ;
assign GPIO_0[2] = TD_DATA[6] ;
assign GPIO_0[3] = TD_CLK27 ;
assign GPIO_0[4] = TD_RESET_N ;

//=======================================================
// Bus controller for AVALON bus-master
//=======================================================
wire [31:0] vga_bus_addr, video_in_bus_addr ; // Avalon addresses
reg  [31:0] bus_addr ;
wire [31:0] vga_out_base_address = 32'h0000_0000 ;  // Avalon address
wire [31:0] video_in_base_address = 32'h0800_0000 ;  // Avalon address
reg [3:0] bus_byte_enable ; // four bit byte read/write mask
reg bus_read  ;       // high when requesting data
reg bus_write ;      //  high when writing data
reg [31:0] bus_write_data ; //  data to send to Avalog bus
wire bus_ack  ;       //  Avalon bus raises this when done
wire [31:0] bus_read_data ; // data from Avalon bus
reg [31:0] timer ;
reg [9:0] state ;
reg last_vs, wait_one;
reg [19:0] vs_count ;
reg last_hs, wait_one_hs ;
reg [19:0] hs_count ;

// pixel address is
reg [9:0] vga_x_cood, vga_y_cood, video_in_x_cood, video_in_y_cood ;
reg [7:0] current_pixel_color1, current_pixel_color2 ;
// compute address
assign vga_bus_addr = vga_out_base_address + {22'b0,video_in_x_cood + vga_x_cood} + ({22'b0,video_in_y_cood + vga_y_cood}<<10) ;
assign video_in_bus_addr = video_in_base_address + {22'b0,video_in_x_cood} + ({22'b0,video_in_y_cood}<<9) ;	 

reg collect_single_frame ; 
reg START;

// M10K block for source 
wire signed     [7:0]   M10K_read_data_source;
reg signed      [7:0]   M10K_write_data_source;
reg             [16:0]   M10K_read_address_source; // 8 bits, 0 to 256
reg             [16:0]   M10K_write_address_source; // 8 bits, 0 to 256
reg             M10K_write_source; // write to M10K block [i]

// M10K block for integral 
wire signed     [INT_WIDTH:0]   M10K_read_data_int;
reg signed      [INT_WIDTH:0]   M10K_write_data_int;
reg             [16:0]   M10K_read_address_int; // 8 bits, 0 to 256
reg             [16:0]   M10K_write_address_int; // 8 bits, 0 to 256
reg             M10K_write_int; // write to M10K block [i]

M10K_1K_8 M10K_source( 
    .q              (M10K_read_data_source),
    .d              (M10K_write_data_source),
    .write_address  (M10K_write_address_source), 
    .read_address   (M10K_read_address_source),
    .we             (M10K_write_source), 
    .clk            (CLOCK2_50)
);

//M10K_25bit#(INT_WIDTH) M10K_int( 
//    .q              (M10K_read_data_int),
//    .d              (M10K_write_data_int),
//    .write_address  (M10K_write_address_int), 
//    .read_address   (M10K_read_address_int),
//    .we             (M10K_write_int), 
//    .clk            (CLOCK2_50)
//);

localparam VID_IN_WIDTH = 320;
localparam VID_IN_HEIGHT = 240;
localparam INT_WIDTH = 24;

wire [7:0] color8b_wire, grey_out_wire, pio_state ;
assign color8b_wire = current_pixel_color1;

convrgb2grey greyscale(
	.color8b(color8b_wire),
	.grey8b(grey_out_wire)
);

// reg [63:0] pixel_counter;
reg [9:0] pio_row_reg, pio_col_reg ;
wire [9:0] pio_row, pio_col ; 
assign pio_row = video_in_y_cood ; 
assign pio_col = video_in_x_cood ; 

wire [7:0] pio_color_read_wire ;
reg [7:0] pio_color_read, pio_state_reg ;
assign pio_color_read_wire = pio_color_read ;
assign pio_state = state ;

wire pio_collectsingle ;
assign pio_collectsingle = collect_single_frame ;

// SRAM data for integral image
//reg [16:0] onchip_sram_int_address;
//reg        onchip_sram_int_write;
//reg [31:0] onchip_sram_int_readdata;
//reg [31:0] onchip_sram_int_writedata;

// data for the integral image calcs
reg [8:0] row, col, col_to_add;
reg [11:0] curr_row_source_data ;
reg [INT_WIDTH:0] sum ;

reg [19:0] int_index; // Used to index into the integral M10K block when we send over PIO

reg [31:0] pio_integral_data;
wire [31:0] pio_integral_data_wire;
assign pio_integral_data_wire = pio_integral_data;

always @(posedge CLOCK2_50) begin //CLOCK_50

	// reset state machine and read/write controls
	if (~KEY[0]) begin
		state <= 0 ;
		bus_read <= 0 ; // set to one if a read opeation from bus
		bus_write <= 0 ; // set to on if a write operation to bus
		// base address of upper-left corner of the screen
		vga_x_cood <= 10'd100 ;
		vga_y_cood <= 10'd50 ;
		video_in_x_cood <= 0 ;
		video_in_y_cood <= 0 ;
		bus_byte_enable <= 4'b0001;

		collect_single_frame <= 1'b0;
		START <= 0;
		// pixel_counter <= 0;

		timer <= 0;
		pio_color_read <= 8'd255 ; 
		pio_col_reg <= 0 ; 
		pio_row_reg <= 0 ;
	
		row <= 0;
		col <= 0;
		col_to_add <= 0;
		curr_row_source_data <= 0;
		sum <= 0;
		int_index <= 0;
		pio_integral_data <= 0;
	end
	else begin
		timer <= timer + 1;
//	end

	////////// STATE 0: UPDATE THE VIDEO INPUT COORDINATES AND START READ /////////////////
	
	// write to the bus-master
	// and put in a small delay to aviod bus hogging
	// timer delay can be set to 2**n-1, so 3, 7, 15, 31
	// bigger numbers mean slower frame update to VGA
	if (state==0 && SW[0] && (timer & 3)==0 ) begin 
		// state <= 1; // unless last pixel and collect_single_frame is high 
		
		// read all the pixels in the video input
		video_in_x_cood <= video_in_x_cood + 10'd1 ;

		if (video_in_x_cood >= 10'd319) begin // check to see if reached end of row 
			video_in_x_cood <= 0 ;
			video_in_y_cood <= video_in_y_cood + 10'd1 ;

			if (video_in_y_cood >= 10'd239) begin // check to see if reached last pixel 
				if (collect_single_frame == 1) begin
					state <= 10;
                    START <= 1; // start doing integral calculation ????
				end
				else begin // continue going through state machine, not taking single frame 
					state <= 1;
					video_in_y_cood <= 10'd0 ;
				end
			end
			else begin // have not reached last pixel, continue going through state machine 
				state <= 1;
			end
		end
		else begin // have not reached end of row, continue going thorugh state machine 
			state <= 1 ; 
		end
		// one byte data
		bus_byte_enable <= 4'b0001;
		// read first pixel
		bus_addr <= video_in_bus_addr ;
		// signal the bus that a read is requested
		bus_read <= 1'b1 ;	
	end

	////////// STATE 1: FINISH READ /////////////////
	
	// finish the  read
	// You MUST do this check
	else if (state==1 && bus_ack==1) begin
        if (collect_single_frame == 1) begin
		    state <= 2 ;
        end 
        else begin
            state <= 8 ;
        end
		bus_read <= 1'b0;
		current_pixel_color1 <= bus_read_data ;
	end

	////////// STATE 2: WRITE TO SOURCE M10K BLOCK /////////////////

    else if (state==2) begin 
        state <= 8;
		// state <= 3; // FOR DEBUGGING, DELETE 

        // write to source M10K block if collecting frame
		// if (collect_single_frame == 1) begin
		M10K_write_address_source <= (video_in_x_cood * 320) + video_in_y_cood ; 
		M10K_write_source <= 1; 
		M10K_write_data_source <= grey_out_wire;
			// pixel_counter <= pixel_counter +1;
		// end
		// else begin
			// M10K_write_source <= 0; 
			// if ( pixel_counter > 76800 ) begin
			// 	SOURCE_READ_START <= 1;
			// end
		// end
    end
	 
	 /*

	////////// STATE 3: FOR DEBUGGING /////////////////

    else if (state==3) begin 
        state <= 4;
        // read from M10K block 
		M10K_read_address_source <= (video_in_x_cood * 320) + video_in_y_cood ; 
		M10K_write_source <= 0; 
    end

	else if (state==4) begin // waiting for read state 
		state <= 5;
	end

	else if (state==5) begin // received read, send over pio port 
		state <= 8; 
		pio_color_read <= M10K_read_data_source ; 
		pio_col_reg <= video_in_x_cood ; 
		pio_row_reg <= video_in_y_cood ; 
	end
	
	*/
	
	// write a pixel to VGA memory
	else if (state==8) begin
		state <= 9 ;
		bus_write <= 1'b1;
		bus_addr <= vga_bus_addr ;
		bus_write_data <= current_pixel_color1 ;
		// bus_write_data <= grey_out_wire ;
		bus_byte_enable <= 4'b0001;

		M10K_write_source <= 0; 
	end
	
	// and finish write
	else if (state==9 && bus_ack==1) begin
		state <= 0 ;
		bus_write <= 1'b0;
	end
	
	else if (state==10) begin
		state <= 11; // If we;ve reached state 10 then we know that we've saved a frame in the source block, so we can move onto the integral image calculation
		bus_write <= 1'b0;
		bus_read <= 1'b0; 
		M10K_write_source <= 0;
		//START_INTEGRAL_COMPUTATION_reg <= 1;
	end
	
	// BEGINNING THE INTEGRAL IMAGE COMPUTATION
	
	else if (state == 11) begin
		M10K_read_address_source <= (row * 320) + col ;
      if ( row > 0 ) begin // if row is not 0, get integral data from row-1, col
			M10K_read_address_int <= ((row - 1) * 320) + col ; // curr row * num col + curr col 
      end

         M10K_write_source <= 0;
         M10K_write_int <= 0;
			state <= 12;
	end
	
	else if (state == 12) begin
		state <= 13;
	end
	
	else if (state == 13) begin
		state <= 14;
	end
	
	else if (state == 14) begin
		curr_row_source_data <= curr_row_source_data + M10K_read_data_source;
      if ( row > 0 ) begin
           // sum <= M10K_read_data_int;
           sum <= M10K_read_data_int + curr_row_source_data + M10K_read_data_source;
		end
      else begin 
           // sum <= 0; //M10K_read_data_source;
           sum <= curr_row_source_data + M10K_read_data_source;
      end
		state <= 15;
	end
	
	else if ( state == 15 ) begin
		// write to M10K block and go to next index
            M10K_write_data_int <= sum;
				pio_color_read	<= M10K_write_data_int;
            M10K_write_address_int <= (row * VID_IN_WIDTH) + col ;
            // M10K_write_address_int <= col ;
            M10K_write_int <= 1;
            col_to_add <= 0 ;

            // next index logic
            if ( col < VID_IN_WIDTH ) begin 
                col <= col + 1;
            end
            else begin 
                col <= 0;
                curr_row_source_data <= 0 ;
                // check to see if reached last row 
                if ( row < 240 ) begin 
                    row <= row + 1;
						  state <= 11;
					 end
                else begin 
                    row <= 0;
						  state <= 16;
						  pio_integral_data <= 1'b1;
                end
            end
	end
	
	else if ( state == 20 ) begin
		state <= 11; // wait state
	end
	
	// BEGIN READING THE INTERGRAL IMAGE M10K block and send it over PIO
	else if (state == 16) begin // set up the read
		M10K_read_address_int <= int_index;
		M10K_write_int <= 0;
		state <= 17;
	end
	
	else if (state == 17) begin // wait for read
		state <= 18; 
	end
	
	else if (state == 18) begin
		pio_integral_data <= M10K_read_data_int;
		if ( int_index < 76799 ) begin
			int_index <= int_index + 1;
			state<= 16;
		end
		else begin
			state <= 19;
		end
	end
	
	else if (state == 19) begin
		pio_integral_data <= 32'b1111;
		state <= 19;
	end
	
	if (~KEY[1]) begin
		bus_write <= 1'b0;
		bus_read <= 1'b0;
		collect_single_frame <= 1'b1;
		state <= 0;
		vga_x_cood <= 10'd100 ;
		vga_y_cood <= 10'd50 ;
		video_in_x_cood <= 0 ;
		video_in_y_cood <= 0 ;
		bus_byte_enable <= 4'b0001;
	end
	else begin
		collect_single_frame <= collect_single_frame; 
	end
	end
	
end // always @(posedge state_clock)


//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),

	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),

	// VGA Subsystem
	.vga_pll_ref_clk_clk 					(CLOCK2_50),
	.vga_pll_ref_reset_reset				(1'b0),
	.vga_CLK										(VGA_CLK),
	.vga_BLANK									(VGA_BLANK_N),
	.vga_SYNC									(VGA_SYNC_N),
	.vga_HS										(VGA_HS),
	.vga_VS										(VGA_VS),
	.vga_R										(VGA_R),
	.vga_G										(VGA_G),
	.vga_B										(VGA_B),
	
	// Video In Subsystem
	.video_in_TD_CLK27 						(TD_CLK27),
	.video_in_TD_DATA							(TD_DATA),
	.video_in_TD_HS							(TD_HS),
	.video_in_TD_VS							(TD_VS),
	.video_in_clk27_reset					(),
	.video_in_TD_RESET						(),
	.video_in_overflow_flag					(),
	
	.ebab_video_in_external_interface_address     (bus_addr),     // 
	.ebab_video_in_external_interface_byte_enable (bus_byte_enable), //  .byte_enable
	.ebab_video_in_external_interface_read        (bus_read),        //  .read
	.ebab_video_in_external_interface_write       (bus_write),       //  .write
	.ebab_video_in_external_interface_write_data  (bus_write_data),  //.write_data
	.ebab_video_in_external_interface_acknowledge (bus_ack), //  .acknowledge
	.ebab_video_in_external_interface_read_data   (bus_read_data),   
	// clock bridge for EBAb_video_in_external_interface_acknowledge
	.clock_bridge_0_in_clk_clk                    (CLOCK_50),
		
	// SDRAM
	.sdram_clk_clk								(DRAM_CLK),
   .sdram_addr									(DRAM_ADDR),
	.sdram_ba									(DRAM_BA),
	.sdram_cas_n								(DRAM_CAS_N),
	.sdram_cke									(DRAM_CKE),
	.sdram_cs_n									(DRAM_CS_N),
	.sdram_dq									(DRAM_DQ),
	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
	.sdram_ras_n								(DRAM_RAS_N),
	.sdram_we_n									(DRAM_WE_N),
	
//	// On chip sram for integral image
//	.onchip_memory_int_address    (onchip_sram_int_address),
//	.onchip_memory_int_clken      (1'b1),
//	.onchip_memory_int_chipselect (1'b1),
//	.onchip_memory_int_write      (onchip_sram_int_write),
//	.onchip_memory_int_readdata   (onchip_sram_int_readdata),
//	.onchip_memory_int_writedata  (onchip_sram_int_writedata),
//	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT),
	
//	.pio_source_pixel_export (source_pixel)
	.pio_col_external_connection_export(pio_col),
	.pio_color_external_connection_export(pio_color_read_wire),
	.pio_row_external_connection_export(pio_row),
	.pio_state_external_connection_export(pio_state),
	.pio_collectsingle_external_connection_export(pio_collectsingle),
	.pio_integral_data_external_connection_export(pio_integral_data_wire)
);

endmodule


////////////////////////////////////////
module convrgb2grey(
	input [7:0] color8b,
	output [7:0] grey8b
);

wire [8:0] sum;
assign sum = (color8b[7:5]) + (color8b[4:2]) + (color8b[1:0]); // [red 7:5, green 4:2, blue 1:0]
assign grey8b = sum[7:0]; // unsigned 

endmodule 



// module convgrey2rgb();

////////////////////////////////////////
module M10K_1K_8( // for source
    output reg [7:0] q,
    input [7:0] d,
    input [16:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
    reg [7:0] mem [76799:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d ;
		  end
        q <= mem[read_address] ; // q doesn't get d in this clock cycle
    end
endmodule


//////////////////////////////////////////
module M10K_25bit#(
	parameter WIDTH = 16
)( // for integral 
    output reg [WIDTH:0] q,
    input [WIDTH:0] d,
    input [16:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
    reg [WIDTH:0] mem [76799:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d ;
		  end
        q <= mem[read_address] ; // q doesn't get d in this clock cycle
    end
endmodule


module compute (
    input           clk, reset, START,
    input  signed   [7:0]    M10K_read_data_source,
    output signed   [7:0]    M10K_write_data_source_wire,
    output          [16:0]   M10K_read_address_source_wire, // 8 bits, 0 to 256
    output          [16:0]   M10K_write_address_source_wire, // 8 bits, 0 to 256
    output          M10K_write_source_wire, // write to M10K block [i]

    input  signed   [24:0]   M10K_read_data_int,
    output signed   [24:0]   M10K_write_data_int_wire,
    output          [16:0]   M10K_read_address_int_wire, // 8 bits, 0 to 256
    output          [16:0]   M10K_write_address_int_wire, // 8 bits, 0 to 256
    output          M10K_write_int_wire // write to M10K block [i]
);

reg signed      [7:0]   M10K_data_buffer_source;
reg signed      [7:0]   M10K_write_data_source;
reg             [16:0]   M10K_read_address_source; // 8 bits, 0 to 256
reg             [16:0]   M10K_write_address_source; // 8 bits, 0 to 256
reg             M10K_write_source; // write to M10K block [i]

assign M10K_write_data_source_wire    = M10K_write_data_source;
assign M10K_read_address_source_wire  = M10K_read_address_source;
assign M10K_write_address_source_wire = M10K_write_address_source;
assign M10K_write_source_wire         = M10K_write_source;



reg signed      [24:0]   M10K_data_buffer_int;
reg signed      [24:0]   M10K_write_data_int;
reg             [16:0]   M10K_read_address_int; // 8 bits, 0 to 256
reg             [16:0]   M10K_write_address_int; // 8 bits, 0 to 256
reg             M10K_write_int; // write to M10K block [i]

assign M10K_write_data_int_wire    = M10K_write_data_int;
assign M10K_read_address_int_wire  = M10K_read_address_int;
assign M10K_write_address_int_wire = M10K_write_address_int;
assign M10K_write_int_wire         = M10K_write_int;

localparam VID_IN_WIDTH = 320;
localparam VID_IN_HEIGHT = 240;

reg [8:0] row, col, col_to_add;

reg [4:0] state_reg ;
wire [4:0] state ;
assign state = state_reg;

reg [11:0] curr_row_source_data ;
reg [24:0] sum ;

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
        curr_row_source_data <= 0;
    end
    else begin
        if (state == 0) begin // reset state
            if(START) begin
                state_reg <= 1;
            end
            else begin state_reg <= state_reg; end
        end
        else if ( state == 1 ) begin // request values to read
            M10K_read_address_source <= (row * VID_IN_WIDTH) + col ;
            // M10K_read_address_source <= col ;

            if ( row > 0 ) begin // if row is not 0, get integral data from row-1, col
                M10K_read_address_int <= ((row - 1) * VID_IN_WIDTH) + col ; // curr row * num col + curr col 
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
            // curr_row_source_data[col] <= M10K_read_data_source;
            curr_row_source_data <= curr_row_source_data + M10K_read_data_source;
            if ( row > 0 ) begin
                // sum <= M10K_read_data_int;
                sum <= M10K_read_data_int + curr_row_source_data + M10K_read_data_source;
            end
            else begin 
                // sum <= 0; //M10K_read_data_source;
                sum <= curr_row_source_data + M10K_read_data_source;
            end
            state_reg <= state + 1;
        end
        else if ( state == 5 ) begin // repeat in state until added all curr_row_source_data
            // if (col_to_add <= col) begin // include this source 
            //     sum <= sum + curr_row_source_data[col_to_add];
            //     col_to_add <= col_to_add + 1;
            //     state_reg <= 5;
            // end
            // else begin // write to M10K block and go to next index
            //     M10K_write_data_int <= sum; 
            //     M10K_write_address_int <= (row * VID_IN_WIDTH) + col ;
            //     // M10K_write_address_int <= col ;
            //     M10K_write_int <= 1;
            //     state_reg <= 6; 
            //     col_to_add <= 0 ;

            //     // next index logic
            //     if ( col < VID_IN_WIDTH ) begin 
            //         col <= col + 1;
            //     end
            //     else begin 
            //         col <= 0;
            //         // check to see if reached last row 
            //         if ( row < VID_IN_HEIGHT ) begin 
            //             row <= row + 1;
            //         end
            //         else begin 
            //             row <= 0;
            //         end
            //     end
            // end

            // write to M10K block and go to next index
            M10K_write_data_int <= sum; 
            M10K_write_address_int <= (row * VID_IN_WIDTH) + col ;
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
                curr_row_source_data <= 0 ;
                // check to see if reached last row 
                if ( row < VID_IN_HEIGHT ) begin 
                    row <= row + 1;
                end
                else begin 
                    row <= 0;
                end
            end
        end
        else if ( state == 6 ) begin // wait state
            state_reg <= 1; 
        end
    end 
end

endmodule
