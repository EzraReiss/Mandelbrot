`define NUM_ITERATORS 160
`define COLS_PER_BANK ((640 + `NUM_ITERATORS - 1) / `NUM_ITERATORS) // ceil(640/N)
`define X_PIXEL_MAX (`COLS_PER_BANK - 1)
`define Y_PIXEL_MAX (480 - 1)
`define MEM_MAX (`COLS_PER_BANK * 480 - 1)
`define BASE_STEP 27'sd39000
`define ITER_MAX 1000


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

// VGA clock and reset lines
wire vga_pll_lock ;
wire vga_pll ;
reg  vga_reset ;

// M10k memory clock
wire 					M10k_pll ;
wire 					M10k_pll_locked ;

// Wires for connecting VGA driver to memory
wire 		[9:0]	next_x ;
wire 		[9:0] 	next_y ;

// Per-iterator write signals — stores 10-bit iter_count (color computed on VGA read side)
wire [9:0]  iter_write_data  [`NUM_ITERATORS-1:0];
wire [18:0] iter_write_addr  [`NUM_ITERATORS-1:0];
wire        iter_we          [`NUM_ITERATORS-1:0];
wire        iter_done        [`NUM_ITERATORS-1:0];

// Per-bank M10K read outputs (10-bit iter_count)
wire [9:0]  bank_q           [`NUM_ITERATORS-1:0];

// VGA read: select bank and convert iter_count → color on the read side (one shared instance)
wire [9:0] raw_iter_count;
assign raw_iter_count = bank_q[next_x % `NUM_ITERATORS];

wire [7:0] M10k_out;
color_scheme vga_colorizer (
	.clk(vga_pll),
	.counter(raw_iter_count),
	.color_reg(M10k_out)
);

wire [18:0] bank_read_addr;
assign bank_read_addr = (next_x / `NUM_ITERATORS) + `COLS_PER_BANK * next_y;

// Done when ALL iterators are finished
reg all_done;
integer di;
always @(*) begin
	all_done = 1'b1;
	for (di = 0; di < `NUM_ITERATORS; di = di + 1)
		all_done = all_done & iter_done[di];
end

wire [31:0] pio_reset;
wire pio_sync_reset;

reset_synchronizer synchronizer(
	.clk(M10k_pll),
	.async_rst_in(pio_reset[0]),
	.sync_rst_out(pio_sync_reset)
);


wire [31:0] pio_zoom;
wire signed [31:0] pio_pan_x;
wire signed [31:0] pio_pan_y;

// pio_start, pio_reset, pio_zoom, pio_pan_x/y: ARM→FPGA (ARM writes, FPGA reads)
// pio_done:                                      FPGA→ARM (FPGA writes, ARM reads)
wire [31:0] pio_start;
wire [31:0] pio_done;
wire [31:0] pio_iter_max;

// all_done is in M10k_pll domain; synchronize to HPS/FPGA fabric clock
reg [1:0] done_sync;
always @(posedge CLOCK_50) done_sync <= {done_sync[0], all_done};

assign pio_done[31:1] = 31'b0;
assign pio_done[0]    = done_sync[1];


// Reset logic for VGA (since we commented out the block above)
always@(posedge M10k_pll) begin
	if (~KEY[0]) begin
		vga_reset <= 1'b_1;
	end else begin
		vga_reset <= 1'b_0;
	end
end





// Instantiate VGA driver					
vga_driver DUT   (	.clock(vga_pll), 
							.reset(vga_reset),
							.color_in(M10k_out),	// Pixel color (8-bit) from selected memory bank
							.next_x(next_x),		// This (and next_y) used to specify memory read address
							.next_y(next_y),		// This (and next_x) used to specify memory read address
							.hsync(VGA_HS),
							.vsync(VGA_VS),
							.red(VGA_R),
							.green(VGA_G),
							.blue(VGA_B),
							.sync(VGA_SYNC_N),
							.clk(VGA_CLK),
							.blank(VGA_BLANK_N)
);

// Step size arithmetic (4.23 fixed-point):
//   Y step = BASE_STEP >> zoom          (each zoom level = 2× magnification)
//   X step = BASE_STEP * N_ITER >> zoom  (N iterators each handle every Nth column)
// pio_zoom[4:0] allows zoom levels 0-31 (zoom=0 is widest view, =15 is ~32000× in)
wire signed [26:0] pixel_increment_x;
wire signed [26:0] pixel_increment_y;

// SW[0] selects precision: 0 = 18-bit (fast), 1 = 27-bit (Karatsuba, accurate)
reg [1:0] sw0_sync;
always @(posedge M10k_pll) sw0_sync <= {sw0_sync[0], SW[0]};
wire is_high_resolution = sw0_sync[1];

assign pixel_increment_y = $signed(`BASE_STEP)                    >>> pio_zoom[4:0];
assign pixel_increment_x = $signed(`BASE_STEP * `NUM_ITERATORS)   >>> pio_zoom[4:0];

// Registered iterator offsets — computed once on reset instead of every cycle
reg signed [26:0] iterator_offset [`NUM_ITERATORS-1:0];
reg offsets_valid;

integer oi;
always @(posedge M10k_pll) begin
	if (~KEY[0] || pio_sync_reset) begin
		offsets_valid <= 1'b0;
	end else if (!offsets_valid) begin
		iterator_offset[0] <= 27'sd0;
		for (oi = 1; oi < `NUM_ITERATORS; oi = oi + 1)
			iterator_offset[oi] <= iterator_offset[oi-1] + pixel_increment_y;
		offsets_valid <= 1'b1;
	end
end

genvar gi;
generate 
	for (gi = 0; gi < `NUM_ITERATORS; gi = gi + 1) begin : gen_iter	
		mandelbrot_top #(
			.ITERATOR_ID(gi),
			.NUM_ITERATORS(`NUM_ITERATORS)
		) mandelbrot_unit (
			.reset(~KEY[0] || pio_sync_reset),
			.clk(M10k_pll),
			.x_start(pio_pan_x[26:0] + iterator_offset[gi]),
			.y_start(pio_pan_y[26:0]),
			.pixel_increment_x(pixel_increment_x),
			.pixel_increment_y(pixel_increment_y),
			.iter_max(pio_iter_max[9:0]),
			.is_high_resolution(is_high_resolution),

			.done(iter_done[gi]), 
			.mem_write_data(iter_write_data[gi]),
			.mem_write_address(iter_write_addr[gi]),
			.mem_we(iter_we[gi])
		);

		M10K_1000_8 #(
			.MEM_DEPTH(`COLS_PER_BANK * 480),
			.DATA_WIDTH(10)
		) pixel_data( 
			.q(bank_q[gi]),
			.d(iter_write_data[gi]),
			.write_address(iter_write_addr[gi]),
			.read_address(bank_read_addr),
			.we(iter_we[gi]),
			.clk(M10k_pll)
		);
	end
endgenerate

//=======================================================
//  Structural coding
//=======================================================
// From Qsys

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////
	.vga_pio_locked_export			(vga_pll_lock),           //       vga_pio_locked.export
	.vga_pio_outclk0_clk				(vga_pll),              //      vga_pio_outclk0.clk
	.m10k_pll_locked_export			(M10k_pll_locked),          //      m10k_pll_locked.export
	.m10k_pll_outclk0_clk			(M10k_pll),            //     m10k_pll_outclk0.clk

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),
	

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

	//pio
	.pio_finish_external_connection_export(pio_done),
	.pio_pan_x_external_connection_export (pio_pan_x),
	.pio_pan_y_external_connection_export (pio_pan_y),
	.pio_reset_external_connection_export (pio_reset),
	.pio_start_external_connection_export (pio_start),
	.pio_zoom_external_connection_export  (pio_zoom),
	.pio_iter_max_external_connection_export(pio_iter_max),
	
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
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT)
);
endmodule // end top level

// Declaration of module, include width and signedness of each input/output
module vga_driver (
	input wire clock,
	input wire reset,
	input [7:0] color_in,
	output [9:0] next_x,
	output [9:0] next_y,
	output wire hsync,
	output wire vsync,
	output [7:0] red,
	output [7:0] green,
	output [7:0] blue,
	output sync,
	output clk,
	output blank
);
	
	// Horizontal parameters (measured in clock cycles)
	parameter [9:0] H_ACTIVE  	=  10'd_639 ;
	parameter [9:0] H_FRONT 	=  10'd_15 ;
	parameter [9:0] H_PULSE		=  10'd_95 ;
	parameter [9:0] H_BACK 		=  10'd_47 ;

	// Vertical parameters (measured in lines)
	parameter [9:0] V_ACTIVE  	=  10'd_479 ;
	parameter [9:0] V_FRONT 	=  10'd_9 ;
	parameter [9:0] V_PULSE		=  10'd_1 ;
	parameter [9:0] V_BACK 		=  10'd_32 ;

//	// Horizontal parameters (measured in clock cycles)
//	parameter [9:0] H_ACTIVE  	=  10'd_9 ;
//	parameter [9:0] H_FRONT 	=  10'd_4 ;
//	parameter [9:0] H_PULSE		=  10'd_4 ;
//	parameter [9:0] H_BACK 		=  10'd_4 ;
//	parameter [9:0] H_TOTAL 	=  10'd_799 ;
//
//	// Vertical parameters (measured in lines)
//	parameter [9:0] V_ACTIVE  	=  10'd_1 ;
//	parameter [9:0] V_FRONT 	=  10'd_1 ;
//	parameter [9:0] V_PULSE		=  10'd_1 ;
//	parameter [9:0] V_BACK 		=  10'd_1 ;

	// Parameters for readability
	parameter 	LOW 	= 1'b_0 ;
	parameter 	HIGH	= 1'b_1 ;

	// States (more readable)
	parameter 	[7:0]	H_ACTIVE_STATE 		= 8'd_0 ;
	parameter 	[7:0] 	H_FRONT_STATE		= 8'd_1 ;
	parameter 	[7:0] 	H_PULSE_STATE 		= 8'd_2 ;
	parameter 	[7:0] 	H_BACK_STATE 		= 8'd_3 ;

	parameter 	[7:0]	V_ACTIVE_STATE 		= 8'd_0 ;
	parameter 	[7:0] 	V_FRONT_STATE		= 8'd_1 ;
	parameter 	[7:0] 	V_PULSE_STATE 		= 8'd_2 ;
	parameter 	[7:0] 	V_BACK_STATE 		= 8'd_3 ;

	// Clocked registers
	reg 		hysnc_reg ;
	reg 		vsync_reg ;
	reg 	[7:0]	red_reg ;
	reg 	[7:0]	green_reg ;
	reg 	[7:0]	blue_reg ;
	reg 		line_done ;

	// Control registers
	reg 	[9:0] 	h_counter ;
	reg 	[9:0] 	v_counter ;

	reg 	[7:0]	h_state ;
	reg 	[7:0]	v_state ;

	// State machine
	always@(posedge clock) begin
		// At reset . . .
  		if (reset) begin
			// Zero the counters
			h_counter 	<= 10'd_0 ;
			v_counter 	<= 10'd_0 ;
			// States to ACTIVE
			h_state 	<= H_ACTIVE_STATE  ;
			v_state 	<= V_ACTIVE_STATE  ;
			// Deassert line done
			line_done 	<= LOW ;
  		end
  		else begin
			//////////////////////////////////////////////////////////////////////////
			///////////////////////// HORIZONTAL /////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			if (h_state == H_ACTIVE_STATE) begin
				// Iterate horizontal counter, zero at end of ACTIVE mode
				h_counter <= (h_counter==H_ACTIVE)?10'd_0:(h_counter + 10'd_1) ;
				// Set hsync
				hysnc_reg <= HIGH ;
				// Deassert line done
				line_done <= LOW ;
				// State transition
				h_state <= (h_counter == H_ACTIVE)?H_FRONT_STATE:H_ACTIVE_STATE ;
			end
			// Assert done flag, wait here for reset
			if (h_state == H_FRONT_STATE) begin
				// Iterate horizontal counter, zero at end of H_FRONT mode
				h_counter <= (h_counter==H_FRONT)?10'd_0:(h_counter + 10'd_1) ;
				// Set hsync
				hysnc_reg <= HIGH ;
				// State transition
				h_state <= (h_counter == H_FRONT)?H_PULSE_STATE:H_FRONT_STATE ;
			end
			if (h_state == H_PULSE_STATE) begin
				// Iterate horizontal counter, zero at end of H_FRONT mode
				h_counter <= (h_counter==H_PULSE)?10'd_0:(h_counter + 10'd_1) ;
				// Set hsync
				hysnc_reg <= LOW ;
				// State transition
				h_state <= (h_counter == H_PULSE)?H_BACK_STATE:H_PULSE_STATE ;
			end
			if (h_state == H_BACK_STATE) begin
				// Iterate horizontal counter, zero at end of H_FRONT mode
				h_counter <= (h_counter==H_BACK)?10'd_0:(h_counter + 10'd_1) ;
				// Set hsync
				hysnc_reg <= HIGH ;
				// State transition
				h_state <= (h_counter == H_BACK)?H_ACTIVE_STATE:H_BACK_STATE ;
				// Signal line complete at state transition (offset by 1 for synchronous state transition)
				line_done <= (h_counter == (H_BACK-1))?HIGH:LOW ;
			end
			//////////////////////////////////////////////////////////////////////////
			///////////////////////// VERTICAL ///////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			if (v_state == V_ACTIVE_STATE) begin
				// increment vertical counter at end of line, zero on state transition
				v_counter <= (line_done==HIGH)?((v_counter==V_ACTIVE)?10'd_0:(v_counter + 10'd_1)):v_counter ;
				// set vsync in active mode
				vsync_reg <= HIGH ;
				// state transition - only on end of lines
				v_state <= (line_done==HIGH)?((v_counter==V_ACTIVE)?V_FRONT_STATE:V_ACTIVE_STATE):V_ACTIVE_STATE ;
			end
			if (v_state == V_FRONT_STATE) begin
				// increment vertical counter at end of line, zero on state transition
				v_counter <= (line_done==HIGH)?((v_counter==V_FRONT)?10'd_0:(v_counter + 10'd_1)):v_counter ;
				// set vsync in front porch
				vsync_reg <= HIGH ;
				// state transition
				v_state <= (line_done==HIGH)?((v_counter==V_FRONT)?V_PULSE_STATE:V_FRONT_STATE):V_FRONT_STATE ;
			end
			if (v_state == V_PULSE_STATE) begin
				// increment vertical counter at end of line, zero on state transition
				v_counter <= (line_done==HIGH)?((v_counter==V_PULSE)?10'd_0:(v_counter + 10'd_1)):v_counter ;
				// clear vsync in pulse
				vsync_reg <= LOW ;
				// state transition
				v_state <= (line_done==HIGH)?((v_counter==V_PULSE)?V_BACK_STATE:V_PULSE_STATE):V_PULSE_STATE ;
			end
			if (v_state == V_BACK_STATE) begin
				// increment vertical counter at end of line, zero on state transition
				v_counter <= (line_done==HIGH)?((v_counter==V_BACK)?10'd_0:(v_counter + 10'd_1)):v_counter ;
				// set vsync in back porch
				vsync_reg <= HIGH ;
				// state transition
				v_state <= (line_done==HIGH)?((v_counter==V_BACK)?V_ACTIVE_STATE:V_BACK_STATE):V_BACK_STATE ;
			end

			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////// COLOR OUT ///////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			red_reg 		<= (h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[7:5],5'd_0}:8'd_0):8'd_0 ;
			green_reg 	<= (h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[4:2],5'd_0}:8'd_0):8'd_0 ;
			blue_reg 	<= (h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[1:0],6'd_0}:8'd_0):8'd_0 ;
			
 	 	end
	end
	// Assign output values
	assign hsync = hysnc_reg ;
	assign vsync = vsync_reg ;
	assign red = red_reg ;
	assign green = green_reg ;
	assign blue = blue_reg ;
	assign clk = clock ;
	assign sync = 1'b_0 ;
	assign blank = hysnc_reg & vsync_reg ;
	// The x/y coordinates that should be available on the NEXT cycle
	assign next_x = (h_state==H_ACTIVE_STATE)?h_counter:10'd_0 ;
	assign next_y = (v_state==V_ACTIVE_STATE)?v_counter:10'd_0 ;

endmodule




//============================================================
// M10K module for testing
//============================================================
// See example 12-16 in 
// http://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HDL_style_qts_qii51007.pdf
//============================================================


module reset_synchronizer (
    input clk,
    input async_rst_in,
    output reg sync_rst_out
);

    reg stage1;

    always @(posedge clk) begin
        // Shift the asynchronous signal through two flops
        stage1       <= async_rst_in;
        sync_rst_out <= stage1;
    end

endmodule

/*
AI made this:
opus modified this M10K module to be able to generate non-powers of 2 sizes
so it can make it do 5 BRAMs per M10K for 64 iterators
*/
module M10K_1000_8 #(
    parameter MEM_DEPTH = 307200,
    parameter DATA_WIDTH = 10
)( 
    output reg [DATA_WIDTH-1:0] q,
    input [DATA_WIDTH-1:0] d,
    input [18:0] write_address, read_address,
    input we, clk
);
    localparam BLOCK_DEPTH = (DATA_WIDTH <= 8) ? 1024 :
                             (DATA_WIDTH <= 10) ? 1024 : 512;
    localparam NUM_BLOCKS  = (MEM_DEPTH + BLOCK_DEPTH - 1) / BLOCK_DEPTH;
    localparam SEL_BITS    = (NUM_BLOCKS > 1) ? $clog2(NUM_BLOCKS) : 1;

    wire [9:0]          sub_addr_w   = write_address[9:0];
    wire [9:0]          sub_addr_r   = read_address[9:0];
    wire [SEL_BITS-1:0] block_sel_w  = write_address[SEL_BITS+9:10];
    wire [SEL_BITS-1:0] block_sel_r  = read_address[SEL_BITS+9:10];

    reg [DATA_WIDTH-1:0] sub_q [0:NUM_BLOCKS-1];
    reg [SEL_BITS-1:0]   block_sel_r_reg;

    always @(posedge clk)
        block_sel_r_reg <= block_sel_r;

    genvar i;
    generate
        for (i = 0; i < NUM_BLOCKS; i = i + 1) begin : blk
            reg [DATA_WIDTH-1:0] mem [0:BLOCK_DEPTH-1] /* synthesis ramstyle = "no_rw_check, M10K" */;
            always @(posedge clk) begin
                if (we && block_sel_w == i[SEL_BITS-1:0])
                    mem[sub_addr_w] <= d;
                sub_q[i] <= mem[sub_addr_r];
            end
        end
    endgenerate

    always @(*)
        q = sub_q[block_sel_r_reg];
endmodule

module full_precision_mult_18_bit (
    output signed [35:0] out,
    input  signed [17:0] a,
    input  signed [17:0] b
);
    assign out = a * b;
endmodule

// Variable-precision multiplier: one 18x18 DSP block.
// Low-res: combinational Q4.14 multiply (0 extra cycles).
// High-res: Karatsuba Q4.23 multiply (3 extra cycles).
module karatsuba_multiplier (
    input clk,
    input reset,
    input is_high_resolution,
    input signed [26:0] a,
    input signed [26:0] b,
    output reg signed [26:0] out,
    input in_val,
    input out_rdy,
    output reg out_val,
    output reg in_rdy
);
    localparam [1:0] IDLE   = 2'b00,
                     CALC_0 = 2'b01,
                     CALC_1 = 2'b10,
                     CALC_2 = 2'b11;

    reg [1:0] current_state;
    reg [1:0] next_state;

    reg  signed [17:0] fpm_in_a, fpm_in_b;
    wire signed [35:0] fpm_out;
    full_precision_mult_18_bit fpm(fpm_out, fpm_in_a, fpm_in_b);

    wire signed [17:0] lo_a = a[26:9];
    wire signed [17:0] lo_b = b[26:9];
    wire signed [17:0] lo_out = {fpm_out[35], fpm_out[30:14]};

    wire sign_out = a[26] ^ b[26];
    wire [26:0] abs_a = a[26] ? (~a + 1) : a;
    wire [26:0] abs_b = b[26] ? (~b + 1) : b;

    wire [15:0] AL = abs_a[15:0];
    wire [10:0] AH = abs_a[26:16];
    wire [15:0] BL = abs_b[15:0];
    wire [10:0] BH = abs_b[26:16];

    wire [17:0] AH_ext = {7'b0, AH};
    wire [17:0] BH_ext = {7'b0, BH};
    wire [17:0] AL_ext = {2'b0, AL};
    wire [17:0] BL_ext = {2'b0, BL};
    wire [17:0] SA_ext = {2'b0, AL} + {7'b0, AH};
    wire [17:0] SB_ext = {2'b0, BL} + {7'b0, BH};

    reg [21:0] Z2_reg;
    reg [31:0] Z0_reg;
    reg [33:0] M_reg;

    wire [53:0] Z0_w = {22'b0, Z0_reg};
    wire [53:0] Z2_w = {32'b0, Z2_reg};
    wire [53:0] M_w  = {20'b0, M_reg};
    wire [53:0] Z1_w = M_w - Z0_w - Z2_w;
    wire [53:0] p_mag = Z0_w + (Z1_w << 16) + (Z2_w << 32);

    wire [53:0] p_signed = sign_out ? (~p_mag + 1'b1) : p_mag;
    wire signed [26:0] hi_out = {p_signed[53], p_signed[48:23]};

    always @(posedge clk) begin
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
            case (current_state)
                IDLE: begin
                    if (in_val && is_high_resolution)
                        Z2_reg <= fpm_out[21:0];
                end
                CALC_0:
                    Z0_reg <= fpm_out[31:0];
                CALC_1:
                    M_reg  <= fpm_out[33:0];
                default: ;
            endcase
        end
    end

    always @(*) begin
        next_state = current_state;
        fpm_in_a   = 18'sd0;
        fpm_in_b   = 18'sd0;
        out        = 27'sd0;
        out_val    = 1'b0;
        in_rdy     = 1'b0;

        case (current_state)
            IDLE: begin
                in_rdy = 1'b1;
                if (in_val && !is_high_resolution) begin
                    fpm_in_a = lo_a;
                    fpm_in_b = lo_b;
                    out      = {lo_out, 9'b0};
                    out_val  = 1'b1;
                end else if (in_val && is_high_resolution) begin
                    fpm_in_a   = AH_ext;
                    fpm_in_b   = BH_ext;
                    next_state = CALC_0;
                end
            end
            CALC_0: begin
                fpm_in_a   = AL_ext;
                fpm_in_b   = BL_ext;
                next_state = CALC_1;
            end
            CALC_1: begin
                fpm_in_a   = SA_ext;
                fpm_in_b   = SB_ext;
                next_state = CALC_2;
            end
            CALC_2: begin
                out     = hi_out;
                out_val = 1'b1;
                if (out_rdy)
                    next_state = IDLE;
            end
        endcase
    end
endmodule

// FSM Mandelbrot iterator — uses one karatsuba_multiplier (one 18x18 DSP)
module fsm_iterator (
    input reset,
    input clk,
    input in_val,
    output reg in_rdy,
    input is_high_resolution,

    input signed [26:0] in_c_r,
    input signed [26:0] in_c_i,
    input [9:0] iter_max,
    output reg [9:0] iter_count,
    output escape_condition,
    output reg out_val,
    input out_rdy
);
    localparam [2:0] IDLE   = 3'b000,
                     CALC_1 = 3'b001,
                     CALC_2 = 3'b010,
                     CALC_3 = 3'b011,
                     DONE   = 3'b100;

    reg [2:0] current_state;
    reg [2:0] next_state;

    reg signed [26:0] zi, zr, c_r, c_i;
    reg signed [26:0] zr_sq_reg, zi_sq_reg;
    wire signed [26:0] zr_next, zi_next, z_mag_sq;

    reg  signed [26:0] kara_a, kara_b;
    wire signed [26:0] kara_out;
    reg  kara_in_val, kara_out_rdy;
    wire kara_out_val, kara_in_rdy;

    karatsuba_multiplier mult (
        .clk(clk), .reset(reset),
        .is_high_resolution(is_high_resolution),
        .a(kara_a), .b(kara_b), .out(kara_out),
        .in_val(kara_in_val), .out_rdy(kara_out_rdy),
        .out_val(kara_out_val), .in_rdy(kara_in_rdy)
    );

    always @(posedge clk) begin
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
            case (current_state)
                IDLE: begin
                    if (in_val) begin
                        c_r <= in_c_r;
                        c_i <= in_c_i;
                        zi <= 27'sd0;
                        zr <= 27'sd0;
                        zr_sq_reg <= 27'sd0;
                        zi_sq_reg <= 27'sd0;
                        iter_count <= 0;
                    end
                end
                CALC_1: begin
                    if (kara_out_val)
                        zr_sq_reg <= kara_out;
                end
                CALC_2: begin
                    if (kara_out_val)
                        zi_sq_reg <= kara_out;
                end
                CALC_3: begin
                    if (kara_out_val) begin
                        zi <= zi_next;
                        zr <= zr_next;
                        iter_count <= iter_count + 1;
                    end
                end
                default: ;
            endcase
        end
    end

    always @(*) begin
        next_state = current_state;
        in_rdy = 1'b0;
        out_val = 1'b0;

        case (current_state)
            IDLE: begin
                in_rdy = 1'b1;
                if (in_val)
                    next_state = CALC_1;
            end
            CALC_1: begin
                if (kara_out_val)
                    next_state = CALC_2;
            end
            CALC_2: begin
                if (kara_out_val)
                    next_state = CALC_3;
            end
            CALC_3: begin
                if (kara_out_val) begin
                    if (escape_condition)
                        next_state = DONE;
                    else
                        next_state = CALC_1;
                end
            end
            DONE: begin
                out_val = 1'b1;
                if (out_rdy)
                    next_state = IDLE;
            end
        endcase
    end

    always @(*) begin
        kara_a       = 27'sd0;
        kara_b       = 27'sd0;
        kara_in_val  = 1'b0;
        kara_out_rdy = 1'b0;

        case (current_state)
            CALC_1: begin
                kara_a = zr; kara_b = zr;
                kara_in_val = kara_in_rdy;
                kara_out_rdy = 1'b1;
            end
            CALC_2: begin
                kara_a = zi; kara_b = zi;
                kara_in_val = kara_in_rdy;
                kara_out_rdy = 1'b1;
            end
            CALC_3: begin
                kara_a = zr; kara_b = zi;
                kara_in_val = kara_in_rdy;
                kara_out_rdy = 1'b1;
            end
            default: ;
        endcase
    end

    assign zr_next = zr_sq_reg - zi_sq_reg + c_r;
    assign zi_next = (kara_out <<< 1) + c_i;
    assign z_mag_sq = zr_sq_reg + zi_sq_reg;

    assign escape_condition = (z_mag_sq > $signed(27'h2000000))
                            || z_mag_sq[26]
                            || (iter_count == iter_max - 1)
                            || (zr > $signed(27'h1000000))
                            || (zr < $signed(-27'h1000000))
                            || (zi > $signed(27'h1000000))
                            || (zi < $signed(-27'h1000000));
endmodule

module mandelbrot_top #(
    parameter ITERATOR_ID   = 0,
    parameter NUM_ITERATORS = 1
) (
    input reset,
    input clk,
    input signed [26:0] x_start,
    input signed [26:0] y_start,
    input signed [26:0] pixel_increment_x,
    input signed [26:0] pixel_increment_y,
    input [9:0] iter_max,
    input is_high_resolution,

    output reg done,
    output [9:0] mem_write_data,
    output reg [18:0] mem_write_address,
    output reg mem_we
);
    localparam [1:0] CALC = 2'b01,
                     DONE = 2'b10;

    reg [9:0] iter_max_reg;
    reg       hires_reg;

    reg  [1:0] current_state;
    reg  [1:0] next_state;

    reg signed [26:0] curr_x, curr_y;
    reg [9:0] pixel_x, pixel_y;
    reg [9:0] next_pixel_x, next_pixel_y;
    reg signed [26:0] next_x, next_y;

    reg iterator_reset;
    reg iterator_in_val;
    wire iterator_in_rdy;
    wire iterator_out_val;
    reg iterator_out_rdy;
    wire [9:0] iterator_iter_count;
    wire iterator_escape_condition;

    assign mem_write_data = iterator_iter_count;

    fsm_iterator iter1 (
        .reset(iterator_reset),
        .clk(clk),
        .in_val(iterator_in_val),
        .in_rdy(iterator_in_rdy),
        .is_high_resolution(hires_reg),
        .in_c_r(curr_x),
        .in_c_i(curr_y),
        .iter_count(iterator_iter_count),
        .iter_max(iter_max_reg),
        .escape_condition(iterator_escape_condition),
        .out_val(iterator_out_val),
        .out_rdy(iterator_out_rdy)
    );

    always @(posedge clk) begin
        if (reset) begin
            done <= 1'b0;
            current_state <= CALC;
            curr_x <= x_start;
            curr_y <= y_start;
            pixel_x <= 0;
            pixel_y <= 0;
            iterator_reset <= 1'b1;
            iter_max_reg <= iter_max;
            hires_reg <= is_high_resolution;
            mem_write_address <= 0;
            mem_we <= 1'b0;
            iterator_in_val <= 1'b0;
            iterator_out_rdy <= 1'b0;
        end else begin
            iterator_reset <= 1'b0;
            current_state <= next_state;
            case (current_state)
                CALC: begin
                    if (iterator_in_rdy && !iterator_in_val) begin
                        iterator_in_val <= 1'b1;
                        iterator_out_rdy <= 1'b0;
                        mem_we <= 1'b0;
                        if (mem_write_address != 0 || pixel_x != 0 || pixel_y != 0)
                            mem_write_address <= mem_write_address + 1;
                    end
                    else if (iterator_in_val) begin
                        iterator_in_val <= 1'b0;
                        mem_we <= 1'b0;
                    end
                    else if (iterator_out_val) begin
                        iterator_out_rdy <= 1'b1;
                        mem_we <= 1'b1;
                        curr_x <= next_x;
                        curr_y <= next_y;
                        pixel_x <= next_pixel_x;
                        pixel_y <= next_pixel_y;
                    end
                    else begin
                        iterator_in_val <= 1'b0;
                        iterator_out_rdy <= 1'b1;
                        mem_we <= 1'b0;
                    end
                end
                DONE: begin
                    done <= 1'b1;
                    mem_we <= 1'b0;
                end
            endcase
        end
    end

    always @(*) begin
        if (pixel_x == `X_PIXEL_MAX) begin
            next_pixel_x = 0;
            next_pixel_y = pixel_y + 1;
            next_x = x_start;
            next_y = curr_y - pixel_increment_y;
        end else begin
            next_pixel_x = pixel_x + 1;
            next_x = curr_x + pixel_increment_x;
            next_pixel_y = pixel_y;
            next_y = curr_y;
        end
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            CALC: begin
                if (iterator_out_val && pixel_x == `X_PIXEL_MAX && pixel_y == `Y_PIXEL_MAX)
                    next_state = DONE;
            end
            DONE:    next_state = DONE;
            default: next_state = CALC;
        endcase
    end
endmodule

module color_scheme #(
    parameter ITER_MAX = `ITER_MAX
)(
    input clk,
    input [9:0] counter,
    output reg [7:0] color_reg
);
    always @(*) begin
        if      (counter >= ITER_MAX)      color_reg = 8'b_000_000_00;
        else if (counter >= ITER_MAX >> 1) color_reg = 8'b_011_001_00;
        else if (counter >= ITER_MAX >> 2) color_reg = 8'b_011_001_00;
        else if (counter >= ITER_MAX >> 3) color_reg = 8'b_101_010_01;
        else if (counter >= ITER_MAX >> 4) color_reg = 8'b_011_001_01;
        else if (counter >= ITER_MAX >> 5) color_reg = 8'b_001_001_01;
        else if (counter >= ITER_MAX >> 6) color_reg = 8'b_011_010_10;
        else if (counter >= ITER_MAX >> 7) color_reg = 8'b_010_100_10;
        else                               color_reg = 8'b_010_100_10;
    end
endmodule
