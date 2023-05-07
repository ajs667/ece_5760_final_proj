	component Computer_System is
		port (
			av_config_SDAT                               : inout std_logic                     := 'X';             -- SDAT
			av_config_SCLK                               : out   std_logic;                                        -- SCLK
			clock_bridge_0_in_clk_clk                    : in    std_logic                     := 'X';             -- clk
			ebab_video_in_external_interface_address     : in    std_logic_vector(29 downto 0) := (others => 'X'); -- address
			ebab_video_in_external_interface_byte_enable : in    std_logic                     := 'X';             -- byte_enable
			ebab_video_in_external_interface_read        : in    std_logic                     := 'X';             -- read
			ebab_video_in_external_interface_write       : in    std_logic                     := 'X';             -- write
			ebab_video_in_external_interface_write_data  : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- write_data
			ebab_video_in_external_interface_acknowledge : out   std_logic;                                        -- acknowledge
			ebab_video_in_external_interface_read_data   : out   std_logic_vector(7 downto 0);                     -- read_data
			hps_io_hps_io_emac1_inst_TX_CLK              : out   std_logic;                                        -- hps_io_emac1_inst_TX_CLK
			hps_io_hps_io_emac1_inst_TXD0                : out   std_logic;                                        -- hps_io_emac1_inst_TXD0
			hps_io_hps_io_emac1_inst_TXD1                : out   std_logic;                                        -- hps_io_emac1_inst_TXD1
			hps_io_hps_io_emac1_inst_TXD2                : out   std_logic;                                        -- hps_io_emac1_inst_TXD2
			hps_io_hps_io_emac1_inst_TXD3                : out   std_logic;                                        -- hps_io_emac1_inst_TXD3
			hps_io_hps_io_emac1_inst_RXD0                : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD0
			hps_io_hps_io_emac1_inst_MDIO                : inout std_logic                     := 'X';             -- hps_io_emac1_inst_MDIO
			hps_io_hps_io_emac1_inst_MDC                 : out   std_logic;                                        -- hps_io_emac1_inst_MDC
			hps_io_hps_io_emac1_inst_RX_CTL              : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CTL
			hps_io_hps_io_emac1_inst_TX_CTL              : out   std_logic;                                        -- hps_io_emac1_inst_TX_CTL
			hps_io_hps_io_emac1_inst_RX_CLK              : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RX_CLK
			hps_io_hps_io_emac1_inst_RXD1                : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD1
			hps_io_hps_io_emac1_inst_RXD2                : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD2
			hps_io_hps_io_emac1_inst_RXD3                : in    std_logic                     := 'X';             -- hps_io_emac1_inst_RXD3
			hps_io_hps_io_qspi_inst_IO0                  : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO0
			hps_io_hps_io_qspi_inst_IO1                  : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO1
			hps_io_hps_io_qspi_inst_IO2                  : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO2
			hps_io_hps_io_qspi_inst_IO3                  : inout std_logic                     := 'X';             -- hps_io_qspi_inst_IO3
			hps_io_hps_io_qspi_inst_SS0                  : out   std_logic;                                        -- hps_io_qspi_inst_SS0
			hps_io_hps_io_qspi_inst_CLK                  : out   std_logic;                                        -- hps_io_qspi_inst_CLK
			hps_io_hps_io_sdio_inst_CMD                  : inout std_logic                     := 'X';             -- hps_io_sdio_inst_CMD
			hps_io_hps_io_sdio_inst_D0                   : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D0
			hps_io_hps_io_sdio_inst_D1                   : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D1
			hps_io_hps_io_sdio_inst_CLK                  : out   std_logic;                                        -- hps_io_sdio_inst_CLK
			hps_io_hps_io_sdio_inst_D2                   : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D2
			hps_io_hps_io_sdio_inst_D3                   : inout std_logic                     := 'X';             -- hps_io_sdio_inst_D3
			hps_io_hps_io_usb1_inst_D0                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D0
			hps_io_hps_io_usb1_inst_D1                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D1
			hps_io_hps_io_usb1_inst_D2                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D2
			hps_io_hps_io_usb1_inst_D3                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D3
			hps_io_hps_io_usb1_inst_D4                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D4
			hps_io_hps_io_usb1_inst_D5                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D5
			hps_io_hps_io_usb1_inst_D6                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D6
			hps_io_hps_io_usb1_inst_D7                   : inout std_logic                     := 'X';             -- hps_io_usb1_inst_D7
			hps_io_hps_io_usb1_inst_CLK                  : in    std_logic                     := 'X';             -- hps_io_usb1_inst_CLK
			hps_io_hps_io_usb1_inst_STP                  : out   std_logic;                                        -- hps_io_usb1_inst_STP
			hps_io_hps_io_usb1_inst_DIR                  : in    std_logic                     := 'X';             -- hps_io_usb1_inst_DIR
			hps_io_hps_io_usb1_inst_NXT                  : in    std_logic                     := 'X';             -- hps_io_usb1_inst_NXT
			hps_io_hps_io_spim1_inst_CLK                 : out   std_logic;                                        -- hps_io_spim1_inst_CLK
			hps_io_hps_io_spim1_inst_MOSI                : out   std_logic;                                        -- hps_io_spim1_inst_MOSI
			hps_io_hps_io_spim1_inst_MISO                : in    std_logic                     := 'X';             -- hps_io_spim1_inst_MISO
			hps_io_hps_io_spim1_inst_SS0                 : out   std_logic;                                        -- hps_io_spim1_inst_SS0
			hps_io_hps_io_uart0_inst_RX                  : in    std_logic                     := 'X';             -- hps_io_uart0_inst_RX
			hps_io_hps_io_uart0_inst_TX                  : out   std_logic;                                        -- hps_io_uart0_inst_TX
			hps_io_hps_io_i2c0_inst_SDA                  : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SDA
			hps_io_hps_io_i2c0_inst_SCL                  : inout std_logic                     := 'X';             -- hps_io_i2c0_inst_SCL
			hps_io_hps_io_i2c1_inst_SDA                  : inout std_logic                     := 'X';             -- hps_io_i2c1_inst_SDA
			hps_io_hps_io_i2c1_inst_SCL                  : inout std_logic                     := 'X';             -- hps_io_i2c1_inst_SCL
			hps_io_hps_io_gpio_inst_GPIO09               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO09
			hps_io_hps_io_gpio_inst_GPIO35               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO35
			hps_io_hps_io_gpio_inst_GPIO40               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO40
			hps_io_hps_io_gpio_inst_GPIO41               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO41
			hps_io_hps_io_gpio_inst_GPIO48               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO48
			hps_io_hps_io_gpio_inst_GPIO53               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO53
			hps_io_hps_io_gpio_inst_GPIO54               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO54
			hps_io_hps_io_gpio_inst_GPIO61               : inout std_logic                     := 'X';             -- hps_io_gpio_inst_GPIO61
			memory_mem_a                                 : out   std_logic_vector(14 downto 0);                    -- mem_a
			memory_mem_ba                                : out   std_logic_vector(2 downto 0);                     -- mem_ba
			memory_mem_ck                                : out   std_logic;                                        -- mem_ck
			memory_mem_ck_n                              : out   std_logic;                                        -- mem_ck_n
			memory_mem_cke                               : out   std_logic;                                        -- mem_cke
			memory_mem_cs_n                              : out   std_logic;                                        -- mem_cs_n
			memory_mem_ras_n                             : out   std_logic;                                        -- mem_ras_n
			memory_mem_cas_n                             : out   std_logic;                                        -- mem_cas_n
			memory_mem_we_n                              : out   std_logic;                                        -- mem_we_n
			memory_mem_reset_n                           : out   std_logic;                                        -- mem_reset_n
			memory_mem_dq                                : inout std_logic_vector(31 downto 0) := (others => 'X'); -- mem_dq
			memory_mem_dqs                               : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs
			memory_mem_dqs_n                             : inout std_logic_vector(3 downto 0)  := (others => 'X'); -- mem_dqs_n
			memory_mem_odt                               : out   std_logic;                                        -- mem_odt
			memory_mem_dm                                : out   std_logic_vector(3 downto 0);                     -- mem_dm
			memory_oct_rzqin                             : in    std_logic                     := 'X';             -- oct_rzqin
			pio_col_external_connection_export           : in    std_logic_vector(9 downto 0)  := (others => 'X'); -- export
			pio_collectsingle_external_connection_export : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- export
			pio_color_external_connection_export         : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- export
			pio_row_external_connection_export           : in    std_logic_vector(9 downto 0)  := (others => 'X'); -- export
			pio_state_external_connection_export         : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- export
			sdram_addr                                   : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_ba                                     : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_cas_n                                  : out   std_logic;                                        -- cas_n
			sdram_cke                                    : out   std_logic;                                        -- cke
			sdram_cs_n                                   : out   std_logic;                                        -- cs_n
			sdram_dq                                     : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			sdram_dqm                                    : out   std_logic_vector(1 downto 0);                     -- dqm
			sdram_ras_n                                  : out   std_logic;                                        -- ras_n
			sdram_we_n                                   : out   std_logic;                                        -- we_n
			sdram_clk_clk                                : out   std_logic;                                        -- clk
			system_pll_ref_clk_clk                       : in    std_logic                     := 'X';             -- clk
			system_pll_ref_reset_reset                   : in    std_logic                     := 'X';             -- reset
			vga_CLK                                      : out   std_logic;                                        -- CLK
			vga_HS                                       : out   std_logic;                                        -- HS
			vga_VS                                       : out   std_logic;                                        -- VS
			vga_BLANK                                    : out   std_logic;                                        -- BLANK
			vga_SYNC                                     : out   std_logic;                                        -- SYNC
			vga_R                                        : out   std_logic_vector(7 downto 0);                     -- R
			vga_G                                        : out   std_logic_vector(7 downto 0);                     -- G
			vga_B                                        : out   std_logic_vector(7 downto 0);                     -- B
			vga_pll_ref_clk_clk                          : in    std_logic                     := 'X';             -- clk
			vga_pll_ref_reset_reset                      : in    std_logic                     := 'X';             -- reset
			video_in_TD_CLK27                            : in    std_logic                     := 'X';             -- TD_CLK27
			video_in_TD_DATA                             : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- TD_DATA
			video_in_TD_HS                               : in    std_logic                     := 'X';             -- TD_HS
			video_in_TD_VS                               : in    std_logic                     := 'X';             -- TD_VS
			video_in_clk27_reset                         : in    std_logic                     := 'X';             -- clk27_reset
			video_in_TD_RESET                            : out   std_logic;                                        -- TD_RESET
			video_in_overflow_flag                       : out   std_logic;                                        -- overflow_flag
			pio_integral_data_external_connection_export : in    std_logic_vector(31 downto 0) := (others => 'X')  -- export
		);
	end component Computer_System;

	u0 : component Computer_System
		port map (
			av_config_SDAT                               => CONNECTED_TO_av_config_SDAT,                               --                             av_config.SDAT
			av_config_SCLK                               => CONNECTED_TO_av_config_SCLK,                               --                                      .SCLK
			clock_bridge_0_in_clk_clk                    => CONNECTED_TO_clock_bridge_0_in_clk_clk,                    --                 clock_bridge_0_in_clk.clk
			ebab_video_in_external_interface_address     => CONNECTED_TO_ebab_video_in_external_interface_address,     --      ebab_video_in_external_interface.address
			ebab_video_in_external_interface_byte_enable => CONNECTED_TO_ebab_video_in_external_interface_byte_enable, --                                      .byte_enable
			ebab_video_in_external_interface_read        => CONNECTED_TO_ebab_video_in_external_interface_read,        --                                      .read
			ebab_video_in_external_interface_write       => CONNECTED_TO_ebab_video_in_external_interface_write,       --                                      .write
			ebab_video_in_external_interface_write_data  => CONNECTED_TO_ebab_video_in_external_interface_write_data,  --                                      .write_data
			ebab_video_in_external_interface_acknowledge => CONNECTED_TO_ebab_video_in_external_interface_acknowledge, --                                      .acknowledge
			ebab_video_in_external_interface_read_data   => CONNECTED_TO_ebab_video_in_external_interface_read_data,   --                                      .read_data
			hps_io_hps_io_emac1_inst_TX_CLK              => CONNECTED_TO_hps_io_hps_io_emac1_inst_TX_CLK,              --                                hps_io.hps_io_emac1_inst_TX_CLK
			hps_io_hps_io_emac1_inst_TXD0                => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD0,                --                                      .hps_io_emac1_inst_TXD0
			hps_io_hps_io_emac1_inst_TXD1                => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD1,                --                                      .hps_io_emac1_inst_TXD1
			hps_io_hps_io_emac1_inst_TXD2                => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD2,                --                                      .hps_io_emac1_inst_TXD2
			hps_io_hps_io_emac1_inst_TXD3                => CONNECTED_TO_hps_io_hps_io_emac1_inst_TXD3,                --                                      .hps_io_emac1_inst_TXD3
			hps_io_hps_io_emac1_inst_RXD0                => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD0,                --                                      .hps_io_emac1_inst_RXD0
			hps_io_hps_io_emac1_inst_MDIO                => CONNECTED_TO_hps_io_hps_io_emac1_inst_MDIO,                --                                      .hps_io_emac1_inst_MDIO
			hps_io_hps_io_emac1_inst_MDC                 => CONNECTED_TO_hps_io_hps_io_emac1_inst_MDC,                 --                                      .hps_io_emac1_inst_MDC
			hps_io_hps_io_emac1_inst_RX_CTL              => CONNECTED_TO_hps_io_hps_io_emac1_inst_RX_CTL,              --                                      .hps_io_emac1_inst_RX_CTL
			hps_io_hps_io_emac1_inst_TX_CTL              => CONNECTED_TO_hps_io_hps_io_emac1_inst_TX_CTL,              --                                      .hps_io_emac1_inst_TX_CTL
			hps_io_hps_io_emac1_inst_RX_CLK              => CONNECTED_TO_hps_io_hps_io_emac1_inst_RX_CLK,              --                                      .hps_io_emac1_inst_RX_CLK
			hps_io_hps_io_emac1_inst_RXD1                => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD1,                --                                      .hps_io_emac1_inst_RXD1
			hps_io_hps_io_emac1_inst_RXD2                => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD2,                --                                      .hps_io_emac1_inst_RXD2
			hps_io_hps_io_emac1_inst_RXD3                => CONNECTED_TO_hps_io_hps_io_emac1_inst_RXD3,                --                                      .hps_io_emac1_inst_RXD3
			hps_io_hps_io_qspi_inst_IO0                  => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO0,                  --                                      .hps_io_qspi_inst_IO0
			hps_io_hps_io_qspi_inst_IO1                  => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO1,                  --                                      .hps_io_qspi_inst_IO1
			hps_io_hps_io_qspi_inst_IO2                  => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO2,                  --                                      .hps_io_qspi_inst_IO2
			hps_io_hps_io_qspi_inst_IO3                  => CONNECTED_TO_hps_io_hps_io_qspi_inst_IO3,                  --                                      .hps_io_qspi_inst_IO3
			hps_io_hps_io_qspi_inst_SS0                  => CONNECTED_TO_hps_io_hps_io_qspi_inst_SS0,                  --                                      .hps_io_qspi_inst_SS0
			hps_io_hps_io_qspi_inst_CLK                  => CONNECTED_TO_hps_io_hps_io_qspi_inst_CLK,                  --                                      .hps_io_qspi_inst_CLK
			hps_io_hps_io_sdio_inst_CMD                  => CONNECTED_TO_hps_io_hps_io_sdio_inst_CMD,                  --                                      .hps_io_sdio_inst_CMD
			hps_io_hps_io_sdio_inst_D0                   => CONNECTED_TO_hps_io_hps_io_sdio_inst_D0,                   --                                      .hps_io_sdio_inst_D0
			hps_io_hps_io_sdio_inst_D1                   => CONNECTED_TO_hps_io_hps_io_sdio_inst_D1,                   --                                      .hps_io_sdio_inst_D1
			hps_io_hps_io_sdio_inst_CLK                  => CONNECTED_TO_hps_io_hps_io_sdio_inst_CLK,                  --                                      .hps_io_sdio_inst_CLK
			hps_io_hps_io_sdio_inst_D2                   => CONNECTED_TO_hps_io_hps_io_sdio_inst_D2,                   --                                      .hps_io_sdio_inst_D2
			hps_io_hps_io_sdio_inst_D3                   => CONNECTED_TO_hps_io_hps_io_sdio_inst_D3,                   --                                      .hps_io_sdio_inst_D3
			hps_io_hps_io_usb1_inst_D0                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D0,                   --                                      .hps_io_usb1_inst_D0
			hps_io_hps_io_usb1_inst_D1                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D1,                   --                                      .hps_io_usb1_inst_D1
			hps_io_hps_io_usb1_inst_D2                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D2,                   --                                      .hps_io_usb1_inst_D2
			hps_io_hps_io_usb1_inst_D3                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D3,                   --                                      .hps_io_usb1_inst_D3
			hps_io_hps_io_usb1_inst_D4                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D4,                   --                                      .hps_io_usb1_inst_D4
			hps_io_hps_io_usb1_inst_D5                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D5,                   --                                      .hps_io_usb1_inst_D5
			hps_io_hps_io_usb1_inst_D6                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D6,                   --                                      .hps_io_usb1_inst_D6
			hps_io_hps_io_usb1_inst_D7                   => CONNECTED_TO_hps_io_hps_io_usb1_inst_D7,                   --                                      .hps_io_usb1_inst_D7
			hps_io_hps_io_usb1_inst_CLK                  => CONNECTED_TO_hps_io_hps_io_usb1_inst_CLK,                  --                                      .hps_io_usb1_inst_CLK
			hps_io_hps_io_usb1_inst_STP                  => CONNECTED_TO_hps_io_hps_io_usb1_inst_STP,                  --                                      .hps_io_usb1_inst_STP
			hps_io_hps_io_usb1_inst_DIR                  => CONNECTED_TO_hps_io_hps_io_usb1_inst_DIR,                  --                                      .hps_io_usb1_inst_DIR
			hps_io_hps_io_usb1_inst_NXT                  => CONNECTED_TO_hps_io_hps_io_usb1_inst_NXT,                  --                                      .hps_io_usb1_inst_NXT
			hps_io_hps_io_spim1_inst_CLK                 => CONNECTED_TO_hps_io_hps_io_spim1_inst_CLK,                 --                                      .hps_io_spim1_inst_CLK
			hps_io_hps_io_spim1_inst_MOSI                => CONNECTED_TO_hps_io_hps_io_spim1_inst_MOSI,                --                                      .hps_io_spim1_inst_MOSI
			hps_io_hps_io_spim1_inst_MISO                => CONNECTED_TO_hps_io_hps_io_spim1_inst_MISO,                --                                      .hps_io_spim1_inst_MISO
			hps_io_hps_io_spim1_inst_SS0                 => CONNECTED_TO_hps_io_hps_io_spim1_inst_SS0,                 --                                      .hps_io_spim1_inst_SS0
			hps_io_hps_io_uart0_inst_RX                  => CONNECTED_TO_hps_io_hps_io_uart0_inst_RX,                  --                                      .hps_io_uart0_inst_RX
			hps_io_hps_io_uart0_inst_TX                  => CONNECTED_TO_hps_io_hps_io_uart0_inst_TX,                  --                                      .hps_io_uart0_inst_TX
			hps_io_hps_io_i2c0_inst_SDA                  => CONNECTED_TO_hps_io_hps_io_i2c0_inst_SDA,                  --                                      .hps_io_i2c0_inst_SDA
			hps_io_hps_io_i2c0_inst_SCL                  => CONNECTED_TO_hps_io_hps_io_i2c0_inst_SCL,                  --                                      .hps_io_i2c0_inst_SCL
			hps_io_hps_io_i2c1_inst_SDA                  => CONNECTED_TO_hps_io_hps_io_i2c1_inst_SDA,                  --                                      .hps_io_i2c1_inst_SDA
			hps_io_hps_io_i2c1_inst_SCL                  => CONNECTED_TO_hps_io_hps_io_i2c1_inst_SCL,                  --                                      .hps_io_i2c1_inst_SCL
			hps_io_hps_io_gpio_inst_GPIO09               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO09,               --                                      .hps_io_gpio_inst_GPIO09
			hps_io_hps_io_gpio_inst_GPIO35               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO35,               --                                      .hps_io_gpio_inst_GPIO35
			hps_io_hps_io_gpio_inst_GPIO40               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO40,               --                                      .hps_io_gpio_inst_GPIO40
			hps_io_hps_io_gpio_inst_GPIO41               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO41,               --                                      .hps_io_gpio_inst_GPIO41
			hps_io_hps_io_gpio_inst_GPIO48               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO48,               --                                      .hps_io_gpio_inst_GPIO48
			hps_io_hps_io_gpio_inst_GPIO53               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO53,               --                                      .hps_io_gpio_inst_GPIO53
			hps_io_hps_io_gpio_inst_GPIO54               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO54,               --                                      .hps_io_gpio_inst_GPIO54
			hps_io_hps_io_gpio_inst_GPIO61               => CONNECTED_TO_hps_io_hps_io_gpio_inst_GPIO61,               --                                      .hps_io_gpio_inst_GPIO61
			memory_mem_a                                 => CONNECTED_TO_memory_mem_a,                                 --                                memory.mem_a
			memory_mem_ba                                => CONNECTED_TO_memory_mem_ba,                                --                                      .mem_ba
			memory_mem_ck                                => CONNECTED_TO_memory_mem_ck,                                --                                      .mem_ck
			memory_mem_ck_n                              => CONNECTED_TO_memory_mem_ck_n,                              --                                      .mem_ck_n
			memory_mem_cke                               => CONNECTED_TO_memory_mem_cke,                               --                                      .mem_cke
			memory_mem_cs_n                              => CONNECTED_TO_memory_mem_cs_n,                              --                                      .mem_cs_n
			memory_mem_ras_n                             => CONNECTED_TO_memory_mem_ras_n,                             --                                      .mem_ras_n
			memory_mem_cas_n                             => CONNECTED_TO_memory_mem_cas_n,                             --                                      .mem_cas_n
			memory_mem_we_n                              => CONNECTED_TO_memory_mem_we_n,                              --                                      .mem_we_n
			memory_mem_reset_n                           => CONNECTED_TO_memory_mem_reset_n,                           --                                      .mem_reset_n
			memory_mem_dq                                => CONNECTED_TO_memory_mem_dq,                                --                                      .mem_dq
			memory_mem_dqs                               => CONNECTED_TO_memory_mem_dqs,                               --                                      .mem_dqs
			memory_mem_dqs_n                             => CONNECTED_TO_memory_mem_dqs_n,                             --                                      .mem_dqs_n
			memory_mem_odt                               => CONNECTED_TO_memory_mem_odt,                               --                                      .mem_odt
			memory_mem_dm                                => CONNECTED_TO_memory_mem_dm,                                --                                      .mem_dm
			memory_oct_rzqin                             => CONNECTED_TO_memory_oct_rzqin,                             --                                      .oct_rzqin
			pio_col_external_connection_export           => CONNECTED_TO_pio_col_external_connection_export,           --           pio_col_external_connection.export
			pio_collectsingle_external_connection_export => CONNECTED_TO_pio_collectsingle_external_connection_export, -- pio_collectsingle_external_connection.export
			pio_color_external_connection_export         => CONNECTED_TO_pio_color_external_connection_export,         --         pio_color_external_connection.export
			pio_row_external_connection_export           => CONNECTED_TO_pio_row_external_connection_export,           --           pio_row_external_connection.export
			pio_state_external_connection_export         => CONNECTED_TO_pio_state_external_connection_export,         --         pio_state_external_connection.export
			sdram_addr                                   => CONNECTED_TO_sdram_addr,                                   --                                 sdram.addr
			sdram_ba                                     => CONNECTED_TO_sdram_ba,                                     --                                      .ba
			sdram_cas_n                                  => CONNECTED_TO_sdram_cas_n,                                  --                                      .cas_n
			sdram_cke                                    => CONNECTED_TO_sdram_cke,                                    --                                      .cke
			sdram_cs_n                                   => CONNECTED_TO_sdram_cs_n,                                   --                                      .cs_n
			sdram_dq                                     => CONNECTED_TO_sdram_dq,                                     --                                      .dq
			sdram_dqm                                    => CONNECTED_TO_sdram_dqm,                                    --                                      .dqm
			sdram_ras_n                                  => CONNECTED_TO_sdram_ras_n,                                  --                                      .ras_n
			sdram_we_n                                   => CONNECTED_TO_sdram_we_n,                                   --                                      .we_n
			sdram_clk_clk                                => CONNECTED_TO_sdram_clk_clk,                                --                             sdram_clk.clk
			system_pll_ref_clk_clk                       => CONNECTED_TO_system_pll_ref_clk_clk,                       --                    system_pll_ref_clk.clk
			system_pll_ref_reset_reset                   => CONNECTED_TO_system_pll_ref_reset_reset,                   --                  system_pll_ref_reset.reset
			vga_CLK                                      => CONNECTED_TO_vga_CLK,                                      --                                   vga.CLK
			vga_HS                                       => CONNECTED_TO_vga_HS,                                       --                                      .HS
			vga_VS                                       => CONNECTED_TO_vga_VS,                                       --                                      .VS
			vga_BLANK                                    => CONNECTED_TO_vga_BLANK,                                    --                                      .BLANK
			vga_SYNC                                     => CONNECTED_TO_vga_SYNC,                                     --                                      .SYNC
			vga_R                                        => CONNECTED_TO_vga_R,                                        --                                      .R
			vga_G                                        => CONNECTED_TO_vga_G,                                        --                                      .G
			vga_B                                        => CONNECTED_TO_vga_B,                                        --                                      .B
			vga_pll_ref_clk_clk                          => CONNECTED_TO_vga_pll_ref_clk_clk,                          --                       vga_pll_ref_clk.clk
			vga_pll_ref_reset_reset                      => CONNECTED_TO_vga_pll_ref_reset_reset,                      --                     vga_pll_ref_reset.reset
			video_in_TD_CLK27                            => CONNECTED_TO_video_in_TD_CLK27,                            --                              video_in.TD_CLK27
			video_in_TD_DATA                             => CONNECTED_TO_video_in_TD_DATA,                             --                                      .TD_DATA
			video_in_TD_HS                               => CONNECTED_TO_video_in_TD_HS,                               --                                      .TD_HS
			video_in_TD_VS                               => CONNECTED_TO_video_in_TD_VS,                               --                                      .TD_VS
			video_in_clk27_reset                         => CONNECTED_TO_video_in_clk27_reset,                         --                                      .clk27_reset
			video_in_TD_RESET                            => CONNECTED_TO_video_in_TD_RESET,                            --                                      .TD_RESET
			video_in_overflow_flag                       => CONNECTED_TO_video_in_overflow_flag,                       --                                      .overflow_flag
			pio_integral_data_external_connection_export => CONNECTED_TO_pio_integral_data_external_connection_export  -- pio_integral_data_external_connection.export
		);
