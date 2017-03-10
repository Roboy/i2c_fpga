-- Copyright (C) 2017  Intel Corporation. All rights reserved.
-- Your use of Intel Corporation's design tools, logic functions 
-- and other software and tools, and its AMPP partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Intel Program License 
-- Subscription Agreement, the Intel Quartus Prime License Agreement,
-- the Intel MegaCore Function License Agreement, or other 
-- applicable license agreement, including, without limitation, 
-- that your use is for the sole purpose of programming logic 
-- devices manufactured by Intel and sold by Intel or its 
-- authorized distributors.  Please refer to the applicable 
-- agreement for further details.

-- VENDOR "Altera"
-- PROGRAM "Quartus Prime"
-- VERSION "Version 16.1.2 Build 203 01/18/2017 SJ Lite Edition"

-- DATE "03/09/2017 19:44:47"

-- 
-- Device: Altera EP4CE22F17C6 Package FBGA256
-- 

-- 
-- This VHDL file should be used for ModelSim-Altera (VHDL) only
-- 

LIBRARY ALTERA;
LIBRARY CYCLONEIVE;
LIBRARY IEEE;
USE ALTERA.ALTERA_PRIMITIVES_COMPONENTS.ALL;
USE CYCLONEIVE.CYCLONEIVE_COMPONENTS.ALL;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY 	darkroom_top IS
    PORT (
	spi_mosi : OUT std_logic;
	clock_50 : IN std_logic;
	button0 : IN std_logic;
	spi_miso : IN std_logic;
	MPU_6050_interrupt_in : IN std_logic;
	sensor1_signal : IN std_logic;
	sensor2_signal : IN std_logic;
	sensor3_signal : IN std_logic;
	sensor4_signal : IN std_logic;
	sensor5_signal : IN std_logic;
	sensor6_signal : IN std_logic;
	sensor7_signal : IN std_logic;
	sensor0_signal : IN std_logic;
	spi_sclk : OUT std_logic;
	spi_ss_n : OUT std_logic;
	MPU_6050_interrupt_out : OUT std_logic;
	LED : OUT std_logic_vector(7 DOWNTO 0);
	button1 : IN std_logic
	);
END darkroom_top;

-- Design Ports Information
-- spi_mosi	=>  Location: PIN_E10,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- spi_sclk	=>  Location: PIN_D11,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- spi_ss_n	=>  Location: PIN_B12,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- MPU_6050_interrupt_out	=>  Location: PIN_D12,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- LED[7]	=>  Location: PIN_L3,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LED[6]	=>  Location: PIN_B1,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LED[5]	=>  Location: PIN_F3,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LED[4]	=>  Location: PIN_D1,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LED[3]	=>  Location: PIN_A11,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- LED[2]	=>  Location: PIN_B13,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- LED[1]	=>  Location: PIN_A13,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- LED[0]	=>  Location: PIN_A15,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: 8mA
-- button1	=>  Location: PIN_E1,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- button0	=>  Location: PIN_J15,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- MPU_6050_interrupt_in	=>  Location: PIN_A12,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: Default
-- clock_50	=>  Location: PIN_R8,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor3_signal	=>  Location: PIN_C6,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor2_signal	=>  Location: PIN_D6,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor1_signal	=>  Location: PIN_A6,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor0_signal	=>  Location: PIN_D5,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor6_signal	=>  Location: PIN_F8,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor7_signal	=>  Location: PIN_E9,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor5_signal	=>  Location: PIN_D8,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- sensor4_signal	=>  Location: PIN_E6,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- spi_miso	=>  Location: PIN_B11,	 I/O Standard: 3.3-V LVTTL,	 Current Strength: Default


ARCHITECTURE structure OF darkroom_top IS
SIGNAL gnd : std_logic := '0';
SIGNAL vcc : std_logic := '1';
SIGNAL unknown : std_logic := 'X';
SIGNAL devoe : std_logic := '1';
SIGNAL devclrn : std_logic := '1';
SIGNAL devpor : std_logic := '1';
SIGNAL ww_devoe : std_logic;
SIGNAL ww_devclrn : std_logic;
SIGNAL ww_devpor : std_logic;
SIGNAL ww_spi_mosi : std_logic;
SIGNAL ww_clock_50 : std_logic;
SIGNAL ww_button0 : std_logic;
SIGNAL ww_spi_miso : std_logic;
SIGNAL ww_MPU_6050_interrupt_in : std_logic;
SIGNAL ww_sensor1_signal : std_logic;
SIGNAL ww_sensor2_signal : std_logic;
SIGNAL ww_sensor3_signal : std_logic;
SIGNAL ww_sensor4_signal : std_logic;
SIGNAL ww_sensor5_signal : std_logic;
SIGNAL ww_sensor6_signal : std_logic;
SIGNAL ww_sensor7_signal : std_logic;
SIGNAL ww_sensor0_signal : std_logic;
SIGNAL ww_spi_sclk : std_logic;
SIGNAL ww_spi_ss_n : std_logic;
SIGNAL ww_MPU_6050_interrupt_out : std_logic;
SIGNAL ww_LED : std_logic_vector(7 DOWNTO 0);
SIGNAL ww_button1 : std_logic;
SIGNAL \inst|altpll_component|auto_generated|pll1_INCLK_bus\ : std_logic_vector(1 DOWNTO 0);
SIGNAL \inst|altpll_component|auto_generated|pll1_CLK_bus\ : std_logic_vector(4 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTADATAIN_bus\ : std_logic_vector(17 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTAADDR_bus\ : std_logic_vector(8 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBADDR_bus\ : std_logic_vector(8 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\ : std_logic_vector(17 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTADATAIN_bus\ : std_logic_vector(17 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTAADDR_bus\ : std_logic_vector(8 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBADDR_bus\ : std_logic_vector(8 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\ : std_logic_vector(17 DOWNTO 0);
SIGNAL \sensor1_signal~inputclkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \sensor7_signal~inputclkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \clock_50~inputclkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \button1~input_o\ : std_logic;
SIGNAL \~ALTERA_ASDO_DATA1~~ibuf_o\ : std_logic;
SIGNAL \~ALTERA_ASDO_DATA1~~padout\ : std_logic;
SIGNAL \~ALTERA_FLASH_nCE_nCSO~~ibuf_o\ : std_logic;
SIGNAL \~ALTERA_FLASH_nCE_nCSO~~padout\ : std_logic;
SIGNAL \~ALTERA_DCLK~~padout\ : std_logic;
SIGNAL \~ALTERA_DATA0~~ibuf_o\ : std_logic;
SIGNAL \~ALTERA_DATA0~~padout\ : std_logic;
SIGNAL \~ALTERA_nCEO~~padout\ : std_logic;
SIGNAL \~ALTERA_DCLK~~obuf_o\ : std_logic;
SIGNAL \~ALTERA_nCEO~~obuf_o\ : std_logic;
SIGNAL \clock_50~input_o\ : std_logic;
SIGNAL \clock_50~inputclkctrl_outclk\ : std_logic;
SIGNAL \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~0_combout\ : std_logic;
SIGNAL \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\ : std_logic;
SIGNAL \inst2|clk_cnt~0_combout\ : std_logic;
SIGNAL \inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\ : std_logic;
SIGNAL \inst2|clk_cnt~1_combout\ : std_logic;
SIGNAL \inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\ : std_logic;
SIGNAL \inst2|Equal0~0_combout\ : std_logic;
SIGNAL \inst2|spi_2x_ce~q\ : std_logic;
SIGNAL \inst2|core_n_clk~0_combout\ : std_logic;
SIGNAL \inst2|core_n_clk~q\ : std_logic;
SIGNAL \inst2|core_n_ce~0_combout\ : std_logic;
SIGNAL \inst2|core_n_ce~q\ : std_logic;
SIGNAL \inst2|Selector4~1_combout\ : std_logic;
SIGNAL \button0~input_o\ : std_logic;
SIGNAL \inst2|state_reg[2]~3_combout\ : std_logic;
SIGNAL \inst2|Selector3~0_combout\ : std_logic;
SIGNAL \inst2|Selector4~0_combout\ : std_logic;
SIGNAL \inst2|state_reg~0_combout\ : std_logic;
SIGNAL \inst2|state_reg~1_combout\ : std_logic;
SIGNAL \inst2|Equal1~0_combout\ : std_logic;
SIGNAL \inst2|state_reg~2_combout\ : std_logic;
SIGNAL \inst2|Selector2~0_combout\ : std_logic;
SIGNAL \inst2|wr_ack_reg~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~combout\ : std_logic;
SIGNAL \~GND~combout\ : std_logic;
SIGNAL \inst27~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita8~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~4_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\ : std_logic;
SIGNAL \inst11|LessThan2~0_combout\ : std_logic;
SIGNAL \inst11|Add0~0_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted~8_combout\ : std_logic;
SIGNAL \inst11|write_ack_prev~q\ : std_logic;
SIGNAL \inst11|wren~2_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted[1]~1_combout\ : std_logic;
SIGNAL \inst11|Add0~1\ : std_logic;
SIGNAL \inst11|Add0~2_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted~0_combout\ : std_logic;
SIGNAL \inst11|Add0~3\ : std_logic;
SIGNAL \inst11|Add0~4_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted~4_combout\ : std_logic;
SIGNAL \inst11|Add0~5\ : std_logic;
SIGNAL \inst11|Add0~6_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted~3_combout\ : std_logic;
SIGNAL \inst11|LessThan0~0_combout\ : std_logic;
SIGNAL \inst11|Add0~11\ : std_logic;
SIGNAL \inst11|Add0~12_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted[6]~7_combout\ : std_logic;
SIGNAL \inst11|Add0~13\ : std_logic;
SIGNAL \inst11|Add0~14_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted[7]~6_combout\ : std_logic;
SIGNAL \inst11|LessThan0~1_combout\ : std_logic;
SIGNAL \inst11|Add0~7\ : std_logic;
SIGNAL \inst11|Add0~8_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted~2_combout\ : std_logic;
SIGNAL \inst11|Add0~9\ : std_logic;
SIGNAL \inst11|Add0~10_combout\ : std_logic;
SIGNAL \inst11|numberOfBytesTransmitted[5]~5_combout\ : std_logic;
SIGNAL \inst11|LessThan1~0_combout\ : std_logic;
SIGNAL \inst11|fifo_read~0_combout\ : std_logic;
SIGNAL \inst2|WideOr2~0_combout\ : std_logic;
SIGNAL \inst2|di_req_reg~q\ : std_logic;
SIGNAL \inst2|di_req_o_A~feeder_combout\ : std_logic;
SIGNAL \inst2|di_req_o_A~q\ : std_logic;
SIGNAL \inst2|di_req_o_B~feeder_combout\ : std_logic;
SIGNAL \inst2|di_req_o_B~q\ : std_logic;
SIGNAL \inst2|di_req_o_C~q\ : std_logic;
SIGNAL \inst2|di_req_o_D~feeder_combout\ : std_logic;
SIGNAL \inst2|di_req_o_D~q\ : std_logic;
SIGNAL \inst2|di_req_o_next~0_combout\ : std_logic;
SIGNAL \inst2|di_req_o_reg~q\ : std_logic;
SIGNAL \inst11|fifo_read~1_combout\ : std_logic;
SIGNAL \inst11|fifo_read~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~1_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~2_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~8_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~10_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~7_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~9_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3_combout\ : std_logic;
SIGNAL \sensor7_signal~input_o\ : std_logic;
SIGNAL \sensor7_signal~inputclkctrl_outclk\ : std_logic;
SIGNAL \inst|altpll_component|auto_generated|wire_pll1_fbout\ : std_logic;
SIGNAL \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\ : std_logic;
SIGNAL \inst33|temp[0]~93_combout\ : std_logic;
SIGNAL \inst33|temp[1]~31_combout\ : std_logic;
SIGNAL \inst33|temp[1]~32\ : std_logic;
SIGNAL \inst33|temp[2]~33_combout\ : std_logic;
SIGNAL \inst33|temp[2]~34\ : std_logic;
SIGNAL \inst33|temp[3]~35_combout\ : std_logic;
SIGNAL \inst33|temp[3]~36\ : std_logic;
SIGNAL \inst33|temp[4]~37_combout\ : std_logic;
SIGNAL \inst33|temp[4]~38\ : std_logic;
SIGNAL \inst33|temp[5]~39_combout\ : std_logic;
SIGNAL \inst33|temp[5]~40\ : std_logic;
SIGNAL \inst33|temp[6]~41_combout\ : std_logic;
SIGNAL \inst33|temp[6]~42\ : std_logic;
SIGNAL \inst33|temp[7]~43_combout\ : std_logic;
SIGNAL \inst33|temp[7]~44\ : std_logic;
SIGNAL \inst33|temp[8]~45_combout\ : std_logic;
SIGNAL \inst33|temp[8]~46\ : std_logic;
SIGNAL \inst33|temp[9]~47_combout\ : std_logic;
SIGNAL \inst33|temp[9]~48\ : std_logic;
SIGNAL \inst33|temp[10]~49_combout\ : std_logic;
SIGNAL \inst33|temp[10]~50\ : std_logic;
SIGNAL \inst33|temp[11]~51_combout\ : std_logic;
SIGNAL \inst33|temp[11]~52\ : std_logic;
SIGNAL \inst33|temp[12]~53_combout\ : std_logic;
SIGNAL \inst33|temp[12]~54\ : std_logic;
SIGNAL \inst33|temp[13]~55_combout\ : std_logic;
SIGNAL \inst33|temp[13]~56\ : std_logic;
SIGNAL \inst33|temp[14]~57_combout\ : std_logic;
SIGNAL \inst33|temp[14]~58\ : std_logic;
SIGNAL \inst33|temp[15]~59_combout\ : std_logic;
SIGNAL \inst33|temp[15]~60\ : std_logic;
SIGNAL \inst33|temp[16]~61_combout\ : std_logic;
SIGNAL \inst33|temp[16]~62\ : std_logic;
SIGNAL \inst33|temp[17]~63_combout\ : std_logic;
SIGNAL \inst33|temp[17]~64\ : std_logic;
SIGNAL \inst33|temp[18]~65_combout\ : std_logic;
SIGNAL \inst15|Add0~1\ : std_logic;
SIGNAL \inst15|Add0~3\ : std_logic;
SIGNAL \inst15|Add0~5\ : std_logic;
SIGNAL \inst15|Add0~7\ : std_logic;
SIGNAL \inst15|Add0~9\ : std_logic;
SIGNAL \inst15|Add0~11\ : std_logic;
SIGNAL \inst15|Add0~13\ : std_logic;
SIGNAL \inst15|Add0~15\ : std_logic;
SIGNAL \inst15|Add0~17\ : std_logic;
SIGNAL \inst15|Add0~19\ : std_logic;
SIGNAL \inst15|Add0~21\ : std_logic;
SIGNAL \inst15|Add0~23\ : std_logic;
SIGNAL \inst15|Add0~25\ : std_logic;
SIGNAL \inst15|Add0~27\ : std_logic;
SIGNAL \inst15|Add0~29\ : std_logic;
SIGNAL \inst15|Add0~31\ : std_logic;
SIGNAL \inst15|Add0~33\ : std_logic;
SIGNAL \inst15|Add0~35\ : std_logic;
SIGNAL \inst15|Add0~36_combout\ : std_logic;
SIGNAL \inst15|Add0~30_combout\ : std_logic;
SIGNAL \inst15|Add0~32_combout\ : std_logic;
SIGNAL \inst15|Add0~34_combout\ : std_logic;
SIGNAL \inst15|data_available~2_combout\ : std_logic;
SIGNAL \inst33|temp[18]~66\ : std_logic;
SIGNAL \inst33|temp[19]~67_combout\ : std_logic;
SIGNAL \inst33|temp[19]~68\ : std_logic;
SIGNAL \inst33|temp[20]~69_combout\ : std_logic;
SIGNAL \inst33|temp[20]~70\ : std_logic;
SIGNAL \inst33|temp[21]~71_combout\ : std_logic;
SIGNAL \inst33|temp[21]~72\ : std_logic;
SIGNAL \inst33|temp[22]~73_combout\ : std_logic;
SIGNAL \inst15|Add0~37\ : std_logic;
SIGNAL \inst15|Add0~39\ : std_logic;
SIGNAL \inst15|Add0~41\ : std_logic;
SIGNAL \inst15|Add0~43\ : std_logic;
SIGNAL \inst15|Add0~44_combout\ : std_logic;
SIGNAL \inst15|Add0~42_combout\ : std_logic;
SIGNAL \inst15|Add0~40_combout\ : std_logic;
SIGNAL \inst15|Add0~38_combout\ : std_logic;
SIGNAL \inst15|data_available~3_combout\ : std_logic;
SIGNAL \inst15|Add0~28_combout\ : std_logic;
SIGNAL \inst15|Add0~22_combout\ : std_logic;
SIGNAL \inst15|Add0~26_combout\ : std_logic;
SIGNAL \inst15|Add0~24_combout\ : std_logic;
SIGNAL \inst15|data_available~1_combout\ : std_logic;
SIGNAL \inst15|Add0~20_combout\ : std_logic;
SIGNAL \inst15|Add0~18_combout\ : std_logic;
SIGNAL \inst15|Add0~14_combout\ : std_logic;
SIGNAL \inst15|Add0~16_combout\ : std_logic;
SIGNAL \inst15|data_available~0_combout\ : std_logic;
SIGNAL \inst15|data_available~4_combout\ : std_logic;
SIGNAL \inst15|Add0~4_combout\ : std_logic;
SIGNAL \inst15|Add0~2_combout\ : std_logic;
SIGNAL \inst15|Add0~0_combout\ : std_logic;
SIGNAL \inst15|data_available~8_combout\ : std_logic;
SIGNAL \inst15|Add0~6_combout\ : std_logic;
SIGNAL \inst15|Add0~12_combout\ : std_logic;
SIGNAL \inst15|Add0~8_combout\ : std_logic;
SIGNAL \inst15|data_available~9_combout\ : std_logic;
SIGNAL \inst15|Add0~10_combout\ : std_logic;
SIGNAL \inst15|data_available~10_combout\ : std_logic;
SIGNAL \inst33|temp[22]~74\ : std_logic;
SIGNAL \inst33|temp[23]~75_combout\ : std_logic;
SIGNAL \inst33|temp[23]~76\ : std_logic;
SIGNAL \inst33|temp[24]~77_combout\ : std_logic;
SIGNAL \inst15|Add0~45\ : std_logic;
SIGNAL \inst15|Add0~47\ : std_logic;
SIGNAL \inst15|Add0~48_combout\ : std_logic;
SIGNAL \inst15|Add0~46_combout\ : std_logic;
SIGNAL \inst33|temp[24]~78\ : std_logic;
SIGNAL \inst33|temp[25]~79_combout\ : std_logic;
SIGNAL \inst33|temp[25]~80\ : std_logic;
SIGNAL \inst33|temp[26]~81_combout\ : std_logic;
SIGNAL \inst15|Add0~49\ : std_logic;
SIGNAL \inst15|Add0~51\ : std_logic;
SIGNAL \inst15|Add0~52_combout\ : std_logic;
SIGNAL \inst15|Add0~50_combout\ : std_logic;
SIGNAL \inst15|data_available~5_combout\ : std_logic;
SIGNAL \inst33|temp[26]~82\ : std_logic;
SIGNAL \inst33|temp[27]~83_combout\ : std_logic;
SIGNAL \inst15|Add0~53\ : std_logic;
SIGNAL \inst15|Add0~54_combout\ : std_logic;
SIGNAL \inst33|temp[27]~84\ : std_logic;
SIGNAL \inst33|temp[28]~85_combout\ : std_logic;
SIGNAL \inst33|temp[28]~86\ : std_logic;
SIGNAL \inst33|temp[29]~87_combout\ : std_logic;
SIGNAL \inst33|temp[29]~88\ : std_logic;
SIGNAL \inst33|temp[30]~89_combout\ : std_logic;
SIGNAL \inst15|Add0~55\ : std_logic;
SIGNAL \inst15|Add0~57\ : std_logic;
SIGNAL \inst15|Add0~59\ : std_logic;
SIGNAL \inst15|Add0~60_combout\ : std_logic;
SIGNAL \inst15|Add0~58_combout\ : std_logic;
SIGNAL \inst15|Add0~56_combout\ : std_logic;
SIGNAL \inst15|data_available~6_combout\ : std_logic;
SIGNAL \inst33|temp[30]~90\ : std_logic;
SIGNAL \inst33|temp[31]~91_combout\ : std_logic;
SIGNAL \inst15|Add0~61\ : std_logic;
SIGNAL \inst15|Add0~62_combout\ : std_logic;
SIGNAL \inst15|data_available~7_combout\ : std_logic;
SIGNAL \inst15|data_available~11_combout\ : std_logic;
SIGNAL \inst15|data_available~q\ : std_logic;
SIGNAL \inst24|cur_value~feeder_combout\ : std_logic;
SIGNAL \inst24|cur_value~q\ : std_logic;
SIGNAL \inst24|last_value~q\ : std_logic;
SIGNAL \inst24|level_sig~combout\ : std_logic;
SIGNAL \sensor6_signal~input_o\ : std_logic;
SIGNAL \inst13|Add0~1\ : std_logic;
SIGNAL \inst13|Add0~3\ : std_logic;
SIGNAL \inst13|Add0~5\ : std_logic;
SIGNAL \inst13|Add0~7\ : std_logic;
SIGNAL \inst13|Add0~9\ : std_logic;
SIGNAL \inst13|Add0~10_combout\ : std_logic;
SIGNAL \inst13|Add0~11\ : std_logic;
SIGNAL \inst13|Add0~12_combout\ : std_logic;
SIGNAL \inst13|Add0~8_combout\ : std_logic;
SIGNAL \inst13|Add0~6_combout\ : std_logic;
SIGNAL \inst13|Add0~4_combout\ : std_logic;
SIGNAL \inst13|Add0~0_combout\ : std_logic;
SIGNAL \inst13|Add0~2_combout\ : std_logic;
SIGNAL \inst13|data_available~8_combout\ : std_logic;
SIGNAL \inst13|data_available~9_combout\ : std_logic;
SIGNAL \inst13|Add0~13\ : std_logic;
SIGNAL \inst13|Add0~15\ : std_logic;
SIGNAL \inst13|Add0~17\ : std_logic;
SIGNAL \inst13|Add0~19\ : std_logic;
SIGNAL \inst13|Add0~21\ : std_logic;
SIGNAL \inst13|Add0~23\ : std_logic;
SIGNAL \inst13|Add0~25\ : std_logic;
SIGNAL \inst13|Add0~27\ : std_logic;
SIGNAL \inst13|Add0~29\ : std_logic;
SIGNAL \inst13|Add0~31\ : std_logic;
SIGNAL \inst13|Add0~33\ : std_logic;
SIGNAL \inst13|Add0~35\ : std_logic;
SIGNAL \inst13|Add0~37\ : std_logic;
SIGNAL \inst13|Add0~39\ : std_logic;
SIGNAL \inst13|Add0~41\ : std_logic;
SIGNAL \inst13|Add0~43\ : std_logic;
SIGNAL \inst13|Add0~45\ : std_logic;
SIGNAL \inst13|Add0~46_combout\ : std_logic;
SIGNAL \inst13|Add0~47\ : std_logic;
SIGNAL \inst13|Add0~49\ : std_logic;
SIGNAL \inst13|Add0~51\ : std_logic;
SIGNAL \inst13|Add0~52_combout\ : std_logic;
SIGNAL \inst13|Add0~48_combout\ : std_logic;
SIGNAL \inst13|Add0~50_combout\ : std_logic;
SIGNAL \inst13|data_available~5_combout\ : std_logic;
SIGNAL \inst13|Add0~53\ : std_logic;
SIGNAL \inst13|Add0~55\ : std_logic;
SIGNAL \inst13|Add0~57\ : std_logic;
SIGNAL \inst13|Add0~59\ : std_logic;
SIGNAL \inst13|Add0~60_combout\ : std_logic;
SIGNAL \inst13|Add0~56_combout\ : std_logic;
SIGNAL \inst13|Add0~58_combout\ : std_logic;
SIGNAL \inst13|Add0~54_combout\ : std_logic;
SIGNAL \inst13|data_available~6_combout\ : std_logic;
SIGNAL \inst13|Add0~61\ : std_logic;
SIGNAL \inst13|Add0~62_combout\ : std_logic;
SIGNAL \inst13|Add0~36_combout\ : std_logic;
SIGNAL \inst13|Add0~32_combout\ : std_logic;
SIGNAL \inst13|Add0~34_combout\ : std_logic;
SIGNAL \inst13|Add0~30_combout\ : std_logic;
SIGNAL \inst13|data_available~2_combout\ : std_logic;
SIGNAL \inst13|Add0~44_combout\ : std_logic;
SIGNAL \inst13|Add0~42_combout\ : std_logic;
SIGNAL \inst13|Add0~38_combout\ : std_logic;
SIGNAL \inst13|Add0~40_combout\ : std_logic;
SIGNAL \inst13|data_available~3_combout\ : std_logic;
SIGNAL \inst13|Add0~22_combout\ : std_logic;
SIGNAL \inst13|Add0~28_combout\ : std_logic;
SIGNAL \inst13|Add0~26_combout\ : std_logic;
SIGNAL \inst13|Add0~24_combout\ : std_logic;
SIGNAL \inst13|data_available~1_combout\ : std_logic;
SIGNAL \inst13|Add0~20_combout\ : std_logic;
SIGNAL \inst13|Add0~18_combout\ : std_logic;
SIGNAL \inst13|Add0~16_combout\ : std_logic;
SIGNAL \inst13|Add0~14_combout\ : std_logic;
SIGNAL \inst13|data_available~0_combout\ : std_logic;
SIGNAL \inst13|data_available~4_combout\ : std_logic;
SIGNAL \inst13|data_available~7_combout\ : std_logic;
SIGNAL \inst13|data_available~10_combout\ : std_logic;
SIGNAL \inst13|data_available~q\ : std_logic;
SIGNAL \inst23|cur_value~q\ : std_logic;
SIGNAL \inst23|last_value~q\ : std_logic;
SIGNAL \inst23|level_sig~combout\ : std_logic;
SIGNAL \sensor4_signal~input_o\ : std_logic;
SIGNAL \inst10|Add0~1\ : std_logic;
SIGNAL \inst10|Add0~3\ : std_logic;
SIGNAL \inst10|Add0~5\ : std_logic;
SIGNAL \inst10|Add0~7\ : std_logic;
SIGNAL \inst10|Add0~9\ : std_logic;
SIGNAL \inst10|Add0~11\ : std_logic;
SIGNAL \inst10|Add0~12_combout\ : std_logic;
SIGNAL \inst10|Add0~13\ : std_logic;
SIGNAL \inst10|Add0~15\ : std_logic;
SIGNAL \inst10|Add0~17\ : std_logic;
SIGNAL \inst10|Add0~19\ : std_logic;
SIGNAL \inst10|Add0~21\ : std_logic;
SIGNAL \inst10|Add0~23\ : std_logic;
SIGNAL \inst10|Add0~25\ : std_logic;
SIGNAL \inst10|Add0~27\ : std_logic;
SIGNAL \inst10|Add0~29\ : std_logic;
SIGNAL \inst10|Add0~31\ : std_logic;
SIGNAL \inst10|Add0~33\ : std_logic;
SIGNAL \inst10|Add0~35\ : std_logic;
SIGNAL \inst10|Add0~37\ : std_logic;
SIGNAL \inst10|Add0~39\ : std_logic;
SIGNAL \inst10|Add0~41\ : std_logic;
SIGNAL \inst10|Add0~43\ : std_logic;
SIGNAL \inst10|Add0~45\ : std_logic;
SIGNAL \inst10|Add0~47\ : std_logic;
SIGNAL \inst10|Add0~49\ : std_logic;
SIGNAL \inst10|Add0~51\ : std_logic;
SIGNAL \inst10|Add0~53\ : std_logic;
SIGNAL \inst10|Add0~54_combout\ : std_logic;
SIGNAL \inst10|Add0~55\ : std_logic;
SIGNAL \inst10|Add0~57\ : std_logic;
SIGNAL \inst10|Add0~59\ : std_logic;
SIGNAL \inst10|Add0~60_combout\ : std_logic;
SIGNAL \inst10|Add0~58_combout\ : std_logic;
SIGNAL \inst10|Add0~56_combout\ : std_logic;
SIGNAL \inst10|data_available~6_combout\ : std_logic;
SIGNAL \inst10|Add0~46_combout\ : std_logic;
SIGNAL \inst10|Add0~50_combout\ : std_logic;
SIGNAL \inst10|Add0~48_combout\ : std_logic;
SIGNAL \inst10|Add0~52_combout\ : std_logic;
SIGNAL \inst10|data_available~5_combout\ : std_logic;
SIGNAL \inst10|Add0~61\ : std_logic;
SIGNAL \inst10|Add0~62_combout\ : std_logic;
SIGNAL \inst10|Add0~24_combout\ : std_logic;
SIGNAL \inst10|Add0~28_combout\ : std_logic;
SIGNAL \inst10|Add0~26_combout\ : std_logic;
SIGNAL \inst10|Add0~22_combout\ : std_logic;
SIGNAL \inst10|data_available~1_combout\ : std_logic;
SIGNAL \inst10|Add0~32_combout\ : std_logic;
SIGNAL \inst10|Add0~36_combout\ : std_logic;
SIGNAL \inst10|Add0~30_combout\ : std_logic;
SIGNAL \inst10|Add0~34_combout\ : std_logic;
SIGNAL \inst10|data_available~2_combout\ : std_logic;
SIGNAL \inst10|Add0~14_combout\ : std_logic;
SIGNAL \inst10|Add0~20_combout\ : std_logic;
SIGNAL \inst10|Add0~16_combout\ : std_logic;
SIGNAL \inst10|Add0~18_combout\ : std_logic;
SIGNAL \inst10|data_available~0_combout\ : std_logic;
SIGNAL \inst10|Add0~38_combout\ : std_logic;
SIGNAL \inst10|Add0~44_combout\ : std_logic;
SIGNAL \inst10|Add0~40_combout\ : std_logic;
SIGNAL \inst10|Add0~42_combout\ : std_logic;
SIGNAL \inst10|data_available~3_combout\ : std_logic;
SIGNAL \inst10|data_available~4_combout\ : std_logic;
SIGNAL \inst10|data_available~7_combout\ : std_logic;
SIGNAL \inst10|Add0~10_combout\ : std_logic;
SIGNAL \inst10|Add0~8_combout\ : std_logic;
SIGNAL \inst10|Add0~6_combout\ : std_logic;
SIGNAL \inst10|Add0~4_combout\ : std_logic;
SIGNAL \inst10|Add0~0_combout\ : std_logic;
SIGNAL \inst10|Add0~2_combout\ : std_logic;
SIGNAL \inst10|data_available~8_combout\ : std_logic;
SIGNAL \inst10|data_available~9_combout\ : std_logic;
SIGNAL \inst10|data_available~10_combout\ : std_logic;
SIGNAL \inst10|data_available~q\ : std_logic;
SIGNAL \inst21|cur_value~q\ : std_logic;
SIGNAL \inst21|last_value~q\ : std_logic;
SIGNAL \sensor5_signal~input_o\ : std_logic;
SIGNAL \inst12|Add0~1\ : std_logic;
SIGNAL \inst12|Add0~3\ : std_logic;
SIGNAL \inst12|Add0~5\ : std_logic;
SIGNAL \inst12|Add0~7\ : std_logic;
SIGNAL \inst12|Add0~9\ : std_logic;
SIGNAL \inst12|Add0~11\ : std_logic;
SIGNAL \inst12|Add0~12_combout\ : std_logic;
SIGNAL \inst12|Add0~10_combout\ : std_logic;
SIGNAL \inst12|Add0~4_combout\ : std_logic;
SIGNAL \inst12|Add0~0_combout\ : std_logic;
SIGNAL \inst12|Add0~2_combout\ : std_logic;
SIGNAL \inst12|data_available~8_combout\ : std_logic;
SIGNAL \inst12|Add0~6_combout\ : std_logic;
SIGNAL \inst12|Add0~8_combout\ : std_logic;
SIGNAL \inst12|data_available~9_combout\ : std_logic;
SIGNAL \inst12|Add0~13\ : std_logic;
SIGNAL \inst12|Add0~15\ : std_logic;
SIGNAL \inst12|Add0~17\ : std_logic;
SIGNAL \inst12|Add0~19\ : std_logic;
SIGNAL \inst12|Add0~21\ : std_logic;
SIGNAL \inst12|Add0~23\ : std_logic;
SIGNAL \inst12|Add0~25\ : std_logic;
SIGNAL \inst12|Add0~27\ : std_logic;
SIGNAL \inst12|Add0~29\ : std_logic;
SIGNAL \inst12|Add0~31\ : std_logic;
SIGNAL \inst12|Add0~33\ : std_logic;
SIGNAL \inst12|Add0~35\ : std_logic;
SIGNAL \inst12|Add0~37\ : std_logic;
SIGNAL \inst12|Add0~39\ : std_logic;
SIGNAL \inst12|Add0~41\ : std_logic;
SIGNAL \inst12|Add0~43\ : std_logic;
SIGNAL \inst12|Add0~45\ : std_logic;
SIGNAL \inst12|Add0~47\ : std_logic;
SIGNAL \inst12|Add0~49\ : std_logic;
SIGNAL \inst12|Add0~51\ : std_logic;
SIGNAL \inst12|Add0~53\ : std_logic;
SIGNAL \inst12|Add0~55\ : std_logic;
SIGNAL \inst12|Add0~56_combout\ : std_logic;
SIGNAL \inst12|Add0~57\ : std_logic;
SIGNAL \inst12|Add0~58_combout\ : std_logic;
SIGNAL \inst12|Add0~54_combout\ : std_logic;
SIGNAL \inst12|Add0~59\ : std_logic;
SIGNAL \inst12|Add0~60_combout\ : std_logic;
SIGNAL \inst12|data_available~6_combout\ : std_logic;
SIGNAL \inst12|Add0~48_combout\ : std_logic;
SIGNAL \inst12|Add0~50_combout\ : std_logic;
SIGNAL \inst12|Add0~46_combout\ : std_logic;
SIGNAL \inst12|Add0~52_combout\ : std_logic;
SIGNAL \inst12|data_available~5_combout\ : std_logic;
SIGNAL \inst12|Add0~61\ : std_logic;
SIGNAL \inst12|Add0~62_combout\ : std_logic;
SIGNAL \inst12|Add0~36_combout\ : std_logic;
SIGNAL \inst12|Add0~34_combout\ : std_logic;
SIGNAL \inst12|Add0~30_combout\ : std_logic;
SIGNAL \inst12|Add0~32_combout\ : std_logic;
SIGNAL \inst12|data_available~2_combout\ : std_logic;
SIGNAL \inst12|Add0~38_combout\ : std_logic;
SIGNAL \inst12|Add0~44_combout\ : std_logic;
SIGNAL \inst12|Add0~42_combout\ : std_logic;
SIGNAL \inst12|Add0~40_combout\ : std_logic;
SIGNAL \inst12|data_available~3_combout\ : std_logic;
SIGNAL \inst12|Add0~14_combout\ : std_logic;
SIGNAL \inst12|Add0~20_combout\ : std_logic;
SIGNAL \inst12|Add0~16_combout\ : std_logic;
SIGNAL \inst12|Add0~18_combout\ : std_logic;
SIGNAL \inst12|data_available~0_combout\ : std_logic;
SIGNAL \inst12|Add0~28_combout\ : std_logic;
SIGNAL \inst12|Add0~24_combout\ : std_logic;
SIGNAL \inst12|Add0~26_combout\ : std_logic;
SIGNAL \inst12|Add0~22_combout\ : std_logic;
SIGNAL \inst12|data_available~1_combout\ : std_logic;
SIGNAL \inst12|data_available~4_combout\ : std_logic;
SIGNAL \inst12|data_available~7_combout\ : std_logic;
SIGNAL \inst12|data_available~10_combout\ : std_logic;
SIGNAL \inst12|data_available~q\ : std_logic;
SIGNAL \inst22|cur_value~feeder_combout\ : std_logic;
SIGNAL \inst22|cur_value~q\ : std_logic;
SIGNAL \inst22|last_value~q\ : std_logic;
SIGNAL \inst16|write~0_combout\ : std_logic;
SIGNAL \inst16|WideOr0~1_combout\ : std_logic;
SIGNAL \inst16|write~3_combout\ : std_logic;
SIGNAL \sensor2_signal~input_o\ : std_logic;
SIGNAL \inst8|Add0~1\ : std_logic;
SIGNAL \inst8|Add0~3\ : std_logic;
SIGNAL \inst8|Add0~5\ : std_logic;
SIGNAL \inst8|Add0~7\ : std_logic;
SIGNAL \inst8|Add0~9\ : std_logic;
SIGNAL \inst8|Add0~11\ : std_logic;
SIGNAL \inst8|Add0~12_combout\ : std_logic;
SIGNAL \inst8|Add0~13\ : std_logic;
SIGNAL \inst8|Add0~15\ : std_logic;
SIGNAL \inst8|Add0~17\ : std_logic;
SIGNAL \inst8|Add0~19\ : std_logic;
SIGNAL \inst8|Add0~21\ : std_logic;
SIGNAL \inst8|Add0~23\ : std_logic;
SIGNAL \inst8|Add0~25\ : std_logic;
SIGNAL \inst8|Add0~27\ : std_logic;
SIGNAL \inst8|Add0~29\ : std_logic;
SIGNAL \inst8|Add0~31\ : std_logic;
SIGNAL \inst8|Add0~33\ : std_logic;
SIGNAL \inst8|Add0~35\ : std_logic;
SIGNAL \inst8|Add0~37\ : std_logic;
SIGNAL \inst8|Add0~39\ : std_logic;
SIGNAL \inst8|Add0~41\ : std_logic;
SIGNAL \inst8|Add0~43\ : std_logic;
SIGNAL \inst8|Add0~45\ : std_logic;
SIGNAL \inst8|Add0~47\ : std_logic;
SIGNAL \inst8|Add0~49\ : std_logic;
SIGNAL \inst8|Add0~51\ : std_logic;
SIGNAL \inst8|Add0~53\ : std_logic;
SIGNAL \inst8|Add0~55\ : std_logic;
SIGNAL \inst8|Add0~57\ : std_logic;
SIGNAL \inst8|Add0~59\ : std_logic;
SIGNAL \inst8|Add0~60_combout\ : std_logic;
SIGNAL \inst8|Add0~54_combout\ : std_logic;
SIGNAL \inst8|Add0~56_combout\ : std_logic;
SIGNAL \inst8|Add0~58_combout\ : std_logic;
SIGNAL \inst8|data_available~6_combout\ : std_logic;
SIGNAL \inst8|Add0~22_combout\ : std_logic;
SIGNAL \inst8|Add0~28_combout\ : std_logic;
SIGNAL \inst8|Add0~26_combout\ : std_logic;
SIGNAL \inst8|Add0~24_combout\ : std_logic;
SIGNAL \inst8|data_available~1_combout\ : std_logic;
SIGNAL \inst8|Add0~44_combout\ : std_logic;
SIGNAL \inst8|Add0~42_combout\ : std_logic;
SIGNAL \inst8|Add0~40_combout\ : std_logic;
SIGNAL \inst8|Add0~38_combout\ : std_logic;
SIGNAL \inst8|data_available~3_combout\ : std_logic;
SIGNAL \inst8|Add0~36_combout\ : std_logic;
SIGNAL \inst8|Add0~34_combout\ : std_logic;
SIGNAL \inst8|Add0~30_combout\ : std_logic;
SIGNAL \inst8|Add0~32_combout\ : std_logic;
SIGNAL \inst8|data_available~2_combout\ : std_logic;
SIGNAL \inst8|Add0~20_combout\ : std_logic;
SIGNAL \inst8|Add0~18_combout\ : std_logic;
SIGNAL \inst8|Add0~14_combout\ : std_logic;
SIGNAL \inst8|Add0~16_combout\ : std_logic;
SIGNAL \inst8|data_available~0_combout\ : std_logic;
SIGNAL \inst8|data_available~4_combout\ : std_logic;
SIGNAL \inst8|Add0~50_combout\ : std_logic;
SIGNAL \inst8|Add0~48_combout\ : std_logic;
SIGNAL \inst8|Add0~46_combout\ : std_logic;
SIGNAL \inst8|Add0~52_combout\ : std_logic;
SIGNAL \inst8|data_available~5_combout\ : std_logic;
SIGNAL \inst8|Add0~61\ : std_logic;
SIGNAL \inst8|Add0~62_combout\ : std_logic;
SIGNAL \inst8|data_available~7_combout\ : std_logic;
SIGNAL \inst8|Add0~10_combout\ : std_logic;
SIGNAL \inst8|Add0~2_combout\ : std_logic;
SIGNAL \inst8|Add0~0_combout\ : std_logic;
SIGNAL \inst8|Add0~4_combout\ : std_logic;
SIGNAL \inst8|data_available~8_combout\ : std_logic;
SIGNAL \inst8|Add0~8_combout\ : std_logic;
SIGNAL \inst8|Add0~6_combout\ : std_logic;
SIGNAL \inst8|data_available~9_combout\ : std_logic;
SIGNAL \inst8|data_available~10_combout\ : std_logic;
SIGNAL \inst8|data_available~q\ : std_logic;
SIGNAL \inst19|cur_value~q\ : std_logic;
SIGNAL \inst19|last_value~q\ : std_logic;
SIGNAL \inst19|level_sig~combout\ : std_logic;
SIGNAL \sensor0_signal~input_o\ : std_logic;
SIGNAL \inst6|Add0~1\ : std_logic;
SIGNAL \inst6|Add0~3\ : std_logic;
SIGNAL \inst6|Add0~5\ : std_logic;
SIGNAL \inst6|Add0~7\ : std_logic;
SIGNAL \inst6|Add0~9\ : std_logic;
SIGNAL \inst6|Add0~11\ : std_logic;
SIGNAL \inst6|Add0~13\ : std_logic;
SIGNAL \inst6|Add0~15\ : std_logic;
SIGNAL \inst6|Add0~17\ : std_logic;
SIGNAL \inst6|Add0~19\ : std_logic;
SIGNAL \inst6|Add0~21\ : std_logic;
SIGNAL \inst6|Add0~23\ : std_logic;
SIGNAL \inst6|Add0~25\ : std_logic;
SIGNAL \inst6|Add0~27\ : std_logic;
SIGNAL \inst6|Add0~29\ : std_logic;
SIGNAL \inst6|Add0~31\ : std_logic;
SIGNAL \inst6|Add0~33\ : std_logic;
SIGNAL \inst6|Add0~35\ : std_logic;
SIGNAL \inst6|Add0~37\ : std_logic;
SIGNAL \inst6|Add0~39\ : std_logic;
SIGNAL \inst6|Add0~41\ : std_logic;
SIGNAL \inst6|Add0~43\ : std_logic;
SIGNAL \inst6|Add0~45\ : std_logic;
SIGNAL \inst6|Add0~47\ : std_logic;
SIGNAL \inst6|Add0~49\ : std_logic;
SIGNAL \inst6|Add0~51\ : std_logic;
SIGNAL \inst6|Add0~53\ : std_logic;
SIGNAL \inst6|Add0~54_combout\ : std_logic;
SIGNAL \inst6|Add0~55\ : std_logic;
SIGNAL \inst6|Add0~57\ : std_logic;
SIGNAL \inst6|Add0~59\ : std_logic;
SIGNAL \inst6|Add0~60_combout\ : std_logic;
SIGNAL \inst6|Add0~58_combout\ : std_logic;
SIGNAL \inst6|Add0~56_combout\ : std_logic;
SIGNAL \inst6|data_available~6_combout\ : std_logic;
SIGNAL \inst6|Add0~61\ : std_logic;
SIGNAL \inst6|Add0~62_combout\ : std_logic;
SIGNAL \inst6|Add0~46_combout\ : std_logic;
SIGNAL \inst6|Add0~52_combout\ : std_logic;
SIGNAL \inst6|Add0~48_combout\ : std_logic;
SIGNAL \inst6|Add0~50_combout\ : std_logic;
SIGNAL \inst6|data_available~5_combout\ : std_logic;
SIGNAL \inst6|data_available~7_combout\ : std_logic;
SIGNAL \inst6|Add0~10_combout\ : std_logic;
SIGNAL \inst6|Add0~12_combout\ : std_logic;
SIGNAL \inst6|Add0~8_combout\ : std_logic;
SIGNAL \inst6|Add0~6_combout\ : std_logic;
SIGNAL \inst6|data_available~9_combout\ : std_logic;
SIGNAL \inst6|Add0~4_combout\ : std_logic;
SIGNAL \inst6|Add0~0_combout\ : std_logic;
SIGNAL \inst6|Add0~2_combout\ : std_logic;
SIGNAL \inst6|data_available~8_combout\ : std_logic;
SIGNAL \inst6|data_available~10_combout\ : std_logic;
SIGNAL \inst6|Add0~34_combout\ : std_logic;
SIGNAL \inst6|Add0~36_combout\ : std_logic;
SIGNAL \inst6|Add0~30_combout\ : std_logic;
SIGNAL \inst6|Add0~32_combout\ : std_logic;
SIGNAL \inst6|data_available~2_combout\ : std_logic;
SIGNAL \inst6|Add0~14_combout\ : std_logic;
SIGNAL \inst6|Add0~20_combout\ : std_logic;
SIGNAL \inst6|Add0~16_combout\ : std_logic;
SIGNAL \inst6|Add0~18_combout\ : std_logic;
SIGNAL \inst6|data_available~0_combout\ : std_logic;
SIGNAL \inst6|Add0~42_combout\ : std_logic;
SIGNAL \inst6|Add0~44_combout\ : std_logic;
SIGNAL \inst6|Add0~38_combout\ : std_logic;
SIGNAL \inst6|Add0~40_combout\ : std_logic;
SIGNAL \inst6|data_available~3_combout\ : std_logic;
SIGNAL \inst6|Add0~28_combout\ : std_logic;
SIGNAL \inst6|Add0~22_combout\ : std_logic;
SIGNAL \inst6|Add0~26_combout\ : std_logic;
SIGNAL \inst6|Add0~24_combout\ : std_logic;
SIGNAL \inst6|data_available~1_combout\ : std_logic;
SIGNAL \inst6|data_available~4_combout\ : std_logic;
SIGNAL \inst6|data_available~11_combout\ : std_logic;
SIGNAL \inst6|data_available~q\ : std_logic;
SIGNAL \sensor3_signal~input_o\ : std_logic;
SIGNAL \inst9|Add0~1\ : std_logic;
SIGNAL \inst9|Add0~3\ : std_logic;
SIGNAL \inst9|Add0~5\ : std_logic;
SIGNAL \inst9|Add0~7\ : std_logic;
SIGNAL \inst9|Add0~9\ : std_logic;
SIGNAL \inst9|Add0~11\ : std_logic;
SIGNAL \inst9|Add0~13\ : std_logic;
SIGNAL \inst9|Add0~15\ : std_logic;
SIGNAL \inst9|Add0~17\ : std_logic;
SIGNAL \inst9|Add0~19\ : std_logic;
SIGNAL \inst9|Add0~21\ : std_logic;
SIGNAL \inst9|Add0~23\ : std_logic;
SIGNAL \inst9|Add0~25\ : std_logic;
SIGNAL \inst9|Add0~27\ : std_logic;
SIGNAL \inst9|Add0~29\ : std_logic;
SIGNAL \inst9|Add0~31\ : std_logic;
SIGNAL \inst9|Add0~33\ : std_logic;
SIGNAL \inst9|Add0~35\ : std_logic;
SIGNAL \inst9|Add0~37\ : std_logic;
SIGNAL \inst9|Add0~39\ : std_logic;
SIGNAL \inst9|Add0~41\ : std_logic;
SIGNAL \inst9|Add0~43\ : std_logic;
SIGNAL \inst9|Add0~45\ : std_logic;
SIGNAL \inst9|Add0~47\ : std_logic;
SIGNAL \inst9|Add0~49\ : std_logic;
SIGNAL \inst9|Add0~51\ : std_logic;
SIGNAL \inst9|Add0~53\ : std_logic;
SIGNAL \inst9|Add0~55\ : std_logic;
SIGNAL \inst9|Add0~56_combout\ : std_logic;
SIGNAL \inst9|Add0~57\ : std_logic;
SIGNAL \inst9|Add0~59\ : std_logic;
SIGNAL \inst9|Add0~60_combout\ : std_logic;
SIGNAL \inst9|Add0~58_combout\ : std_logic;
SIGNAL \inst9|Add0~54_combout\ : std_logic;
SIGNAL \inst9|data_available~6_combout\ : std_logic;
SIGNAL \inst9|Add0~61\ : std_logic;
SIGNAL \inst9|Add0~62_combout\ : std_logic;
SIGNAL \inst9|Add0~50_combout\ : std_logic;
SIGNAL \inst9|Add0~52_combout\ : std_logic;
SIGNAL \inst9|Add0~46_combout\ : std_logic;
SIGNAL \inst9|Add0~48_combout\ : std_logic;
SIGNAL \inst9|data_available~5_combout\ : std_logic;
SIGNAL \inst9|data_available~7_combout\ : std_logic;
SIGNAL \inst9|Add0~4_combout\ : std_logic;
SIGNAL \inst9|Add0~0_combout\ : std_logic;
SIGNAL \inst9|Add0~2_combout\ : std_logic;
SIGNAL \inst9|data_available~8_combout\ : std_logic;
SIGNAL \inst9|Add0~12_combout\ : std_logic;
SIGNAL \inst9|Add0~10_combout\ : std_logic;
SIGNAL \inst9|Add0~8_combout\ : std_logic;
SIGNAL \inst9|Add0~6_combout\ : std_logic;
SIGNAL \inst9|data_available~9_combout\ : std_logic;
SIGNAL \inst9|data_available~10_combout\ : std_logic;
SIGNAL \inst9|Add0~20_combout\ : std_logic;
SIGNAL \inst9|Add0~18_combout\ : std_logic;
SIGNAL \inst9|Add0~14_combout\ : std_logic;
SIGNAL \inst9|Add0~16_combout\ : std_logic;
SIGNAL \inst9|data_available~0_combout\ : std_logic;
SIGNAL \inst9|Add0~38_combout\ : std_logic;
SIGNAL \inst9|Add0~40_combout\ : std_logic;
SIGNAL \inst9|Add0~42_combout\ : std_logic;
SIGNAL \inst9|Add0~44_combout\ : std_logic;
SIGNAL \inst9|data_available~3_combout\ : std_logic;
SIGNAL \inst9|Add0~22_combout\ : std_logic;
SIGNAL \inst9|Add0~28_combout\ : std_logic;
SIGNAL \inst9|Add0~26_combout\ : std_logic;
SIGNAL \inst9|Add0~24_combout\ : std_logic;
SIGNAL \inst9|data_available~1_combout\ : std_logic;
SIGNAL \inst9|Add0~34_combout\ : std_logic;
SIGNAL \inst9|Add0~32_combout\ : std_logic;
SIGNAL \inst9|Add0~30_combout\ : std_logic;
SIGNAL \inst9|Add0~36_combout\ : std_logic;
SIGNAL \inst9|data_available~2_combout\ : std_logic;
SIGNAL \inst9|data_available~4_combout\ : std_logic;
SIGNAL \inst9|data_available~11_combout\ : std_logic;
SIGNAL \inst9|data_available~q\ : std_logic;
SIGNAL \inst20|cur_value~feeder_combout\ : std_logic;
SIGNAL \inst20|cur_value~q\ : std_logic;
SIGNAL \inst20|last_value~q\ : std_logic;
SIGNAL \inst20|level_sig~combout\ : std_logic;
SIGNAL \sensor1_signal~input_o\ : std_logic;
SIGNAL \sensor1_signal~inputclkctrl_outclk\ : std_logic;
SIGNAL \inst7|Add0~1\ : std_logic;
SIGNAL \inst7|Add0~3\ : std_logic;
SIGNAL \inst7|Add0~5\ : std_logic;
SIGNAL \inst7|Add0~7\ : std_logic;
SIGNAL \inst7|Add0~8_combout\ : std_logic;
SIGNAL \inst7|Add0~9\ : std_logic;
SIGNAL \inst7|Add0~11\ : std_logic;
SIGNAL \inst7|Add0~12_combout\ : std_logic;
SIGNAL \inst7|Add0~6_combout\ : std_logic;
SIGNAL \inst7|Add0~4_combout\ : std_logic;
SIGNAL \inst7|Add0~2_combout\ : std_logic;
SIGNAL \inst7|Add0~0_combout\ : std_logic;
SIGNAL \inst7|data_available~8_combout\ : std_logic;
SIGNAL \inst7|data_available~9_combout\ : std_logic;
SIGNAL \inst7|Add0~13\ : std_logic;
SIGNAL \inst7|Add0~15\ : std_logic;
SIGNAL \inst7|Add0~17\ : std_logic;
SIGNAL \inst7|Add0~19\ : std_logic;
SIGNAL \inst7|Add0~21\ : std_logic;
SIGNAL \inst7|Add0~23\ : std_logic;
SIGNAL \inst7|Add0~25\ : std_logic;
SIGNAL \inst7|Add0~27\ : std_logic;
SIGNAL \inst7|Add0~29\ : std_logic;
SIGNAL \inst7|Add0~31\ : std_logic;
SIGNAL \inst7|Add0~33\ : std_logic;
SIGNAL \inst7|Add0~34_combout\ : std_logic;
SIGNAL \inst7|Add0~32_combout\ : std_logic;
SIGNAL \inst7|Add0~30_combout\ : std_logic;
SIGNAL \inst7|Add0~35\ : std_logic;
SIGNAL \inst7|Add0~36_combout\ : std_logic;
SIGNAL \inst7|data_available~2_combout\ : std_logic;
SIGNAL \inst7|Add0~37\ : std_logic;
SIGNAL \inst7|Add0~38_combout\ : std_logic;
SIGNAL \inst7|Add0~39\ : std_logic;
SIGNAL \inst7|Add0~41\ : std_logic;
SIGNAL \inst7|Add0~42_combout\ : std_logic;
SIGNAL \inst7|Add0~40_combout\ : std_logic;
SIGNAL \inst7|Add0~43\ : std_logic;
SIGNAL \inst7|Add0~44_combout\ : std_logic;
SIGNAL \inst7|data_available~3_combout\ : std_logic;
SIGNAL \inst7|Add0~22_combout\ : std_logic;
SIGNAL \inst7|Add0~24_combout\ : std_logic;
SIGNAL \inst7|Add0~26_combout\ : std_logic;
SIGNAL \inst7|Add0~28_combout\ : std_logic;
SIGNAL \inst7|data_available~1_combout\ : std_logic;
SIGNAL \inst7|Add0~18_combout\ : std_logic;
SIGNAL \inst7|Add0~16_combout\ : std_logic;
SIGNAL \inst7|Add0~20_combout\ : std_logic;
SIGNAL \inst7|Add0~14_combout\ : std_logic;
SIGNAL \inst7|data_available~0_combout\ : std_logic;
SIGNAL \inst7|data_available~4_combout\ : std_logic;
SIGNAL \inst7|Add0~45\ : std_logic;
SIGNAL \inst7|Add0~47\ : std_logic;
SIGNAL \inst7|Add0~49\ : std_logic;
SIGNAL \inst7|Add0~51\ : std_logic;
SIGNAL \inst7|Add0~52_combout\ : std_logic;
SIGNAL \inst7|Add0~48_combout\ : std_logic;
SIGNAL \inst7|Add0~50_combout\ : std_logic;
SIGNAL \inst7|Add0~46_combout\ : std_logic;
SIGNAL \inst7|data_available~5_combout\ : std_logic;
SIGNAL \inst7|Add0~53\ : std_logic;
SIGNAL \inst7|Add0~55\ : std_logic;
SIGNAL \inst7|Add0~56_combout\ : std_logic;
SIGNAL \inst7|Add0~57\ : std_logic;
SIGNAL \inst7|Add0~58_combout\ : std_logic;
SIGNAL \inst7|Add0~59\ : std_logic;
SIGNAL \inst7|Add0~60_combout\ : std_logic;
SIGNAL \inst7|Add0~54_combout\ : std_logic;
SIGNAL \inst7|data_available~6_combout\ : std_logic;
SIGNAL \inst7|Add0~61\ : std_logic;
SIGNAL \inst7|Add0~62_combout\ : std_logic;
SIGNAL \inst7|data_available~7_combout\ : std_logic;
SIGNAL \inst7|Add0~10_combout\ : std_logic;
SIGNAL \inst7|data_available~10_combout\ : std_logic;
SIGNAL \inst7|data_available~q\ : std_logic;
SIGNAL \inst18|cur_value~feeder_combout\ : std_logic;
SIGNAL \inst18|cur_value~q\ : std_logic;
SIGNAL \inst18|last_value~q\ : std_logic;
SIGNAL \inst18|level_sig~combout\ : std_logic;
SIGNAL \inst16|WideOr0~0_combout\ : std_logic;
SIGNAL \inst16|write~2_combout\ : std_logic;
SIGNAL \inst16|write~1_combout\ : std_logic;
SIGNAL \inst16|LessThan0~0_combout\ : std_logic;
SIGNAL \inst16|write~4_combout\ : std_logic;
SIGNAL \inst16|write~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|full_dff~3_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|full_dff~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|full_dff~2_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|full_dff~1_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|full_dff~4_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6_combout\ : std_logic;
SIGNAL \inst11|always0~0_combout\ : std_logic;
SIGNAL \inst11|wren~0_combout\ : std_logic;
SIGNAL \inst11|wren~1_combout\ : std_logic;
SIGNAL \inst11|wren~q\ : std_logic;
SIGNAL \inst2|wren~0_combout\ : std_logic;
SIGNAL \inst2|wren~q\ : std_logic;
SIGNAL \inst2|Equal4~0_combout\ : std_logic;
SIGNAL \inst16|counter[0]~93_combout\ : std_logic;
SIGNAL \inst16|counter[1]~31_combout\ : std_logic;
SIGNAL \inst16|counter[1]~32\ : std_logic;
SIGNAL \inst16|counter[2]~33_combout\ : std_logic;
SIGNAL \inst16|counter[2]~34\ : std_logic;
SIGNAL \inst16|counter[3]~35_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[3]~feeder_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita8~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[0]~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[0]~0_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[1]~1_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[1]~1_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[2]~2_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[2]~2_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[3]~3_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[3]~3_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[4]~4_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[4]~4_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[5]~5_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[5]~5_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[6]~6_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[6]~6_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[7]~7_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[7]~7_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~COUT\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita7~combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[8]~8_combout\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[8]~8_combout\ : std_logic;
SIGNAL \inst16|counter[3]~36\ : std_logic;
SIGNAL \inst16|counter[4]~37_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[4]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[4]~38\ : std_logic;
SIGNAL \inst16|counter[5]~39_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[5]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[5]~40\ : std_logic;
SIGNAL \inst16|counter[6]~41_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[6]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[6]~42\ : std_logic;
SIGNAL \inst16|counter[7]~43_combout\ : std_logic;
SIGNAL \inst16|counter[7]~44\ : std_logic;
SIGNAL \inst16|counter[8]~45_combout\ : std_logic;
SIGNAL \inst16|counter[8]~46\ : std_logic;
SIGNAL \inst16|counter[9]~47_combout\ : std_logic;
SIGNAL \inst16|counter[9]~48\ : std_logic;
SIGNAL \inst16|counter[10]~49_combout\ : std_logic;
SIGNAL \inst16|counter[10]~50\ : std_logic;
SIGNAL \inst16|counter[11]~51_combout\ : std_logic;
SIGNAL \inst16|counter[11]~52\ : std_logic;
SIGNAL \inst16|counter[12]~53_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[12]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[12]~54\ : std_logic;
SIGNAL \inst16|counter[13]~55_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[13]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[13]~56\ : std_logic;
SIGNAL \inst16|counter[14]~57_combout\ : std_logic;
SIGNAL \inst16|counter[14]~58\ : std_logic;
SIGNAL \inst16|counter[15]~59_combout\ : std_logic;
SIGNAL \inst16|counter[15]~60\ : std_logic;
SIGNAL \inst16|counter[16]~61_combout\ : std_logic;
SIGNAL \inst16|counter[16]~62\ : std_logic;
SIGNAL \inst16|counter[17]~63_combout\ : std_logic;
SIGNAL \inst16|counter[17]~64\ : std_logic;
SIGNAL \inst16|counter[18]~65_combout\ : std_logic;
SIGNAL \inst16|counter[18]~66\ : std_logic;
SIGNAL \inst16|counter[19]~67_combout\ : std_logic;
SIGNAL \inst16|counter[19]~68\ : std_logic;
SIGNAL \inst16|counter[20]~69_combout\ : std_logic;
SIGNAL \inst16|counter[20]~70\ : std_logic;
SIGNAL \inst16|counter[21]~71_combout\ : std_logic;
SIGNAL \inst16|counter[21]~72\ : std_logic;
SIGNAL \inst16|counter[22]~73_combout\ : std_logic;
SIGNAL \inst16|counter[22]~74\ : std_logic;
SIGNAL \inst16|counter[23]~75_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[23]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[23]~76\ : std_logic;
SIGNAL \inst16|counter[24]~77_combout\ : std_logic;
SIGNAL \inst16|counter[24]~78\ : std_logic;
SIGNAL \inst16|counter[25]~79_combout\ : std_logic;
SIGNAL \inst16|counter[25]~80\ : std_logic;
SIGNAL \inst16|counter[26]~81_combout\ : std_logic;
SIGNAL \inst16|counter[26]~82\ : std_logic;
SIGNAL \inst16|counter[27]~83_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[27]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[27]~84\ : std_logic;
SIGNAL \inst16|counter[28]~85_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[28]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[28]~86\ : std_logic;
SIGNAL \inst16|counter[29]~87_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[29]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[29]~88\ : std_logic;
SIGNAL \inst16|counter[30]~89_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[30]~feeder_combout\ : std_logic;
SIGNAL \inst16|counter[30]~90\ : std_logic;
SIGNAL \inst16|counter[31]~91_combout\ : std_logic;
SIGNAL \inst11|Byte~1_combout\ : std_logic;
SIGNAL \inst11|Byte~2_combout\ : std_logic;
SIGNAL \inst11|Byte[3]~0_combout\ : std_logic;
SIGNAL \inst11|Byte~3_combout\ : std_logic;
SIGNAL \inst11|Byte[7]~4_combout\ : std_logic;
SIGNAL \inst11|Byte[7]~5_combout\ : std_logic;
SIGNAL \inst2|Equal3~0_combout\ : std_logic;
SIGNAL \inst2|Selector6~0_combout\ : std_logic;
SIGNAL \inst2|Selector6~1_combout\ : std_logic;
SIGNAL \inst11|Byte~12_combout\ : std_logic;
SIGNAL \inst11|Byte~13_combout\ : std_logic;
SIGNAL \inst11|Byte~14_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[1]~feeder_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[2]~feeder_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[8]~feeder_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[9]~feeder_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[11]~feeder_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[25]~feeder_combout\ : std_logic;
SIGNAL \inst16|sensor_value_out[26]~feeder_combout\ : std_logic;
SIGNAL \inst11|Byte~21_combout\ : std_logic;
SIGNAL \inst11|Byte~22_combout\ : std_logic;
SIGNAL \inst11|Byte~23_combout\ : std_logic;
SIGNAL \inst2|di_reg[1]~feeder_combout\ : std_logic;
SIGNAL \inst11|Byte~24_combout\ : std_logic;
SIGNAL \inst11|Byte~25_combout\ : std_logic;
SIGNAL \inst11|Byte~26_combout\ : std_logic;
SIGNAL \inst2|di_reg[0]~feeder_combout\ : std_logic;
SIGNAL \inst2|core_ce~0_combout\ : std_logic;
SIGNAL \inst2|core_ce~q\ : std_logic;
SIGNAL \spi_miso~input_o\ : std_logic;
SIGNAL \inst2|rx_bit_reg~0_combout\ : std_logic;
SIGNAL \inst2|rx_bit_reg~q\ : std_logic;
SIGNAL \inst2|WideOr1~0_combout\ : std_logic;
SIGNAL \inst2|Selector13~0_combout\ : std_logic;
SIGNAL \inst2|sh_reg[6]~0_combout\ : std_logic;
SIGNAL \inst2|Selector12~0_combout\ : std_logic;
SIGNAL \inst11|Byte~18_combout\ : std_logic;
SIGNAL \inst11|Byte~19_combout\ : std_logic;
SIGNAL \inst11|Byte~20_combout\ : std_logic;
SIGNAL \inst2|di_reg[2]~feeder_combout\ : std_logic;
SIGNAL \inst2|Selector11~0_combout\ : std_logic;
SIGNAL \inst11|Byte~15_combout\ : std_logic;
SIGNAL \inst11|Byte~16_combout\ : std_logic;
SIGNAL \inst11|Byte~17_combout\ : std_logic;
SIGNAL \inst2|di_reg[3]~feeder_combout\ : std_logic;
SIGNAL \inst2|Selector10~0_combout\ : std_logic;
SIGNAL \inst2|Selector9~0_combout\ : std_logic;
SIGNAL \inst11|Byte~9_combout\ : std_logic;
SIGNAL \inst11|Byte~10_combout\ : std_logic;
SIGNAL \inst11|Byte~11_combout\ : std_logic;
SIGNAL \inst2|Selector8~0_combout\ : std_logic;
SIGNAL \inst11|Byte~6_combout\ : std_logic;
SIGNAL \inst11|Byte~7_combout\ : std_logic;
SIGNAL \inst11|Byte~8_combout\ : std_logic;
SIGNAL \inst2|di_reg[6]~feeder_combout\ : std_logic;
SIGNAL \inst2|Selector7~0_combout\ : std_logic;
SIGNAL \inst2|Selector6~2_combout\ : std_logic;
SIGNAL \inst2|spi_mosi_o~0_combout\ : std_logic;
SIGNAL \inst2|Selector1~0_combout\ : std_logic;
SIGNAL \inst2|Selector1~1_combout\ : std_logic;
SIGNAL \inst2|sck_ena_reg~q\ : std_logic;
SIGNAL \inst2|spi_clk_reg~0_combout\ : std_logic;
SIGNAL \inst2|spi_clk_reg~q\ : std_logic;
SIGNAL \inst2|WideOr0~0_combout\ : std_logic;
SIGNAL \inst2|Selector0~0_combout\ : std_logic;
SIGNAL \inst2|ssel_ena_reg~q\ : std_logic;
SIGNAL \and1~combout\ : std_logic;
SIGNAL \MPU_6050_interrupt_in~input_o\ : std_logic;
SIGNAL \and2~combout\ : std_logic;
SIGNAL \inst15|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst16|sensor_value_out\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst6|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst7|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst9|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst11|numberOfBytesTransmitted\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst8|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst2|state_reg\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \inst11|Byte\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\ : std_logic_vector(8 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst12|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\ : std_logic_vector(8 DOWNTO 0);
SIGNAL \inst33|temp\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst|altpll_component|auto_generated|wire_pll1_clk\ : std_logic_vector(4 DOWNTO 0);
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\ : std_logic_vector(8 DOWNTO 0);
SIGNAL \inst16|counter\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst13|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst2|di_reg\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst10|t_0\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst2|sh_reg\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \ALT_INV_sensor7_signal~inputclkctrl_outclk\ : std_logic;
SIGNAL \ALT_INV_sensor1_signal~inputclkctrl_outclk\ : std_logic;
SIGNAL \ALT_INV_sensor4_signal~input_o\ : std_logic;
SIGNAL \ALT_INV_sensor5_signal~input_o\ : std_logic;
SIGNAL \ALT_INV_sensor6_signal~input_o\ : std_logic;
SIGNAL \ALT_INV_sensor0_signal~input_o\ : std_logic;
SIGNAL \ALT_INV_sensor2_signal~input_o\ : std_logic;
SIGNAL \ALT_INV_sensor3_signal~input_o\ : std_logic;
SIGNAL \ALT_INV_button0~input_o\ : std_logic;
SIGNAL \inst2|ALT_INV_core_n_ce~q\ : std_logic;
SIGNAL \inst3|scfifo_component|auto_generated|dpfifo|ALT_INV_empty_dff~q\ : std_logic;

BEGIN

spi_mosi <= ww_spi_mosi;
ww_clock_50 <= clock_50;
ww_button0 <= button0;
ww_spi_miso <= spi_miso;
ww_MPU_6050_interrupt_in <= MPU_6050_interrupt_in;
ww_sensor1_signal <= sensor1_signal;
ww_sensor2_signal <= sensor2_signal;
ww_sensor3_signal <= sensor3_signal;
ww_sensor4_signal <= sensor4_signal;
ww_sensor5_signal <= sensor5_signal;
ww_sensor6_signal <= sensor6_signal;
ww_sensor7_signal <= sensor7_signal;
ww_sensor0_signal <= sensor0_signal;
spi_sclk <= ww_spi_sclk;
spi_ss_n <= ww_spi_ss_n;
MPU_6050_interrupt_out <= ww_MPU_6050_interrupt_out;
LED <= ww_LED;
ww_button1 <= button1;
ww_devoe <= devoe;
ww_devclrn <= devclrn;
ww_devpor <= devpor;

\inst|altpll_component|auto_generated|pll1_INCLK_bus\ <= (gnd & \clock_50~input_o\);

\inst|altpll_component|auto_generated|wire_pll1_clk\(0) <= \inst|altpll_component|auto_generated|pll1_CLK_bus\(0);
\inst|altpll_component|auto_generated|wire_pll1_clk\(1) <= \inst|altpll_component|auto_generated|pll1_CLK_bus\(1);
\inst|altpll_component|auto_generated|wire_pll1_clk\(2) <= \inst|altpll_component|auto_generated|pll1_CLK_bus\(2);
\inst|altpll_component|auto_generated|wire_pll1_clk\(3) <= \inst|altpll_component|auto_generated|pll1_CLK_bus\(3);
\inst|altpll_component|auto_generated|wire_pll1_clk\(4) <= \inst|altpll_component|auto_generated|pll1_CLK_bus\(4);

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTADATAIN_bus\ <= (\inst16|sensor_value_out\(31) & \inst16|sensor_value_out\(30) & \inst16|sensor_value_out\(29) & \inst16|sensor_value_out\(28) & \inst16|sensor_value_out\(27) & 
\inst16|sensor_value_out\(23) & \inst16|sensor_value_out\(22) & \inst16|sensor_value_out\(21) & \inst16|sensor_value_out\(20) & \inst16|sensor_value_out\(15) & \inst16|sensor_value_out\(14) & \inst16|sensor_value_out\(13) & 
\inst16|sensor_value_out\(12) & \inst16|sensor_value_out\(7) & \inst16|sensor_value_out\(6) & \inst16|sensor_value_out\(5) & \inst16|sensor_value_out\(4) & \inst16|sensor_value_out\(3));

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTAADDR_bus\ <= (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(8) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(7) & 
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(6) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(5) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(4) & 
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(3) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(2) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(1) & 
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(0));

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBADDR_bus\ <= (\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[8]~8_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[7]~7_combout\ & 
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[6]~6_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[5]~5_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[4]~4_combout\ & 
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[3]~3_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[2]~2_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[1]~1_combout\ & 
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[0]~0_combout\);

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(3) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(0);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(4) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(1);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(5) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(2);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(6) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(3);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(7) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(4);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(12) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(5);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(13) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(6);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(14) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(7);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(15) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(8);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(20) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(9);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(21) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(10);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(22) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(11);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(23) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(12);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(27) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(13);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(28) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(14);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(29) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(15);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(30) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(16);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(31) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\(17);

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTADATAIN_bus\ <= (gnd & gnd & gnd & gnd & \inst16|sensor_value_out\(26) & \inst16|sensor_value_out\(25) & \inst16|sensor_value_out\(24) & \inst16|sensor_value_out\(19) & 
\inst16|sensor_value_out\(18) & \inst16|sensor_value_out\(17) & \inst16|sensor_value_out\(16) & \inst16|sensor_value_out\(11) & \inst16|sensor_value_out\(10) & \inst16|sensor_value_out\(9) & \inst16|sensor_value_out\(8) & 
\inst16|sensor_value_out\(2) & \inst16|sensor_value_out\(1) & \inst16|sensor_value_out\(0));

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTAADDR_bus\ <= (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(8) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(7) & 
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(6) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(5) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(4) & 
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(3) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(2) & \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(1) & 
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(0));

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBADDR_bus\ <= (\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[8]~8_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[7]~7_combout\ & 
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[6]~6_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[5]~5_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[4]~4_combout\ & 
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[3]~3_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[2]~2_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[1]~1_combout\ & 
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[0]~0_combout\);

\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(0) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(0);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(1) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(1);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(2) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(2);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(8) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(3);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(9) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(4);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(10) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(5);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(11) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(6);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(16) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(7);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(17) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(8);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(18) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(9);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(19) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(10);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(24) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(11);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(25) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(12);
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(26) <= \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\(13);

\sensor1_signal~inputclkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \sensor1_signal~input_o\);

\sensor7_signal~inputclkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \sensor7_signal~input_o\);

\inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \inst|altpll_component|auto_generated|wire_pll1_clk\(0));

\clock_50~inputclkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \clock_50~input_o\);
\ALT_INV_sensor7_signal~inputclkctrl_outclk\ <= NOT \sensor7_signal~inputclkctrl_outclk\;
\ALT_INV_sensor1_signal~inputclkctrl_outclk\ <= NOT \sensor1_signal~inputclkctrl_outclk\;
\ALT_INV_sensor4_signal~input_o\ <= NOT \sensor4_signal~input_o\;
\ALT_INV_sensor5_signal~input_o\ <= NOT \sensor5_signal~input_o\;
\ALT_INV_sensor6_signal~input_o\ <= NOT \sensor6_signal~input_o\;
\ALT_INV_sensor0_signal~input_o\ <= NOT \sensor0_signal~input_o\;
\ALT_INV_sensor2_signal~input_o\ <= NOT \sensor2_signal~input_o\;
\ALT_INV_sensor3_signal~input_o\ <= NOT \sensor3_signal~input_o\;
\ALT_INV_button0~input_o\ <= NOT \button0~input_o\;
\inst2|ALT_INV_core_n_ce~q\ <= NOT \inst2|core_n_ce~q\;
\inst3|scfifo_component|auto_generated|dpfifo|ALT_INV_empty_dff~q\ <= NOT \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\;

-- Location: IOOBUF_X45_Y34_N16
\spi_mosi~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|spi_mosi_o~0_combout\,
	devoe => ww_devoe,
	o => ww_spi_mosi);

-- Location: IOOBUF_X51_Y34_N16
\spi_sclk~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|spi_clk_reg~q\,
	devoe => ww_devoe,
	o => ww_spi_sclk);

-- Location: IOOBUF_X43_Y34_N23
\spi_ss_n~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \and1~combout\,
	devoe => ww_devoe,
	o => ww_spi_ss_n);

-- Location: IOOBUF_X51_Y34_N23
\MPU_6050_interrupt_out~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \and2~combout\,
	devoe => ww_devoe,
	o => ww_MPU_6050_interrupt_out);

-- Location: IOOBUF_X0_Y10_N23
\LED[7]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	devoe => ww_devoe,
	o => ww_LED(7));

-- Location: IOOBUF_X0_Y28_N9
\LED[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|scfifo_component|auto_generated|dpfifo|ALT_INV_empty_dff~q\,
	devoe => ww_devoe,
	o => ww_LED(6));

-- Location: IOOBUF_X0_Y26_N16
\LED[5]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst11|fifo_read~q\,
	devoe => ww_devoe,
	o => ww_LED(5));

-- Location: IOOBUF_X0_Y25_N9
\LED[4]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst16|write~q\,
	devoe => ww_devoe,
	o => ww_LED(4));

-- Location: IOOBUF_X40_Y34_N2
\LED[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst9|data_available~q\,
	devoe => ww_devoe,
	o => ww_LED(3));

-- Location: IOOBUF_X49_Y34_N9
\LED[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst8|data_available~q\,
	devoe => ww_devoe,
	o => ww_LED(2));

-- Location: IOOBUF_X49_Y34_N2
\LED[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst7|data_available~q\,
	devoe => ww_devoe,
	o => ww_LED(1));

-- Location: IOOBUF_X38_Y34_N16
\LED[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst6|data_available~q\,
	devoe => ww_devoe,
	o => ww_LED(0));

-- Location: IOIBUF_X27_Y0_N22
\clock_50~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_clock_50,
	o => \clock_50~input_o\);

-- Location: CLKCTRL_G15
\clock_50~inputclkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \clock_50~inputclkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \clock_50~inputclkctrl_outclk\);

-- Location: LCCOMB_X28_Y23_N30
\inst2|spi_2x_ce_gen_proc:clk_cnt[1]~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~0_combout\ = \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\ $ (\inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\,
	datad => \inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\,
	combout => \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~0_combout\);

-- Location: FF_X28_Y23_N31
\inst2|spi_2x_ce_gen_proc:clk_cnt[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\);

-- Location: LCCOMB_X28_Y23_N10
\inst2|clk_cnt~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|clk_cnt~0_combout\ = (\inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\ & (\inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\ $ (\inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\))) # (!\inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\ & (\inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\ & 
-- \inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\,
	datac => \inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\,
	datad => \inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\,
	combout => \inst2|clk_cnt~0_combout\);

-- Location: FF_X28_Y23_N11
\inst2|spi_2x_ce_gen_proc:clk_cnt[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|clk_cnt~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\);

-- Location: LCCOMB_X28_Y23_N20
\inst2|clk_cnt~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|clk_cnt~1_combout\ = (!\inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\ & ((\inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\) # (!\inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000101000001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\,
	datac => \inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\,
	datad => \inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\,
	combout => \inst2|clk_cnt~1_combout\);

-- Location: FF_X28_Y23_N21
\inst2|spi_2x_ce_gen_proc:clk_cnt[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|clk_cnt~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\);

-- Location: LCCOMB_X28_Y23_N6
\inst2|Equal0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Equal0~0_combout\ = (\inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\) # ((\inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\) # (!\inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110011111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|spi_2x_ce_gen_proc:clk_cnt[0]~q\,
	datac => \inst2|spi_2x_ce_gen_proc:clk_cnt[1]~q\,
	datad => \inst2|spi_2x_ce_gen_proc:clk_cnt[2]~q\,
	combout => \inst2|Equal0~0_combout\);

-- Location: FF_X28_Y23_N7
\inst2|spi_2x_ce\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Equal0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|spi_2x_ce~q\);

-- Location: LCCOMB_X28_Y23_N2
\inst2|core_n_clk~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|core_n_clk~0_combout\ = \inst2|core_n_clk~q\ $ (!\inst2|spi_2x_ce~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|core_n_clk~q\,
	datad => \inst2|spi_2x_ce~q\,
	combout => \inst2|core_n_clk~0_combout\);

-- Location: FF_X28_Y23_N3
\inst2|core_n_clk\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|core_n_clk~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|core_n_clk~q\);

-- Location: LCCOMB_X31_Y23_N26
\inst2|core_n_ce~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|core_n_ce~0_combout\ = (\inst2|spi_2x_ce~q\) # (!\inst2|core_n_clk~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111101010101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|core_n_clk~q\,
	datad => \inst2|spi_2x_ce~q\,
	combout => \inst2|core_n_ce~0_combout\);

-- Location: FF_X31_Y23_N27
\inst2|core_n_ce\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|core_n_ce~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|core_n_ce~q\);

-- Location: LCCOMB_X34_Y22_N12
\inst2|Selector4~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector4~1_combout\ = (\inst2|state_reg\(0) & (((\inst2|state_reg\(1) & !\inst2|state_reg\(3))))) # (!\inst2|state_reg\(0) & (!\inst2|state_reg\(1) & (\inst2|state_reg\(2) $ (\inst2|state_reg\(3)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000111000010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(2),
	datab => \inst2|state_reg\(0),
	datac => \inst2|state_reg\(1),
	datad => \inst2|state_reg\(3),
	combout => \inst2|Selector4~1_combout\);

-- Location: IOIBUF_X53_Y14_N1
\button0~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_button0,
	o => \button0~input_o\);

-- Location: LCCOMB_X34_Y22_N2
\inst2|state_reg[2]~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|state_reg[2]~3_combout\ = (!\button0~input_o\) # (!\inst2|core_n_ce~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011111100111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|core_n_ce~q\,
	datac => \button0~input_o\,
	combout => \inst2|state_reg[2]~3_combout\);

-- Location: FF_X34_Y22_N13
\inst2|state_reg[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector4~1_combout\,
	sclr => \ALT_INV_button0~input_o\,
	ena => \inst2|state_reg[2]~3_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|state_reg\(1));

-- Location: LCCOMB_X34_Y22_N22
\inst2|Selector3~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector3~0_combout\ = (\inst2|state_reg\(3) & (!\inst2|state_reg\(0) & (!\inst2|state_reg\(2) & !\inst2|state_reg\(1)))) # (!\inst2|state_reg\(3) & (\inst2|state_reg\(2) & ((\inst2|state_reg\(0)) # (\inst2|state_reg\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101000001000010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|state_reg\(0),
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|Selector3~0_combout\);

-- Location: FF_X34_Y22_N23
\inst2|state_reg[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector3~0_combout\,
	sclr => \ALT_INV_button0~input_o\,
	ena => \inst2|state_reg[2]~3_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|state_reg\(2));

-- Location: LCCOMB_X32_Y22_N4
\inst2|Selector4~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector4~0_combout\ = (!\inst2|state_reg\(2) & !\inst2|state_reg\(1))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|Selector4~0_combout\);

-- Location: LCCOMB_X32_Y22_N26
\inst2|state_reg~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|state_reg~0_combout\ = (!\inst2|state_reg\(0) & ((\inst2|Selector4~0_combout\ & ((\inst2|wren~q\) # (\inst2|state_reg\(3)))) # (!\inst2|Selector4~0_combout\ & ((!\inst2|state_reg\(3))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011000000100011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wren~q\,
	datab => \inst2|state_reg\(0),
	datac => \inst2|Selector4~0_combout\,
	datad => \inst2|state_reg\(3),
	combout => \inst2|state_reg~0_combout\);

-- Location: LCCOMB_X34_Y22_N16
\inst2|state_reg~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|state_reg~1_combout\ = (\inst2|core_n_ce~q\ & (\inst2|state_reg\(0))) # (!\inst2|core_n_ce~q\ & ((\inst2|state_reg~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111001111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|core_n_ce~q\,
	datac => \inst2|state_reg\(0),
	datad => \inst2|state_reg~0_combout\,
	combout => \inst2|state_reg~1_combout\);

-- Location: FF_X34_Y22_N17
\inst2|state_reg[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|state_reg~1_combout\,
	sclr => \ALT_INV_button0~input_o\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|state_reg\(0));

-- Location: LCCOMB_X34_Y22_N8
\inst2|Equal1~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Equal1~0_combout\ = (\inst2|state_reg\(3) & (\inst2|state_reg\(0) & (!\inst2|state_reg\(2) & !\inst2|state_reg\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|state_reg\(0),
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|Equal1~0_combout\);

-- Location: LCCOMB_X34_Y22_N10
\inst2|state_reg~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|state_reg~2_combout\ = (\inst2|core_n_ce~q\ & (((\inst2|state_reg\(3))))) # (!\inst2|core_n_ce~q\ & ((\inst2|Selector2~0_combout\) # ((\inst2|Equal1~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111001111100010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Selector2~0_combout\,
	datab => \inst2|core_n_ce~q\,
	datac => \inst2|state_reg\(3),
	datad => \inst2|Equal1~0_combout\,
	combout => \inst2|state_reg~2_combout\);

-- Location: FF_X34_Y22_N11
\inst2|state_reg[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|state_reg~2_combout\,
	sclr => \ALT_INV_button0~input_o\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|state_reg\(3));

-- Location: LCCOMB_X32_Y22_N20
\inst2|Selector2~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector2~0_combout\ = (!\inst2|state_reg\(3) & (!\inst2|state_reg\(1) & (!\inst2|state_reg\(2) & \inst2|wren~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|state_reg\(1),
	datac => \inst2|state_reg\(2),
	datad => \inst2|wren~q\,
	combout => \inst2|Selector2~0_combout\);

-- Location: FF_X32_Y22_N21
\inst2|wr_ack_reg\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector2~0_combout\,
	ena => \inst2|ALT_INV_core_n_ce~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|wr_ack_reg~q\);

-- Location: LCCOMB_X28_Y22_N4
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~combout\ = \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0) $ (((VCC) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\)))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~COUT\ = CARRY(\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ $ (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001110011001",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0),
	datad => VCC,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~COUT\);

-- Location: LCCOMB_X25_Y22_N4
\~GND\ : cycloneive_lcell_comb
-- Equation(s):
-- \~GND~combout\ = GND

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	combout => \~GND~combout\);

-- Location: LCCOMB_X24_Y22_N24
\inst27~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst27~0_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\) # (!\button0~input_o\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000011111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	datad => \button0~input_o\,
	combout => \inst27~0_combout\);

-- Location: LCCOMB_X28_Y22_N8
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~COUT\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2) & VCC)))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2) $ (((VCC) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\)))))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~COUT\ & (\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ $ 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001001",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~COUT\);

-- Location: LCCOMB_X28_Y22_N10
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3) $ (((\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\) # (VCC))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~COUT\ & 
-- ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3)) # ((GND))))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3) $ (\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001101111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~COUT\);

-- Location: FF_X28_Y22_N11
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3));

-- Location: LCCOMB_X28_Y22_N12
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4) & ((VCC)))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4) $ (((VCC) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\)))))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4) $ (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001001",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita3~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~COUT\);

-- Location: LCCOMB_X28_Y22_N14
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5) $ (((\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\) # (VCC))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~COUT\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5)) # (GND))))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ $ (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110001101111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~COUT\);

-- Location: FF_X28_Y22_N15
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5));

-- Location: LCCOMB_X28_Y22_N16
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~COUT\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6) & VCC)))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6) $ (((VCC) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\)))))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~COUT\ & (\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ $ 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001001",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita5~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~COUT\);

-- Location: FF_X28_Y22_N17
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6));

-- Location: LCCOMB_X28_Y22_N18
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7) $ (((\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\) # (VCC))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~COUT\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7)) # (GND))))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ $ (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110001101111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita6~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~COUT\);

-- Location: FF_X28_Y22_N19
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7));

-- Location: LCCOMB_X28_Y22_N20
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita8~combout\ = \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~COUT\ $ 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8),
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita7~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita8~combout\);

-- Location: FF_X28_Y22_N21
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita8~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8));

-- Location: LCCOMB_X28_Y22_N30
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~4_combout\ = (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6) & !\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000110011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~4_combout\);

-- Location: LCCOMB_X28_Y22_N24
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~4_combout\ & (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7) & 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5) & !\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~4_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\);

-- Location: LCCOMB_X28_Y22_N2
\inst11|LessThan2~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|LessThan2~0_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3) & ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2)) # 
-- ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0)) # (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101010101000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1),
	combout => \inst11|LessThan2~0_combout\);

-- Location: LCCOMB_X30_Y20_N8
\inst11|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~0_combout\ = \inst11|numberOfBytesTransmitted\(0) $ (VCC)
-- \inst11|Add0~1\ = CARRY(\inst11|numberOfBytesTransmitted\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010110101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(0),
	datad => VCC,
	combout => \inst11|Add0~0_combout\,
	cout => \inst11|Add0~1\);

-- Location: LCCOMB_X30_Y22_N2
\inst11|numberOfBytesTransmitted~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted~8_combout\ = (\inst11|Add0~0_combout\ & ((\inst11|LessThan0~1_combout\) # ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\ & !\inst11|LessThan2~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datab => \inst11|LessThan2~0_combout\,
	datac => \inst11|Add0~0_combout\,
	datad => \inst11|LessThan0~1_combout\,
	combout => \inst11|numberOfBytesTransmitted~8_combout\);

-- Location: FF_X31_Y22_N7
\inst11|write_ack_prev\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst2|wr_ack_reg~q\,
	clrn => \button0~input_o\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|write_ack_prev~q\);

-- Location: LCCOMB_X31_Y22_N6
\inst11|wren~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|wren~2_combout\ = (\inst2|wr_ack_reg~q\ & !\inst11|write_ack_prev~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000101000001010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wr_ack_reg~q\,
	datac => \inst11|write_ack_prev~q\,
	combout => \inst11|wren~2_combout\);

-- Location: LCCOMB_X30_Y22_N14
\inst11|numberOfBytesTransmitted[1]~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted[1]~1_combout\ = (\inst11|wren~2_combout\) # ((!\inst11|LessThan0~1_combout\ & ((\inst11|LessThan2~0_combout\) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100110001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datab => \inst11|LessThan0~1_combout\,
	datac => \inst11|LessThan2~0_combout\,
	datad => \inst11|wren~2_combout\,
	combout => \inst11|numberOfBytesTransmitted[1]~1_combout\);

-- Location: FF_X30_Y22_N3
\inst11|numberOfBytesTransmitted[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted~8_combout\,
	clrn => \button0~input_o\,
	ena => \inst11|numberOfBytesTransmitted[1]~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(0));

-- Location: LCCOMB_X30_Y20_N10
\inst11|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~2_combout\ = (\inst11|numberOfBytesTransmitted\(1) & ((\inst11|Add0~1\) # (GND))) # (!\inst11|numberOfBytesTransmitted\(1) & (!\inst11|Add0~1\))
-- \inst11|Add0~3\ = CARRY((\inst11|numberOfBytesTransmitted\(1)) # (!\inst11|Add0~1\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110101111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datad => VCC,
	cin => \inst11|Add0~1\,
	combout => \inst11|Add0~2_combout\,
	cout => \inst11|Add0~3\);

-- Location: LCCOMB_X30_Y22_N26
\inst11|numberOfBytesTransmitted~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted~0_combout\ = ((!\inst11|LessThan0~1_combout\ & ((\inst11|LessThan2~0_combout\) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\)))) # (!\inst11|Add0~2_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011000111111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datab => \inst11|LessThan0~1_combout\,
	datac => \inst11|LessThan2~0_combout\,
	datad => \inst11|Add0~2_combout\,
	combout => \inst11|numberOfBytesTransmitted~0_combout\);

-- Location: FF_X30_Y22_N27
\inst11|numberOfBytesTransmitted[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted~0_combout\,
	clrn => \button0~input_o\,
	ena => \inst11|numberOfBytesTransmitted[1]~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(1));

-- Location: LCCOMB_X30_Y20_N12
\inst11|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~4_combout\ = (\inst11|numberOfBytesTransmitted\(2) & (\inst11|Add0~3\ $ (GND))) # (!\inst11|numberOfBytesTransmitted\(2) & (!\inst11|Add0~3\ & VCC))
-- \inst11|Add0~5\ = CARRY((\inst11|numberOfBytesTransmitted\(2) & !\inst11|Add0~3\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst11|numberOfBytesTransmitted\(2),
	datad => VCC,
	cin => \inst11|Add0~3\,
	combout => \inst11|Add0~4_combout\,
	cout => \inst11|Add0~5\);

-- Location: LCCOMB_X30_Y22_N8
\inst11|numberOfBytesTransmitted~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted~4_combout\ = (\inst11|Add0~4_combout\ & ((\inst11|LessThan0~1_combout\) # ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\ & !\inst11|LessThan2~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datab => \inst11|LessThan2~0_combout\,
	datac => \inst11|Add0~4_combout\,
	datad => \inst11|LessThan0~1_combout\,
	combout => \inst11|numberOfBytesTransmitted~4_combout\);

-- Location: FF_X30_Y22_N9
\inst11|numberOfBytesTransmitted[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted~4_combout\,
	clrn => \button0~input_o\,
	ena => \inst11|numberOfBytesTransmitted[1]~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(2));

-- Location: LCCOMB_X30_Y20_N14
\inst11|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~6_combout\ = (\inst11|numberOfBytesTransmitted\(3) & (!\inst11|Add0~5\)) # (!\inst11|numberOfBytesTransmitted\(3) & ((\inst11|Add0~5\) # (GND)))
-- \inst11|Add0~7\ = CARRY((!\inst11|Add0~5\) # (!\inst11|numberOfBytesTransmitted\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(3),
	datad => VCC,
	cin => \inst11|Add0~5\,
	combout => \inst11|Add0~6_combout\,
	cout => \inst11|Add0~7\);

-- Location: LCCOMB_X30_Y22_N6
\inst11|numberOfBytesTransmitted~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted~3_combout\ = (\inst11|Add0~6_combout\ & ((\inst11|LessThan0~1_combout\) # ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\ & !\inst11|LessThan2~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100111000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datab => \inst11|LessThan0~1_combout\,
	datac => \inst11|LessThan2~0_combout\,
	datad => \inst11|Add0~6_combout\,
	combout => \inst11|numberOfBytesTransmitted~3_combout\);

-- Location: FF_X30_Y22_N7
\inst11|numberOfBytesTransmitted[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted~3_combout\,
	clrn => \button0~input_o\,
	ena => \inst11|numberOfBytesTransmitted[1]~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(3));

-- Location: LCCOMB_X30_Y20_N24
\inst11|LessThan0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|LessThan0~0_combout\ = (!\inst11|numberOfBytesTransmitted\(3) & (!\inst11|numberOfBytesTransmitted\(4) & (\inst11|numberOfBytesTransmitted\(1) & !\inst11|numberOfBytesTransmitted\(2))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(3),
	datab => \inst11|numberOfBytesTransmitted\(4),
	datac => \inst11|numberOfBytesTransmitted\(1),
	datad => \inst11|numberOfBytesTransmitted\(2),
	combout => \inst11|LessThan0~0_combout\);

-- Location: LCCOMB_X30_Y20_N18
\inst11|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~10_combout\ = (\inst11|numberOfBytesTransmitted\(5) & ((\inst11|Add0~9\) # (GND))) # (!\inst11|numberOfBytesTransmitted\(5) & (!\inst11|Add0~9\))
-- \inst11|Add0~11\ = CARRY((\inst11|numberOfBytesTransmitted\(5)) # (!\inst11|Add0~9\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110101111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(5),
	datad => VCC,
	cin => \inst11|Add0~9\,
	combout => \inst11|Add0~10_combout\,
	cout => \inst11|Add0~11\);

-- Location: LCCOMB_X30_Y20_N20
\inst11|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~12_combout\ = (\inst11|numberOfBytesTransmitted\(6) & (\inst11|Add0~11\ $ (GND))) # (!\inst11|numberOfBytesTransmitted\(6) & (!\inst11|Add0~11\ & VCC))
-- \inst11|Add0~13\ = CARRY((\inst11|numberOfBytesTransmitted\(6) & !\inst11|Add0~11\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(6),
	datad => VCC,
	cin => \inst11|Add0~11\,
	combout => \inst11|Add0~12_combout\,
	cout => \inst11|Add0~13\);

-- Location: LCCOMB_X30_Y20_N30
\inst11|numberOfBytesTransmitted[6]~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted[6]~7_combout\ = (!\inst11|always0~0_combout\ & ((\inst11|wren~2_combout\ & (\inst11|Add0~12_combout\)) # (!\inst11|wren~2_combout\ & ((\inst11|numberOfBytesTransmitted\(6))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100010001010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|always0~0_combout\,
	datab => \inst11|Add0~12_combout\,
	datac => \inst11|numberOfBytesTransmitted\(6),
	datad => \inst11|wren~2_combout\,
	combout => \inst11|numberOfBytesTransmitted[6]~7_combout\);

-- Location: FF_X30_Y20_N31
\inst11|numberOfBytesTransmitted[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted[6]~7_combout\,
	clrn => \button0~input_o\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(6));

-- Location: LCCOMB_X30_Y20_N22
\inst11|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~14_combout\ = \inst11|Add0~13\ $ (\inst11|numberOfBytesTransmitted\(7))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datad => \inst11|numberOfBytesTransmitted\(7),
	cin => \inst11|Add0~13\,
	combout => \inst11|Add0~14_combout\);

-- Location: LCCOMB_X30_Y20_N28
\inst11|numberOfBytesTransmitted[7]~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted[7]~6_combout\ = (!\inst11|always0~0_combout\ & ((\inst11|wren~2_combout\ & (\inst11|Add0~14_combout\)) # (!\inst11|wren~2_combout\ & ((\inst11|numberOfBytesTransmitted\(7))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100010001010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|always0~0_combout\,
	datab => \inst11|Add0~14_combout\,
	datac => \inst11|numberOfBytesTransmitted\(7),
	datad => \inst11|wren~2_combout\,
	combout => \inst11|numberOfBytesTransmitted[7]~6_combout\);

-- Location: FF_X30_Y20_N29
\inst11|numberOfBytesTransmitted[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted[7]~6_combout\,
	clrn => \button0~input_o\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(7));

-- Location: LCCOMB_X30_Y20_N0
\inst11|LessThan0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|LessThan0~1_combout\ = (!\inst11|numberOfBytesTransmitted\(6) & (!\inst11|numberOfBytesTransmitted\(7) & ((\inst11|numberOfBytesTransmitted\(5)) # (\inst11|LessThan0~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000001110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(5),
	datab => \inst11|LessThan0~0_combout\,
	datac => \inst11|numberOfBytesTransmitted\(6),
	datad => \inst11|numberOfBytesTransmitted\(7),
	combout => \inst11|LessThan0~1_combout\);

-- Location: LCCOMB_X30_Y20_N16
\inst11|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Add0~8_combout\ = (\inst11|numberOfBytesTransmitted\(4) & (\inst11|Add0~7\ $ (GND))) # (!\inst11|numberOfBytesTransmitted\(4) & (!\inst11|Add0~7\ & VCC))
-- \inst11|Add0~9\ = CARRY((\inst11|numberOfBytesTransmitted\(4) & !\inst11|Add0~7\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst11|numberOfBytesTransmitted\(4),
	datad => VCC,
	cin => \inst11|Add0~7\,
	combout => \inst11|Add0~8_combout\,
	cout => \inst11|Add0~9\);

-- Location: LCCOMB_X30_Y22_N4
\inst11|numberOfBytesTransmitted~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted~2_combout\ = (\inst11|Add0~8_combout\ & ((\inst11|LessThan0~1_combout\) # ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\ & !\inst11|LessThan2~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100111000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datab => \inst11|LessThan0~1_combout\,
	datac => \inst11|LessThan2~0_combout\,
	datad => \inst11|Add0~8_combout\,
	combout => \inst11|numberOfBytesTransmitted~2_combout\);

-- Location: FF_X30_Y22_N5
\inst11|numberOfBytesTransmitted[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted~2_combout\,
	clrn => \button0~input_o\,
	ena => \inst11|numberOfBytesTransmitted[1]~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(4));

-- Location: LCCOMB_X30_Y20_N26
\inst11|numberOfBytesTransmitted[5]~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|numberOfBytesTransmitted[5]~5_combout\ = (\inst11|always0~0_combout\) # ((\inst11|wren~2_combout\ & (!\inst11|Add0~10_combout\)) # (!\inst11|wren~2_combout\ & ((\inst11|numberOfBytesTransmitted\(5)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011101111111010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|always0~0_combout\,
	datab => \inst11|Add0~10_combout\,
	datac => \inst11|numberOfBytesTransmitted\(5),
	datad => \inst11|wren~2_combout\,
	combout => \inst11|numberOfBytesTransmitted[5]~5_combout\);

-- Location: FF_X30_Y20_N27
\inst11|numberOfBytesTransmitted[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|numberOfBytesTransmitted[5]~5_combout\,
	clrn => \button0~input_o\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|numberOfBytesTransmitted\(5));

-- Location: LCCOMB_X30_Y20_N2
\inst11|LessThan1~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|LessThan1~0_combout\ = (((\inst11|numberOfBytesTransmitted\(6)) # (\inst11|numberOfBytesTransmitted\(7))) # (!\inst11|LessThan0~0_combout\)) # (!\inst11|numberOfBytesTransmitted\(5))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111110111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(5),
	datab => \inst11|LessThan0~0_combout\,
	datac => \inst11|numberOfBytesTransmitted\(6),
	datad => \inst11|numberOfBytesTransmitted\(7),
	combout => \inst11|LessThan1~0_combout\);

-- Location: LCCOMB_X30_Y22_N28
\inst11|fifo_read~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|fifo_read~0_combout\ = (\inst11|numberOfBytesTransmitted\(1) & \inst11|numberOfBytesTransmitted\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst11|numberOfBytesTransmitted\(1),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|fifo_read~0_combout\);

-- Location: LCCOMB_X32_Y22_N22
\inst2|WideOr2~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|WideOr2~0_combout\ = (\inst2|state_reg\(3) & (((\inst2|state_reg\(2)) # (\inst2|state_reg\(1))))) # (!\inst2|state_reg\(3) & (((!\inst2|state_reg\(0) & !\inst2|state_reg\(1))) # (!\inst2|state_reg\(2))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010111110110101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|state_reg\(0),
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|WideOr2~0_combout\);

-- Location: FF_X32_Y22_N23
\inst2|di_req_reg\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|WideOr2~0_combout\,
	ena => \inst2|ALT_INV_core_n_ce~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_req_reg~q\);

-- Location: LCCOMB_X31_Y22_N28
\inst2|di_req_o_A~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_req_o_A~feeder_combout\ = \inst2|di_req_reg~q\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst2|di_req_reg~q\,
	combout => \inst2|di_req_o_A~feeder_combout\);

-- Location: FF_X31_Y22_N29
\inst2|di_req_o_A\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_req_o_A~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_req_o_A~q\);

-- Location: LCCOMB_X31_Y22_N18
\inst2|di_req_o_B~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_req_o_B~feeder_combout\ = \inst2|di_req_o_A~q\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst2|di_req_o_A~q\,
	combout => \inst2|di_req_o_B~feeder_combout\);

-- Location: FF_X31_Y22_N19
\inst2|di_req_o_B\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_req_o_B~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_req_o_B~q\);

-- Location: FF_X30_Y20_N5
\inst2|di_req_o_C\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst2|di_req_o_B~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_req_o_C~q\);

-- Location: LCCOMB_X31_Y22_N14
\inst2|di_req_o_D~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_req_o_D~feeder_combout\ = \inst2|di_req_o_C~q\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst2|di_req_o_C~q\,
	combout => \inst2|di_req_o_D~feeder_combout\);

-- Location: FF_X31_Y22_N15
\inst2|di_req_o_D\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_req_o_D~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_req_o_D~q\);

-- Location: LCCOMB_X31_Y22_N4
\inst2|di_req_o_next~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_req_o_next~0_combout\ = ((\inst2|di_req_o_D~q\) # (!\inst2|di_req_o_B~q\)) # (!\inst2|di_req_o_A~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111001111111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|di_req_o_A~q\,
	datac => \inst2|di_req_o_D~q\,
	datad => \inst2|di_req_o_B~q\,
	combout => \inst2|di_req_o_next~0_combout\);

-- Location: FF_X31_Y22_N5
\inst2|di_req_o_reg\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_req_o_next~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_req_o_reg~q\);

-- Location: LCCOMB_X30_Y22_N0
\inst11|fifo_read~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|fifo_read~1_combout\ = (\inst11|LessThan0~1_combout\ & (!\inst2|di_req_o_reg~q\ & ((\inst11|fifo_read~0_combout\) # (!\inst11|LessThan1~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000011010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|LessThan1~0_combout\,
	datab => \inst11|fifo_read~0_combout\,
	datac => \inst11|LessThan0~1_combout\,
	datad => \inst2|di_req_o_reg~q\,
	combout => \inst11|fifo_read~1_combout\);

-- Location: FF_X30_Y22_N1
\inst11|fifo_read\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|fifo_read~1_combout\,
	ena => \button0~input_o\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|fifo_read~q\);

-- Location: LCCOMB_X27_Y22_N16
\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ = (\inst11|fifo_read~q\ & \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|fifo_read~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\);

-- Location: LCCOMB_X24_Y22_N14
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~combout\ = (!\inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3_combout\ & \button0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3_combout\,
	datad => \button0~input_o\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~combout\);

-- Location: FF_X24_Y22_N15
\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\);

-- Location: LCCOMB_X24_Y22_N18
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff~q\) # 
-- ((!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\)))) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & 
-- ((!\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\))) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff~q\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000110111101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0_combout\);

-- Location: LCCOMB_X24_Y22_N0
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~1_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0_combout\ & (!\inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\ & \button0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000110000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	datad => \button0~input_o\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~1_combout\);

-- Location: FF_X24_Y22_N1
\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff~q\);

-- Location: LCCOMB_X24_Y22_N26
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~2_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & 
-- ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\))) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff~q\)))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & (((\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\ & !\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000001110010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_0_dff~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~2_combout\);

-- Location: LCCOMB_X28_Y22_N22
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~8_combout\ = (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3) & (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2) & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0) & \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~8_combout\);

-- Location: LCCOMB_X24_Y22_N20
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~10_combout\ = (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~8_combout\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & \inst11|fifo_read~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~8_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datad => \inst11|fifo_read~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~10_combout\);

-- Location: LCCOMB_X24_Y22_N28
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~7_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & 
-- ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff~q\))) # (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\)))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & (((\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff~q\ & !\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000011011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_1_dff~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~7_combout\);

-- Location: LCCOMB_X24_Y22_N12
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~9_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~7_combout\) # ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~10_combout\ & 
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000011101100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~10_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~7_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datad => \inst27~0_combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~9_combout\);

-- Location: FF_X24_Y22_N13
\inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~9_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff~q\);

-- Location: LCCOMB_X24_Y22_N30
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~2_combout\) # ((!\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111010011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~2_combout\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_is_2_dff~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3_combout\);

-- Location: IOIBUF_X29_Y34_N15
\sensor7_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor7_signal,
	o => \sensor7_signal~input_o\);

-- Location: CLKCTRL_G12
\sensor7_signal~inputclkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \sensor7_signal~inputclkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \sensor7_signal~inputclkctrl_outclk\);

-- Location: PLL_4
\inst|altpll_component|auto_generated|pll1\ : cycloneive_pll
-- pragma translate_off
GENERIC MAP (
	auto_settings => "false",
	bandwidth_type => "medium",
	c0_high => 250,
	c0_initial => 1,
	c0_low => 250,
	c0_mode => "even",
	c0_ph => 0,
	c1_high => 0,
	c1_initial => 0,
	c1_low => 0,
	c1_mode => "bypass",
	c1_ph => 0,
	c1_use_casc_in => "off",
	c2_high => 0,
	c2_initial => 0,
	c2_low => 0,
	c2_mode => "bypass",
	c2_ph => 0,
	c2_use_casc_in => "off",
	c3_high => 0,
	c3_initial => 0,
	c3_low => 0,
	c3_mode => "bypass",
	c3_ph => 0,
	c3_use_casc_in => "off",
	c4_high => 0,
	c4_initial => 0,
	c4_low => 0,
	c4_mode => "bypass",
	c4_ph => 0,
	c4_use_casc_in => "off",
	charge_pump_current_bits => 1,
	clk0_counter => "c0",
	clk0_divide_by => 50,
	clk0_duty_cycle => 50,
	clk0_multiply_by => 1,
	clk0_phase_shift => "0",
	clk1_counter => "unused",
	clk1_divide_by => 0,
	clk1_duty_cycle => 50,
	clk1_multiply_by => 0,
	clk1_phase_shift => "0",
	clk2_counter => "unused",
	clk2_divide_by => 0,
	clk2_duty_cycle => 50,
	clk2_multiply_by => 0,
	clk2_phase_shift => "0",
	clk3_counter => "unused",
	clk3_divide_by => 0,
	clk3_duty_cycle => 50,
	clk3_multiply_by => 0,
	clk3_phase_shift => "0",
	clk4_counter => "unused",
	clk4_divide_by => 0,
	clk4_duty_cycle => 50,
	clk4_multiply_by => 0,
	clk4_phase_shift => "0",
	compensate_clock => "clock0",
	inclk0_input_frequency => 20000,
	inclk1_input_frequency => 0,
	loop_filter_c_bits => 0,
	loop_filter_r_bits => 27,
	m => 10,
	m_initial => 1,
	m_ph => 0,
	n => 1,
	operation_mode => "normal",
	pfd_max => 200000,
	pfd_min => 3076,
	self_reset_on_loss_lock => "off",
	simulation_type => "timing",
	switch_over_type => "auto",
	vco_center => 1538,
	vco_divide_by => 0,
	vco_frequency_control => "auto",
	vco_max => 3333,
	vco_min => 1538,
	vco_multiply_by => 0,
	vco_phase_shift_step => 250,
	vco_post_scale => 2)
-- pragma translate_on
PORT MAP (
	fbin => \inst|altpll_component|auto_generated|wire_pll1_fbout\,
	inclk => \inst|altpll_component|auto_generated|pll1_INCLK_bus\,
	fbout => \inst|altpll_component|auto_generated|wire_pll1_fbout\,
	clk => \inst|altpll_component|auto_generated|pll1_CLK_bus\);

-- Location: CLKCTRL_G18
\inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\);

-- Location: LCCOMB_X17_Y26_N8
\inst33|temp[0]~93\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[0]~93_combout\ = !\inst33|temp\(0)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111100001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst33|temp\(0),
	combout => \inst33|temp[0]~93_combout\);

-- Location: FF_X17_Y26_N9
\inst33|temp[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[0]~93_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(0));

-- Location: LCCOMB_X15_Y30_N2
\inst33|temp[1]~31\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[1]~31_combout\ = (\inst33|temp\(0) & (\inst33|temp\(1) $ (VCC))) # (!\inst33|temp\(0) & (\inst33|temp\(1) & VCC))
-- \inst33|temp[1]~32\ = CARRY((\inst33|temp\(0) & \inst33|temp\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(0),
	datab => \inst33|temp\(1),
	datad => VCC,
	combout => \inst33|temp[1]~31_combout\,
	cout => \inst33|temp[1]~32\);

-- Location: FF_X15_Y30_N3
\inst33|temp[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[1]~31_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(1));

-- Location: LCCOMB_X15_Y30_N4
\inst33|temp[2]~33\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[2]~33_combout\ = (\inst33|temp\(2) & (!\inst33|temp[1]~32\)) # (!\inst33|temp\(2) & ((\inst33|temp[1]~32\) # (GND)))
-- \inst33|temp[2]~34\ = CARRY((!\inst33|temp[1]~32\) # (!\inst33|temp\(2)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(2),
	datad => VCC,
	cin => \inst33|temp[1]~32\,
	combout => \inst33|temp[2]~33_combout\,
	cout => \inst33|temp[2]~34\);

-- Location: FF_X15_Y30_N5
\inst33|temp[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[2]~33_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(2));

-- Location: LCCOMB_X15_Y30_N6
\inst33|temp[3]~35\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[3]~35_combout\ = (\inst33|temp\(3) & (\inst33|temp[2]~34\ $ (GND))) # (!\inst33|temp\(3) & (!\inst33|temp[2]~34\ & VCC))
-- \inst33|temp[3]~36\ = CARRY((\inst33|temp\(3) & !\inst33|temp[2]~34\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(3),
	datad => VCC,
	cin => \inst33|temp[2]~34\,
	combout => \inst33|temp[3]~35_combout\,
	cout => \inst33|temp[3]~36\);

-- Location: FF_X15_Y30_N7
\inst33|temp[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[3]~35_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(3));

-- Location: LCCOMB_X15_Y30_N8
\inst33|temp[4]~37\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[4]~37_combout\ = (\inst33|temp\(4) & (!\inst33|temp[3]~36\)) # (!\inst33|temp\(4) & ((\inst33|temp[3]~36\) # (GND)))
-- \inst33|temp[4]~38\ = CARRY((!\inst33|temp[3]~36\) # (!\inst33|temp\(4)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(4),
	datad => VCC,
	cin => \inst33|temp[3]~36\,
	combout => \inst33|temp[4]~37_combout\,
	cout => \inst33|temp[4]~38\);

-- Location: FF_X15_Y30_N9
\inst33|temp[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[4]~37_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(4));

-- Location: LCCOMB_X15_Y30_N10
\inst33|temp[5]~39\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[5]~39_combout\ = (\inst33|temp\(5) & (\inst33|temp[4]~38\ $ (GND))) # (!\inst33|temp\(5) & (!\inst33|temp[4]~38\ & VCC))
-- \inst33|temp[5]~40\ = CARRY((\inst33|temp\(5) & !\inst33|temp[4]~38\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(5),
	datad => VCC,
	cin => \inst33|temp[4]~38\,
	combout => \inst33|temp[5]~39_combout\,
	cout => \inst33|temp[5]~40\);

-- Location: FF_X15_Y30_N11
\inst33|temp[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[5]~39_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(5));

-- Location: LCCOMB_X15_Y30_N12
\inst33|temp[6]~41\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[6]~41_combout\ = (\inst33|temp\(6) & (!\inst33|temp[5]~40\)) # (!\inst33|temp\(6) & ((\inst33|temp[5]~40\) # (GND)))
-- \inst33|temp[6]~42\ = CARRY((!\inst33|temp[5]~40\) # (!\inst33|temp\(6)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(6),
	datad => VCC,
	cin => \inst33|temp[5]~40\,
	combout => \inst33|temp[6]~41_combout\,
	cout => \inst33|temp[6]~42\);

-- Location: FF_X15_Y30_N13
\inst33|temp[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[6]~41_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(6));

-- Location: LCCOMB_X15_Y30_N14
\inst33|temp[7]~43\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[7]~43_combout\ = (\inst33|temp\(7) & (\inst33|temp[6]~42\ $ (GND))) # (!\inst33|temp\(7) & (!\inst33|temp[6]~42\ & VCC))
-- \inst33|temp[7]~44\ = CARRY((\inst33|temp\(7) & !\inst33|temp[6]~42\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(7),
	datad => VCC,
	cin => \inst33|temp[6]~42\,
	combout => \inst33|temp[7]~43_combout\,
	cout => \inst33|temp[7]~44\);

-- Location: FF_X15_Y30_N15
\inst33|temp[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[7]~43_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(7));

-- Location: LCCOMB_X15_Y30_N16
\inst33|temp[8]~45\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[8]~45_combout\ = (\inst33|temp\(8) & (!\inst33|temp[7]~44\)) # (!\inst33|temp\(8) & ((\inst33|temp[7]~44\) # (GND)))
-- \inst33|temp[8]~46\ = CARRY((!\inst33|temp[7]~44\) # (!\inst33|temp\(8)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(8),
	datad => VCC,
	cin => \inst33|temp[7]~44\,
	combout => \inst33|temp[8]~45_combout\,
	cout => \inst33|temp[8]~46\);

-- Location: FF_X15_Y30_N17
\inst33|temp[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[8]~45_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(8));

-- Location: LCCOMB_X15_Y30_N18
\inst33|temp[9]~47\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[9]~47_combout\ = (\inst33|temp\(9) & (\inst33|temp[8]~46\ $ (GND))) # (!\inst33|temp\(9) & (!\inst33|temp[8]~46\ & VCC))
-- \inst33|temp[9]~48\ = CARRY((\inst33|temp\(9) & !\inst33|temp[8]~46\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(9),
	datad => VCC,
	cin => \inst33|temp[8]~46\,
	combout => \inst33|temp[9]~47_combout\,
	cout => \inst33|temp[9]~48\);

-- Location: FF_X15_Y30_N19
\inst33|temp[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[9]~47_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(9));

-- Location: LCCOMB_X15_Y30_N20
\inst33|temp[10]~49\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[10]~49_combout\ = (\inst33|temp\(10) & (!\inst33|temp[9]~48\)) # (!\inst33|temp\(10) & ((\inst33|temp[9]~48\) # (GND)))
-- \inst33|temp[10]~50\ = CARRY((!\inst33|temp[9]~48\) # (!\inst33|temp\(10)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(10),
	datad => VCC,
	cin => \inst33|temp[9]~48\,
	combout => \inst33|temp[10]~49_combout\,
	cout => \inst33|temp[10]~50\);

-- Location: FF_X15_Y30_N21
\inst33|temp[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[10]~49_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(10));

-- Location: LCCOMB_X15_Y30_N22
\inst33|temp[11]~51\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[11]~51_combout\ = (\inst33|temp\(11) & (\inst33|temp[10]~50\ $ (GND))) # (!\inst33|temp\(11) & (!\inst33|temp[10]~50\ & VCC))
-- \inst33|temp[11]~52\ = CARRY((\inst33|temp\(11) & !\inst33|temp[10]~50\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(11),
	datad => VCC,
	cin => \inst33|temp[10]~50\,
	combout => \inst33|temp[11]~51_combout\,
	cout => \inst33|temp[11]~52\);

-- Location: FF_X15_Y30_N23
\inst33|temp[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[11]~51_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(11));

-- Location: LCCOMB_X15_Y30_N24
\inst33|temp[12]~53\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[12]~53_combout\ = (\inst33|temp\(12) & (!\inst33|temp[11]~52\)) # (!\inst33|temp\(12) & ((\inst33|temp[11]~52\) # (GND)))
-- \inst33|temp[12]~54\ = CARRY((!\inst33|temp[11]~52\) # (!\inst33|temp\(12)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(12),
	datad => VCC,
	cin => \inst33|temp[11]~52\,
	combout => \inst33|temp[12]~53_combout\,
	cout => \inst33|temp[12]~54\);

-- Location: FF_X15_Y30_N25
\inst33|temp[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[12]~53_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(12));

-- Location: LCCOMB_X15_Y30_N26
\inst33|temp[13]~55\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[13]~55_combout\ = (\inst33|temp\(13) & (\inst33|temp[12]~54\ $ (GND))) # (!\inst33|temp\(13) & (!\inst33|temp[12]~54\ & VCC))
-- \inst33|temp[13]~56\ = CARRY((\inst33|temp\(13) & !\inst33|temp[12]~54\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(13),
	datad => VCC,
	cin => \inst33|temp[12]~54\,
	combout => \inst33|temp[13]~55_combout\,
	cout => \inst33|temp[13]~56\);

-- Location: FF_X15_Y30_N27
\inst33|temp[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[13]~55_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(13));

-- Location: LCCOMB_X15_Y30_N28
\inst33|temp[14]~57\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[14]~57_combout\ = (\inst33|temp\(14) & (!\inst33|temp[13]~56\)) # (!\inst33|temp\(14) & ((\inst33|temp[13]~56\) # (GND)))
-- \inst33|temp[14]~58\ = CARRY((!\inst33|temp[13]~56\) # (!\inst33|temp\(14)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(14),
	datad => VCC,
	cin => \inst33|temp[13]~56\,
	combout => \inst33|temp[14]~57_combout\,
	cout => \inst33|temp[14]~58\);

-- Location: FF_X15_Y30_N29
\inst33|temp[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[14]~57_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(14));

-- Location: LCCOMB_X15_Y30_N30
\inst33|temp[15]~59\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[15]~59_combout\ = (\inst33|temp\(15) & (\inst33|temp[14]~58\ $ (GND))) # (!\inst33|temp\(15) & (!\inst33|temp[14]~58\ & VCC))
-- \inst33|temp[15]~60\ = CARRY((\inst33|temp\(15) & !\inst33|temp[14]~58\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(15),
	datad => VCC,
	cin => \inst33|temp[14]~58\,
	combout => \inst33|temp[15]~59_combout\,
	cout => \inst33|temp[15]~60\);

-- Location: FF_X15_Y30_N31
\inst33|temp[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[15]~59_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(15));

-- Location: LCCOMB_X15_Y29_N0
\inst33|temp[16]~61\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[16]~61_combout\ = (\inst33|temp\(16) & (!\inst33|temp[15]~60\)) # (!\inst33|temp\(16) & ((\inst33|temp[15]~60\) # (GND)))
-- \inst33|temp[16]~62\ = CARRY((!\inst33|temp[15]~60\) # (!\inst33|temp\(16)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(16),
	datad => VCC,
	cin => \inst33|temp[15]~60\,
	combout => \inst33|temp[16]~61_combout\,
	cout => \inst33|temp[16]~62\);

-- Location: FF_X15_Y29_N1
\inst33|temp[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[16]~61_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(16));

-- Location: LCCOMB_X15_Y29_N2
\inst33|temp[17]~63\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[17]~63_combout\ = (\inst33|temp\(17) & (\inst33|temp[16]~62\ $ (GND))) # (!\inst33|temp\(17) & (!\inst33|temp[16]~62\ & VCC))
-- \inst33|temp[17]~64\ = CARRY((\inst33|temp\(17) & !\inst33|temp[16]~62\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(17),
	datad => VCC,
	cin => \inst33|temp[16]~62\,
	combout => \inst33|temp[17]~63_combout\,
	cout => \inst33|temp[17]~64\);

-- Location: FF_X15_Y29_N3
\inst33|temp[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[17]~63_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(17));

-- Location: LCCOMB_X15_Y29_N4
\inst33|temp[18]~65\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[18]~65_combout\ = (\inst33|temp\(18) & (!\inst33|temp[17]~64\)) # (!\inst33|temp\(18) & ((\inst33|temp[17]~64\) # (GND)))
-- \inst33|temp[18]~66\ = CARRY((!\inst33|temp[17]~64\) # (!\inst33|temp\(18)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(18),
	datad => VCC,
	cin => \inst33|temp[17]~64\,
	combout => \inst33|temp[18]~65_combout\,
	cout => \inst33|temp[18]~66\);

-- Location: FF_X15_Y29_N5
\inst33|temp[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[18]~65_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(18));

-- Location: FF_X20_Y25_N5
\inst15|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(18));

-- Location: FF_X20_Y25_N3
\inst15|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(17));

-- Location: FF_X20_Y25_N1
\inst15|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(16));

-- Location: FF_X20_Y26_N31
\inst15|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(15));

-- Location: FF_X20_Y26_N29
\inst15|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(14));

-- Location: FF_X20_Y26_N27
\inst15|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(13));

-- Location: FF_X20_Y26_N25
\inst15|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(12));

-- Location: FF_X20_Y26_N23
\inst15|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(11));

-- Location: FF_X20_Y26_N21
\inst15|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(10));

-- Location: FF_X20_Y26_N19
\inst15|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(9));

-- Location: FF_X20_Y26_N17
\inst15|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(8));

-- Location: FF_X20_Y26_N15
\inst15|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(7));

-- Location: FF_X20_Y26_N13
\inst15|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(6));

-- Location: FF_X20_Y26_N11
\inst15|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(5));

-- Location: FF_X20_Y26_N9
\inst15|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(4));

-- Location: FF_X20_Y26_N7
\inst15|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(3));

-- Location: FF_X20_Y26_N5
\inst15|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(2));

-- Location: FF_X20_Y26_N3
\inst15|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(1));

-- Location: FF_X20_Y26_N1
\inst15|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(0));

-- Location: LCCOMB_X20_Y26_N0
\inst15|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~0_combout\ = (\inst33|temp\(0) & ((GND) # (!\inst15|t_0\(0)))) # (!\inst33|temp\(0) & (\inst15|t_0\(0) $ (GND)))
-- \inst15|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst15|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010111011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(0),
	datab => \inst15|t_0\(0),
	datad => VCC,
	combout => \inst15|Add0~0_combout\,
	cout => \inst15|Add0~1\);

-- Location: LCCOMB_X20_Y26_N2
\inst15|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~2_combout\ = (\inst15|t_0\(1) & ((\inst33|temp\(1) & (!\inst15|Add0~1\)) # (!\inst33|temp\(1) & ((\inst15|Add0~1\) # (GND))))) # (!\inst15|t_0\(1) & ((\inst33|temp\(1) & (\inst15|Add0~1\ & VCC)) # (!\inst33|temp\(1) & (!\inst15|Add0~1\))))
-- \inst15|Add0~3\ = CARRY((\inst15|t_0\(1) & ((!\inst15|Add0~1\) # (!\inst33|temp\(1)))) # (!\inst15|t_0\(1) & (!\inst33|temp\(1) & !\inst15|Add0~1\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(1),
	datab => \inst33|temp\(1),
	datad => VCC,
	cin => \inst15|Add0~1\,
	combout => \inst15|Add0~2_combout\,
	cout => \inst15|Add0~3\);

-- Location: LCCOMB_X20_Y26_N4
\inst15|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~4_combout\ = ((\inst33|temp\(2) $ (\inst15|t_0\(2) $ (\inst15|Add0~3\)))) # (GND)
-- \inst15|Add0~5\ = CARRY((\inst33|temp\(2) & ((!\inst15|Add0~3\) # (!\inst15|t_0\(2)))) # (!\inst33|temp\(2) & (!\inst15|t_0\(2) & !\inst15|Add0~3\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(2),
	datab => \inst15|t_0\(2),
	datad => VCC,
	cin => \inst15|Add0~3\,
	combout => \inst15|Add0~4_combout\,
	cout => \inst15|Add0~5\);

-- Location: LCCOMB_X20_Y26_N6
\inst15|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~6_combout\ = (\inst15|t_0\(3) & ((\inst33|temp\(3) & (!\inst15|Add0~5\)) # (!\inst33|temp\(3) & ((\inst15|Add0~5\) # (GND))))) # (!\inst15|t_0\(3) & ((\inst33|temp\(3) & (\inst15|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst15|Add0~5\))))
-- \inst15|Add0~7\ = CARRY((\inst15|t_0\(3) & ((!\inst15|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst15|t_0\(3) & (!\inst33|temp\(3) & !\inst15|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst15|Add0~5\,
	combout => \inst15|Add0~6_combout\,
	cout => \inst15|Add0~7\);

-- Location: LCCOMB_X20_Y26_N8
\inst15|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~8_combout\ = ((\inst33|temp\(4) $ (\inst15|t_0\(4) $ (\inst15|Add0~7\)))) # (GND)
-- \inst15|Add0~9\ = CARRY((\inst33|temp\(4) & ((!\inst15|Add0~7\) # (!\inst15|t_0\(4)))) # (!\inst33|temp\(4) & (!\inst15|t_0\(4) & !\inst15|Add0~7\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(4),
	datab => \inst15|t_0\(4),
	datad => VCC,
	cin => \inst15|Add0~7\,
	combout => \inst15|Add0~8_combout\,
	cout => \inst15|Add0~9\);

-- Location: LCCOMB_X20_Y26_N10
\inst15|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~10_combout\ = (\inst15|t_0\(5) & ((\inst33|temp\(5) & (!\inst15|Add0~9\)) # (!\inst33|temp\(5) & ((\inst15|Add0~9\) # (GND))))) # (!\inst15|t_0\(5) & ((\inst33|temp\(5) & (\inst15|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst15|Add0~9\))))
-- \inst15|Add0~11\ = CARRY((\inst15|t_0\(5) & ((!\inst15|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst15|t_0\(5) & (!\inst33|temp\(5) & !\inst15|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst15|Add0~9\,
	combout => \inst15|Add0~10_combout\,
	cout => \inst15|Add0~11\);

-- Location: LCCOMB_X20_Y26_N12
\inst15|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~12_combout\ = ((\inst15|t_0\(6) $ (\inst33|temp\(6) $ (\inst15|Add0~11\)))) # (GND)
-- \inst15|Add0~13\ = CARRY((\inst15|t_0\(6) & (\inst33|temp\(6) & !\inst15|Add0~11\)) # (!\inst15|t_0\(6) & ((\inst33|temp\(6)) # (!\inst15|Add0~11\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(6),
	datab => \inst33|temp\(6),
	datad => VCC,
	cin => \inst15|Add0~11\,
	combout => \inst15|Add0~12_combout\,
	cout => \inst15|Add0~13\);

-- Location: LCCOMB_X20_Y26_N14
\inst15|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~14_combout\ = (\inst33|temp\(7) & ((\inst15|t_0\(7) & (!\inst15|Add0~13\)) # (!\inst15|t_0\(7) & (\inst15|Add0~13\ & VCC)))) # (!\inst33|temp\(7) & ((\inst15|t_0\(7) & ((\inst15|Add0~13\) # (GND))) # (!\inst15|t_0\(7) & (!\inst15|Add0~13\))))
-- \inst15|Add0~15\ = CARRY((\inst33|temp\(7) & (\inst15|t_0\(7) & !\inst15|Add0~13\)) # (!\inst33|temp\(7) & ((\inst15|t_0\(7)) # (!\inst15|Add0~13\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(7),
	datab => \inst15|t_0\(7),
	datad => VCC,
	cin => \inst15|Add0~13\,
	combout => \inst15|Add0~14_combout\,
	cout => \inst15|Add0~15\);

-- Location: LCCOMB_X20_Y26_N16
\inst15|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~16_combout\ = ((\inst33|temp\(8) $ (\inst15|t_0\(8) $ (\inst15|Add0~15\)))) # (GND)
-- \inst15|Add0~17\ = CARRY((\inst33|temp\(8) & ((!\inst15|Add0~15\) # (!\inst15|t_0\(8)))) # (!\inst33|temp\(8) & (!\inst15|t_0\(8) & !\inst15|Add0~15\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(8),
	datab => \inst15|t_0\(8),
	datad => VCC,
	cin => \inst15|Add0~15\,
	combout => \inst15|Add0~16_combout\,
	cout => \inst15|Add0~17\);

-- Location: LCCOMB_X20_Y26_N18
\inst15|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~18_combout\ = (\inst33|temp\(9) & ((\inst15|t_0\(9) & (!\inst15|Add0~17\)) # (!\inst15|t_0\(9) & (\inst15|Add0~17\ & VCC)))) # (!\inst33|temp\(9) & ((\inst15|t_0\(9) & ((\inst15|Add0~17\) # (GND))) # (!\inst15|t_0\(9) & (!\inst15|Add0~17\))))
-- \inst15|Add0~19\ = CARRY((\inst33|temp\(9) & (\inst15|t_0\(9) & !\inst15|Add0~17\)) # (!\inst33|temp\(9) & ((\inst15|t_0\(9)) # (!\inst15|Add0~17\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(9),
	datab => \inst15|t_0\(9),
	datad => VCC,
	cin => \inst15|Add0~17\,
	combout => \inst15|Add0~18_combout\,
	cout => \inst15|Add0~19\);

-- Location: LCCOMB_X20_Y26_N20
\inst15|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~20_combout\ = ((\inst33|temp\(10) $ (\inst15|t_0\(10) $ (\inst15|Add0~19\)))) # (GND)
-- \inst15|Add0~21\ = CARRY((\inst33|temp\(10) & ((!\inst15|Add0~19\) # (!\inst15|t_0\(10)))) # (!\inst33|temp\(10) & (!\inst15|t_0\(10) & !\inst15|Add0~19\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(10),
	datab => \inst15|t_0\(10),
	datad => VCC,
	cin => \inst15|Add0~19\,
	combout => \inst15|Add0~20_combout\,
	cout => \inst15|Add0~21\);

-- Location: LCCOMB_X20_Y26_N22
\inst15|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~22_combout\ = (\inst15|t_0\(11) & ((\inst33|temp\(11) & (!\inst15|Add0~21\)) # (!\inst33|temp\(11) & ((\inst15|Add0~21\) # (GND))))) # (!\inst15|t_0\(11) & ((\inst33|temp\(11) & (\inst15|Add0~21\ & VCC)) # (!\inst33|temp\(11) & 
-- (!\inst15|Add0~21\))))
-- \inst15|Add0~23\ = CARRY((\inst15|t_0\(11) & ((!\inst15|Add0~21\) # (!\inst33|temp\(11)))) # (!\inst15|t_0\(11) & (!\inst33|temp\(11) & !\inst15|Add0~21\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(11),
	datab => \inst33|temp\(11),
	datad => VCC,
	cin => \inst15|Add0~21\,
	combout => \inst15|Add0~22_combout\,
	cout => \inst15|Add0~23\);

-- Location: LCCOMB_X20_Y26_N24
\inst15|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~24_combout\ = ((\inst15|t_0\(12) $ (\inst33|temp\(12) $ (\inst15|Add0~23\)))) # (GND)
-- \inst15|Add0~25\ = CARRY((\inst15|t_0\(12) & (\inst33|temp\(12) & !\inst15|Add0~23\)) # (!\inst15|t_0\(12) & ((\inst33|temp\(12)) # (!\inst15|Add0~23\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(12),
	datab => \inst33|temp\(12),
	datad => VCC,
	cin => \inst15|Add0~23\,
	combout => \inst15|Add0~24_combout\,
	cout => \inst15|Add0~25\);

-- Location: LCCOMB_X20_Y26_N26
\inst15|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~26_combout\ = (\inst33|temp\(13) & ((\inst15|t_0\(13) & (!\inst15|Add0~25\)) # (!\inst15|t_0\(13) & (\inst15|Add0~25\ & VCC)))) # (!\inst33|temp\(13) & ((\inst15|t_0\(13) & ((\inst15|Add0~25\) # (GND))) # (!\inst15|t_0\(13) & 
-- (!\inst15|Add0~25\))))
-- \inst15|Add0~27\ = CARRY((\inst33|temp\(13) & (\inst15|t_0\(13) & !\inst15|Add0~25\)) # (!\inst33|temp\(13) & ((\inst15|t_0\(13)) # (!\inst15|Add0~25\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(13),
	datab => \inst15|t_0\(13),
	datad => VCC,
	cin => \inst15|Add0~25\,
	combout => \inst15|Add0~26_combout\,
	cout => \inst15|Add0~27\);

-- Location: LCCOMB_X20_Y26_N28
\inst15|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst15|t_0\(14) $ (\inst15|Add0~27\)))) # (GND)
-- \inst15|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst15|Add0~27\) # (!\inst15|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst15|t_0\(14) & !\inst15|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst15|t_0\(14),
	datad => VCC,
	cin => \inst15|Add0~27\,
	combout => \inst15|Add0~28_combout\,
	cout => \inst15|Add0~29\);

-- Location: LCCOMB_X20_Y26_N30
\inst15|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~30_combout\ = (\inst15|t_0\(15) & ((\inst33|temp\(15) & (!\inst15|Add0~29\)) # (!\inst33|temp\(15) & ((\inst15|Add0~29\) # (GND))))) # (!\inst15|t_0\(15) & ((\inst33|temp\(15) & (\inst15|Add0~29\ & VCC)) # (!\inst33|temp\(15) & 
-- (!\inst15|Add0~29\))))
-- \inst15|Add0~31\ = CARRY((\inst15|t_0\(15) & ((!\inst15|Add0~29\) # (!\inst33|temp\(15)))) # (!\inst15|t_0\(15) & (!\inst33|temp\(15) & !\inst15|Add0~29\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(15),
	datab => \inst33|temp\(15),
	datad => VCC,
	cin => \inst15|Add0~29\,
	combout => \inst15|Add0~30_combout\,
	cout => \inst15|Add0~31\);

-- Location: LCCOMB_X20_Y25_N0
\inst15|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~32_combout\ = ((\inst33|temp\(16) $ (\inst15|t_0\(16) $ (\inst15|Add0~31\)))) # (GND)
-- \inst15|Add0~33\ = CARRY((\inst33|temp\(16) & ((!\inst15|Add0~31\) # (!\inst15|t_0\(16)))) # (!\inst33|temp\(16) & (!\inst15|t_0\(16) & !\inst15|Add0~31\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(16),
	datab => \inst15|t_0\(16),
	datad => VCC,
	cin => \inst15|Add0~31\,
	combout => \inst15|Add0~32_combout\,
	cout => \inst15|Add0~33\);

-- Location: LCCOMB_X20_Y25_N2
\inst15|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~34_combout\ = (\inst33|temp\(17) & ((\inst15|t_0\(17) & (!\inst15|Add0~33\)) # (!\inst15|t_0\(17) & (\inst15|Add0~33\ & VCC)))) # (!\inst33|temp\(17) & ((\inst15|t_0\(17) & ((\inst15|Add0~33\) # (GND))) # (!\inst15|t_0\(17) & 
-- (!\inst15|Add0~33\))))
-- \inst15|Add0~35\ = CARRY((\inst33|temp\(17) & (\inst15|t_0\(17) & !\inst15|Add0~33\)) # (!\inst33|temp\(17) & ((\inst15|t_0\(17)) # (!\inst15|Add0~33\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(17),
	datab => \inst15|t_0\(17),
	datad => VCC,
	cin => \inst15|Add0~33\,
	combout => \inst15|Add0~34_combout\,
	cout => \inst15|Add0~35\);

-- Location: LCCOMB_X20_Y25_N4
\inst15|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~36_combout\ = ((\inst15|t_0\(18) $ (\inst33|temp\(18) $ (\inst15|Add0~35\)))) # (GND)
-- \inst15|Add0~37\ = CARRY((\inst15|t_0\(18) & (\inst33|temp\(18) & !\inst15|Add0~35\)) # (!\inst15|t_0\(18) & ((\inst33|temp\(18)) # (!\inst15|Add0~35\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(18),
	datab => \inst33|temp\(18),
	datad => VCC,
	cin => \inst15|Add0~35\,
	combout => \inst15|Add0~36_combout\,
	cout => \inst15|Add0~37\);

-- Location: LCCOMB_X19_Y25_N10
\inst15|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~2_combout\ = (!\inst15|Add0~36_combout\ & (!\inst15|Add0~30_combout\ & (!\inst15|Add0~32_combout\ & !\inst15|Add0~34_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|Add0~36_combout\,
	datab => \inst15|Add0~30_combout\,
	datac => \inst15|Add0~32_combout\,
	datad => \inst15|Add0~34_combout\,
	combout => \inst15|data_available~2_combout\);

-- Location: LCCOMB_X15_Y29_N6
\inst33|temp[19]~67\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[19]~67_combout\ = (\inst33|temp\(19) & (\inst33|temp[18]~66\ $ (GND))) # (!\inst33|temp\(19) & (!\inst33|temp[18]~66\ & VCC))
-- \inst33|temp[19]~68\ = CARRY((\inst33|temp\(19) & !\inst33|temp[18]~66\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(19),
	datad => VCC,
	cin => \inst33|temp[18]~66\,
	combout => \inst33|temp[19]~67_combout\,
	cout => \inst33|temp[19]~68\);

-- Location: FF_X15_Y29_N7
\inst33|temp[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[19]~67_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(19));

-- Location: LCCOMB_X15_Y29_N8
\inst33|temp[20]~69\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[20]~69_combout\ = (\inst33|temp\(20) & (!\inst33|temp[19]~68\)) # (!\inst33|temp\(20) & ((\inst33|temp[19]~68\) # (GND)))
-- \inst33|temp[20]~70\ = CARRY((!\inst33|temp[19]~68\) # (!\inst33|temp\(20)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst33|temp[19]~68\,
	combout => \inst33|temp[20]~69_combout\,
	cout => \inst33|temp[20]~70\);

-- Location: FF_X15_Y29_N9
\inst33|temp[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[20]~69_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(20));

-- Location: LCCOMB_X15_Y29_N10
\inst33|temp[21]~71\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[21]~71_combout\ = (\inst33|temp\(21) & (\inst33|temp[20]~70\ $ (GND))) # (!\inst33|temp\(21) & (!\inst33|temp[20]~70\ & VCC))
-- \inst33|temp[21]~72\ = CARRY((\inst33|temp\(21) & !\inst33|temp[20]~70\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(21),
	datad => VCC,
	cin => \inst33|temp[20]~70\,
	combout => \inst33|temp[21]~71_combout\,
	cout => \inst33|temp[21]~72\);

-- Location: FF_X15_Y29_N11
\inst33|temp[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[21]~71_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(21));

-- Location: LCCOMB_X15_Y29_N12
\inst33|temp[22]~73\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[22]~73_combout\ = (\inst33|temp\(22) & (!\inst33|temp[21]~72\)) # (!\inst33|temp\(22) & ((\inst33|temp[21]~72\) # (GND)))
-- \inst33|temp[22]~74\ = CARRY((!\inst33|temp[21]~72\) # (!\inst33|temp\(22)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(22),
	datad => VCC,
	cin => \inst33|temp[21]~72\,
	combout => \inst33|temp[22]~73_combout\,
	cout => \inst33|temp[22]~74\);

-- Location: FF_X15_Y29_N13
\inst33|temp[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[22]~73_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(22));

-- Location: FF_X20_Y25_N13
\inst15|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(22));

-- Location: FF_X20_Y25_N11
\inst15|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(21));

-- Location: FF_X20_Y25_N9
\inst15|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(20));

-- Location: FF_X20_Y25_N7
\inst15|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(19));

-- Location: LCCOMB_X20_Y25_N6
\inst15|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~38_combout\ = (\inst15|t_0\(19) & ((\inst33|temp\(19) & (!\inst15|Add0~37\)) # (!\inst33|temp\(19) & ((\inst15|Add0~37\) # (GND))))) # (!\inst15|t_0\(19) & ((\inst33|temp\(19) & (\inst15|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst15|Add0~37\))))
-- \inst15|Add0~39\ = CARRY((\inst15|t_0\(19) & ((!\inst15|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst15|t_0\(19) & (!\inst33|temp\(19) & !\inst15|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst15|Add0~37\,
	combout => \inst15|Add0~38_combout\,
	cout => \inst15|Add0~39\);

-- Location: LCCOMB_X20_Y25_N8
\inst15|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~40_combout\ = ((\inst15|t_0\(20) $ (\inst33|temp\(20) $ (\inst15|Add0~39\)))) # (GND)
-- \inst15|Add0~41\ = CARRY((\inst15|t_0\(20) & (\inst33|temp\(20) & !\inst15|Add0~39\)) # (!\inst15|t_0\(20) & ((\inst33|temp\(20)) # (!\inst15|Add0~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(20),
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst15|Add0~39\,
	combout => \inst15|Add0~40_combout\,
	cout => \inst15|Add0~41\);

-- Location: LCCOMB_X20_Y25_N10
\inst15|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~42_combout\ = (\inst15|t_0\(21) & ((\inst33|temp\(21) & (!\inst15|Add0~41\)) # (!\inst33|temp\(21) & ((\inst15|Add0~41\) # (GND))))) # (!\inst15|t_0\(21) & ((\inst33|temp\(21) & (\inst15|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst15|Add0~41\))))
-- \inst15|Add0~43\ = CARRY((\inst15|t_0\(21) & ((!\inst15|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst15|t_0\(21) & (!\inst33|temp\(21) & !\inst15|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst15|Add0~41\,
	combout => \inst15|Add0~42_combout\,
	cout => \inst15|Add0~43\);

-- Location: LCCOMB_X20_Y25_N12
\inst15|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~44_combout\ = ((\inst15|t_0\(22) $ (\inst33|temp\(22) $ (\inst15|Add0~43\)))) # (GND)
-- \inst15|Add0~45\ = CARRY((\inst15|t_0\(22) & (\inst33|temp\(22) & !\inst15|Add0~43\)) # (!\inst15|t_0\(22) & ((\inst33|temp\(22)) # (!\inst15|Add0~43\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(22),
	datab => \inst33|temp\(22),
	datad => VCC,
	cin => \inst15|Add0~43\,
	combout => \inst15|Add0~44_combout\,
	cout => \inst15|Add0~45\);

-- Location: LCCOMB_X19_Y25_N4
\inst15|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~3_combout\ = (!\inst15|Add0~44_combout\ & (!\inst15|Add0~42_combout\ & (!\inst15|Add0~40_combout\ & !\inst15|Add0~38_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|Add0~44_combout\,
	datab => \inst15|Add0~42_combout\,
	datac => \inst15|Add0~40_combout\,
	datad => \inst15|Add0~38_combout\,
	combout => \inst15|data_available~3_combout\);

-- Location: LCCOMB_X19_Y26_N18
\inst15|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~1_combout\ = (!\inst15|Add0~28_combout\ & (!\inst15|Add0~22_combout\ & (!\inst15|Add0~26_combout\ & !\inst15|Add0~24_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|Add0~28_combout\,
	datab => \inst15|Add0~22_combout\,
	datac => \inst15|Add0~26_combout\,
	datad => \inst15|Add0~24_combout\,
	combout => \inst15|data_available~1_combout\);

-- Location: LCCOMB_X19_Y26_N8
\inst15|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~0_combout\ = (!\inst15|Add0~20_combout\ & (!\inst15|Add0~18_combout\ & (!\inst15|Add0~14_combout\ & !\inst15|Add0~16_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|Add0~20_combout\,
	datab => \inst15|Add0~18_combout\,
	datac => \inst15|Add0~14_combout\,
	datad => \inst15|Add0~16_combout\,
	combout => \inst15|data_available~0_combout\);

-- Location: LCCOMB_X19_Y25_N6
\inst15|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~4_combout\ = (\inst15|data_available~2_combout\ & (\inst15|data_available~3_combout\ & (\inst15|data_available~1_combout\ & \inst15|data_available~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|data_available~2_combout\,
	datab => \inst15|data_available~3_combout\,
	datac => \inst15|data_available~1_combout\,
	datad => \inst15|data_available~0_combout\,
	combout => \inst15|data_available~4_combout\);

-- Location: LCCOMB_X19_Y25_N22
\inst15|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~8_combout\ = (\inst15|Add0~4_combout\) # ((\inst15|Add0~2_combout\ & \inst15|Add0~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst15|Add0~4_combout\,
	datac => \inst15|Add0~2_combout\,
	datad => \inst15|Add0~0_combout\,
	combout => \inst15|data_available~8_combout\);

-- Location: LCCOMB_X19_Y25_N8
\inst15|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~9_combout\ = (\inst15|Add0~6_combout\ & ((\inst15|Add0~12_combout\) # (\inst15|Add0~8_combout\))) # (!\inst15|Add0~6_combout\ & (\inst15|Add0~12_combout\ & \inst15|Add0~8_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst15|Add0~6_combout\,
	datac => \inst15|Add0~12_combout\,
	datad => \inst15|Add0~8_combout\,
	combout => \inst15|data_available~9_combout\);

-- Location: LCCOMB_X19_Y25_N26
\inst15|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~10_combout\ = (\inst15|Add0~12_combout\ & (((!\inst15|data_available~8_combout\ & !\inst15|data_available~9_combout\)) # (!\inst15|Add0~10_combout\))) # (!\inst15|Add0~12_combout\ & (\inst15|data_available~8_combout\ & 
-- (\inst15|data_available~9_combout\ & \inst15|Add0~10_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001100011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|data_available~8_combout\,
	datab => \inst15|data_available~9_combout\,
	datac => \inst15|Add0~12_combout\,
	datad => \inst15|Add0~10_combout\,
	combout => \inst15|data_available~10_combout\);

-- Location: LCCOMB_X15_Y29_N14
\inst33|temp[23]~75\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[23]~75_combout\ = (\inst33|temp\(23) & (\inst33|temp[22]~74\ $ (GND))) # (!\inst33|temp\(23) & (!\inst33|temp[22]~74\ & VCC))
-- \inst33|temp[23]~76\ = CARRY((\inst33|temp\(23) & !\inst33|temp[22]~74\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(23),
	datad => VCC,
	cin => \inst33|temp[22]~74\,
	combout => \inst33|temp[23]~75_combout\,
	cout => \inst33|temp[23]~76\);

-- Location: FF_X15_Y29_N15
\inst33|temp[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[23]~75_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(23));

-- Location: LCCOMB_X15_Y29_N16
\inst33|temp[24]~77\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[24]~77_combout\ = (\inst33|temp\(24) & (!\inst33|temp[23]~76\)) # (!\inst33|temp\(24) & ((\inst33|temp[23]~76\) # (GND)))
-- \inst33|temp[24]~78\ = CARRY((!\inst33|temp[23]~76\) # (!\inst33|temp\(24)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(24),
	datad => VCC,
	cin => \inst33|temp[23]~76\,
	combout => \inst33|temp[24]~77_combout\,
	cout => \inst33|temp[24]~78\);

-- Location: FF_X15_Y29_N17
\inst33|temp[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[24]~77_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(24));

-- Location: FF_X20_Y25_N17
\inst15|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(24));

-- Location: FF_X20_Y25_N15
\inst15|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(23));

-- Location: LCCOMB_X20_Y25_N14
\inst15|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~46_combout\ = (\inst33|temp\(23) & ((\inst15|t_0\(23) & (!\inst15|Add0~45\)) # (!\inst15|t_0\(23) & (\inst15|Add0~45\ & VCC)))) # (!\inst33|temp\(23) & ((\inst15|t_0\(23) & ((\inst15|Add0~45\) # (GND))) # (!\inst15|t_0\(23) & 
-- (!\inst15|Add0~45\))))
-- \inst15|Add0~47\ = CARRY((\inst33|temp\(23) & (\inst15|t_0\(23) & !\inst15|Add0~45\)) # (!\inst33|temp\(23) & ((\inst15|t_0\(23)) # (!\inst15|Add0~45\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(23),
	datab => \inst15|t_0\(23),
	datad => VCC,
	cin => \inst15|Add0~45\,
	combout => \inst15|Add0~46_combout\,
	cout => \inst15|Add0~47\);

-- Location: LCCOMB_X20_Y25_N16
\inst15|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~48_combout\ = ((\inst33|temp\(24) $ (\inst15|t_0\(24) $ (\inst15|Add0~47\)))) # (GND)
-- \inst15|Add0~49\ = CARRY((\inst33|temp\(24) & ((!\inst15|Add0~47\) # (!\inst15|t_0\(24)))) # (!\inst33|temp\(24) & (!\inst15|t_0\(24) & !\inst15|Add0~47\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(24),
	datab => \inst15|t_0\(24),
	datad => VCC,
	cin => \inst15|Add0~47\,
	combout => \inst15|Add0~48_combout\,
	cout => \inst15|Add0~49\);

-- Location: LCCOMB_X15_Y29_N18
\inst33|temp[25]~79\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[25]~79_combout\ = (\inst33|temp\(25) & (\inst33|temp[24]~78\ $ (GND))) # (!\inst33|temp\(25) & (!\inst33|temp[24]~78\ & VCC))
-- \inst33|temp[25]~80\ = CARRY((\inst33|temp\(25) & !\inst33|temp[24]~78\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(25),
	datad => VCC,
	cin => \inst33|temp[24]~78\,
	combout => \inst33|temp[25]~79_combout\,
	cout => \inst33|temp[25]~80\);

-- Location: FF_X15_Y29_N19
\inst33|temp[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[25]~79_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(25));

-- Location: LCCOMB_X15_Y29_N20
\inst33|temp[26]~81\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[26]~81_combout\ = (\inst33|temp\(26) & (!\inst33|temp[25]~80\)) # (!\inst33|temp\(26) & ((\inst33|temp[25]~80\) # (GND)))
-- \inst33|temp[26]~82\ = CARRY((!\inst33|temp[25]~80\) # (!\inst33|temp\(26)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(26),
	datad => VCC,
	cin => \inst33|temp[25]~80\,
	combout => \inst33|temp[26]~81_combout\,
	cout => \inst33|temp[26]~82\);

-- Location: FF_X15_Y29_N21
\inst33|temp[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[26]~81_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(26));

-- Location: FF_X20_Y25_N21
\inst15|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(26));

-- Location: FF_X20_Y25_N19
\inst15|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(25));

-- Location: LCCOMB_X20_Y25_N18
\inst15|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst15|t_0\(25) & (!\inst15|Add0~49\)) # (!\inst15|t_0\(25) & (\inst15|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst15|t_0\(25) & ((\inst15|Add0~49\) # (GND))) # (!\inst15|t_0\(25) & 
-- (!\inst15|Add0~49\))))
-- \inst15|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst15|t_0\(25) & !\inst15|Add0~49\)) # (!\inst33|temp\(25) & ((\inst15|t_0\(25)) # (!\inst15|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst15|t_0\(25),
	datad => VCC,
	cin => \inst15|Add0~49\,
	combout => \inst15|Add0~50_combout\,
	cout => \inst15|Add0~51\);

-- Location: LCCOMB_X20_Y25_N20
\inst15|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~52_combout\ = ((\inst33|temp\(26) $ (\inst15|t_0\(26) $ (\inst15|Add0~51\)))) # (GND)
-- \inst15|Add0~53\ = CARRY((\inst33|temp\(26) & ((!\inst15|Add0~51\) # (!\inst15|t_0\(26)))) # (!\inst33|temp\(26) & (!\inst15|t_0\(26) & !\inst15|Add0~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(26),
	datab => \inst15|t_0\(26),
	datad => VCC,
	cin => \inst15|Add0~51\,
	combout => \inst15|Add0~52_combout\,
	cout => \inst15|Add0~53\);

-- Location: LCCOMB_X19_Y25_N0
\inst15|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~5_combout\ = (!\inst15|Add0~48_combout\ & (!\inst15|Add0~46_combout\ & (!\inst15|Add0~52_combout\ & !\inst15|Add0~50_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|Add0~48_combout\,
	datab => \inst15|Add0~46_combout\,
	datac => \inst15|Add0~52_combout\,
	datad => \inst15|Add0~50_combout\,
	combout => \inst15|data_available~5_combout\);

-- Location: LCCOMB_X15_Y29_N22
\inst33|temp[27]~83\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[27]~83_combout\ = (\inst33|temp\(27) & (\inst33|temp[26]~82\ $ (GND))) # (!\inst33|temp\(27) & (!\inst33|temp[26]~82\ & VCC))
-- \inst33|temp[27]~84\ = CARRY((\inst33|temp\(27) & !\inst33|temp[26]~82\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(27),
	datad => VCC,
	cin => \inst33|temp[26]~82\,
	combout => \inst33|temp[27]~83_combout\,
	cout => \inst33|temp[27]~84\);

-- Location: FF_X15_Y29_N23
\inst33|temp[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[27]~83_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(27));

-- Location: FF_X20_Y25_N23
\inst15|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(27));

-- Location: LCCOMB_X20_Y25_N22
\inst15|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~54_combout\ = (\inst15|t_0\(27) & ((\inst33|temp\(27) & (!\inst15|Add0~53\)) # (!\inst33|temp\(27) & ((\inst15|Add0~53\) # (GND))))) # (!\inst15|t_0\(27) & ((\inst33|temp\(27) & (\inst15|Add0~53\ & VCC)) # (!\inst33|temp\(27) & 
-- (!\inst15|Add0~53\))))
-- \inst15|Add0~55\ = CARRY((\inst15|t_0\(27) & ((!\inst15|Add0~53\) # (!\inst33|temp\(27)))) # (!\inst15|t_0\(27) & (!\inst33|temp\(27) & !\inst15|Add0~53\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(27),
	datab => \inst33|temp\(27),
	datad => VCC,
	cin => \inst15|Add0~53\,
	combout => \inst15|Add0~54_combout\,
	cout => \inst15|Add0~55\);

-- Location: LCCOMB_X15_Y29_N24
\inst33|temp[28]~85\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[28]~85_combout\ = (\inst33|temp\(28) & (!\inst33|temp[27]~84\)) # (!\inst33|temp\(28) & ((\inst33|temp[27]~84\) # (GND)))
-- \inst33|temp[28]~86\ = CARRY((!\inst33|temp[27]~84\) # (!\inst33|temp\(28)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(28),
	datad => VCC,
	cin => \inst33|temp[27]~84\,
	combout => \inst33|temp[28]~85_combout\,
	cout => \inst33|temp[28]~86\);

-- Location: FF_X15_Y29_N25
\inst33|temp[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[28]~85_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(28));

-- Location: LCCOMB_X15_Y29_N26
\inst33|temp[29]~87\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[29]~87_combout\ = (\inst33|temp\(29) & (\inst33|temp[28]~86\ $ (GND))) # (!\inst33|temp\(29) & (!\inst33|temp[28]~86\ & VCC))
-- \inst33|temp[29]~88\ = CARRY((\inst33|temp\(29) & !\inst33|temp[28]~86\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(29),
	datad => VCC,
	cin => \inst33|temp[28]~86\,
	combout => \inst33|temp[29]~87_combout\,
	cout => \inst33|temp[29]~88\);

-- Location: FF_X15_Y29_N27
\inst33|temp[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[29]~87_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(29));

-- Location: LCCOMB_X15_Y29_N28
\inst33|temp[30]~89\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[30]~89_combout\ = (\inst33|temp\(30) & (!\inst33|temp[29]~88\)) # (!\inst33|temp\(30) & ((\inst33|temp[29]~88\) # (GND)))
-- \inst33|temp[30]~90\ = CARRY((!\inst33|temp[29]~88\) # (!\inst33|temp\(30)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(30),
	datad => VCC,
	cin => \inst33|temp[29]~88\,
	combout => \inst33|temp[30]~89_combout\,
	cout => \inst33|temp[30]~90\);

-- Location: FF_X15_Y29_N29
\inst33|temp[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[30]~89_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(30));

-- Location: FF_X20_Y25_N29
\inst15|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(30));

-- Location: FF_X20_Y25_N27
\inst15|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(29));

-- Location: FF_X20_Y25_N25
\inst15|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(28));

-- Location: LCCOMB_X20_Y25_N24
\inst15|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~56_combout\ = ((\inst33|temp\(28) $ (\inst15|t_0\(28) $ (\inst15|Add0~55\)))) # (GND)
-- \inst15|Add0~57\ = CARRY((\inst33|temp\(28) & ((!\inst15|Add0~55\) # (!\inst15|t_0\(28)))) # (!\inst33|temp\(28) & (!\inst15|t_0\(28) & !\inst15|Add0~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(28),
	datab => \inst15|t_0\(28),
	datad => VCC,
	cin => \inst15|Add0~55\,
	combout => \inst15|Add0~56_combout\,
	cout => \inst15|Add0~57\);

-- Location: LCCOMB_X20_Y25_N26
\inst15|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~58_combout\ = (\inst15|t_0\(29) & ((\inst33|temp\(29) & (!\inst15|Add0~57\)) # (!\inst33|temp\(29) & ((\inst15|Add0~57\) # (GND))))) # (!\inst15|t_0\(29) & ((\inst33|temp\(29) & (\inst15|Add0~57\ & VCC)) # (!\inst33|temp\(29) & 
-- (!\inst15|Add0~57\))))
-- \inst15|Add0~59\ = CARRY((\inst15|t_0\(29) & ((!\inst15|Add0~57\) # (!\inst33|temp\(29)))) # (!\inst15|t_0\(29) & (!\inst33|temp\(29) & !\inst15|Add0~57\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|t_0\(29),
	datab => \inst33|temp\(29),
	datad => VCC,
	cin => \inst15|Add0~57\,
	combout => \inst15|Add0~58_combout\,
	cout => \inst15|Add0~59\);

-- Location: LCCOMB_X20_Y25_N28
\inst15|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst15|t_0\(30) $ (\inst15|Add0~59\)))) # (GND)
-- \inst15|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst15|Add0~59\) # (!\inst15|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst15|t_0\(30) & !\inst15|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst15|t_0\(30),
	datad => VCC,
	cin => \inst15|Add0~59\,
	combout => \inst15|Add0~60_combout\,
	cout => \inst15|Add0~61\);

-- Location: LCCOMB_X19_Y25_N18
\inst15|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~6_combout\ = (!\inst15|Add0~54_combout\ & (!\inst15|Add0~60_combout\ & (!\inst15|Add0~58_combout\ & !\inst15|Add0~56_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|Add0~54_combout\,
	datab => \inst15|Add0~60_combout\,
	datac => \inst15|Add0~58_combout\,
	datad => \inst15|Add0~56_combout\,
	combout => \inst15|data_available~6_combout\);

-- Location: LCCOMB_X15_Y29_N30
\inst33|temp[31]~91\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst33|temp[31]~91_combout\ = \inst33|temp\(31) $ (!\inst33|temp[30]~90\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(31),
	cin => \inst33|temp[30]~90\,
	combout => \inst33|temp[31]~91_combout\);

-- Location: FF_X15_Y29_N31
\inst33|temp[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst33|temp[31]~91_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst33|temp\(31));

-- Location: FF_X20_Y25_N31
\inst15|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor7_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|t_0\(31));

-- Location: LCCOMB_X20_Y25_N30
\inst15|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|Add0~62_combout\ = \inst33|temp\(31) $ (\inst15|Add0~61\ $ (!\inst15|t_0\(31)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101010100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(31),
	datad => \inst15|t_0\(31),
	cin => \inst15|Add0~61\,
	combout => \inst15|Add0~62_combout\);

-- Location: LCCOMB_X19_Y25_N20
\inst15|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~7_combout\ = (\inst15|data_available~5_combout\ & (\inst15|data_available~6_combout\ & !\inst15|Add0~62_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst15|data_available~5_combout\,
	datac => \inst15|data_available~6_combout\,
	datad => \inst15|Add0~62_combout\,
	combout => \inst15|data_available~7_combout\);

-- Location: LCCOMB_X19_Y25_N24
\inst15|data_available~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst15|data_available~11_combout\ = (\inst15|data_available~4_combout\ & (\inst15|data_available~10_combout\ & \inst15|data_available~7_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst15|data_available~4_combout\,
	datac => \inst15|data_available~10_combout\,
	datad => \inst15|data_available~7_combout\,
	combout => \inst15|data_available~11_combout\);

-- Location: FF_X19_Y25_N25
\inst15|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor7_signal~inputclkctrl_outclk\,
	d => \inst15|data_available~11_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst15|data_available~q\);

-- Location: LCCOMB_X23_Y27_N12
\inst24|cur_value~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst24|cur_value~feeder_combout\ = \inst15|data_available~q\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst15|data_available~q\,
	combout => \inst24|cur_value~feeder_combout\);

-- Location: FF_X23_Y27_N13
\inst24|cur_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst24|cur_value~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst24|cur_value~q\);

-- Location: FF_X23_Y27_N23
\inst24|last_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst24|cur_value~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst24|last_value~q\);

-- Location: LCCOMB_X23_Y27_N22
\inst24|level_sig\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst24|level_sig~combout\ = (!\inst24|last_value~q\ & \inst24|cur_value~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst24|last_value~q\,
	datad => \inst24|cur_value~q\,
	combout => \inst24|level_sig~combout\);

-- Location: IOIBUF_X20_Y34_N15
\sensor6_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor6_signal,
	o => \sensor6_signal~input_o\);

-- Location: FF_X20_Y28_N11
\inst13|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(5));

-- Location: FF_X20_Y28_N9
\inst13|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(4));

-- Location: FF_X20_Y28_N7
\inst13|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(3));

-- Location: FF_X20_Y28_N5
\inst13|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(2));

-- Location: FF_X20_Y28_N3
\inst13|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(1));

-- Location: FF_X20_Y28_N1
\inst13|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(0));

-- Location: LCCOMB_X20_Y28_N0
\inst13|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~0_combout\ = (\inst33|temp\(0) & ((GND) # (!\inst13|t_0\(0)))) # (!\inst33|temp\(0) & (\inst13|t_0\(0) $ (GND)))
-- \inst13|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst13|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010111011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(0),
	datab => \inst13|t_0\(0),
	datad => VCC,
	combout => \inst13|Add0~0_combout\,
	cout => \inst13|Add0~1\);

-- Location: LCCOMB_X20_Y28_N2
\inst13|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~2_combout\ = (\inst33|temp\(1) & ((\inst13|t_0\(1) & (!\inst13|Add0~1\)) # (!\inst13|t_0\(1) & (\inst13|Add0~1\ & VCC)))) # (!\inst33|temp\(1) & ((\inst13|t_0\(1) & ((\inst13|Add0~1\) # (GND))) # (!\inst13|t_0\(1) & (!\inst13|Add0~1\))))
-- \inst13|Add0~3\ = CARRY((\inst33|temp\(1) & (\inst13|t_0\(1) & !\inst13|Add0~1\)) # (!\inst33|temp\(1) & ((\inst13|t_0\(1)) # (!\inst13|Add0~1\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(1),
	datab => \inst13|t_0\(1),
	datad => VCC,
	cin => \inst13|Add0~1\,
	combout => \inst13|Add0~2_combout\,
	cout => \inst13|Add0~3\);

-- Location: LCCOMB_X20_Y28_N4
\inst13|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~4_combout\ = ((\inst33|temp\(2) $ (\inst13|t_0\(2) $ (\inst13|Add0~3\)))) # (GND)
-- \inst13|Add0~5\ = CARRY((\inst33|temp\(2) & ((!\inst13|Add0~3\) # (!\inst13|t_0\(2)))) # (!\inst33|temp\(2) & (!\inst13|t_0\(2) & !\inst13|Add0~3\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(2),
	datab => \inst13|t_0\(2),
	datad => VCC,
	cin => \inst13|Add0~3\,
	combout => \inst13|Add0~4_combout\,
	cout => \inst13|Add0~5\);

-- Location: LCCOMB_X20_Y28_N6
\inst13|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~6_combout\ = (\inst13|t_0\(3) & ((\inst33|temp\(3) & (!\inst13|Add0~5\)) # (!\inst33|temp\(3) & ((\inst13|Add0~5\) # (GND))))) # (!\inst13|t_0\(3) & ((\inst33|temp\(3) & (\inst13|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst13|Add0~5\))))
-- \inst13|Add0~7\ = CARRY((\inst13|t_0\(3) & ((!\inst13|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst13|t_0\(3) & (!\inst33|temp\(3) & !\inst13|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst13|Add0~5\,
	combout => \inst13|Add0~6_combout\,
	cout => \inst13|Add0~7\);

-- Location: LCCOMB_X20_Y28_N8
\inst13|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~8_combout\ = ((\inst33|temp\(4) $ (\inst13|t_0\(4) $ (\inst13|Add0~7\)))) # (GND)
-- \inst13|Add0~9\ = CARRY((\inst33|temp\(4) & ((!\inst13|Add0~7\) # (!\inst13|t_0\(4)))) # (!\inst33|temp\(4) & (!\inst13|t_0\(4) & !\inst13|Add0~7\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(4),
	datab => \inst13|t_0\(4),
	datad => VCC,
	cin => \inst13|Add0~7\,
	combout => \inst13|Add0~8_combout\,
	cout => \inst13|Add0~9\);

-- Location: LCCOMB_X20_Y28_N10
\inst13|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~10_combout\ = (\inst13|t_0\(5) & ((\inst33|temp\(5) & (!\inst13|Add0~9\)) # (!\inst33|temp\(5) & ((\inst13|Add0~9\) # (GND))))) # (!\inst13|t_0\(5) & ((\inst33|temp\(5) & (\inst13|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst13|Add0~9\))))
-- \inst13|Add0~11\ = CARRY((\inst13|t_0\(5) & ((!\inst13|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst13|t_0\(5) & (!\inst33|temp\(5) & !\inst13|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst13|Add0~9\,
	combout => \inst13|Add0~10_combout\,
	cout => \inst13|Add0~11\);

-- Location: FF_X20_Y28_N13
\inst13|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(6));

-- Location: LCCOMB_X20_Y28_N12
\inst13|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~12_combout\ = ((\inst13|t_0\(6) $ (\inst33|temp\(6) $ (\inst13|Add0~11\)))) # (GND)
-- \inst13|Add0~13\ = CARRY((\inst13|t_0\(6) & (\inst33|temp\(6) & !\inst13|Add0~11\)) # (!\inst13|t_0\(6) & ((\inst33|temp\(6)) # (!\inst13|Add0~11\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(6),
	datab => \inst33|temp\(6),
	datad => VCC,
	cin => \inst13|Add0~11\,
	combout => \inst13|Add0~12_combout\,
	cout => \inst13|Add0~13\);

-- Location: LCCOMB_X21_Y28_N20
\inst13|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~8_combout\ = (\inst13|Add0~4_combout\) # ((\inst13|Add0~0_combout\ & \inst13|Add0~2_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst13|Add0~4_combout\,
	datac => \inst13|Add0~0_combout\,
	datad => \inst13|Add0~2_combout\,
	combout => \inst13|data_available~8_combout\);

-- Location: LCCOMB_X21_Y28_N22
\inst13|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~9_combout\ = (\inst13|Add0~12_combout\ & ((\inst13|Add0~8_combout\) # ((\inst13|Add0~6_combout\) # (\inst13|data_available~8_combout\)))) # (!\inst13|Add0~12_combout\ & (\inst13|Add0~8_combout\ & (\inst13|Add0~6_combout\ & 
-- \inst13|data_available~8_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110101010101000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~12_combout\,
	datab => \inst13|Add0~8_combout\,
	datac => \inst13|Add0~6_combout\,
	datad => \inst13|data_available~8_combout\,
	combout => \inst13|data_available~9_combout\);

-- Location: FF_X20_Y27_N15
\inst13|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(23));

-- Location: FF_X20_Y27_N13
\inst13|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(22));

-- Location: FF_X20_Y27_N11
\inst13|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(21));

-- Location: FF_X20_Y27_N9
\inst13|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(20));

-- Location: FF_X20_Y27_N7
\inst13|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(19));

-- Location: FF_X20_Y27_N5
\inst13|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(18));

-- Location: FF_X20_Y27_N3
\inst13|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(17));

-- Location: FF_X20_Y27_N1
\inst13|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(16));

-- Location: FF_X20_Y28_N31
\inst13|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(15));

-- Location: FF_X20_Y28_N29
\inst13|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(14));

-- Location: FF_X20_Y28_N27
\inst13|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(13));

-- Location: FF_X20_Y28_N25
\inst13|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(12));

-- Location: FF_X20_Y28_N23
\inst13|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(11));

-- Location: FF_X20_Y28_N21
\inst13|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(10));

-- Location: FF_X20_Y28_N19
\inst13|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(9));

-- Location: FF_X20_Y28_N17
\inst13|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(8));

-- Location: FF_X20_Y28_N15
\inst13|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(7));

-- Location: LCCOMB_X20_Y28_N14
\inst13|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~14_combout\ = (\inst33|temp\(7) & ((\inst13|t_0\(7) & (!\inst13|Add0~13\)) # (!\inst13|t_0\(7) & (\inst13|Add0~13\ & VCC)))) # (!\inst33|temp\(7) & ((\inst13|t_0\(7) & ((\inst13|Add0~13\) # (GND))) # (!\inst13|t_0\(7) & (!\inst13|Add0~13\))))
-- \inst13|Add0~15\ = CARRY((\inst33|temp\(7) & (\inst13|t_0\(7) & !\inst13|Add0~13\)) # (!\inst33|temp\(7) & ((\inst13|t_0\(7)) # (!\inst13|Add0~13\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(7),
	datab => \inst13|t_0\(7),
	datad => VCC,
	cin => \inst13|Add0~13\,
	combout => \inst13|Add0~14_combout\,
	cout => \inst13|Add0~15\);

-- Location: LCCOMB_X20_Y28_N16
\inst13|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~16_combout\ = ((\inst33|temp\(8) $ (\inst13|t_0\(8) $ (\inst13|Add0~15\)))) # (GND)
-- \inst13|Add0~17\ = CARRY((\inst33|temp\(8) & ((!\inst13|Add0~15\) # (!\inst13|t_0\(8)))) # (!\inst33|temp\(8) & (!\inst13|t_0\(8) & !\inst13|Add0~15\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(8),
	datab => \inst13|t_0\(8),
	datad => VCC,
	cin => \inst13|Add0~15\,
	combout => \inst13|Add0~16_combout\,
	cout => \inst13|Add0~17\);

-- Location: LCCOMB_X20_Y28_N18
\inst13|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~18_combout\ = (\inst13|t_0\(9) & ((\inst33|temp\(9) & (!\inst13|Add0~17\)) # (!\inst33|temp\(9) & ((\inst13|Add0~17\) # (GND))))) # (!\inst13|t_0\(9) & ((\inst33|temp\(9) & (\inst13|Add0~17\ & VCC)) # (!\inst33|temp\(9) & 
-- (!\inst13|Add0~17\))))
-- \inst13|Add0~19\ = CARRY((\inst13|t_0\(9) & ((!\inst13|Add0~17\) # (!\inst33|temp\(9)))) # (!\inst13|t_0\(9) & (!\inst33|temp\(9) & !\inst13|Add0~17\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(9),
	datab => \inst33|temp\(9),
	datad => VCC,
	cin => \inst13|Add0~17\,
	combout => \inst13|Add0~18_combout\,
	cout => \inst13|Add0~19\);

-- Location: LCCOMB_X20_Y28_N20
\inst13|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~20_combout\ = ((\inst13|t_0\(10) $ (\inst33|temp\(10) $ (\inst13|Add0~19\)))) # (GND)
-- \inst13|Add0~21\ = CARRY((\inst13|t_0\(10) & (\inst33|temp\(10) & !\inst13|Add0~19\)) # (!\inst13|t_0\(10) & ((\inst33|temp\(10)) # (!\inst13|Add0~19\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(10),
	datab => \inst33|temp\(10),
	datad => VCC,
	cin => \inst13|Add0~19\,
	combout => \inst13|Add0~20_combout\,
	cout => \inst13|Add0~21\);

-- Location: LCCOMB_X20_Y28_N22
\inst13|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~22_combout\ = (\inst13|t_0\(11) & ((\inst33|temp\(11) & (!\inst13|Add0~21\)) # (!\inst33|temp\(11) & ((\inst13|Add0~21\) # (GND))))) # (!\inst13|t_0\(11) & ((\inst33|temp\(11) & (\inst13|Add0~21\ & VCC)) # (!\inst33|temp\(11) & 
-- (!\inst13|Add0~21\))))
-- \inst13|Add0~23\ = CARRY((\inst13|t_0\(11) & ((!\inst13|Add0~21\) # (!\inst33|temp\(11)))) # (!\inst13|t_0\(11) & (!\inst33|temp\(11) & !\inst13|Add0~21\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(11),
	datab => \inst33|temp\(11),
	datad => VCC,
	cin => \inst13|Add0~21\,
	combout => \inst13|Add0~22_combout\,
	cout => \inst13|Add0~23\);

-- Location: LCCOMB_X20_Y28_N24
\inst13|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~24_combout\ = ((\inst33|temp\(12) $ (\inst13|t_0\(12) $ (\inst13|Add0~23\)))) # (GND)
-- \inst13|Add0~25\ = CARRY((\inst33|temp\(12) & ((!\inst13|Add0~23\) # (!\inst13|t_0\(12)))) # (!\inst33|temp\(12) & (!\inst13|t_0\(12) & !\inst13|Add0~23\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(12),
	datab => \inst13|t_0\(12),
	datad => VCC,
	cin => \inst13|Add0~23\,
	combout => \inst13|Add0~24_combout\,
	cout => \inst13|Add0~25\);

-- Location: LCCOMB_X20_Y28_N26
\inst13|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~26_combout\ = (\inst33|temp\(13) & ((\inst13|t_0\(13) & (!\inst13|Add0~25\)) # (!\inst13|t_0\(13) & (\inst13|Add0~25\ & VCC)))) # (!\inst33|temp\(13) & ((\inst13|t_0\(13) & ((\inst13|Add0~25\) # (GND))) # (!\inst13|t_0\(13) & 
-- (!\inst13|Add0~25\))))
-- \inst13|Add0~27\ = CARRY((\inst33|temp\(13) & (\inst13|t_0\(13) & !\inst13|Add0~25\)) # (!\inst33|temp\(13) & ((\inst13|t_0\(13)) # (!\inst13|Add0~25\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(13),
	datab => \inst13|t_0\(13),
	datad => VCC,
	cin => \inst13|Add0~25\,
	combout => \inst13|Add0~26_combout\,
	cout => \inst13|Add0~27\);

-- Location: LCCOMB_X20_Y28_N28
\inst13|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst13|t_0\(14) $ (\inst13|Add0~27\)))) # (GND)
-- \inst13|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst13|Add0~27\) # (!\inst13|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst13|t_0\(14) & !\inst13|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst13|t_0\(14),
	datad => VCC,
	cin => \inst13|Add0~27\,
	combout => \inst13|Add0~28_combout\,
	cout => \inst13|Add0~29\);

-- Location: LCCOMB_X20_Y28_N30
\inst13|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~30_combout\ = (\inst13|t_0\(15) & ((\inst33|temp\(15) & (!\inst13|Add0~29\)) # (!\inst33|temp\(15) & ((\inst13|Add0~29\) # (GND))))) # (!\inst13|t_0\(15) & ((\inst33|temp\(15) & (\inst13|Add0~29\ & VCC)) # (!\inst33|temp\(15) & 
-- (!\inst13|Add0~29\))))
-- \inst13|Add0~31\ = CARRY((\inst13|t_0\(15) & ((!\inst13|Add0~29\) # (!\inst33|temp\(15)))) # (!\inst13|t_0\(15) & (!\inst33|temp\(15) & !\inst13|Add0~29\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(15),
	datab => \inst33|temp\(15),
	datad => VCC,
	cin => \inst13|Add0~29\,
	combout => \inst13|Add0~30_combout\,
	cout => \inst13|Add0~31\);

-- Location: LCCOMB_X20_Y27_N0
\inst13|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~32_combout\ = ((\inst13|t_0\(16) $ (\inst33|temp\(16) $ (\inst13|Add0~31\)))) # (GND)
-- \inst13|Add0~33\ = CARRY((\inst13|t_0\(16) & (\inst33|temp\(16) & !\inst13|Add0~31\)) # (!\inst13|t_0\(16) & ((\inst33|temp\(16)) # (!\inst13|Add0~31\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(16),
	datab => \inst33|temp\(16),
	datad => VCC,
	cin => \inst13|Add0~31\,
	combout => \inst13|Add0~32_combout\,
	cout => \inst13|Add0~33\);

-- Location: LCCOMB_X20_Y27_N2
\inst13|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~34_combout\ = (\inst33|temp\(17) & ((\inst13|t_0\(17) & (!\inst13|Add0~33\)) # (!\inst13|t_0\(17) & (\inst13|Add0~33\ & VCC)))) # (!\inst33|temp\(17) & ((\inst13|t_0\(17) & ((\inst13|Add0~33\) # (GND))) # (!\inst13|t_0\(17) & 
-- (!\inst13|Add0~33\))))
-- \inst13|Add0~35\ = CARRY((\inst33|temp\(17) & (\inst13|t_0\(17) & !\inst13|Add0~33\)) # (!\inst33|temp\(17) & ((\inst13|t_0\(17)) # (!\inst13|Add0~33\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(17),
	datab => \inst13|t_0\(17),
	datad => VCC,
	cin => \inst13|Add0~33\,
	combout => \inst13|Add0~34_combout\,
	cout => \inst13|Add0~35\);

-- Location: LCCOMB_X20_Y27_N4
\inst13|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~36_combout\ = ((\inst33|temp\(18) $ (\inst13|t_0\(18) $ (\inst13|Add0~35\)))) # (GND)
-- \inst13|Add0~37\ = CARRY((\inst33|temp\(18) & ((!\inst13|Add0~35\) # (!\inst13|t_0\(18)))) # (!\inst33|temp\(18) & (!\inst13|t_0\(18) & !\inst13|Add0~35\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(18),
	datab => \inst13|t_0\(18),
	datad => VCC,
	cin => \inst13|Add0~35\,
	combout => \inst13|Add0~36_combout\,
	cout => \inst13|Add0~37\);

-- Location: LCCOMB_X20_Y27_N6
\inst13|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~38_combout\ = (\inst13|t_0\(19) & ((\inst33|temp\(19) & (!\inst13|Add0~37\)) # (!\inst33|temp\(19) & ((\inst13|Add0~37\) # (GND))))) # (!\inst13|t_0\(19) & ((\inst33|temp\(19) & (\inst13|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst13|Add0~37\))))
-- \inst13|Add0~39\ = CARRY((\inst13|t_0\(19) & ((!\inst13|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst13|t_0\(19) & (!\inst33|temp\(19) & !\inst13|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst13|Add0~37\,
	combout => \inst13|Add0~38_combout\,
	cout => \inst13|Add0~39\);

-- Location: LCCOMB_X20_Y27_N8
\inst13|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~40_combout\ = ((\inst13|t_0\(20) $ (\inst33|temp\(20) $ (\inst13|Add0~39\)))) # (GND)
-- \inst13|Add0~41\ = CARRY((\inst13|t_0\(20) & (\inst33|temp\(20) & !\inst13|Add0~39\)) # (!\inst13|t_0\(20) & ((\inst33|temp\(20)) # (!\inst13|Add0~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(20),
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst13|Add0~39\,
	combout => \inst13|Add0~40_combout\,
	cout => \inst13|Add0~41\);

-- Location: LCCOMB_X20_Y27_N10
\inst13|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~42_combout\ = (\inst13|t_0\(21) & ((\inst33|temp\(21) & (!\inst13|Add0~41\)) # (!\inst33|temp\(21) & ((\inst13|Add0~41\) # (GND))))) # (!\inst13|t_0\(21) & ((\inst33|temp\(21) & (\inst13|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst13|Add0~41\))))
-- \inst13|Add0~43\ = CARRY((\inst13|t_0\(21) & ((!\inst13|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst13|t_0\(21) & (!\inst33|temp\(21) & !\inst13|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst13|Add0~41\,
	combout => \inst13|Add0~42_combout\,
	cout => \inst13|Add0~43\);

-- Location: LCCOMB_X20_Y27_N12
\inst13|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~44_combout\ = ((\inst33|temp\(22) $ (\inst13|t_0\(22) $ (\inst13|Add0~43\)))) # (GND)
-- \inst13|Add0~45\ = CARRY((\inst33|temp\(22) & ((!\inst13|Add0~43\) # (!\inst13|t_0\(22)))) # (!\inst33|temp\(22) & (!\inst13|t_0\(22) & !\inst13|Add0~43\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(22),
	datab => \inst13|t_0\(22),
	datad => VCC,
	cin => \inst13|Add0~43\,
	combout => \inst13|Add0~44_combout\,
	cout => \inst13|Add0~45\);

-- Location: LCCOMB_X20_Y27_N14
\inst13|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~46_combout\ = (\inst33|temp\(23) & ((\inst13|t_0\(23) & (!\inst13|Add0~45\)) # (!\inst13|t_0\(23) & (\inst13|Add0~45\ & VCC)))) # (!\inst33|temp\(23) & ((\inst13|t_0\(23) & ((\inst13|Add0~45\) # (GND))) # (!\inst13|t_0\(23) & 
-- (!\inst13|Add0~45\))))
-- \inst13|Add0~47\ = CARRY((\inst33|temp\(23) & (\inst13|t_0\(23) & !\inst13|Add0~45\)) # (!\inst33|temp\(23) & ((\inst13|t_0\(23)) # (!\inst13|Add0~45\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(23),
	datab => \inst13|t_0\(23),
	datad => VCC,
	cin => \inst13|Add0~45\,
	combout => \inst13|Add0~46_combout\,
	cout => \inst13|Add0~47\);

-- Location: FF_X20_Y27_N21
\inst13|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(26));

-- Location: FF_X20_Y27_N19
\inst13|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(25));

-- Location: FF_X20_Y27_N17
\inst13|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(24));

-- Location: LCCOMB_X20_Y27_N16
\inst13|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~48_combout\ = ((\inst13|t_0\(24) $ (\inst33|temp\(24) $ (\inst13|Add0~47\)))) # (GND)
-- \inst13|Add0~49\ = CARRY((\inst13|t_0\(24) & (\inst33|temp\(24) & !\inst13|Add0~47\)) # (!\inst13|t_0\(24) & ((\inst33|temp\(24)) # (!\inst13|Add0~47\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(24),
	datab => \inst33|temp\(24),
	datad => VCC,
	cin => \inst13|Add0~47\,
	combout => \inst13|Add0~48_combout\,
	cout => \inst13|Add0~49\);

-- Location: LCCOMB_X20_Y27_N18
\inst13|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst13|t_0\(25) & (!\inst13|Add0~49\)) # (!\inst13|t_0\(25) & (\inst13|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst13|t_0\(25) & ((\inst13|Add0~49\) # (GND))) # (!\inst13|t_0\(25) & 
-- (!\inst13|Add0~49\))))
-- \inst13|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst13|t_0\(25) & !\inst13|Add0~49\)) # (!\inst33|temp\(25) & ((\inst13|t_0\(25)) # (!\inst13|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst13|t_0\(25),
	datad => VCC,
	cin => \inst13|Add0~49\,
	combout => \inst13|Add0~50_combout\,
	cout => \inst13|Add0~51\);

-- Location: LCCOMB_X20_Y27_N20
\inst13|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~52_combout\ = ((\inst33|temp\(26) $ (\inst13|t_0\(26) $ (\inst13|Add0~51\)))) # (GND)
-- \inst13|Add0~53\ = CARRY((\inst33|temp\(26) & ((!\inst13|Add0~51\) # (!\inst13|t_0\(26)))) # (!\inst33|temp\(26) & (!\inst13|t_0\(26) & !\inst13|Add0~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(26),
	datab => \inst13|t_0\(26),
	datad => VCC,
	cin => \inst13|Add0~51\,
	combout => \inst13|Add0~52_combout\,
	cout => \inst13|Add0~53\);

-- Location: LCCOMB_X21_Y27_N18
\inst13|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~5_combout\ = (!\inst13|Add0~46_combout\ & (!\inst13|Add0~52_combout\ & (!\inst13|Add0~48_combout\ & !\inst13|Add0~50_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~46_combout\,
	datab => \inst13|Add0~52_combout\,
	datac => \inst13|Add0~48_combout\,
	datad => \inst13|Add0~50_combout\,
	combout => \inst13|data_available~5_combout\);

-- Location: FF_X20_Y27_N29
\inst13|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(30));

-- Location: FF_X20_Y27_N27
\inst13|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(29));

-- Location: FF_X20_Y27_N25
\inst13|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(28));

-- Location: FF_X20_Y27_N23
\inst13|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(27));

-- Location: LCCOMB_X20_Y27_N22
\inst13|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~54_combout\ = (\inst13|t_0\(27) & ((\inst33|temp\(27) & (!\inst13|Add0~53\)) # (!\inst33|temp\(27) & ((\inst13|Add0~53\) # (GND))))) # (!\inst13|t_0\(27) & ((\inst33|temp\(27) & (\inst13|Add0~53\ & VCC)) # (!\inst33|temp\(27) & 
-- (!\inst13|Add0~53\))))
-- \inst13|Add0~55\ = CARRY((\inst13|t_0\(27) & ((!\inst13|Add0~53\) # (!\inst33|temp\(27)))) # (!\inst13|t_0\(27) & (!\inst33|temp\(27) & !\inst13|Add0~53\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(27),
	datab => \inst33|temp\(27),
	datad => VCC,
	cin => \inst13|Add0~53\,
	combout => \inst13|Add0~54_combout\,
	cout => \inst13|Add0~55\);

-- Location: LCCOMB_X20_Y27_N24
\inst13|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~56_combout\ = ((\inst13|t_0\(28) $ (\inst33|temp\(28) $ (\inst13|Add0~55\)))) # (GND)
-- \inst13|Add0~57\ = CARRY((\inst13|t_0\(28) & (\inst33|temp\(28) & !\inst13|Add0~55\)) # (!\inst13|t_0\(28) & ((\inst33|temp\(28)) # (!\inst13|Add0~55\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(28),
	datab => \inst33|temp\(28),
	datad => VCC,
	cin => \inst13|Add0~55\,
	combout => \inst13|Add0~56_combout\,
	cout => \inst13|Add0~57\);

-- Location: LCCOMB_X20_Y27_N26
\inst13|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~58_combout\ = (\inst13|t_0\(29) & ((\inst33|temp\(29) & (!\inst13|Add0~57\)) # (!\inst33|temp\(29) & ((\inst13|Add0~57\) # (GND))))) # (!\inst13|t_0\(29) & ((\inst33|temp\(29) & (\inst13|Add0~57\ & VCC)) # (!\inst33|temp\(29) & 
-- (!\inst13|Add0~57\))))
-- \inst13|Add0~59\ = CARRY((\inst13|t_0\(29) & ((!\inst13|Add0~57\) # (!\inst33|temp\(29)))) # (!\inst13|t_0\(29) & (!\inst33|temp\(29) & !\inst13|Add0~57\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|t_0\(29),
	datab => \inst33|temp\(29),
	datad => VCC,
	cin => \inst13|Add0~57\,
	combout => \inst13|Add0~58_combout\,
	cout => \inst13|Add0~59\);

-- Location: LCCOMB_X20_Y27_N28
\inst13|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst13|t_0\(30) $ (\inst13|Add0~59\)))) # (GND)
-- \inst13|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst13|Add0~59\) # (!\inst13|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst13|t_0\(30) & !\inst13|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst13|t_0\(30),
	datad => VCC,
	cin => \inst13|Add0~59\,
	combout => \inst13|Add0~60_combout\,
	cout => \inst13|Add0~61\);

-- Location: LCCOMB_X21_Y27_N20
\inst13|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~6_combout\ = (!\inst13|Add0~60_combout\ & (!\inst13|Add0~56_combout\ & (!\inst13|Add0~58_combout\ & !\inst13|Add0~54_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~60_combout\,
	datab => \inst13|Add0~56_combout\,
	datac => \inst13|Add0~58_combout\,
	datad => \inst13|Add0~54_combout\,
	combout => \inst13|data_available~6_combout\);

-- Location: FF_X20_Y27_N31
\inst13|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor6_signal~input_o\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|t_0\(31));

-- Location: LCCOMB_X20_Y27_N30
\inst13|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|Add0~62_combout\ = \inst33|temp\(31) $ (\inst13|Add0~61\ $ (!\inst13|t_0\(31)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101010100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(31),
	datad => \inst13|t_0\(31),
	cin => \inst13|Add0~61\,
	combout => \inst13|Add0~62_combout\);

-- Location: LCCOMB_X21_Y28_N6
\inst13|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~2_combout\ = (!\inst13|Add0~36_combout\ & (!\inst13|Add0~32_combout\ & (!\inst13|Add0~34_combout\ & !\inst13|Add0~30_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~36_combout\,
	datab => \inst13|Add0~32_combout\,
	datac => \inst13|Add0~34_combout\,
	datad => \inst13|Add0~30_combout\,
	combout => \inst13|data_available~2_combout\);

-- Location: LCCOMB_X21_Y27_N0
\inst13|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~3_combout\ = (!\inst13|Add0~44_combout\ & (!\inst13|Add0~42_combout\ & (!\inst13|Add0~38_combout\ & !\inst13|Add0~40_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~44_combout\,
	datab => \inst13|Add0~42_combout\,
	datac => \inst13|Add0~38_combout\,
	datad => \inst13|Add0~40_combout\,
	combout => \inst13|data_available~3_combout\);

-- Location: LCCOMB_X21_Y28_N4
\inst13|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~1_combout\ = (!\inst13|Add0~22_combout\ & (!\inst13|Add0~28_combout\ & (!\inst13|Add0~26_combout\ & !\inst13|Add0~24_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~22_combout\,
	datab => \inst13|Add0~28_combout\,
	datac => \inst13|Add0~26_combout\,
	datad => \inst13|Add0~24_combout\,
	combout => \inst13|data_available~1_combout\);

-- Location: LCCOMB_X21_Y28_N10
\inst13|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~0_combout\ = (!\inst13|Add0~20_combout\ & (!\inst13|Add0~18_combout\ & (!\inst13|Add0~16_combout\ & !\inst13|Add0~14_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~20_combout\,
	datab => \inst13|Add0~18_combout\,
	datac => \inst13|Add0~16_combout\,
	datad => \inst13|Add0~14_combout\,
	combout => \inst13|data_available~0_combout\);

-- Location: LCCOMB_X21_Y28_N16
\inst13|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~4_combout\ = (\inst13|data_available~2_combout\ & (\inst13|data_available~3_combout\ & (\inst13|data_available~1_combout\ & \inst13|data_available~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|data_available~2_combout\,
	datab => \inst13|data_available~3_combout\,
	datac => \inst13|data_available~1_combout\,
	datad => \inst13|data_available~0_combout\,
	combout => \inst13|data_available~4_combout\);

-- Location: LCCOMB_X21_Y28_N26
\inst13|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~7_combout\ = (\inst13|data_available~5_combout\ & (\inst13|data_available~6_combout\ & (!\inst13|Add0~62_combout\ & \inst13|data_available~4_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000100000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|data_available~5_combout\,
	datab => \inst13|data_available~6_combout\,
	datac => \inst13|Add0~62_combout\,
	datad => \inst13|data_available~4_combout\,
	combout => \inst13|data_available~7_combout\);

-- Location: LCCOMB_X21_Y28_N24
\inst13|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst13|data_available~10_combout\ = (\inst13|data_available~7_combout\ & (\inst13|Add0~12_combout\ $ (((\inst13|Add0~10_combout\ & \inst13|data_available~9_combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0111000010000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst13|Add0~10_combout\,
	datab => \inst13|data_available~9_combout\,
	datac => \inst13|data_available~7_combout\,
	datad => \inst13|Add0~12_combout\,
	combout => \inst13|data_available~10_combout\);

-- Location: FF_X21_Y28_N25
\inst13|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor6_signal~input_o\,
	d => \inst13|data_available~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst13|data_available~q\);

-- Location: FF_X23_Y27_N1
\inst23|cur_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst13|data_available~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst23|cur_value~q\);

-- Location: FF_X23_Y27_N19
\inst23|last_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst23|cur_value~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst23|last_value~q\);

-- Location: LCCOMB_X23_Y27_N18
\inst23|level_sig\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst23|level_sig~combout\ = (!\inst23|last_value~q\ & \inst23|cur_value~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst23|last_value~q\,
	datad => \inst23|cur_value~q\,
	combout => \inst23|level_sig~combout\);

-- Location: IOIBUF_X14_Y34_N15
\sensor4_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor4_signal,
	o => \sensor4_signal~input_o\);

-- Location: FF_X16_Y23_N13
\inst10|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(6));

-- Location: FF_X16_Y23_N11
\inst10|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(5));

-- Location: FF_X16_Y23_N9
\inst10|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(4));

-- Location: FF_X16_Y23_N7
\inst10|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(3));

-- Location: FF_X16_Y23_N5
\inst10|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(2));

-- Location: FF_X16_Y23_N3
\inst10|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(1));

-- Location: FF_X16_Y23_N1
\inst10|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(0));

-- Location: LCCOMB_X16_Y23_N0
\inst10|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~0_combout\ = (\inst10|t_0\(0) & (\inst33|temp\(0) $ (VCC))) # (!\inst10|t_0\(0) & ((\inst33|temp\(0)) # (GND)))
-- \inst10|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst10|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011011011101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(0),
	datab => \inst33|temp\(0),
	datad => VCC,
	combout => \inst10|Add0~0_combout\,
	cout => \inst10|Add0~1\);

-- Location: LCCOMB_X16_Y23_N2
\inst10|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~2_combout\ = (\inst33|temp\(1) & ((\inst10|t_0\(1) & (!\inst10|Add0~1\)) # (!\inst10|t_0\(1) & (\inst10|Add0~1\ & VCC)))) # (!\inst33|temp\(1) & ((\inst10|t_0\(1) & ((\inst10|Add0~1\) # (GND))) # (!\inst10|t_0\(1) & (!\inst10|Add0~1\))))
-- \inst10|Add0~3\ = CARRY((\inst33|temp\(1) & (\inst10|t_0\(1) & !\inst10|Add0~1\)) # (!\inst33|temp\(1) & ((\inst10|t_0\(1)) # (!\inst10|Add0~1\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(1),
	datab => \inst10|t_0\(1),
	datad => VCC,
	cin => \inst10|Add0~1\,
	combout => \inst10|Add0~2_combout\,
	cout => \inst10|Add0~3\);

-- Location: LCCOMB_X16_Y23_N4
\inst10|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~4_combout\ = ((\inst33|temp\(2) $ (\inst10|t_0\(2) $ (\inst10|Add0~3\)))) # (GND)
-- \inst10|Add0~5\ = CARRY((\inst33|temp\(2) & ((!\inst10|Add0~3\) # (!\inst10|t_0\(2)))) # (!\inst33|temp\(2) & (!\inst10|t_0\(2) & !\inst10|Add0~3\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(2),
	datab => \inst10|t_0\(2),
	datad => VCC,
	cin => \inst10|Add0~3\,
	combout => \inst10|Add0~4_combout\,
	cout => \inst10|Add0~5\);

-- Location: LCCOMB_X16_Y23_N6
\inst10|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~6_combout\ = (\inst10|t_0\(3) & ((\inst33|temp\(3) & (!\inst10|Add0~5\)) # (!\inst33|temp\(3) & ((\inst10|Add0~5\) # (GND))))) # (!\inst10|t_0\(3) & ((\inst33|temp\(3) & (\inst10|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst10|Add0~5\))))
-- \inst10|Add0~7\ = CARRY((\inst10|t_0\(3) & ((!\inst10|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst10|t_0\(3) & (!\inst33|temp\(3) & !\inst10|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst10|Add0~5\,
	combout => \inst10|Add0~6_combout\,
	cout => \inst10|Add0~7\);

-- Location: LCCOMB_X16_Y23_N8
\inst10|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~8_combout\ = ((\inst10|t_0\(4) $ (\inst33|temp\(4) $ (\inst10|Add0~7\)))) # (GND)
-- \inst10|Add0~9\ = CARRY((\inst10|t_0\(4) & (\inst33|temp\(4) & !\inst10|Add0~7\)) # (!\inst10|t_0\(4) & ((\inst33|temp\(4)) # (!\inst10|Add0~7\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(4),
	datab => \inst33|temp\(4),
	datad => VCC,
	cin => \inst10|Add0~7\,
	combout => \inst10|Add0~8_combout\,
	cout => \inst10|Add0~9\);

-- Location: LCCOMB_X16_Y23_N10
\inst10|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~10_combout\ = (\inst10|t_0\(5) & ((\inst33|temp\(5) & (!\inst10|Add0~9\)) # (!\inst33|temp\(5) & ((\inst10|Add0~9\) # (GND))))) # (!\inst10|t_0\(5) & ((\inst33|temp\(5) & (\inst10|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst10|Add0~9\))))
-- \inst10|Add0~11\ = CARRY((\inst10|t_0\(5) & ((!\inst10|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst10|t_0\(5) & (!\inst33|temp\(5) & !\inst10|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst10|Add0~9\,
	combout => \inst10|Add0~10_combout\,
	cout => \inst10|Add0~11\);

-- Location: LCCOMB_X16_Y23_N12
\inst10|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~12_combout\ = ((\inst10|t_0\(6) $ (\inst33|temp\(6) $ (\inst10|Add0~11\)))) # (GND)
-- \inst10|Add0~13\ = CARRY((\inst10|t_0\(6) & (\inst33|temp\(6) & !\inst10|Add0~11\)) # (!\inst10|t_0\(6) & ((\inst33|temp\(6)) # (!\inst10|Add0~11\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(6),
	datab => \inst33|temp\(6),
	datad => VCC,
	cin => \inst10|Add0~11\,
	combout => \inst10|Add0~12_combout\,
	cout => \inst10|Add0~13\);

-- Location: FF_X16_Y22_N23
\inst10|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(27));

-- Location: FF_X16_Y22_N21
\inst10|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(26));

-- Location: FF_X16_Y22_N19
\inst10|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(25));

-- Location: FF_X16_Y22_N17
\inst10|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(24));

-- Location: FF_X16_Y22_N15
\inst10|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(23));

-- Location: FF_X16_Y22_N13
\inst10|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(22));

-- Location: FF_X16_Y22_N11
\inst10|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(21));

-- Location: FF_X16_Y22_N9
\inst10|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(20));

-- Location: FF_X16_Y22_N7
\inst10|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(19));

-- Location: FF_X16_Y22_N5
\inst10|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(18));

-- Location: FF_X16_Y22_N3
\inst10|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(17));

-- Location: FF_X16_Y22_N1
\inst10|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(16));

-- Location: FF_X16_Y23_N31
\inst10|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(15));

-- Location: FF_X16_Y23_N29
\inst10|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(14));

-- Location: FF_X16_Y23_N27
\inst10|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(13));

-- Location: FF_X16_Y23_N25
\inst10|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(12));

-- Location: FF_X16_Y23_N23
\inst10|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(11));

-- Location: FF_X16_Y23_N21
\inst10|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(10));

-- Location: FF_X16_Y23_N19
\inst10|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(9));

-- Location: FF_X16_Y23_N17
\inst10|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(8));

-- Location: FF_X16_Y23_N15
\inst10|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(7));

-- Location: LCCOMB_X16_Y23_N14
\inst10|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~14_combout\ = (\inst10|t_0\(7) & ((\inst33|temp\(7) & (!\inst10|Add0~13\)) # (!\inst33|temp\(7) & ((\inst10|Add0~13\) # (GND))))) # (!\inst10|t_0\(7) & ((\inst33|temp\(7) & (\inst10|Add0~13\ & VCC)) # (!\inst33|temp\(7) & 
-- (!\inst10|Add0~13\))))
-- \inst10|Add0~15\ = CARRY((\inst10|t_0\(7) & ((!\inst10|Add0~13\) # (!\inst33|temp\(7)))) # (!\inst10|t_0\(7) & (!\inst33|temp\(7) & !\inst10|Add0~13\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(7),
	datab => \inst33|temp\(7),
	datad => VCC,
	cin => \inst10|Add0~13\,
	combout => \inst10|Add0~14_combout\,
	cout => \inst10|Add0~15\);

-- Location: LCCOMB_X16_Y23_N16
\inst10|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~16_combout\ = ((\inst10|t_0\(8) $ (\inst33|temp\(8) $ (\inst10|Add0~15\)))) # (GND)
-- \inst10|Add0~17\ = CARRY((\inst10|t_0\(8) & (\inst33|temp\(8) & !\inst10|Add0~15\)) # (!\inst10|t_0\(8) & ((\inst33|temp\(8)) # (!\inst10|Add0~15\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(8),
	datab => \inst33|temp\(8),
	datad => VCC,
	cin => \inst10|Add0~15\,
	combout => \inst10|Add0~16_combout\,
	cout => \inst10|Add0~17\);

-- Location: LCCOMB_X16_Y23_N18
\inst10|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~18_combout\ = (\inst33|temp\(9) & ((\inst10|t_0\(9) & (!\inst10|Add0~17\)) # (!\inst10|t_0\(9) & (\inst10|Add0~17\ & VCC)))) # (!\inst33|temp\(9) & ((\inst10|t_0\(9) & ((\inst10|Add0~17\) # (GND))) # (!\inst10|t_0\(9) & (!\inst10|Add0~17\))))
-- \inst10|Add0~19\ = CARRY((\inst33|temp\(9) & (\inst10|t_0\(9) & !\inst10|Add0~17\)) # (!\inst33|temp\(9) & ((\inst10|t_0\(9)) # (!\inst10|Add0~17\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(9),
	datab => \inst10|t_0\(9),
	datad => VCC,
	cin => \inst10|Add0~17\,
	combout => \inst10|Add0~18_combout\,
	cout => \inst10|Add0~19\);

-- Location: LCCOMB_X16_Y23_N20
\inst10|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~20_combout\ = ((\inst33|temp\(10) $ (\inst10|t_0\(10) $ (\inst10|Add0~19\)))) # (GND)
-- \inst10|Add0~21\ = CARRY((\inst33|temp\(10) & ((!\inst10|Add0~19\) # (!\inst10|t_0\(10)))) # (!\inst33|temp\(10) & (!\inst10|t_0\(10) & !\inst10|Add0~19\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(10),
	datab => \inst10|t_0\(10),
	datad => VCC,
	cin => \inst10|Add0~19\,
	combout => \inst10|Add0~20_combout\,
	cout => \inst10|Add0~21\);

-- Location: LCCOMB_X16_Y23_N22
\inst10|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~22_combout\ = (\inst33|temp\(11) & ((\inst10|t_0\(11) & (!\inst10|Add0~21\)) # (!\inst10|t_0\(11) & (\inst10|Add0~21\ & VCC)))) # (!\inst33|temp\(11) & ((\inst10|t_0\(11) & ((\inst10|Add0~21\) # (GND))) # (!\inst10|t_0\(11) & 
-- (!\inst10|Add0~21\))))
-- \inst10|Add0~23\ = CARRY((\inst33|temp\(11) & (\inst10|t_0\(11) & !\inst10|Add0~21\)) # (!\inst33|temp\(11) & ((\inst10|t_0\(11)) # (!\inst10|Add0~21\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(11),
	datab => \inst10|t_0\(11),
	datad => VCC,
	cin => \inst10|Add0~21\,
	combout => \inst10|Add0~22_combout\,
	cout => \inst10|Add0~23\);

-- Location: LCCOMB_X16_Y23_N24
\inst10|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~24_combout\ = ((\inst33|temp\(12) $ (\inst10|t_0\(12) $ (\inst10|Add0~23\)))) # (GND)
-- \inst10|Add0~25\ = CARRY((\inst33|temp\(12) & ((!\inst10|Add0~23\) # (!\inst10|t_0\(12)))) # (!\inst33|temp\(12) & (!\inst10|t_0\(12) & !\inst10|Add0~23\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(12),
	datab => \inst10|t_0\(12),
	datad => VCC,
	cin => \inst10|Add0~23\,
	combout => \inst10|Add0~24_combout\,
	cout => \inst10|Add0~25\);

-- Location: LCCOMB_X16_Y23_N26
\inst10|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~26_combout\ = (\inst33|temp\(13) & ((\inst10|t_0\(13) & (!\inst10|Add0~25\)) # (!\inst10|t_0\(13) & (\inst10|Add0~25\ & VCC)))) # (!\inst33|temp\(13) & ((\inst10|t_0\(13) & ((\inst10|Add0~25\) # (GND))) # (!\inst10|t_0\(13) & 
-- (!\inst10|Add0~25\))))
-- \inst10|Add0~27\ = CARRY((\inst33|temp\(13) & (\inst10|t_0\(13) & !\inst10|Add0~25\)) # (!\inst33|temp\(13) & ((\inst10|t_0\(13)) # (!\inst10|Add0~25\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(13),
	datab => \inst10|t_0\(13),
	datad => VCC,
	cin => \inst10|Add0~25\,
	combout => \inst10|Add0~26_combout\,
	cout => \inst10|Add0~27\);

-- Location: LCCOMB_X16_Y23_N28
\inst10|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst10|t_0\(14) $ (\inst10|Add0~27\)))) # (GND)
-- \inst10|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst10|Add0~27\) # (!\inst10|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst10|t_0\(14) & !\inst10|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst10|t_0\(14),
	datad => VCC,
	cin => \inst10|Add0~27\,
	combout => \inst10|Add0~28_combout\,
	cout => \inst10|Add0~29\);

-- Location: LCCOMB_X16_Y23_N30
\inst10|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~30_combout\ = (\inst10|t_0\(15) & ((\inst33|temp\(15) & (!\inst10|Add0~29\)) # (!\inst33|temp\(15) & ((\inst10|Add0~29\) # (GND))))) # (!\inst10|t_0\(15) & ((\inst33|temp\(15) & (\inst10|Add0~29\ & VCC)) # (!\inst33|temp\(15) & 
-- (!\inst10|Add0~29\))))
-- \inst10|Add0~31\ = CARRY((\inst10|t_0\(15) & ((!\inst10|Add0~29\) # (!\inst33|temp\(15)))) # (!\inst10|t_0\(15) & (!\inst33|temp\(15) & !\inst10|Add0~29\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(15),
	datab => \inst33|temp\(15),
	datad => VCC,
	cin => \inst10|Add0~29\,
	combout => \inst10|Add0~30_combout\,
	cout => \inst10|Add0~31\);

-- Location: LCCOMB_X16_Y22_N0
\inst10|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~32_combout\ = ((\inst10|t_0\(16) $ (\inst33|temp\(16) $ (\inst10|Add0~31\)))) # (GND)
-- \inst10|Add0~33\ = CARRY((\inst10|t_0\(16) & (\inst33|temp\(16) & !\inst10|Add0~31\)) # (!\inst10|t_0\(16) & ((\inst33|temp\(16)) # (!\inst10|Add0~31\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(16),
	datab => \inst33|temp\(16),
	datad => VCC,
	cin => \inst10|Add0~31\,
	combout => \inst10|Add0~32_combout\,
	cout => \inst10|Add0~33\);

-- Location: LCCOMB_X16_Y22_N2
\inst10|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~34_combout\ = (\inst33|temp\(17) & ((\inst10|t_0\(17) & (!\inst10|Add0~33\)) # (!\inst10|t_0\(17) & (\inst10|Add0~33\ & VCC)))) # (!\inst33|temp\(17) & ((\inst10|t_0\(17) & ((\inst10|Add0~33\) # (GND))) # (!\inst10|t_0\(17) & 
-- (!\inst10|Add0~33\))))
-- \inst10|Add0~35\ = CARRY((\inst33|temp\(17) & (\inst10|t_0\(17) & !\inst10|Add0~33\)) # (!\inst33|temp\(17) & ((\inst10|t_0\(17)) # (!\inst10|Add0~33\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(17),
	datab => \inst10|t_0\(17),
	datad => VCC,
	cin => \inst10|Add0~33\,
	combout => \inst10|Add0~34_combout\,
	cout => \inst10|Add0~35\);

-- Location: LCCOMB_X16_Y22_N4
\inst10|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~36_combout\ = ((\inst10|t_0\(18) $ (\inst33|temp\(18) $ (\inst10|Add0~35\)))) # (GND)
-- \inst10|Add0~37\ = CARRY((\inst10|t_0\(18) & (\inst33|temp\(18) & !\inst10|Add0~35\)) # (!\inst10|t_0\(18) & ((\inst33|temp\(18)) # (!\inst10|Add0~35\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(18),
	datab => \inst33|temp\(18),
	datad => VCC,
	cin => \inst10|Add0~35\,
	combout => \inst10|Add0~36_combout\,
	cout => \inst10|Add0~37\);

-- Location: LCCOMB_X16_Y22_N6
\inst10|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~38_combout\ = (\inst10|t_0\(19) & ((\inst33|temp\(19) & (!\inst10|Add0~37\)) # (!\inst33|temp\(19) & ((\inst10|Add0~37\) # (GND))))) # (!\inst10|t_0\(19) & ((\inst33|temp\(19) & (\inst10|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst10|Add0~37\))))
-- \inst10|Add0~39\ = CARRY((\inst10|t_0\(19) & ((!\inst10|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst10|t_0\(19) & (!\inst33|temp\(19) & !\inst10|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst10|Add0~37\,
	combout => \inst10|Add0~38_combout\,
	cout => \inst10|Add0~39\);

-- Location: LCCOMB_X16_Y22_N8
\inst10|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~40_combout\ = ((\inst10|t_0\(20) $ (\inst33|temp\(20) $ (\inst10|Add0~39\)))) # (GND)
-- \inst10|Add0~41\ = CARRY((\inst10|t_0\(20) & (\inst33|temp\(20) & !\inst10|Add0~39\)) # (!\inst10|t_0\(20) & ((\inst33|temp\(20)) # (!\inst10|Add0~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(20),
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst10|Add0~39\,
	combout => \inst10|Add0~40_combout\,
	cout => \inst10|Add0~41\);

-- Location: LCCOMB_X16_Y22_N10
\inst10|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~42_combout\ = (\inst10|t_0\(21) & ((\inst33|temp\(21) & (!\inst10|Add0~41\)) # (!\inst33|temp\(21) & ((\inst10|Add0~41\) # (GND))))) # (!\inst10|t_0\(21) & ((\inst33|temp\(21) & (\inst10|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst10|Add0~41\))))
-- \inst10|Add0~43\ = CARRY((\inst10|t_0\(21) & ((!\inst10|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst10|t_0\(21) & (!\inst33|temp\(21) & !\inst10|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst10|Add0~41\,
	combout => \inst10|Add0~42_combout\,
	cout => \inst10|Add0~43\);

-- Location: LCCOMB_X16_Y22_N12
\inst10|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~44_combout\ = ((\inst33|temp\(22) $ (\inst10|t_0\(22) $ (\inst10|Add0~43\)))) # (GND)
-- \inst10|Add0~45\ = CARRY((\inst33|temp\(22) & ((!\inst10|Add0~43\) # (!\inst10|t_0\(22)))) # (!\inst33|temp\(22) & (!\inst10|t_0\(22) & !\inst10|Add0~43\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(22),
	datab => \inst10|t_0\(22),
	datad => VCC,
	cin => \inst10|Add0~43\,
	combout => \inst10|Add0~44_combout\,
	cout => \inst10|Add0~45\);

-- Location: LCCOMB_X16_Y22_N14
\inst10|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~46_combout\ = (\inst10|t_0\(23) & ((\inst33|temp\(23) & (!\inst10|Add0~45\)) # (!\inst33|temp\(23) & ((\inst10|Add0~45\) # (GND))))) # (!\inst10|t_0\(23) & ((\inst33|temp\(23) & (\inst10|Add0~45\ & VCC)) # (!\inst33|temp\(23) & 
-- (!\inst10|Add0~45\))))
-- \inst10|Add0~47\ = CARRY((\inst10|t_0\(23) & ((!\inst10|Add0~45\) # (!\inst33|temp\(23)))) # (!\inst10|t_0\(23) & (!\inst33|temp\(23) & !\inst10|Add0~45\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(23),
	datab => \inst33|temp\(23),
	datad => VCC,
	cin => \inst10|Add0~45\,
	combout => \inst10|Add0~46_combout\,
	cout => \inst10|Add0~47\);

-- Location: LCCOMB_X16_Y22_N16
\inst10|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~48_combout\ = ((\inst10|t_0\(24) $ (\inst33|temp\(24) $ (\inst10|Add0~47\)))) # (GND)
-- \inst10|Add0~49\ = CARRY((\inst10|t_0\(24) & (\inst33|temp\(24) & !\inst10|Add0~47\)) # (!\inst10|t_0\(24) & ((\inst33|temp\(24)) # (!\inst10|Add0~47\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(24),
	datab => \inst33|temp\(24),
	datad => VCC,
	cin => \inst10|Add0~47\,
	combout => \inst10|Add0~48_combout\,
	cout => \inst10|Add0~49\);

-- Location: LCCOMB_X16_Y22_N18
\inst10|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst10|t_0\(25) & (!\inst10|Add0~49\)) # (!\inst10|t_0\(25) & (\inst10|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst10|t_0\(25) & ((\inst10|Add0~49\) # (GND))) # (!\inst10|t_0\(25) & 
-- (!\inst10|Add0~49\))))
-- \inst10|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst10|t_0\(25) & !\inst10|Add0~49\)) # (!\inst33|temp\(25) & ((\inst10|t_0\(25)) # (!\inst10|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst10|t_0\(25),
	datad => VCC,
	cin => \inst10|Add0~49\,
	combout => \inst10|Add0~50_combout\,
	cout => \inst10|Add0~51\);

-- Location: LCCOMB_X16_Y22_N20
\inst10|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~52_combout\ = ((\inst33|temp\(26) $ (\inst10|t_0\(26) $ (\inst10|Add0~51\)))) # (GND)
-- \inst10|Add0~53\ = CARRY((\inst33|temp\(26) & ((!\inst10|Add0~51\) # (!\inst10|t_0\(26)))) # (!\inst33|temp\(26) & (!\inst10|t_0\(26) & !\inst10|Add0~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(26),
	datab => \inst10|t_0\(26),
	datad => VCC,
	cin => \inst10|Add0~51\,
	combout => \inst10|Add0~52_combout\,
	cout => \inst10|Add0~53\);

-- Location: LCCOMB_X16_Y22_N22
\inst10|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~54_combout\ = (\inst33|temp\(27) & ((\inst10|t_0\(27) & (!\inst10|Add0~53\)) # (!\inst10|t_0\(27) & (\inst10|Add0~53\ & VCC)))) # (!\inst33|temp\(27) & ((\inst10|t_0\(27) & ((\inst10|Add0~53\) # (GND))) # (!\inst10|t_0\(27) & 
-- (!\inst10|Add0~53\))))
-- \inst10|Add0~55\ = CARRY((\inst33|temp\(27) & (\inst10|t_0\(27) & !\inst10|Add0~53\)) # (!\inst33|temp\(27) & ((\inst10|t_0\(27)) # (!\inst10|Add0~53\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(27),
	datab => \inst10|t_0\(27),
	datad => VCC,
	cin => \inst10|Add0~53\,
	combout => \inst10|Add0~54_combout\,
	cout => \inst10|Add0~55\);

-- Location: FF_X16_Y22_N29
\inst10|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(30));

-- Location: FF_X16_Y22_N27
\inst10|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(29));

-- Location: FF_X16_Y22_N25
\inst10|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(28));

-- Location: LCCOMB_X16_Y22_N24
\inst10|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~56_combout\ = ((\inst33|temp\(28) $ (\inst10|t_0\(28) $ (\inst10|Add0~55\)))) # (GND)
-- \inst10|Add0~57\ = CARRY((\inst33|temp\(28) & ((!\inst10|Add0~55\) # (!\inst10|t_0\(28)))) # (!\inst33|temp\(28) & (!\inst10|t_0\(28) & !\inst10|Add0~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(28),
	datab => \inst10|t_0\(28),
	datad => VCC,
	cin => \inst10|Add0~55\,
	combout => \inst10|Add0~56_combout\,
	cout => \inst10|Add0~57\);

-- Location: LCCOMB_X16_Y22_N26
\inst10|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~58_combout\ = (\inst10|t_0\(29) & ((\inst33|temp\(29) & (!\inst10|Add0~57\)) # (!\inst33|temp\(29) & ((\inst10|Add0~57\) # (GND))))) # (!\inst10|t_0\(29) & ((\inst33|temp\(29) & (\inst10|Add0~57\ & VCC)) # (!\inst33|temp\(29) & 
-- (!\inst10|Add0~57\))))
-- \inst10|Add0~59\ = CARRY((\inst10|t_0\(29) & ((!\inst10|Add0~57\) # (!\inst33|temp\(29)))) # (!\inst10|t_0\(29) & (!\inst33|temp\(29) & !\inst10|Add0~57\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|t_0\(29),
	datab => \inst33|temp\(29),
	datad => VCC,
	cin => \inst10|Add0~57\,
	combout => \inst10|Add0~58_combout\,
	cout => \inst10|Add0~59\);

-- Location: LCCOMB_X16_Y22_N28
\inst10|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst10|t_0\(30) $ (\inst10|Add0~59\)))) # (GND)
-- \inst10|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst10|Add0~59\) # (!\inst10|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst10|t_0\(30) & !\inst10|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst10|t_0\(30),
	datad => VCC,
	cin => \inst10|Add0~59\,
	combout => \inst10|Add0~60_combout\,
	cout => \inst10|Add0~61\);

-- Location: LCCOMB_X17_Y22_N12
\inst10|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~6_combout\ = (!\inst10|Add0~54_combout\ & (!\inst10|Add0~60_combout\ & (!\inst10|Add0~58_combout\ & !\inst10|Add0~56_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~54_combout\,
	datab => \inst10|Add0~60_combout\,
	datac => \inst10|Add0~58_combout\,
	datad => \inst10|Add0~56_combout\,
	combout => \inst10|data_available~6_combout\);

-- Location: LCCOMB_X17_Y22_N2
\inst10|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~5_combout\ = (!\inst10|Add0~46_combout\ & (!\inst10|Add0~50_combout\ & (!\inst10|Add0~48_combout\ & !\inst10|Add0~52_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~46_combout\,
	datab => \inst10|Add0~50_combout\,
	datac => \inst10|Add0~48_combout\,
	datad => \inst10|Add0~52_combout\,
	combout => \inst10|data_available~5_combout\);

-- Location: FF_X16_Y22_N31
\inst10|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor4_signal~input_o\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|t_0\(31));

-- Location: LCCOMB_X16_Y22_N30
\inst10|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|Add0~62_combout\ = \inst33|temp\(31) $ (\inst10|Add0~61\ $ (!\inst10|t_0\(31)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101010100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(31),
	datad => \inst10|t_0\(31),
	cin => \inst10|Add0~61\,
	combout => \inst10|Add0~62_combout\);

-- Location: LCCOMB_X17_Y23_N6
\inst10|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~1_combout\ = (!\inst10|Add0~24_combout\ & (!\inst10|Add0~28_combout\ & (!\inst10|Add0~26_combout\ & !\inst10|Add0~22_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~24_combout\,
	datab => \inst10|Add0~28_combout\,
	datac => \inst10|Add0~26_combout\,
	datad => \inst10|Add0~22_combout\,
	combout => \inst10|data_available~1_combout\);

-- Location: LCCOMB_X17_Y22_N0
\inst10|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~2_combout\ = (!\inst10|Add0~32_combout\ & (!\inst10|Add0~36_combout\ & (!\inst10|Add0~30_combout\ & !\inst10|Add0~34_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~32_combout\,
	datab => \inst10|Add0~36_combout\,
	datac => \inst10|Add0~30_combout\,
	datad => \inst10|Add0~34_combout\,
	combout => \inst10|data_available~2_combout\);

-- Location: LCCOMB_X17_Y23_N4
\inst10|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~0_combout\ = (!\inst10|Add0~14_combout\ & (!\inst10|Add0~20_combout\ & (!\inst10|Add0~16_combout\ & !\inst10|Add0~18_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~14_combout\,
	datab => \inst10|Add0~20_combout\,
	datac => \inst10|Add0~16_combout\,
	datad => \inst10|Add0~18_combout\,
	combout => \inst10|data_available~0_combout\);

-- Location: LCCOMB_X17_Y23_N0
\inst10|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~3_combout\ = (!\inst10|Add0~38_combout\ & (!\inst10|Add0~44_combout\ & (!\inst10|Add0~40_combout\ & !\inst10|Add0~42_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~38_combout\,
	datab => \inst10|Add0~44_combout\,
	datac => \inst10|Add0~40_combout\,
	datad => \inst10|Add0~42_combout\,
	combout => \inst10|data_available~3_combout\);

-- Location: LCCOMB_X17_Y23_N10
\inst10|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~4_combout\ = (\inst10|data_available~1_combout\ & (\inst10|data_available~2_combout\ & (\inst10|data_available~0_combout\ & \inst10|data_available~3_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|data_available~1_combout\,
	datab => \inst10|data_available~2_combout\,
	datac => \inst10|data_available~0_combout\,
	datad => \inst10|data_available~3_combout\,
	combout => \inst10|data_available~4_combout\);

-- Location: LCCOMB_X17_Y23_N20
\inst10|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~7_combout\ = (\inst10|data_available~6_combout\ & (\inst10|data_available~5_combout\ & (!\inst10|Add0~62_combout\ & \inst10|data_available~4_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000100000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|data_available~6_combout\,
	datab => \inst10|data_available~5_combout\,
	datac => \inst10|Add0~62_combout\,
	datad => \inst10|data_available~4_combout\,
	combout => \inst10|data_available~7_combout\);

-- Location: LCCOMB_X17_Y23_N22
\inst10|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~8_combout\ = (\inst10|Add0~4_combout\) # ((\inst10|Add0~0_combout\ & \inst10|Add0~2_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst10|Add0~4_combout\,
	datac => \inst10|Add0~0_combout\,
	datad => \inst10|Add0~2_combout\,
	combout => \inst10|data_available~8_combout\);

-- Location: LCCOMB_X17_Y23_N24
\inst10|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~9_combout\ = (\inst10|Add0~8_combout\ & ((\inst10|Add0~12_combout\) # ((\inst10|Add0~6_combout\ & \inst10|data_available~8_combout\)))) # (!\inst10|Add0~8_combout\ & (\inst10|Add0~12_combout\ & ((\inst10|Add0~6_combout\) # 
-- (\inst10|data_available~8_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111010000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~8_combout\,
	datab => \inst10|Add0~6_combout\,
	datac => \inst10|data_available~8_combout\,
	datad => \inst10|Add0~12_combout\,
	combout => \inst10|data_available~9_combout\);

-- Location: LCCOMB_X17_Y23_N26
\inst10|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst10|data_available~10_combout\ = (\inst10|data_available~7_combout\ & (\inst10|Add0~12_combout\ $ (((\inst10|Add0~10_combout\ & \inst10|data_available~9_combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100100010001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst10|Add0~12_combout\,
	datab => \inst10|data_available~7_combout\,
	datac => \inst10|Add0~10_combout\,
	datad => \inst10|data_available~9_combout\,
	combout => \inst10|data_available~10_combout\);

-- Location: FF_X17_Y23_N27
\inst10|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor4_signal~input_o\,
	d => \inst10|data_available~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst10|data_available~q\);

-- Location: FF_X17_Y23_N9
\inst21|cur_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst10|data_available~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst21|cur_value~q\);

-- Location: FF_X23_Y27_N25
\inst21|last_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst21|cur_value~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst21|last_value~q\);

-- Location: IOIBUF_X23_Y34_N22
\sensor5_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor5_signal,
	o => \sensor5_signal~input_o\);

-- Location: FF_X18_Y30_N13
\inst12|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(6));

-- Location: FF_X18_Y30_N11
\inst12|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(5));

-- Location: FF_X18_Y30_N9
\inst12|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(4));

-- Location: FF_X18_Y30_N7
\inst12|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(3));

-- Location: FF_X18_Y30_N5
\inst12|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(2));

-- Location: FF_X18_Y30_N3
\inst12|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(1));

-- Location: FF_X18_Y30_N1
\inst12|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(0));

-- Location: LCCOMB_X18_Y30_N0
\inst12|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~0_combout\ = (\inst33|temp\(0) & ((GND) # (!\inst12|t_0\(0)))) # (!\inst33|temp\(0) & (\inst12|t_0\(0) $ (GND)))
-- \inst12|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst12|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010111011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(0),
	datab => \inst12|t_0\(0),
	datad => VCC,
	combout => \inst12|Add0~0_combout\,
	cout => \inst12|Add0~1\);

-- Location: LCCOMB_X18_Y30_N2
\inst12|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~2_combout\ = (\inst33|temp\(1) & ((\inst12|t_0\(1) & (!\inst12|Add0~1\)) # (!\inst12|t_0\(1) & (\inst12|Add0~1\ & VCC)))) # (!\inst33|temp\(1) & ((\inst12|t_0\(1) & ((\inst12|Add0~1\) # (GND))) # (!\inst12|t_0\(1) & (!\inst12|Add0~1\))))
-- \inst12|Add0~3\ = CARRY((\inst33|temp\(1) & (\inst12|t_0\(1) & !\inst12|Add0~1\)) # (!\inst33|temp\(1) & ((\inst12|t_0\(1)) # (!\inst12|Add0~1\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(1),
	datab => \inst12|t_0\(1),
	datad => VCC,
	cin => \inst12|Add0~1\,
	combout => \inst12|Add0~2_combout\,
	cout => \inst12|Add0~3\);

-- Location: LCCOMB_X18_Y30_N4
\inst12|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~4_combout\ = ((\inst33|temp\(2) $ (\inst12|t_0\(2) $ (\inst12|Add0~3\)))) # (GND)
-- \inst12|Add0~5\ = CARRY((\inst33|temp\(2) & ((!\inst12|Add0~3\) # (!\inst12|t_0\(2)))) # (!\inst33|temp\(2) & (!\inst12|t_0\(2) & !\inst12|Add0~3\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(2),
	datab => \inst12|t_0\(2),
	datad => VCC,
	cin => \inst12|Add0~3\,
	combout => \inst12|Add0~4_combout\,
	cout => \inst12|Add0~5\);

-- Location: LCCOMB_X18_Y30_N6
\inst12|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~6_combout\ = (\inst12|t_0\(3) & ((\inst33|temp\(3) & (!\inst12|Add0~5\)) # (!\inst33|temp\(3) & ((\inst12|Add0~5\) # (GND))))) # (!\inst12|t_0\(3) & ((\inst33|temp\(3) & (\inst12|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst12|Add0~5\))))
-- \inst12|Add0~7\ = CARRY((\inst12|t_0\(3) & ((!\inst12|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst12|t_0\(3) & (!\inst33|temp\(3) & !\inst12|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst12|Add0~5\,
	combout => \inst12|Add0~6_combout\,
	cout => \inst12|Add0~7\);

-- Location: LCCOMB_X18_Y30_N8
\inst12|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~8_combout\ = ((\inst12|t_0\(4) $ (\inst33|temp\(4) $ (\inst12|Add0~7\)))) # (GND)
-- \inst12|Add0~9\ = CARRY((\inst12|t_0\(4) & (\inst33|temp\(4) & !\inst12|Add0~7\)) # (!\inst12|t_0\(4) & ((\inst33|temp\(4)) # (!\inst12|Add0~7\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(4),
	datab => \inst33|temp\(4),
	datad => VCC,
	cin => \inst12|Add0~7\,
	combout => \inst12|Add0~8_combout\,
	cout => \inst12|Add0~9\);

-- Location: LCCOMB_X18_Y30_N10
\inst12|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~10_combout\ = (\inst12|t_0\(5) & ((\inst33|temp\(5) & (!\inst12|Add0~9\)) # (!\inst33|temp\(5) & ((\inst12|Add0~9\) # (GND))))) # (!\inst12|t_0\(5) & ((\inst33|temp\(5) & (\inst12|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst12|Add0~9\))))
-- \inst12|Add0~11\ = CARRY((\inst12|t_0\(5) & ((!\inst12|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst12|t_0\(5) & (!\inst33|temp\(5) & !\inst12|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst12|Add0~9\,
	combout => \inst12|Add0~10_combout\,
	cout => \inst12|Add0~11\);

-- Location: LCCOMB_X18_Y30_N12
\inst12|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~12_combout\ = ((\inst12|t_0\(6) $ (\inst33|temp\(6) $ (\inst12|Add0~11\)))) # (GND)
-- \inst12|Add0~13\ = CARRY((\inst12|t_0\(6) & (\inst33|temp\(6) & !\inst12|Add0~11\)) # (!\inst12|t_0\(6) & ((\inst33|temp\(6)) # (!\inst12|Add0~11\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(6),
	datab => \inst33|temp\(6),
	datad => VCC,
	cin => \inst12|Add0~11\,
	combout => \inst12|Add0~12_combout\,
	cout => \inst12|Add0~13\);

-- Location: LCCOMB_X17_Y30_N2
\inst12|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~8_combout\ = (\inst12|Add0~4_combout\) # ((\inst12|Add0~0_combout\ & \inst12|Add0~2_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~4_combout\,
	datac => \inst12|Add0~0_combout\,
	datad => \inst12|Add0~2_combout\,
	combout => \inst12|data_available~8_combout\);

-- Location: LCCOMB_X19_Y30_N26
\inst12|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~9_combout\ = (\inst12|Add0~12_combout\ & ((\inst12|data_available~8_combout\) # ((\inst12|Add0~6_combout\) # (\inst12|Add0~8_combout\)))) # (!\inst12|Add0~12_combout\ & (\inst12|data_available~8_combout\ & (\inst12|Add0~6_combout\ & 
-- \inst12|Add0~8_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110101010101000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~12_combout\,
	datab => \inst12|data_available~8_combout\,
	datac => \inst12|Add0~6_combout\,
	datad => \inst12|Add0~8_combout\,
	combout => \inst12|data_available~9_combout\);

-- Location: FF_X18_Y29_N25
\inst12|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(28));

-- Location: FF_X18_Y29_N23
\inst12|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(27));

-- Location: FF_X18_Y29_N21
\inst12|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(26));

-- Location: FF_X18_Y29_N19
\inst12|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(25));

-- Location: FF_X18_Y29_N17
\inst12|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(24));

-- Location: FF_X18_Y29_N15
\inst12|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(23));

-- Location: FF_X18_Y29_N13
\inst12|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(22));

-- Location: FF_X18_Y29_N11
\inst12|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(21));

-- Location: FF_X18_Y29_N9
\inst12|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(20));

-- Location: FF_X18_Y29_N7
\inst12|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(19));

-- Location: FF_X18_Y29_N5
\inst12|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(18));

-- Location: FF_X18_Y29_N3
\inst12|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(17));

-- Location: FF_X18_Y29_N1
\inst12|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(16));

-- Location: FF_X18_Y30_N31
\inst12|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(15));

-- Location: FF_X18_Y30_N29
\inst12|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(14));

-- Location: FF_X18_Y30_N27
\inst12|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(13));

-- Location: FF_X18_Y30_N25
\inst12|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(12));

-- Location: FF_X18_Y30_N23
\inst12|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(11));

-- Location: FF_X18_Y30_N21
\inst12|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(10));

-- Location: FF_X18_Y30_N19
\inst12|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(9));

-- Location: FF_X18_Y30_N17
\inst12|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(8));

-- Location: FF_X18_Y30_N15
\inst12|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(7));

-- Location: LCCOMB_X18_Y30_N14
\inst12|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~14_combout\ = (\inst12|t_0\(7) & ((\inst33|temp\(7) & (!\inst12|Add0~13\)) # (!\inst33|temp\(7) & ((\inst12|Add0~13\) # (GND))))) # (!\inst12|t_0\(7) & ((\inst33|temp\(7) & (\inst12|Add0~13\ & VCC)) # (!\inst33|temp\(7) & 
-- (!\inst12|Add0~13\))))
-- \inst12|Add0~15\ = CARRY((\inst12|t_0\(7) & ((!\inst12|Add0~13\) # (!\inst33|temp\(7)))) # (!\inst12|t_0\(7) & (!\inst33|temp\(7) & !\inst12|Add0~13\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(7),
	datab => \inst33|temp\(7),
	datad => VCC,
	cin => \inst12|Add0~13\,
	combout => \inst12|Add0~14_combout\,
	cout => \inst12|Add0~15\);

-- Location: LCCOMB_X18_Y30_N16
\inst12|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~16_combout\ = ((\inst33|temp\(8) $ (\inst12|t_0\(8) $ (\inst12|Add0~15\)))) # (GND)
-- \inst12|Add0~17\ = CARRY((\inst33|temp\(8) & ((!\inst12|Add0~15\) # (!\inst12|t_0\(8)))) # (!\inst33|temp\(8) & (!\inst12|t_0\(8) & !\inst12|Add0~15\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(8),
	datab => \inst12|t_0\(8),
	datad => VCC,
	cin => \inst12|Add0~15\,
	combout => \inst12|Add0~16_combout\,
	cout => \inst12|Add0~17\);

-- Location: LCCOMB_X18_Y30_N18
\inst12|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~18_combout\ = (\inst33|temp\(9) & ((\inst12|t_0\(9) & (!\inst12|Add0~17\)) # (!\inst12|t_0\(9) & (\inst12|Add0~17\ & VCC)))) # (!\inst33|temp\(9) & ((\inst12|t_0\(9) & ((\inst12|Add0~17\) # (GND))) # (!\inst12|t_0\(9) & (!\inst12|Add0~17\))))
-- \inst12|Add0~19\ = CARRY((\inst33|temp\(9) & (\inst12|t_0\(9) & !\inst12|Add0~17\)) # (!\inst33|temp\(9) & ((\inst12|t_0\(9)) # (!\inst12|Add0~17\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(9),
	datab => \inst12|t_0\(9),
	datad => VCC,
	cin => \inst12|Add0~17\,
	combout => \inst12|Add0~18_combout\,
	cout => \inst12|Add0~19\);

-- Location: LCCOMB_X18_Y30_N20
\inst12|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~20_combout\ = ((\inst33|temp\(10) $ (\inst12|t_0\(10) $ (\inst12|Add0~19\)))) # (GND)
-- \inst12|Add0~21\ = CARRY((\inst33|temp\(10) & ((!\inst12|Add0~19\) # (!\inst12|t_0\(10)))) # (!\inst33|temp\(10) & (!\inst12|t_0\(10) & !\inst12|Add0~19\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(10),
	datab => \inst12|t_0\(10),
	datad => VCC,
	cin => \inst12|Add0~19\,
	combout => \inst12|Add0~20_combout\,
	cout => \inst12|Add0~21\);

-- Location: LCCOMB_X18_Y30_N22
\inst12|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~22_combout\ = (\inst12|t_0\(11) & ((\inst33|temp\(11) & (!\inst12|Add0~21\)) # (!\inst33|temp\(11) & ((\inst12|Add0~21\) # (GND))))) # (!\inst12|t_0\(11) & ((\inst33|temp\(11) & (\inst12|Add0~21\ & VCC)) # (!\inst33|temp\(11) & 
-- (!\inst12|Add0~21\))))
-- \inst12|Add0~23\ = CARRY((\inst12|t_0\(11) & ((!\inst12|Add0~21\) # (!\inst33|temp\(11)))) # (!\inst12|t_0\(11) & (!\inst33|temp\(11) & !\inst12|Add0~21\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(11),
	datab => \inst33|temp\(11),
	datad => VCC,
	cin => \inst12|Add0~21\,
	combout => \inst12|Add0~22_combout\,
	cout => \inst12|Add0~23\);

-- Location: LCCOMB_X18_Y30_N24
\inst12|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~24_combout\ = ((\inst33|temp\(12) $ (\inst12|t_0\(12) $ (\inst12|Add0~23\)))) # (GND)
-- \inst12|Add0~25\ = CARRY((\inst33|temp\(12) & ((!\inst12|Add0~23\) # (!\inst12|t_0\(12)))) # (!\inst33|temp\(12) & (!\inst12|t_0\(12) & !\inst12|Add0~23\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(12),
	datab => \inst12|t_0\(12),
	datad => VCC,
	cin => \inst12|Add0~23\,
	combout => \inst12|Add0~24_combout\,
	cout => \inst12|Add0~25\);

-- Location: LCCOMB_X18_Y30_N26
\inst12|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~26_combout\ = (\inst33|temp\(13) & ((\inst12|t_0\(13) & (!\inst12|Add0~25\)) # (!\inst12|t_0\(13) & (\inst12|Add0~25\ & VCC)))) # (!\inst33|temp\(13) & ((\inst12|t_0\(13) & ((\inst12|Add0~25\) # (GND))) # (!\inst12|t_0\(13) & 
-- (!\inst12|Add0~25\))))
-- \inst12|Add0~27\ = CARRY((\inst33|temp\(13) & (\inst12|t_0\(13) & !\inst12|Add0~25\)) # (!\inst33|temp\(13) & ((\inst12|t_0\(13)) # (!\inst12|Add0~25\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(13),
	datab => \inst12|t_0\(13),
	datad => VCC,
	cin => \inst12|Add0~25\,
	combout => \inst12|Add0~26_combout\,
	cout => \inst12|Add0~27\);

-- Location: LCCOMB_X18_Y30_N28
\inst12|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst12|t_0\(14) $ (\inst12|Add0~27\)))) # (GND)
-- \inst12|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst12|Add0~27\) # (!\inst12|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst12|t_0\(14) & !\inst12|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst12|t_0\(14),
	datad => VCC,
	cin => \inst12|Add0~27\,
	combout => \inst12|Add0~28_combout\,
	cout => \inst12|Add0~29\);

-- Location: LCCOMB_X18_Y30_N30
\inst12|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~30_combout\ = (\inst12|t_0\(15) & ((\inst33|temp\(15) & (!\inst12|Add0~29\)) # (!\inst33|temp\(15) & ((\inst12|Add0~29\) # (GND))))) # (!\inst12|t_0\(15) & ((\inst33|temp\(15) & (\inst12|Add0~29\ & VCC)) # (!\inst33|temp\(15) & 
-- (!\inst12|Add0~29\))))
-- \inst12|Add0~31\ = CARRY((\inst12|t_0\(15) & ((!\inst12|Add0~29\) # (!\inst33|temp\(15)))) # (!\inst12|t_0\(15) & (!\inst33|temp\(15) & !\inst12|Add0~29\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(15),
	datab => \inst33|temp\(15),
	datad => VCC,
	cin => \inst12|Add0~29\,
	combout => \inst12|Add0~30_combout\,
	cout => \inst12|Add0~31\);

-- Location: LCCOMB_X18_Y29_N0
\inst12|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~32_combout\ = ((\inst12|t_0\(16) $ (\inst33|temp\(16) $ (\inst12|Add0~31\)))) # (GND)
-- \inst12|Add0~33\ = CARRY((\inst12|t_0\(16) & (\inst33|temp\(16) & !\inst12|Add0~31\)) # (!\inst12|t_0\(16) & ((\inst33|temp\(16)) # (!\inst12|Add0~31\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(16),
	datab => \inst33|temp\(16),
	datad => VCC,
	cin => \inst12|Add0~31\,
	combout => \inst12|Add0~32_combout\,
	cout => \inst12|Add0~33\);

-- Location: LCCOMB_X18_Y29_N2
\inst12|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~34_combout\ = (\inst33|temp\(17) & ((\inst12|t_0\(17) & (!\inst12|Add0~33\)) # (!\inst12|t_0\(17) & (\inst12|Add0~33\ & VCC)))) # (!\inst33|temp\(17) & ((\inst12|t_0\(17) & ((\inst12|Add0~33\) # (GND))) # (!\inst12|t_0\(17) & 
-- (!\inst12|Add0~33\))))
-- \inst12|Add0~35\ = CARRY((\inst33|temp\(17) & (\inst12|t_0\(17) & !\inst12|Add0~33\)) # (!\inst33|temp\(17) & ((\inst12|t_0\(17)) # (!\inst12|Add0~33\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(17),
	datab => \inst12|t_0\(17),
	datad => VCC,
	cin => \inst12|Add0~33\,
	combout => \inst12|Add0~34_combout\,
	cout => \inst12|Add0~35\);

-- Location: LCCOMB_X18_Y29_N4
\inst12|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~36_combout\ = ((\inst33|temp\(18) $ (\inst12|t_0\(18) $ (\inst12|Add0~35\)))) # (GND)
-- \inst12|Add0~37\ = CARRY((\inst33|temp\(18) & ((!\inst12|Add0~35\) # (!\inst12|t_0\(18)))) # (!\inst33|temp\(18) & (!\inst12|t_0\(18) & !\inst12|Add0~35\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(18),
	datab => \inst12|t_0\(18),
	datad => VCC,
	cin => \inst12|Add0~35\,
	combout => \inst12|Add0~36_combout\,
	cout => \inst12|Add0~37\);

-- Location: LCCOMB_X18_Y29_N6
\inst12|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~38_combout\ = (\inst12|t_0\(19) & ((\inst33|temp\(19) & (!\inst12|Add0~37\)) # (!\inst33|temp\(19) & ((\inst12|Add0~37\) # (GND))))) # (!\inst12|t_0\(19) & ((\inst33|temp\(19) & (\inst12|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst12|Add0~37\))))
-- \inst12|Add0~39\ = CARRY((\inst12|t_0\(19) & ((!\inst12|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst12|t_0\(19) & (!\inst33|temp\(19) & !\inst12|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst12|Add0~37\,
	combout => \inst12|Add0~38_combout\,
	cout => \inst12|Add0~39\);

-- Location: LCCOMB_X18_Y29_N8
\inst12|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~40_combout\ = ((\inst12|t_0\(20) $ (\inst33|temp\(20) $ (\inst12|Add0~39\)))) # (GND)
-- \inst12|Add0~41\ = CARRY((\inst12|t_0\(20) & (\inst33|temp\(20) & !\inst12|Add0~39\)) # (!\inst12|t_0\(20) & ((\inst33|temp\(20)) # (!\inst12|Add0~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(20),
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst12|Add0~39\,
	combout => \inst12|Add0~40_combout\,
	cout => \inst12|Add0~41\);

-- Location: LCCOMB_X18_Y29_N10
\inst12|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~42_combout\ = (\inst12|t_0\(21) & ((\inst33|temp\(21) & (!\inst12|Add0~41\)) # (!\inst33|temp\(21) & ((\inst12|Add0~41\) # (GND))))) # (!\inst12|t_0\(21) & ((\inst33|temp\(21) & (\inst12|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst12|Add0~41\))))
-- \inst12|Add0~43\ = CARRY((\inst12|t_0\(21) & ((!\inst12|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst12|t_0\(21) & (!\inst33|temp\(21) & !\inst12|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst12|Add0~41\,
	combout => \inst12|Add0~42_combout\,
	cout => \inst12|Add0~43\);

-- Location: LCCOMB_X18_Y29_N12
\inst12|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~44_combout\ = ((\inst12|t_0\(22) $ (\inst33|temp\(22) $ (\inst12|Add0~43\)))) # (GND)
-- \inst12|Add0~45\ = CARRY((\inst12|t_0\(22) & (\inst33|temp\(22) & !\inst12|Add0~43\)) # (!\inst12|t_0\(22) & ((\inst33|temp\(22)) # (!\inst12|Add0~43\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(22),
	datab => \inst33|temp\(22),
	datad => VCC,
	cin => \inst12|Add0~43\,
	combout => \inst12|Add0~44_combout\,
	cout => \inst12|Add0~45\);

-- Location: LCCOMB_X18_Y29_N14
\inst12|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~46_combout\ = (\inst12|t_0\(23) & ((\inst33|temp\(23) & (!\inst12|Add0~45\)) # (!\inst33|temp\(23) & ((\inst12|Add0~45\) # (GND))))) # (!\inst12|t_0\(23) & ((\inst33|temp\(23) & (\inst12|Add0~45\ & VCC)) # (!\inst33|temp\(23) & 
-- (!\inst12|Add0~45\))))
-- \inst12|Add0~47\ = CARRY((\inst12|t_0\(23) & ((!\inst12|Add0~45\) # (!\inst33|temp\(23)))) # (!\inst12|t_0\(23) & (!\inst33|temp\(23) & !\inst12|Add0~45\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(23),
	datab => \inst33|temp\(23),
	datad => VCC,
	cin => \inst12|Add0~45\,
	combout => \inst12|Add0~46_combout\,
	cout => \inst12|Add0~47\);

-- Location: LCCOMB_X18_Y29_N16
\inst12|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~48_combout\ = ((\inst12|t_0\(24) $ (\inst33|temp\(24) $ (\inst12|Add0~47\)))) # (GND)
-- \inst12|Add0~49\ = CARRY((\inst12|t_0\(24) & (\inst33|temp\(24) & !\inst12|Add0~47\)) # (!\inst12|t_0\(24) & ((\inst33|temp\(24)) # (!\inst12|Add0~47\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(24),
	datab => \inst33|temp\(24),
	datad => VCC,
	cin => \inst12|Add0~47\,
	combout => \inst12|Add0~48_combout\,
	cout => \inst12|Add0~49\);

-- Location: LCCOMB_X18_Y29_N18
\inst12|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst12|t_0\(25) & (!\inst12|Add0~49\)) # (!\inst12|t_0\(25) & (\inst12|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst12|t_0\(25) & ((\inst12|Add0~49\) # (GND))) # (!\inst12|t_0\(25) & 
-- (!\inst12|Add0~49\))))
-- \inst12|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst12|t_0\(25) & !\inst12|Add0~49\)) # (!\inst33|temp\(25) & ((\inst12|t_0\(25)) # (!\inst12|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst12|t_0\(25),
	datad => VCC,
	cin => \inst12|Add0~49\,
	combout => \inst12|Add0~50_combout\,
	cout => \inst12|Add0~51\);

-- Location: LCCOMB_X18_Y29_N20
\inst12|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~52_combout\ = ((\inst33|temp\(26) $ (\inst12|t_0\(26) $ (\inst12|Add0~51\)))) # (GND)
-- \inst12|Add0~53\ = CARRY((\inst33|temp\(26) & ((!\inst12|Add0~51\) # (!\inst12|t_0\(26)))) # (!\inst33|temp\(26) & (!\inst12|t_0\(26) & !\inst12|Add0~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(26),
	datab => \inst12|t_0\(26),
	datad => VCC,
	cin => \inst12|Add0~51\,
	combout => \inst12|Add0~52_combout\,
	cout => \inst12|Add0~53\);

-- Location: LCCOMB_X18_Y29_N22
\inst12|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~54_combout\ = (\inst33|temp\(27) & ((\inst12|t_0\(27) & (!\inst12|Add0~53\)) # (!\inst12|t_0\(27) & (\inst12|Add0~53\ & VCC)))) # (!\inst33|temp\(27) & ((\inst12|t_0\(27) & ((\inst12|Add0~53\) # (GND))) # (!\inst12|t_0\(27) & 
-- (!\inst12|Add0~53\))))
-- \inst12|Add0~55\ = CARRY((\inst33|temp\(27) & (\inst12|t_0\(27) & !\inst12|Add0~53\)) # (!\inst33|temp\(27) & ((\inst12|t_0\(27)) # (!\inst12|Add0~53\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(27),
	datab => \inst12|t_0\(27),
	datad => VCC,
	cin => \inst12|Add0~53\,
	combout => \inst12|Add0~54_combout\,
	cout => \inst12|Add0~55\);

-- Location: LCCOMB_X18_Y29_N24
\inst12|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~56_combout\ = ((\inst33|temp\(28) $ (\inst12|t_0\(28) $ (\inst12|Add0~55\)))) # (GND)
-- \inst12|Add0~57\ = CARRY((\inst33|temp\(28) & ((!\inst12|Add0~55\) # (!\inst12|t_0\(28)))) # (!\inst33|temp\(28) & (!\inst12|t_0\(28) & !\inst12|Add0~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(28),
	datab => \inst12|t_0\(28),
	datad => VCC,
	cin => \inst12|Add0~55\,
	combout => \inst12|Add0~56_combout\,
	cout => \inst12|Add0~57\);

-- Location: FF_X18_Y29_N27
\inst12|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(29));

-- Location: LCCOMB_X18_Y29_N26
\inst12|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~58_combout\ = (\inst12|t_0\(29) & ((\inst33|temp\(29) & (!\inst12|Add0~57\)) # (!\inst33|temp\(29) & ((\inst12|Add0~57\) # (GND))))) # (!\inst12|t_0\(29) & ((\inst33|temp\(29) & (\inst12|Add0~57\ & VCC)) # (!\inst33|temp\(29) & 
-- (!\inst12|Add0~57\))))
-- \inst12|Add0~59\ = CARRY((\inst12|t_0\(29) & ((!\inst12|Add0~57\) # (!\inst33|temp\(29)))) # (!\inst12|t_0\(29) & (!\inst33|temp\(29) & !\inst12|Add0~57\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(29),
	datab => \inst33|temp\(29),
	datad => VCC,
	cin => \inst12|Add0~57\,
	combout => \inst12|Add0~58_combout\,
	cout => \inst12|Add0~59\);

-- Location: FF_X18_Y29_N29
\inst12|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(30));

-- Location: LCCOMB_X18_Y29_N28
\inst12|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst12|t_0\(30) $ (\inst12|Add0~59\)))) # (GND)
-- \inst12|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst12|Add0~59\) # (!\inst12|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst12|t_0\(30) & !\inst12|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst12|t_0\(30),
	datad => VCC,
	cin => \inst12|Add0~59\,
	combout => \inst12|Add0~60_combout\,
	cout => \inst12|Add0~61\);

-- Location: LCCOMB_X19_Y30_N6
\inst12|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~6_combout\ = (!\inst12|Add0~56_combout\ & (!\inst12|Add0~58_combout\ & (!\inst12|Add0~54_combout\ & !\inst12|Add0~60_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~56_combout\,
	datab => \inst12|Add0~58_combout\,
	datac => \inst12|Add0~54_combout\,
	datad => \inst12|Add0~60_combout\,
	combout => \inst12|data_available~6_combout\);

-- Location: LCCOMB_X19_Y30_N20
\inst12|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~5_combout\ = (!\inst12|Add0~48_combout\ & (!\inst12|Add0~50_combout\ & (!\inst12|Add0~46_combout\ & !\inst12|Add0~52_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~48_combout\,
	datab => \inst12|Add0~50_combout\,
	datac => \inst12|Add0~46_combout\,
	datad => \inst12|Add0~52_combout\,
	combout => \inst12|data_available~5_combout\);

-- Location: FF_X18_Y29_N31
\inst12|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor5_signal~input_o\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|t_0\(31));

-- Location: LCCOMB_X18_Y29_N30
\inst12|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|Add0~62_combout\ = \inst12|t_0\(31) $ (\inst33|temp\(31) $ (!\inst12|Add0~61\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101101001",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|t_0\(31),
	datab => \inst33|temp\(31),
	cin => \inst12|Add0~61\,
	combout => \inst12|Add0~62_combout\);

-- Location: LCCOMB_X17_Y30_N24
\inst12|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~2_combout\ = (!\inst12|Add0~36_combout\ & (!\inst12|Add0~34_combout\ & (!\inst12|Add0~30_combout\ & !\inst12|Add0~32_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~36_combout\,
	datab => \inst12|Add0~34_combout\,
	datac => \inst12|Add0~30_combout\,
	datad => \inst12|Add0~32_combout\,
	combout => \inst12|data_available~2_combout\);

-- Location: LCCOMB_X17_Y29_N0
\inst12|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~3_combout\ = (!\inst12|Add0~38_combout\ & (!\inst12|Add0~44_combout\ & (!\inst12|Add0~42_combout\ & !\inst12|Add0~40_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~38_combout\,
	datab => \inst12|Add0~44_combout\,
	datac => \inst12|Add0~42_combout\,
	datad => \inst12|Add0~40_combout\,
	combout => \inst12|data_available~3_combout\);

-- Location: LCCOMB_X19_Y30_N22
\inst12|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~0_combout\ = (!\inst12|Add0~14_combout\ & (!\inst12|Add0~20_combout\ & (!\inst12|Add0~16_combout\ & !\inst12|Add0~18_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~14_combout\,
	datab => \inst12|Add0~20_combout\,
	datac => \inst12|Add0~16_combout\,
	datad => \inst12|Add0~18_combout\,
	combout => \inst12|data_available~0_combout\);

-- Location: LCCOMB_X19_Y30_N16
\inst12|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~1_combout\ = (!\inst12|Add0~28_combout\ & (!\inst12|Add0~24_combout\ & (!\inst12|Add0~26_combout\ & !\inst12|Add0~22_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~28_combout\,
	datab => \inst12|Add0~24_combout\,
	datac => \inst12|Add0~26_combout\,
	datad => \inst12|Add0~22_combout\,
	combout => \inst12|data_available~1_combout\);

-- Location: LCCOMB_X19_Y30_N10
\inst12|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~4_combout\ = (\inst12|data_available~2_combout\ & (\inst12|data_available~3_combout\ & (\inst12|data_available~0_combout\ & \inst12|data_available~1_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|data_available~2_combout\,
	datab => \inst12|data_available~3_combout\,
	datac => \inst12|data_available~0_combout\,
	datad => \inst12|data_available~1_combout\,
	combout => \inst12|data_available~4_combout\);

-- Location: LCCOMB_X19_Y30_N24
\inst12|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~7_combout\ = (\inst12|data_available~6_combout\ & (\inst12|data_available~5_combout\ & (!\inst12|Add0~62_combout\ & \inst12|data_available~4_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000100000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|data_available~6_combout\,
	datab => \inst12|data_available~5_combout\,
	datac => \inst12|Add0~62_combout\,
	datad => \inst12|data_available~4_combout\,
	combout => \inst12|data_available~7_combout\);

-- Location: LCCOMB_X19_Y30_N4
\inst12|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst12|data_available~10_combout\ = (\inst12|data_available~7_combout\ & (\inst12|Add0~12_combout\ $ (((\inst12|Add0~10_combout\ & \inst12|data_available~9_combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110101000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst12|Add0~12_combout\,
	datab => \inst12|Add0~10_combout\,
	datac => \inst12|data_available~9_combout\,
	datad => \inst12|data_available~7_combout\,
	combout => \inst12|data_available~10_combout\);

-- Location: FF_X19_Y30_N5
\inst12|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor5_signal~input_o\,
	d => \inst12|data_available~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst12|data_available~q\);

-- Location: LCCOMB_X23_Y27_N28
\inst22|cur_value~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst22|cur_value~feeder_combout\ = \inst12|data_available~q\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst12|data_available~q\,
	combout => \inst22|cur_value~feeder_combout\);

-- Location: FF_X23_Y27_N29
\inst22|cur_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst22|cur_value~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst22|cur_value~q\);

-- Location: FF_X23_Y27_N27
\inst22|last_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst22|cur_value~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst22|last_value~q\);

-- Location: LCCOMB_X23_Y27_N26
\inst16|write~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|write~0_combout\ = (\inst21|last_value~q\ & (((\inst22|last_value~q\) # (!\inst22|cur_value~q\)))) # (!\inst21|last_value~q\ & (!\inst21|cur_value~q\ & ((\inst22|last_value~q\) # (!\inst22|cur_value~q\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011000010111011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst21|last_value~q\,
	datab => \inst21|cur_value~q\,
	datac => \inst22|last_value~q\,
	datad => \inst22|cur_value~q\,
	combout => \inst16|write~0_combout\);

-- Location: LCCOMB_X23_Y27_N24
\inst16|WideOr0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|WideOr0~1_combout\ = (\inst22|last_value~q\ & (\inst21|cur_value~q\ & (!\inst21|last_value~q\))) # (!\inst22|last_value~q\ & (\inst22|cur_value~q\ $ (((\inst21|cur_value~q\ & !\inst21|last_value~q\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101100100001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst22|last_value~q\,
	datab => \inst21|cur_value~q\,
	datac => \inst21|last_value~q\,
	datad => \inst22|cur_value~q\,
	combout => \inst16|WideOr0~1_combout\);

-- Location: LCCOMB_X23_Y27_N10
\inst16|write~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|write~3_combout\ = (\inst24|level_sig~combout\ & (!\inst23|level_sig~combout\ & (\inst16|write~0_combout\))) # (!\inst24|level_sig~combout\ & ((\inst23|level_sig~combout\ & (\inst16|write~0_combout\)) # (!\inst23|level_sig~combout\ & 
-- ((\inst16|WideOr0~1_combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0111000101100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst24|level_sig~combout\,
	datab => \inst23|level_sig~combout\,
	datac => \inst16|write~0_combout\,
	datad => \inst16|WideOr0~1_combout\,
	combout => \inst16|write~3_combout\);

-- Location: IOIBUF_X9_Y34_N8
\sensor2_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor2_signal,
	o => \sensor2_signal~input_o\);

-- Location: FF_X15_Y27_N13
\inst8|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(6));

-- Location: FF_X15_Y27_N11
\inst8|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(5));

-- Location: FF_X15_Y27_N9
\inst8|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(4));

-- Location: FF_X15_Y27_N7
\inst8|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(3));

-- Location: FF_X15_Y27_N5
\inst8|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(2));

-- Location: FF_X15_Y27_N3
\inst8|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(1));

-- Location: FF_X15_Y27_N1
\inst8|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(0));

-- Location: LCCOMB_X15_Y27_N0
\inst8|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~0_combout\ = (\inst33|temp\(0) & ((GND) # (!\inst8|t_0\(0)))) # (!\inst33|temp\(0) & (\inst8|t_0\(0) $ (GND)))
-- \inst8|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst8|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010111011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(0),
	datab => \inst8|t_0\(0),
	datad => VCC,
	combout => \inst8|Add0~0_combout\,
	cout => \inst8|Add0~1\);

-- Location: LCCOMB_X15_Y27_N2
\inst8|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~2_combout\ = (\inst33|temp\(1) & ((\inst8|t_0\(1) & (!\inst8|Add0~1\)) # (!\inst8|t_0\(1) & (\inst8|Add0~1\ & VCC)))) # (!\inst33|temp\(1) & ((\inst8|t_0\(1) & ((\inst8|Add0~1\) # (GND))) # (!\inst8|t_0\(1) & (!\inst8|Add0~1\))))
-- \inst8|Add0~3\ = CARRY((\inst33|temp\(1) & (\inst8|t_0\(1) & !\inst8|Add0~1\)) # (!\inst33|temp\(1) & ((\inst8|t_0\(1)) # (!\inst8|Add0~1\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(1),
	datab => \inst8|t_0\(1),
	datad => VCC,
	cin => \inst8|Add0~1\,
	combout => \inst8|Add0~2_combout\,
	cout => \inst8|Add0~3\);

-- Location: LCCOMB_X15_Y27_N4
\inst8|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~4_combout\ = ((\inst8|t_0\(2) $ (\inst33|temp\(2) $ (\inst8|Add0~3\)))) # (GND)
-- \inst8|Add0~5\ = CARRY((\inst8|t_0\(2) & (\inst33|temp\(2) & !\inst8|Add0~3\)) # (!\inst8|t_0\(2) & ((\inst33|temp\(2)) # (!\inst8|Add0~3\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(2),
	datab => \inst33|temp\(2),
	datad => VCC,
	cin => \inst8|Add0~3\,
	combout => \inst8|Add0~4_combout\,
	cout => \inst8|Add0~5\);

-- Location: LCCOMB_X15_Y27_N6
\inst8|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~6_combout\ = (\inst8|t_0\(3) & ((\inst33|temp\(3) & (!\inst8|Add0~5\)) # (!\inst33|temp\(3) & ((\inst8|Add0~5\) # (GND))))) # (!\inst8|t_0\(3) & ((\inst33|temp\(3) & (\inst8|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst8|Add0~5\))))
-- \inst8|Add0~7\ = CARRY((\inst8|t_0\(3) & ((!\inst8|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst8|t_0\(3) & (!\inst33|temp\(3) & !\inst8|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst8|Add0~5\,
	combout => \inst8|Add0~6_combout\,
	cout => \inst8|Add0~7\);

-- Location: LCCOMB_X15_Y27_N8
\inst8|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~8_combout\ = ((\inst8|t_0\(4) $ (\inst33|temp\(4) $ (\inst8|Add0~7\)))) # (GND)
-- \inst8|Add0~9\ = CARRY((\inst8|t_0\(4) & (\inst33|temp\(4) & !\inst8|Add0~7\)) # (!\inst8|t_0\(4) & ((\inst33|temp\(4)) # (!\inst8|Add0~7\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(4),
	datab => \inst33|temp\(4),
	datad => VCC,
	cin => \inst8|Add0~7\,
	combout => \inst8|Add0~8_combout\,
	cout => \inst8|Add0~9\);

-- Location: LCCOMB_X15_Y27_N10
\inst8|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~10_combout\ = (\inst8|t_0\(5) & ((\inst33|temp\(5) & (!\inst8|Add0~9\)) # (!\inst33|temp\(5) & ((\inst8|Add0~9\) # (GND))))) # (!\inst8|t_0\(5) & ((\inst33|temp\(5) & (\inst8|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst8|Add0~9\))))
-- \inst8|Add0~11\ = CARRY((\inst8|t_0\(5) & ((!\inst8|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst8|t_0\(5) & (!\inst33|temp\(5) & !\inst8|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst8|Add0~9\,
	combout => \inst8|Add0~10_combout\,
	cout => \inst8|Add0~11\);

-- Location: LCCOMB_X15_Y27_N12
\inst8|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~12_combout\ = ((\inst33|temp\(6) $ (\inst8|t_0\(6) $ (\inst8|Add0~11\)))) # (GND)
-- \inst8|Add0~13\ = CARRY((\inst33|temp\(6) & ((!\inst8|Add0~11\) # (!\inst8|t_0\(6)))) # (!\inst33|temp\(6) & (!\inst8|t_0\(6) & !\inst8|Add0~11\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(6),
	datab => \inst8|t_0\(6),
	datad => VCC,
	cin => \inst8|Add0~11\,
	combout => \inst8|Add0~12_combout\,
	cout => \inst8|Add0~13\);

-- Location: FF_X15_Y26_N29
\inst8|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(30));

-- Location: FF_X15_Y26_N27
\inst8|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(29));

-- Location: FF_X15_Y26_N25
\inst8|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(28));

-- Location: FF_X15_Y26_N23
\inst8|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(27));

-- Location: FF_X15_Y26_N21
\inst8|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(26));

-- Location: FF_X15_Y26_N19
\inst8|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(25));

-- Location: FF_X15_Y26_N17
\inst8|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(24));

-- Location: FF_X15_Y26_N15
\inst8|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(23));

-- Location: FF_X15_Y26_N13
\inst8|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(22));

-- Location: FF_X15_Y26_N11
\inst8|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(21));

-- Location: FF_X15_Y26_N9
\inst8|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(20));

-- Location: FF_X15_Y26_N7
\inst8|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(19));

-- Location: FF_X15_Y26_N5
\inst8|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(18));

-- Location: FF_X15_Y26_N3
\inst8|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(17));

-- Location: FF_X15_Y26_N1
\inst8|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(16));

-- Location: FF_X15_Y27_N31
\inst8|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(15));

-- Location: FF_X15_Y27_N29
\inst8|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(14));

-- Location: FF_X15_Y27_N27
\inst8|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(13));

-- Location: FF_X15_Y27_N25
\inst8|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(12));

-- Location: FF_X15_Y27_N23
\inst8|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(11));

-- Location: FF_X15_Y27_N21
\inst8|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(10));

-- Location: FF_X15_Y27_N19
\inst8|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(9));

-- Location: FF_X15_Y27_N17
\inst8|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(8));

-- Location: FF_X15_Y27_N15
\inst8|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(7));

-- Location: LCCOMB_X15_Y27_N14
\inst8|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~14_combout\ = (\inst8|t_0\(7) & ((\inst33|temp\(7) & (!\inst8|Add0~13\)) # (!\inst33|temp\(7) & ((\inst8|Add0~13\) # (GND))))) # (!\inst8|t_0\(7) & ((\inst33|temp\(7) & (\inst8|Add0~13\ & VCC)) # (!\inst33|temp\(7) & (!\inst8|Add0~13\))))
-- \inst8|Add0~15\ = CARRY((\inst8|t_0\(7) & ((!\inst8|Add0~13\) # (!\inst33|temp\(7)))) # (!\inst8|t_0\(7) & (!\inst33|temp\(7) & !\inst8|Add0~13\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(7),
	datab => \inst33|temp\(7),
	datad => VCC,
	cin => \inst8|Add0~13\,
	combout => \inst8|Add0~14_combout\,
	cout => \inst8|Add0~15\);

-- Location: LCCOMB_X15_Y27_N16
\inst8|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~16_combout\ = ((\inst8|t_0\(8) $ (\inst33|temp\(8) $ (\inst8|Add0~15\)))) # (GND)
-- \inst8|Add0~17\ = CARRY((\inst8|t_0\(8) & (\inst33|temp\(8) & !\inst8|Add0~15\)) # (!\inst8|t_0\(8) & ((\inst33|temp\(8)) # (!\inst8|Add0~15\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(8),
	datab => \inst33|temp\(8),
	datad => VCC,
	cin => \inst8|Add0~15\,
	combout => \inst8|Add0~16_combout\,
	cout => \inst8|Add0~17\);

-- Location: LCCOMB_X15_Y27_N18
\inst8|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~18_combout\ = (\inst33|temp\(9) & ((\inst8|t_0\(9) & (!\inst8|Add0~17\)) # (!\inst8|t_0\(9) & (\inst8|Add0~17\ & VCC)))) # (!\inst33|temp\(9) & ((\inst8|t_0\(9) & ((\inst8|Add0~17\) # (GND))) # (!\inst8|t_0\(9) & (!\inst8|Add0~17\))))
-- \inst8|Add0~19\ = CARRY((\inst33|temp\(9) & (\inst8|t_0\(9) & !\inst8|Add0~17\)) # (!\inst33|temp\(9) & ((\inst8|t_0\(9)) # (!\inst8|Add0~17\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(9),
	datab => \inst8|t_0\(9),
	datad => VCC,
	cin => \inst8|Add0~17\,
	combout => \inst8|Add0~18_combout\,
	cout => \inst8|Add0~19\);

-- Location: LCCOMB_X15_Y27_N20
\inst8|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~20_combout\ = ((\inst33|temp\(10) $ (\inst8|t_0\(10) $ (\inst8|Add0~19\)))) # (GND)
-- \inst8|Add0~21\ = CARRY((\inst33|temp\(10) & ((!\inst8|Add0~19\) # (!\inst8|t_0\(10)))) # (!\inst33|temp\(10) & (!\inst8|t_0\(10) & !\inst8|Add0~19\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(10),
	datab => \inst8|t_0\(10),
	datad => VCC,
	cin => \inst8|Add0~19\,
	combout => \inst8|Add0~20_combout\,
	cout => \inst8|Add0~21\);

-- Location: LCCOMB_X15_Y27_N22
\inst8|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~22_combout\ = (\inst33|temp\(11) & ((\inst8|t_0\(11) & (!\inst8|Add0~21\)) # (!\inst8|t_0\(11) & (\inst8|Add0~21\ & VCC)))) # (!\inst33|temp\(11) & ((\inst8|t_0\(11) & ((\inst8|Add0~21\) # (GND))) # (!\inst8|t_0\(11) & (!\inst8|Add0~21\))))
-- \inst8|Add0~23\ = CARRY((\inst33|temp\(11) & (\inst8|t_0\(11) & !\inst8|Add0~21\)) # (!\inst33|temp\(11) & ((\inst8|t_0\(11)) # (!\inst8|Add0~21\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(11),
	datab => \inst8|t_0\(11),
	datad => VCC,
	cin => \inst8|Add0~21\,
	combout => \inst8|Add0~22_combout\,
	cout => \inst8|Add0~23\);

-- Location: LCCOMB_X15_Y27_N24
\inst8|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~24_combout\ = ((\inst33|temp\(12) $ (\inst8|t_0\(12) $ (\inst8|Add0~23\)))) # (GND)
-- \inst8|Add0~25\ = CARRY((\inst33|temp\(12) & ((!\inst8|Add0~23\) # (!\inst8|t_0\(12)))) # (!\inst33|temp\(12) & (!\inst8|t_0\(12) & !\inst8|Add0~23\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(12),
	datab => \inst8|t_0\(12),
	datad => VCC,
	cin => \inst8|Add0~23\,
	combout => \inst8|Add0~24_combout\,
	cout => \inst8|Add0~25\);

-- Location: LCCOMB_X15_Y27_N26
\inst8|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~26_combout\ = (\inst8|t_0\(13) & ((\inst33|temp\(13) & (!\inst8|Add0~25\)) # (!\inst33|temp\(13) & ((\inst8|Add0~25\) # (GND))))) # (!\inst8|t_0\(13) & ((\inst33|temp\(13) & (\inst8|Add0~25\ & VCC)) # (!\inst33|temp\(13) & 
-- (!\inst8|Add0~25\))))
-- \inst8|Add0~27\ = CARRY((\inst8|t_0\(13) & ((!\inst8|Add0~25\) # (!\inst33|temp\(13)))) # (!\inst8|t_0\(13) & (!\inst33|temp\(13) & !\inst8|Add0~25\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(13),
	datab => \inst33|temp\(13),
	datad => VCC,
	cin => \inst8|Add0~25\,
	combout => \inst8|Add0~26_combout\,
	cout => \inst8|Add0~27\);

-- Location: LCCOMB_X15_Y27_N28
\inst8|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst8|t_0\(14) $ (\inst8|Add0~27\)))) # (GND)
-- \inst8|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst8|Add0~27\) # (!\inst8|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst8|t_0\(14) & !\inst8|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst8|t_0\(14),
	datad => VCC,
	cin => \inst8|Add0~27\,
	combout => \inst8|Add0~28_combout\,
	cout => \inst8|Add0~29\);

-- Location: LCCOMB_X15_Y27_N30
\inst8|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~30_combout\ = (\inst8|t_0\(15) & ((\inst33|temp\(15) & (!\inst8|Add0~29\)) # (!\inst33|temp\(15) & ((\inst8|Add0~29\) # (GND))))) # (!\inst8|t_0\(15) & ((\inst33|temp\(15) & (\inst8|Add0~29\ & VCC)) # (!\inst33|temp\(15) & 
-- (!\inst8|Add0~29\))))
-- \inst8|Add0~31\ = CARRY((\inst8|t_0\(15) & ((!\inst8|Add0~29\) # (!\inst33|temp\(15)))) # (!\inst8|t_0\(15) & (!\inst33|temp\(15) & !\inst8|Add0~29\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(15),
	datab => \inst33|temp\(15),
	datad => VCC,
	cin => \inst8|Add0~29\,
	combout => \inst8|Add0~30_combout\,
	cout => \inst8|Add0~31\);

-- Location: LCCOMB_X15_Y26_N0
\inst8|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~32_combout\ = ((\inst8|t_0\(16) $ (\inst33|temp\(16) $ (\inst8|Add0~31\)))) # (GND)
-- \inst8|Add0~33\ = CARRY((\inst8|t_0\(16) & (\inst33|temp\(16) & !\inst8|Add0~31\)) # (!\inst8|t_0\(16) & ((\inst33|temp\(16)) # (!\inst8|Add0~31\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(16),
	datab => \inst33|temp\(16),
	datad => VCC,
	cin => \inst8|Add0~31\,
	combout => \inst8|Add0~32_combout\,
	cout => \inst8|Add0~33\);

-- Location: LCCOMB_X15_Y26_N2
\inst8|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~34_combout\ = (\inst8|t_0\(17) & ((\inst33|temp\(17) & (!\inst8|Add0~33\)) # (!\inst33|temp\(17) & ((\inst8|Add0~33\) # (GND))))) # (!\inst8|t_0\(17) & ((\inst33|temp\(17) & (\inst8|Add0~33\ & VCC)) # (!\inst33|temp\(17) & 
-- (!\inst8|Add0~33\))))
-- \inst8|Add0~35\ = CARRY((\inst8|t_0\(17) & ((!\inst8|Add0~33\) # (!\inst33|temp\(17)))) # (!\inst8|t_0\(17) & (!\inst33|temp\(17) & !\inst8|Add0~33\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(17),
	datab => \inst33|temp\(17),
	datad => VCC,
	cin => \inst8|Add0~33\,
	combout => \inst8|Add0~34_combout\,
	cout => \inst8|Add0~35\);

-- Location: LCCOMB_X15_Y26_N4
\inst8|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~36_combout\ = ((\inst8|t_0\(18) $ (\inst33|temp\(18) $ (\inst8|Add0~35\)))) # (GND)
-- \inst8|Add0~37\ = CARRY((\inst8|t_0\(18) & (\inst33|temp\(18) & !\inst8|Add0~35\)) # (!\inst8|t_0\(18) & ((\inst33|temp\(18)) # (!\inst8|Add0~35\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(18),
	datab => \inst33|temp\(18),
	datad => VCC,
	cin => \inst8|Add0~35\,
	combout => \inst8|Add0~36_combout\,
	cout => \inst8|Add0~37\);

-- Location: LCCOMB_X15_Y26_N6
\inst8|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~38_combout\ = (\inst8|t_0\(19) & ((\inst33|temp\(19) & (!\inst8|Add0~37\)) # (!\inst33|temp\(19) & ((\inst8|Add0~37\) # (GND))))) # (!\inst8|t_0\(19) & ((\inst33|temp\(19) & (\inst8|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst8|Add0~37\))))
-- \inst8|Add0~39\ = CARRY((\inst8|t_0\(19) & ((!\inst8|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst8|t_0\(19) & (!\inst33|temp\(19) & !\inst8|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst8|Add0~37\,
	combout => \inst8|Add0~38_combout\,
	cout => \inst8|Add0~39\);

-- Location: LCCOMB_X15_Y26_N8
\inst8|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~40_combout\ = ((\inst8|t_0\(20) $ (\inst33|temp\(20) $ (\inst8|Add0~39\)))) # (GND)
-- \inst8|Add0~41\ = CARRY((\inst8|t_0\(20) & (\inst33|temp\(20) & !\inst8|Add0~39\)) # (!\inst8|t_0\(20) & ((\inst33|temp\(20)) # (!\inst8|Add0~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(20),
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst8|Add0~39\,
	combout => \inst8|Add0~40_combout\,
	cout => \inst8|Add0~41\);

-- Location: LCCOMB_X15_Y26_N10
\inst8|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~42_combout\ = (\inst8|t_0\(21) & ((\inst33|temp\(21) & (!\inst8|Add0~41\)) # (!\inst33|temp\(21) & ((\inst8|Add0~41\) # (GND))))) # (!\inst8|t_0\(21) & ((\inst33|temp\(21) & (\inst8|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst8|Add0~41\))))
-- \inst8|Add0~43\ = CARRY((\inst8|t_0\(21) & ((!\inst8|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst8|t_0\(21) & (!\inst33|temp\(21) & !\inst8|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst8|Add0~41\,
	combout => \inst8|Add0~42_combout\,
	cout => \inst8|Add0~43\);

-- Location: LCCOMB_X15_Y26_N12
\inst8|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~44_combout\ = ((\inst8|t_0\(22) $ (\inst33|temp\(22) $ (\inst8|Add0~43\)))) # (GND)
-- \inst8|Add0~45\ = CARRY((\inst8|t_0\(22) & (\inst33|temp\(22) & !\inst8|Add0~43\)) # (!\inst8|t_0\(22) & ((\inst33|temp\(22)) # (!\inst8|Add0~43\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(22),
	datab => \inst33|temp\(22),
	datad => VCC,
	cin => \inst8|Add0~43\,
	combout => \inst8|Add0~44_combout\,
	cout => \inst8|Add0~45\);

-- Location: LCCOMB_X15_Y26_N14
\inst8|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~46_combout\ = (\inst8|t_0\(23) & ((\inst33|temp\(23) & (!\inst8|Add0~45\)) # (!\inst33|temp\(23) & ((\inst8|Add0~45\) # (GND))))) # (!\inst8|t_0\(23) & ((\inst33|temp\(23) & (\inst8|Add0~45\ & VCC)) # (!\inst33|temp\(23) & 
-- (!\inst8|Add0~45\))))
-- \inst8|Add0~47\ = CARRY((\inst8|t_0\(23) & ((!\inst8|Add0~45\) # (!\inst33|temp\(23)))) # (!\inst8|t_0\(23) & (!\inst33|temp\(23) & !\inst8|Add0~45\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(23),
	datab => \inst33|temp\(23),
	datad => VCC,
	cin => \inst8|Add0~45\,
	combout => \inst8|Add0~46_combout\,
	cout => \inst8|Add0~47\);

-- Location: LCCOMB_X15_Y26_N16
\inst8|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~48_combout\ = ((\inst33|temp\(24) $ (\inst8|t_0\(24) $ (\inst8|Add0~47\)))) # (GND)
-- \inst8|Add0~49\ = CARRY((\inst33|temp\(24) & ((!\inst8|Add0~47\) # (!\inst8|t_0\(24)))) # (!\inst33|temp\(24) & (!\inst8|t_0\(24) & !\inst8|Add0~47\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(24),
	datab => \inst8|t_0\(24),
	datad => VCC,
	cin => \inst8|Add0~47\,
	combout => \inst8|Add0~48_combout\,
	cout => \inst8|Add0~49\);

-- Location: LCCOMB_X15_Y26_N18
\inst8|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst8|t_0\(25) & (!\inst8|Add0~49\)) # (!\inst8|t_0\(25) & (\inst8|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst8|t_0\(25) & ((\inst8|Add0~49\) # (GND))) # (!\inst8|t_0\(25) & (!\inst8|Add0~49\))))
-- \inst8|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst8|t_0\(25) & !\inst8|Add0~49\)) # (!\inst33|temp\(25) & ((\inst8|t_0\(25)) # (!\inst8|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst8|t_0\(25),
	datad => VCC,
	cin => \inst8|Add0~49\,
	combout => \inst8|Add0~50_combout\,
	cout => \inst8|Add0~51\);

-- Location: LCCOMB_X15_Y26_N20
\inst8|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~52_combout\ = ((\inst33|temp\(26) $ (\inst8|t_0\(26) $ (\inst8|Add0~51\)))) # (GND)
-- \inst8|Add0~53\ = CARRY((\inst33|temp\(26) & ((!\inst8|Add0~51\) # (!\inst8|t_0\(26)))) # (!\inst33|temp\(26) & (!\inst8|t_0\(26) & !\inst8|Add0~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(26),
	datab => \inst8|t_0\(26),
	datad => VCC,
	cin => \inst8|Add0~51\,
	combout => \inst8|Add0~52_combout\,
	cout => \inst8|Add0~53\);

-- Location: LCCOMB_X15_Y26_N22
\inst8|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~54_combout\ = (\inst33|temp\(27) & ((\inst8|t_0\(27) & (!\inst8|Add0~53\)) # (!\inst8|t_0\(27) & (\inst8|Add0~53\ & VCC)))) # (!\inst33|temp\(27) & ((\inst8|t_0\(27) & ((\inst8|Add0~53\) # (GND))) # (!\inst8|t_0\(27) & (!\inst8|Add0~53\))))
-- \inst8|Add0~55\ = CARRY((\inst33|temp\(27) & (\inst8|t_0\(27) & !\inst8|Add0~53\)) # (!\inst33|temp\(27) & ((\inst8|t_0\(27)) # (!\inst8|Add0~53\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(27),
	datab => \inst8|t_0\(27),
	datad => VCC,
	cin => \inst8|Add0~53\,
	combout => \inst8|Add0~54_combout\,
	cout => \inst8|Add0~55\);

-- Location: LCCOMB_X15_Y26_N24
\inst8|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~56_combout\ = ((\inst33|temp\(28) $ (\inst8|t_0\(28) $ (\inst8|Add0~55\)))) # (GND)
-- \inst8|Add0~57\ = CARRY((\inst33|temp\(28) & ((!\inst8|Add0~55\) # (!\inst8|t_0\(28)))) # (!\inst33|temp\(28) & (!\inst8|t_0\(28) & !\inst8|Add0~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(28),
	datab => \inst8|t_0\(28),
	datad => VCC,
	cin => \inst8|Add0~55\,
	combout => \inst8|Add0~56_combout\,
	cout => \inst8|Add0~57\);

-- Location: LCCOMB_X15_Y26_N26
\inst8|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~58_combout\ = (\inst33|temp\(29) & ((\inst8|t_0\(29) & (!\inst8|Add0~57\)) # (!\inst8|t_0\(29) & (\inst8|Add0~57\ & VCC)))) # (!\inst33|temp\(29) & ((\inst8|t_0\(29) & ((\inst8|Add0~57\) # (GND))) # (!\inst8|t_0\(29) & (!\inst8|Add0~57\))))
-- \inst8|Add0~59\ = CARRY((\inst33|temp\(29) & (\inst8|t_0\(29) & !\inst8|Add0~57\)) # (!\inst33|temp\(29) & ((\inst8|t_0\(29)) # (!\inst8|Add0~57\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(29),
	datab => \inst8|t_0\(29),
	datad => VCC,
	cin => \inst8|Add0~57\,
	combout => \inst8|Add0~58_combout\,
	cout => \inst8|Add0~59\);

-- Location: LCCOMB_X15_Y26_N28
\inst8|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst8|t_0\(30) $ (\inst8|Add0~59\)))) # (GND)
-- \inst8|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst8|Add0~59\) # (!\inst8|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst8|t_0\(30) & !\inst8|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst8|t_0\(30),
	datad => VCC,
	cin => \inst8|Add0~59\,
	combout => \inst8|Add0~60_combout\,
	cout => \inst8|Add0~61\);

-- Location: LCCOMB_X14_Y26_N2
\inst8|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~6_combout\ = (!\inst8|Add0~60_combout\ & (!\inst8|Add0~54_combout\ & (!\inst8|Add0~56_combout\ & !\inst8|Add0~58_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~60_combout\,
	datab => \inst8|Add0~54_combout\,
	datac => \inst8|Add0~56_combout\,
	datad => \inst8|Add0~58_combout\,
	combout => \inst8|data_available~6_combout\);

-- Location: LCCOMB_X14_Y27_N12
\inst8|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~1_combout\ = (!\inst8|Add0~22_combout\ & (!\inst8|Add0~28_combout\ & (!\inst8|Add0~26_combout\ & !\inst8|Add0~24_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~22_combout\,
	datab => \inst8|Add0~28_combout\,
	datac => \inst8|Add0~26_combout\,
	datad => \inst8|Add0~24_combout\,
	combout => \inst8|data_available~1_combout\);

-- Location: LCCOMB_X14_Y26_N0
\inst8|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~3_combout\ = (!\inst8|Add0~44_combout\ & (!\inst8|Add0~42_combout\ & (!\inst8|Add0~40_combout\ & !\inst8|Add0~38_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~44_combout\,
	datab => \inst8|Add0~42_combout\,
	datac => \inst8|Add0~40_combout\,
	datad => \inst8|Add0~38_combout\,
	combout => \inst8|data_available~3_combout\);

-- Location: LCCOMB_X14_Y27_N22
\inst8|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~2_combout\ = (!\inst8|Add0~36_combout\ & (!\inst8|Add0~34_combout\ & (!\inst8|Add0~30_combout\ & !\inst8|Add0~32_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~36_combout\,
	datab => \inst8|Add0~34_combout\,
	datac => \inst8|Add0~30_combout\,
	datad => \inst8|Add0~32_combout\,
	combout => \inst8|data_available~2_combout\);

-- Location: LCCOMB_X14_Y27_N10
\inst8|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~0_combout\ = (!\inst8|Add0~20_combout\ & (!\inst8|Add0~18_combout\ & (!\inst8|Add0~14_combout\ & !\inst8|Add0~16_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~20_combout\,
	datab => \inst8|Add0~18_combout\,
	datac => \inst8|Add0~14_combout\,
	datad => \inst8|Add0~16_combout\,
	combout => \inst8|data_available~0_combout\);

-- Location: LCCOMB_X14_Y27_N16
\inst8|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~4_combout\ = (\inst8|data_available~1_combout\ & (\inst8|data_available~3_combout\ & (\inst8|data_available~2_combout\ & \inst8|data_available~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|data_available~1_combout\,
	datab => \inst8|data_available~3_combout\,
	datac => \inst8|data_available~2_combout\,
	datad => \inst8|data_available~0_combout\,
	combout => \inst8|data_available~4_combout\);

-- Location: LCCOMB_X14_Y27_N26
\inst8|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~5_combout\ = (!\inst8|Add0~50_combout\ & (!\inst8|Add0~48_combout\ & (!\inst8|Add0~46_combout\ & !\inst8|Add0~52_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~50_combout\,
	datab => \inst8|Add0~48_combout\,
	datac => \inst8|Add0~46_combout\,
	datad => \inst8|Add0~52_combout\,
	combout => \inst8|data_available~5_combout\);

-- Location: FF_X15_Y26_N31
\inst8|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor2_signal~input_o\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|t_0\(31));

-- Location: LCCOMB_X15_Y26_N30
\inst8|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|Add0~62_combout\ = \inst8|t_0\(31) $ (\inst33|temp\(31) $ (!\inst8|Add0~61\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101101001",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|t_0\(31),
	datab => \inst33|temp\(31),
	cin => \inst8|Add0~61\,
	combout => \inst8|Add0~62_combout\);

-- Location: LCCOMB_X14_Y27_N20
\inst8|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~7_combout\ = (\inst8|data_available~6_combout\ & (\inst8|data_available~4_combout\ & (\inst8|data_available~5_combout\ & !\inst8|Add0~62_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000010000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|data_available~6_combout\,
	datab => \inst8|data_available~4_combout\,
	datac => \inst8|data_available~5_combout\,
	datad => \inst8|Add0~62_combout\,
	combout => \inst8|data_available~7_combout\);

-- Location: LCCOMB_X14_Y27_N6
\inst8|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~8_combout\ = (\inst8|Add0~4_combout\) # ((\inst8|Add0~2_combout\ & \inst8|Add0~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111110100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~2_combout\,
	datac => \inst8|Add0~0_combout\,
	datad => \inst8|Add0~4_combout\,
	combout => \inst8|data_available~8_combout\);

-- Location: LCCOMB_X14_Y27_N0
\inst8|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~9_combout\ = (\inst8|data_available~8_combout\ & ((\inst8|Add0~12_combout\) # ((\inst8|Add0~8_combout\ & \inst8|Add0~6_combout\)))) # (!\inst8|data_available~8_combout\ & (\inst8|Add0~12_combout\ & ((\inst8|Add0~8_combout\) # 
-- (\inst8|Add0~6_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111010000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|data_available~8_combout\,
	datab => \inst8|Add0~8_combout\,
	datac => \inst8|Add0~6_combout\,
	datad => \inst8|Add0~12_combout\,
	combout => \inst8|data_available~9_combout\);

-- Location: LCCOMB_X14_Y27_N24
\inst8|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst8|data_available~10_combout\ = (\inst8|data_available~7_combout\ & (\inst8|Add0~12_combout\ $ (((\inst8|Add0~10_combout\ & \inst8|data_available~9_combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100100010001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst8|Add0~12_combout\,
	datab => \inst8|data_available~7_combout\,
	datac => \inst8|Add0~10_combout\,
	datad => \inst8|data_available~9_combout\,
	combout => \inst8|data_available~10_combout\);

-- Location: FF_X14_Y27_N25
\inst8|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor2_signal~input_o\,
	d => \inst8|data_available~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst8|data_available~q\);

-- Location: FF_X26_Y26_N5
\inst19|cur_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst8|data_available~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst19|cur_value~q\);

-- Location: FF_X26_Y26_N7
\inst19|last_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst19|cur_value~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst19|last_value~q\);

-- Location: LCCOMB_X26_Y26_N6
\inst19|level_sig\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst19|level_sig~combout\ = (\inst19|cur_value~q\ & !\inst19|last_value~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000110000001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst19|cur_value~q\,
	datac => \inst19|last_value~q\,
	combout => \inst19|level_sig~combout\);

-- Location: IOIBUF_X5_Y34_N15
\sensor0_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor0_signal,
	o => \sensor0_signal~input_o\);

-- Location: FF_X16_Y29_N23
\inst6|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(27));

-- Location: FF_X16_Y29_N21
\inst6|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(26));

-- Location: FF_X16_Y29_N19
\inst6|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(25));

-- Location: FF_X16_Y29_N17
\inst6|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(24));

-- Location: FF_X16_Y29_N15
\inst6|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(23));

-- Location: FF_X16_Y29_N13
\inst6|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(22));

-- Location: FF_X16_Y29_N11
\inst6|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(21));

-- Location: FF_X16_Y29_N9
\inst6|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(20));

-- Location: FF_X16_Y29_N7
\inst6|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(19));

-- Location: FF_X16_Y29_N5
\inst6|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(18));

-- Location: FF_X16_Y29_N3
\inst6|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(17));

-- Location: FF_X16_Y29_N1
\inst6|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(16));

-- Location: FF_X16_Y30_N31
\inst6|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(15));

-- Location: FF_X16_Y30_N29
\inst6|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(14));

-- Location: FF_X16_Y30_N27
\inst6|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(13));

-- Location: FF_X16_Y30_N25
\inst6|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(12));

-- Location: FF_X16_Y30_N23
\inst6|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(11));

-- Location: FF_X16_Y30_N21
\inst6|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(10));

-- Location: FF_X16_Y30_N19
\inst6|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(9));

-- Location: FF_X16_Y30_N17
\inst6|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(8));

-- Location: FF_X16_Y30_N15
\inst6|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(7));

-- Location: FF_X16_Y30_N13
\inst6|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(6));

-- Location: FF_X16_Y30_N11
\inst6|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(5));

-- Location: FF_X16_Y30_N9
\inst6|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(4));

-- Location: FF_X16_Y30_N7
\inst6|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(3));

-- Location: FF_X16_Y30_N5
\inst6|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(2));

-- Location: FF_X16_Y30_N3
\inst6|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(1));

-- Location: FF_X16_Y30_N1
\inst6|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(0));

-- Location: LCCOMB_X16_Y30_N0
\inst6|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~0_combout\ = (\inst33|temp\(0) & ((GND) # (!\inst6|t_0\(0)))) # (!\inst33|temp\(0) & (\inst6|t_0\(0) $ (GND)))
-- \inst6|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst6|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010111011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(0),
	datab => \inst6|t_0\(0),
	datad => VCC,
	combout => \inst6|Add0~0_combout\,
	cout => \inst6|Add0~1\);

-- Location: LCCOMB_X16_Y30_N2
\inst6|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~2_combout\ = (\inst6|t_0\(1) & ((\inst33|temp\(1) & (!\inst6|Add0~1\)) # (!\inst33|temp\(1) & ((\inst6|Add0~1\) # (GND))))) # (!\inst6|t_0\(1) & ((\inst33|temp\(1) & (\inst6|Add0~1\ & VCC)) # (!\inst33|temp\(1) & (!\inst6|Add0~1\))))
-- \inst6|Add0~3\ = CARRY((\inst6|t_0\(1) & ((!\inst6|Add0~1\) # (!\inst33|temp\(1)))) # (!\inst6|t_0\(1) & (!\inst33|temp\(1) & !\inst6|Add0~1\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(1),
	datab => \inst33|temp\(1),
	datad => VCC,
	cin => \inst6|Add0~1\,
	combout => \inst6|Add0~2_combout\,
	cout => \inst6|Add0~3\);

-- Location: LCCOMB_X16_Y30_N4
\inst6|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~4_combout\ = ((\inst33|temp\(2) $ (\inst6|t_0\(2) $ (\inst6|Add0~3\)))) # (GND)
-- \inst6|Add0~5\ = CARRY((\inst33|temp\(2) & ((!\inst6|Add0~3\) # (!\inst6|t_0\(2)))) # (!\inst33|temp\(2) & (!\inst6|t_0\(2) & !\inst6|Add0~3\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(2),
	datab => \inst6|t_0\(2),
	datad => VCC,
	cin => \inst6|Add0~3\,
	combout => \inst6|Add0~4_combout\,
	cout => \inst6|Add0~5\);

-- Location: LCCOMB_X16_Y30_N6
\inst6|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~6_combout\ = (\inst6|t_0\(3) & ((\inst33|temp\(3) & (!\inst6|Add0~5\)) # (!\inst33|temp\(3) & ((\inst6|Add0~5\) # (GND))))) # (!\inst6|t_0\(3) & ((\inst33|temp\(3) & (\inst6|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst6|Add0~5\))))
-- \inst6|Add0~7\ = CARRY((\inst6|t_0\(3) & ((!\inst6|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst6|t_0\(3) & (!\inst33|temp\(3) & !\inst6|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst6|Add0~5\,
	combout => \inst6|Add0~6_combout\,
	cout => \inst6|Add0~7\);

-- Location: LCCOMB_X16_Y30_N8
\inst6|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~8_combout\ = ((\inst6|t_0\(4) $ (\inst33|temp\(4) $ (\inst6|Add0~7\)))) # (GND)
-- \inst6|Add0~9\ = CARRY((\inst6|t_0\(4) & (\inst33|temp\(4) & !\inst6|Add0~7\)) # (!\inst6|t_0\(4) & ((\inst33|temp\(4)) # (!\inst6|Add0~7\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(4),
	datab => \inst33|temp\(4),
	datad => VCC,
	cin => \inst6|Add0~7\,
	combout => \inst6|Add0~8_combout\,
	cout => \inst6|Add0~9\);

-- Location: LCCOMB_X16_Y30_N10
\inst6|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~10_combout\ = (\inst6|t_0\(5) & ((\inst33|temp\(5) & (!\inst6|Add0~9\)) # (!\inst33|temp\(5) & ((\inst6|Add0~9\) # (GND))))) # (!\inst6|t_0\(5) & ((\inst33|temp\(5) & (\inst6|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst6|Add0~9\))))
-- \inst6|Add0~11\ = CARRY((\inst6|t_0\(5) & ((!\inst6|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst6|t_0\(5) & (!\inst33|temp\(5) & !\inst6|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst6|Add0~9\,
	combout => \inst6|Add0~10_combout\,
	cout => \inst6|Add0~11\);

-- Location: LCCOMB_X16_Y30_N12
\inst6|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~12_combout\ = ((\inst6|t_0\(6) $ (\inst33|temp\(6) $ (\inst6|Add0~11\)))) # (GND)
-- \inst6|Add0~13\ = CARRY((\inst6|t_0\(6) & (\inst33|temp\(6) & !\inst6|Add0~11\)) # (!\inst6|t_0\(6) & ((\inst33|temp\(6)) # (!\inst6|Add0~11\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(6),
	datab => \inst33|temp\(6),
	datad => VCC,
	cin => \inst6|Add0~11\,
	combout => \inst6|Add0~12_combout\,
	cout => \inst6|Add0~13\);

-- Location: LCCOMB_X16_Y30_N14
\inst6|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~14_combout\ = (\inst6|t_0\(7) & ((\inst33|temp\(7) & (!\inst6|Add0~13\)) # (!\inst33|temp\(7) & ((\inst6|Add0~13\) # (GND))))) # (!\inst6|t_0\(7) & ((\inst33|temp\(7) & (\inst6|Add0~13\ & VCC)) # (!\inst33|temp\(7) & (!\inst6|Add0~13\))))
-- \inst6|Add0~15\ = CARRY((\inst6|t_0\(7) & ((!\inst6|Add0~13\) # (!\inst33|temp\(7)))) # (!\inst6|t_0\(7) & (!\inst33|temp\(7) & !\inst6|Add0~13\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(7),
	datab => \inst33|temp\(7),
	datad => VCC,
	cin => \inst6|Add0~13\,
	combout => \inst6|Add0~14_combout\,
	cout => \inst6|Add0~15\);

-- Location: LCCOMB_X16_Y30_N16
\inst6|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~16_combout\ = ((\inst6|t_0\(8) $ (\inst33|temp\(8) $ (\inst6|Add0~15\)))) # (GND)
-- \inst6|Add0~17\ = CARRY((\inst6|t_0\(8) & (\inst33|temp\(8) & !\inst6|Add0~15\)) # (!\inst6|t_0\(8) & ((\inst33|temp\(8)) # (!\inst6|Add0~15\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(8),
	datab => \inst33|temp\(8),
	datad => VCC,
	cin => \inst6|Add0~15\,
	combout => \inst6|Add0~16_combout\,
	cout => \inst6|Add0~17\);

-- Location: LCCOMB_X16_Y30_N18
\inst6|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~18_combout\ = (\inst33|temp\(9) & ((\inst6|t_0\(9) & (!\inst6|Add0~17\)) # (!\inst6|t_0\(9) & (\inst6|Add0~17\ & VCC)))) # (!\inst33|temp\(9) & ((\inst6|t_0\(9) & ((\inst6|Add0~17\) # (GND))) # (!\inst6|t_0\(9) & (!\inst6|Add0~17\))))
-- \inst6|Add0~19\ = CARRY((\inst33|temp\(9) & (\inst6|t_0\(9) & !\inst6|Add0~17\)) # (!\inst33|temp\(9) & ((\inst6|t_0\(9)) # (!\inst6|Add0~17\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(9),
	datab => \inst6|t_0\(9),
	datad => VCC,
	cin => \inst6|Add0~17\,
	combout => \inst6|Add0~18_combout\,
	cout => \inst6|Add0~19\);

-- Location: LCCOMB_X16_Y30_N20
\inst6|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~20_combout\ = ((\inst33|temp\(10) $ (\inst6|t_0\(10) $ (\inst6|Add0~19\)))) # (GND)
-- \inst6|Add0~21\ = CARRY((\inst33|temp\(10) & ((!\inst6|Add0~19\) # (!\inst6|t_0\(10)))) # (!\inst33|temp\(10) & (!\inst6|t_0\(10) & !\inst6|Add0~19\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(10),
	datab => \inst6|t_0\(10),
	datad => VCC,
	cin => \inst6|Add0~19\,
	combout => \inst6|Add0~20_combout\,
	cout => \inst6|Add0~21\);

-- Location: LCCOMB_X16_Y30_N22
\inst6|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~22_combout\ = (\inst33|temp\(11) & ((\inst6|t_0\(11) & (!\inst6|Add0~21\)) # (!\inst6|t_0\(11) & (\inst6|Add0~21\ & VCC)))) # (!\inst33|temp\(11) & ((\inst6|t_0\(11) & ((\inst6|Add0~21\) # (GND))) # (!\inst6|t_0\(11) & (!\inst6|Add0~21\))))
-- \inst6|Add0~23\ = CARRY((\inst33|temp\(11) & (\inst6|t_0\(11) & !\inst6|Add0~21\)) # (!\inst33|temp\(11) & ((\inst6|t_0\(11)) # (!\inst6|Add0~21\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(11),
	datab => \inst6|t_0\(11),
	datad => VCC,
	cin => \inst6|Add0~21\,
	combout => \inst6|Add0~22_combout\,
	cout => \inst6|Add0~23\);

-- Location: LCCOMB_X16_Y30_N24
\inst6|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~24_combout\ = ((\inst33|temp\(12) $ (\inst6|t_0\(12) $ (\inst6|Add0~23\)))) # (GND)
-- \inst6|Add0~25\ = CARRY((\inst33|temp\(12) & ((!\inst6|Add0~23\) # (!\inst6|t_0\(12)))) # (!\inst33|temp\(12) & (!\inst6|t_0\(12) & !\inst6|Add0~23\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(12),
	datab => \inst6|t_0\(12),
	datad => VCC,
	cin => \inst6|Add0~23\,
	combout => \inst6|Add0~24_combout\,
	cout => \inst6|Add0~25\);

-- Location: LCCOMB_X16_Y30_N26
\inst6|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~26_combout\ = (\inst33|temp\(13) & ((\inst6|t_0\(13) & (!\inst6|Add0~25\)) # (!\inst6|t_0\(13) & (\inst6|Add0~25\ & VCC)))) # (!\inst33|temp\(13) & ((\inst6|t_0\(13) & ((\inst6|Add0~25\) # (GND))) # (!\inst6|t_0\(13) & (!\inst6|Add0~25\))))
-- \inst6|Add0~27\ = CARRY((\inst33|temp\(13) & (\inst6|t_0\(13) & !\inst6|Add0~25\)) # (!\inst33|temp\(13) & ((\inst6|t_0\(13)) # (!\inst6|Add0~25\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(13),
	datab => \inst6|t_0\(13),
	datad => VCC,
	cin => \inst6|Add0~25\,
	combout => \inst6|Add0~26_combout\,
	cout => \inst6|Add0~27\);

-- Location: LCCOMB_X16_Y30_N28
\inst6|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst6|t_0\(14) $ (\inst6|Add0~27\)))) # (GND)
-- \inst6|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst6|Add0~27\) # (!\inst6|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst6|t_0\(14) & !\inst6|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst6|t_0\(14),
	datad => VCC,
	cin => \inst6|Add0~27\,
	combout => \inst6|Add0~28_combout\,
	cout => \inst6|Add0~29\);

-- Location: LCCOMB_X16_Y30_N30
\inst6|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~30_combout\ = (\inst33|temp\(15) & ((\inst6|t_0\(15) & (!\inst6|Add0~29\)) # (!\inst6|t_0\(15) & (\inst6|Add0~29\ & VCC)))) # (!\inst33|temp\(15) & ((\inst6|t_0\(15) & ((\inst6|Add0~29\) # (GND))) # (!\inst6|t_0\(15) & (!\inst6|Add0~29\))))
-- \inst6|Add0~31\ = CARRY((\inst33|temp\(15) & (\inst6|t_0\(15) & !\inst6|Add0~29\)) # (!\inst33|temp\(15) & ((\inst6|t_0\(15)) # (!\inst6|Add0~29\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(15),
	datab => \inst6|t_0\(15),
	datad => VCC,
	cin => \inst6|Add0~29\,
	combout => \inst6|Add0~30_combout\,
	cout => \inst6|Add0~31\);

-- Location: LCCOMB_X16_Y29_N0
\inst6|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~32_combout\ = ((\inst33|temp\(16) $ (\inst6|t_0\(16) $ (\inst6|Add0~31\)))) # (GND)
-- \inst6|Add0~33\ = CARRY((\inst33|temp\(16) & ((!\inst6|Add0~31\) # (!\inst6|t_0\(16)))) # (!\inst33|temp\(16) & (!\inst6|t_0\(16) & !\inst6|Add0~31\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(16),
	datab => \inst6|t_0\(16),
	datad => VCC,
	cin => \inst6|Add0~31\,
	combout => \inst6|Add0~32_combout\,
	cout => \inst6|Add0~33\);

-- Location: LCCOMB_X16_Y29_N2
\inst6|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~34_combout\ = (\inst33|temp\(17) & ((\inst6|t_0\(17) & (!\inst6|Add0~33\)) # (!\inst6|t_0\(17) & (\inst6|Add0~33\ & VCC)))) # (!\inst33|temp\(17) & ((\inst6|t_0\(17) & ((\inst6|Add0~33\) # (GND))) # (!\inst6|t_0\(17) & (!\inst6|Add0~33\))))
-- \inst6|Add0~35\ = CARRY((\inst33|temp\(17) & (\inst6|t_0\(17) & !\inst6|Add0~33\)) # (!\inst33|temp\(17) & ((\inst6|t_0\(17)) # (!\inst6|Add0~33\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(17),
	datab => \inst6|t_0\(17),
	datad => VCC,
	cin => \inst6|Add0~33\,
	combout => \inst6|Add0~34_combout\,
	cout => \inst6|Add0~35\);

-- Location: LCCOMB_X16_Y29_N4
\inst6|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~36_combout\ = ((\inst33|temp\(18) $ (\inst6|t_0\(18) $ (\inst6|Add0~35\)))) # (GND)
-- \inst6|Add0~37\ = CARRY((\inst33|temp\(18) & ((!\inst6|Add0~35\) # (!\inst6|t_0\(18)))) # (!\inst33|temp\(18) & (!\inst6|t_0\(18) & !\inst6|Add0~35\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(18),
	datab => \inst6|t_0\(18),
	datad => VCC,
	cin => \inst6|Add0~35\,
	combout => \inst6|Add0~36_combout\,
	cout => \inst6|Add0~37\);

-- Location: LCCOMB_X16_Y29_N6
\inst6|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~38_combout\ = (\inst6|t_0\(19) & ((\inst33|temp\(19) & (!\inst6|Add0~37\)) # (!\inst33|temp\(19) & ((\inst6|Add0~37\) # (GND))))) # (!\inst6|t_0\(19) & ((\inst33|temp\(19) & (\inst6|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst6|Add0~37\))))
-- \inst6|Add0~39\ = CARRY((\inst6|t_0\(19) & ((!\inst6|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst6|t_0\(19) & (!\inst33|temp\(19) & !\inst6|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst6|Add0~37\,
	combout => \inst6|Add0~38_combout\,
	cout => \inst6|Add0~39\);

-- Location: LCCOMB_X16_Y29_N8
\inst6|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~40_combout\ = ((\inst6|t_0\(20) $ (\inst33|temp\(20) $ (\inst6|Add0~39\)))) # (GND)
-- \inst6|Add0~41\ = CARRY((\inst6|t_0\(20) & (\inst33|temp\(20) & !\inst6|Add0~39\)) # (!\inst6|t_0\(20) & ((\inst33|temp\(20)) # (!\inst6|Add0~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(20),
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst6|Add0~39\,
	combout => \inst6|Add0~40_combout\,
	cout => \inst6|Add0~41\);

-- Location: LCCOMB_X16_Y29_N10
\inst6|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~42_combout\ = (\inst6|t_0\(21) & ((\inst33|temp\(21) & (!\inst6|Add0~41\)) # (!\inst33|temp\(21) & ((\inst6|Add0~41\) # (GND))))) # (!\inst6|t_0\(21) & ((\inst33|temp\(21) & (\inst6|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst6|Add0~41\))))
-- \inst6|Add0~43\ = CARRY((\inst6|t_0\(21) & ((!\inst6|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst6|t_0\(21) & (!\inst33|temp\(21) & !\inst6|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst6|Add0~41\,
	combout => \inst6|Add0~42_combout\,
	cout => \inst6|Add0~43\);

-- Location: LCCOMB_X16_Y29_N12
\inst6|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~44_combout\ = ((\inst6|t_0\(22) $ (\inst33|temp\(22) $ (\inst6|Add0~43\)))) # (GND)
-- \inst6|Add0~45\ = CARRY((\inst6|t_0\(22) & (\inst33|temp\(22) & !\inst6|Add0~43\)) # (!\inst6|t_0\(22) & ((\inst33|temp\(22)) # (!\inst6|Add0~43\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(22),
	datab => \inst33|temp\(22),
	datad => VCC,
	cin => \inst6|Add0~43\,
	combout => \inst6|Add0~44_combout\,
	cout => \inst6|Add0~45\);

-- Location: LCCOMB_X16_Y29_N14
\inst6|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~46_combout\ = (\inst6|t_0\(23) & ((\inst33|temp\(23) & (!\inst6|Add0~45\)) # (!\inst33|temp\(23) & ((\inst6|Add0~45\) # (GND))))) # (!\inst6|t_0\(23) & ((\inst33|temp\(23) & (\inst6|Add0~45\ & VCC)) # (!\inst33|temp\(23) & 
-- (!\inst6|Add0~45\))))
-- \inst6|Add0~47\ = CARRY((\inst6|t_0\(23) & ((!\inst6|Add0~45\) # (!\inst33|temp\(23)))) # (!\inst6|t_0\(23) & (!\inst33|temp\(23) & !\inst6|Add0~45\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(23),
	datab => \inst33|temp\(23),
	datad => VCC,
	cin => \inst6|Add0~45\,
	combout => \inst6|Add0~46_combout\,
	cout => \inst6|Add0~47\);

-- Location: LCCOMB_X16_Y29_N16
\inst6|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~48_combout\ = ((\inst6|t_0\(24) $ (\inst33|temp\(24) $ (\inst6|Add0~47\)))) # (GND)
-- \inst6|Add0~49\ = CARRY((\inst6|t_0\(24) & (\inst33|temp\(24) & !\inst6|Add0~47\)) # (!\inst6|t_0\(24) & ((\inst33|temp\(24)) # (!\inst6|Add0~47\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(24),
	datab => \inst33|temp\(24),
	datad => VCC,
	cin => \inst6|Add0~47\,
	combout => \inst6|Add0~48_combout\,
	cout => \inst6|Add0~49\);

-- Location: LCCOMB_X16_Y29_N18
\inst6|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst6|t_0\(25) & (!\inst6|Add0~49\)) # (!\inst6|t_0\(25) & (\inst6|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst6|t_0\(25) & ((\inst6|Add0~49\) # (GND))) # (!\inst6|t_0\(25) & (!\inst6|Add0~49\))))
-- \inst6|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst6|t_0\(25) & !\inst6|Add0~49\)) # (!\inst33|temp\(25) & ((\inst6|t_0\(25)) # (!\inst6|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst6|t_0\(25),
	datad => VCC,
	cin => \inst6|Add0~49\,
	combout => \inst6|Add0~50_combout\,
	cout => \inst6|Add0~51\);

-- Location: LCCOMB_X16_Y29_N20
\inst6|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~52_combout\ = ((\inst6|t_0\(26) $ (\inst33|temp\(26) $ (\inst6|Add0~51\)))) # (GND)
-- \inst6|Add0~53\ = CARRY((\inst6|t_0\(26) & (\inst33|temp\(26) & !\inst6|Add0~51\)) # (!\inst6|t_0\(26) & ((\inst33|temp\(26)) # (!\inst6|Add0~51\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(26),
	datab => \inst33|temp\(26),
	datad => VCC,
	cin => \inst6|Add0~51\,
	combout => \inst6|Add0~52_combout\,
	cout => \inst6|Add0~53\);

-- Location: LCCOMB_X16_Y29_N22
\inst6|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~54_combout\ = (\inst33|temp\(27) & ((\inst6|t_0\(27) & (!\inst6|Add0~53\)) # (!\inst6|t_0\(27) & (\inst6|Add0~53\ & VCC)))) # (!\inst33|temp\(27) & ((\inst6|t_0\(27) & ((\inst6|Add0~53\) # (GND))) # (!\inst6|t_0\(27) & (!\inst6|Add0~53\))))
-- \inst6|Add0~55\ = CARRY((\inst33|temp\(27) & (\inst6|t_0\(27) & !\inst6|Add0~53\)) # (!\inst33|temp\(27) & ((\inst6|t_0\(27)) # (!\inst6|Add0~53\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(27),
	datab => \inst6|t_0\(27),
	datad => VCC,
	cin => \inst6|Add0~53\,
	combout => \inst6|Add0~54_combout\,
	cout => \inst6|Add0~55\);

-- Location: FF_X16_Y29_N29
\inst6|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(30));

-- Location: FF_X16_Y29_N27
\inst6|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(29));

-- Location: FF_X16_Y29_N25
\inst6|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(28));

-- Location: LCCOMB_X16_Y29_N24
\inst6|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~56_combout\ = ((\inst33|temp\(28) $ (\inst6|t_0\(28) $ (\inst6|Add0~55\)))) # (GND)
-- \inst6|Add0~57\ = CARRY((\inst33|temp\(28) & ((!\inst6|Add0~55\) # (!\inst6|t_0\(28)))) # (!\inst33|temp\(28) & (!\inst6|t_0\(28) & !\inst6|Add0~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(28),
	datab => \inst6|t_0\(28),
	datad => VCC,
	cin => \inst6|Add0~55\,
	combout => \inst6|Add0~56_combout\,
	cout => \inst6|Add0~57\);

-- Location: LCCOMB_X16_Y29_N26
\inst6|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~58_combout\ = (\inst6|t_0\(29) & ((\inst33|temp\(29) & (!\inst6|Add0~57\)) # (!\inst33|temp\(29) & ((\inst6|Add0~57\) # (GND))))) # (!\inst6|t_0\(29) & ((\inst33|temp\(29) & (\inst6|Add0~57\ & VCC)) # (!\inst33|temp\(29) & 
-- (!\inst6|Add0~57\))))
-- \inst6|Add0~59\ = CARRY((\inst6|t_0\(29) & ((!\inst6|Add0~57\) # (!\inst33|temp\(29)))) # (!\inst6|t_0\(29) & (!\inst33|temp\(29) & !\inst6|Add0~57\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|t_0\(29),
	datab => \inst33|temp\(29),
	datad => VCC,
	cin => \inst6|Add0~57\,
	combout => \inst6|Add0~58_combout\,
	cout => \inst6|Add0~59\);

-- Location: LCCOMB_X16_Y29_N28
\inst6|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst6|t_0\(30) $ (\inst6|Add0~59\)))) # (GND)
-- \inst6|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst6|Add0~59\) # (!\inst6|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst6|t_0\(30) & !\inst6|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst6|t_0\(30),
	datad => VCC,
	cin => \inst6|Add0~59\,
	combout => \inst6|Add0~60_combout\,
	cout => \inst6|Add0~61\);

-- Location: LCCOMB_X17_Y29_N26
\inst6|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~6_combout\ = (!\inst6|Add0~54_combout\ & (!\inst6|Add0~60_combout\ & (!\inst6|Add0~58_combout\ & !\inst6|Add0~56_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~54_combout\,
	datab => \inst6|Add0~60_combout\,
	datac => \inst6|Add0~58_combout\,
	datad => \inst6|Add0~56_combout\,
	combout => \inst6|data_available~6_combout\);

-- Location: FF_X16_Y29_N31
\inst6|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor0_signal~input_o\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|t_0\(31));

-- Location: LCCOMB_X16_Y29_N30
\inst6|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|Add0~62_combout\ = \inst33|temp\(31) $ (\inst6|Add0~61\ $ (!\inst6|t_0\(31)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101010100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(31),
	datad => \inst6|t_0\(31),
	cin => \inst6|Add0~61\,
	combout => \inst6|Add0~62_combout\);

-- Location: LCCOMB_X17_Y29_N8
\inst6|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~5_combout\ = (!\inst6|Add0~46_combout\ & (!\inst6|Add0~52_combout\ & (!\inst6|Add0~48_combout\ & !\inst6|Add0~50_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~46_combout\,
	datab => \inst6|Add0~52_combout\,
	datac => \inst6|Add0~48_combout\,
	datad => \inst6|Add0~50_combout\,
	combout => \inst6|data_available~5_combout\);

-- Location: LCCOMB_X17_Y29_N12
\inst6|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~7_combout\ = (\inst6|data_available~6_combout\ & (!\inst6|Add0~62_combout\ & \inst6|data_available~5_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010000000100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|data_available~6_combout\,
	datab => \inst6|Add0~62_combout\,
	datac => \inst6|data_available~5_combout\,
	combout => \inst6|data_available~7_combout\);

-- Location: LCCOMB_X17_Y30_N30
\inst6|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~9_combout\ = (\inst6|Add0~8_combout\ & ((\inst6|Add0~6_combout\) # (\inst6|Add0~12_combout\))) # (!\inst6|Add0~8_combout\ & (\inst6|Add0~6_combout\ & \inst6|Add0~12_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~8_combout\,
	datac => \inst6|Add0~6_combout\,
	datad => \inst6|Add0~12_combout\,
	combout => \inst6|data_available~9_combout\);

-- Location: LCCOMB_X17_Y30_N12
\inst6|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~8_combout\ = (\inst6|Add0~4_combout\) # ((\inst6|Add0~0_combout\ & \inst6|Add0~2_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst6|Add0~4_combout\,
	datac => \inst6|Add0~0_combout\,
	datad => \inst6|Add0~2_combout\,
	combout => \inst6|data_available~8_combout\);

-- Location: LCCOMB_X17_Y29_N22
\inst6|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~10_combout\ = (\inst6|Add0~10_combout\ & ((\inst6|Add0~12_combout\ & (!\inst6|data_available~9_combout\ & !\inst6|data_available~8_combout\)) # (!\inst6|Add0~12_combout\ & (\inst6|data_available~9_combout\ & 
-- \inst6|data_available~8_combout\)))) # (!\inst6|Add0~10_combout\ & (\inst6|Add0~12_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110010001001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~10_combout\,
	datab => \inst6|Add0~12_combout\,
	datac => \inst6|data_available~9_combout\,
	datad => \inst6|data_available~8_combout\,
	combout => \inst6|data_available~10_combout\);

-- Location: LCCOMB_X17_Y29_N10
\inst6|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~2_combout\ = (!\inst6|Add0~34_combout\ & (!\inst6|Add0~36_combout\ & (!\inst6|Add0~30_combout\ & !\inst6|Add0~32_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~34_combout\,
	datab => \inst6|Add0~36_combout\,
	datac => \inst6|Add0~30_combout\,
	datad => \inst6|Add0~32_combout\,
	combout => \inst6|data_available~2_combout\);

-- Location: LCCOMB_X17_Y30_N8
\inst6|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~0_combout\ = (!\inst6|Add0~14_combout\ & (!\inst6|Add0~20_combout\ & (!\inst6|Add0~16_combout\ & !\inst6|Add0~18_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~14_combout\,
	datab => \inst6|Add0~20_combout\,
	datac => \inst6|Add0~16_combout\,
	datad => \inst6|Add0~18_combout\,
	combout => \inst6|data_available~0_combout\);

-- Location: LCCOMB_X17_Y29_N4
\inst6|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~3_combout\ = (!\inst6|Add0~42_combout\ & (!\inst6|Add0~44_combout\ & (!\inst6|Add0~38_combout\ & !\inst6|Add0~40_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~42_combout\,
	datab => \inst6|Add0~44_combout\,
	datac => \inst6|Add0~38_combout\,
	datad => \inst6|Add0~40_combout\,
	combout => \inst6|data_available~3_combout\);

-- Location: LCCOMB_X17_Y30_N18
\inst6|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~1_combout\ = (!\inst6|Add0~28_combout\ & (!\inst6|Add0~22_combout\ & (!\inst6|Add0~26_combout\ & !\inst6|Add0~24_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|Add0~28_combout\,
	datab => \inst6|Add0~22_combout\,
	datac => \inst6|Add0~26_combout\,
	datad => \inst6|Add0~24_combout\,
	combout => \inst6|data_available~1_combout\);

-- Location: LCCOMB_X17_Y29_N6
\inst6|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~4_combout\ = (\inst6|data_available~2_combout\ & (\inst6|data_available~0_combout\ & (\inst6|data_available~3_combout\ & \inst6|data_available~1_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|data_available~2_combout\,
	datab => \inst6|data_available~0_combout\,
	datac => \inst6|data_available~3_combout\,
	datad => \inst6|data_available~1_combout\,
	combout => \inst6|data_available~4_combout\);

-- Location: LCCOMB_X17_Y29_N16
\inst6|data_available~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst6|data_available~11_combout\ = (\inst6|data_available~7_combout\ & (\inst6|data_available~10_combout\ & \inst6|data_available~4_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|data_available~7_combout\,
	datac => \inst6|data_available~10_combout\,
	datad => \inst6|data_available~4_combout\,
	combout => \inst6|data_available~11_combout\);

-- Location: FF_X17_Y29_N17
\inst6|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor0_signal~input_o\,
	d => \inst6|data_available~11_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst6|data_available~q\);

-- Location: IOIBUF_X18_Y34_N22
\sensor3_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor3_signal,
	o => \sensor3_signal~input_o\);

-- Location: FF_X20_Y30_N25
\inst9|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(28));

-- Location: FF_X20_Y30_N23
\inst9|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(27));

-- Location: FF_X20_Y30_N21
\inst9|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(26));

-- Location: FF_X20_Y30_N19
\inst9|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(25));

-- Location: FF_X20_Y30_N17
\inst9|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(24));

-- Location: FF_X20_Y30_N15
\inst9|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(23));

-- Location: FF_X20_Y30_N13
\inst9|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(22));

-- Location: FF_X20_Y30_N11
\inst9|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(21));

-- Location: FF_X20_Y30_N9
\inst9|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(20));

-- Location: FF_X20_Y30_N7
\inst9|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(19));

-- Location: FF_X20_Y30_N5
\inst9|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(18));

-- Location: FF_X20_Y30_N3
\inst9|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(17));

-- Location: FF_X20_Y30_N1
\inst9|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(16));

-- Location: FF_X20_Y31_N31
\inst9|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(15));

-- Location: FF_X20_Y31_N29
\inst9|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(14));

-- Location: FF_X20_Y31_N27
\inst9|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(13));

-- Location: FF_X20_Y31_N25
\inst9|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(12));

-- Location: FF_X20_Y31_N23
\inst9|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(11));

-- Location: FF_X20_Y31_N21
\inst9|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(10));

-- Location: FF_X20_Y31_N19
\inst9|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(9));

-- Location: FF_X20_Y31_N17
\inst9|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(8));

-- Location: FF_X20_Y31_N15
\inst9|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(7));

-- Location: FF_X20_Y31_N13
\inst9|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(6));

-- Location: FF_X20_Y31_N11
\inst9|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(5));

-- Location: FF_X20_Y31_N9
\inst9|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(4));

-- Location: FF_X20_Y31_N7
\inst9|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(3));

-- Location: FF_X20_Y31_N5
\inst9|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(2));

-- Location: FF_X20_Y31_N3
\inst9|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(1));

-- Location: FF_X20_Y31_N1
\inst9|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(0));

-- Location: LCCOMB_X20_Y31_N0
\inst9|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~0_combout\ = (\inst9|t_0\(0) & (\inst33|temp\(0) $ (VCC))) # (!\inst9|t_0\(0) & ((\inst33|temp\(0)) # (GND)))
-- \inst9|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst9|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011011011101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(0),
	datab => \inst33|temp\(0),
	datad => VCC,
	combout => \inst9|Add0~0_combout\,
	cout => \inst9|Add0~1\);

-- Location: LCCOMB_X20_Y31_N2
\inst9|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~2_combout\ = (\inst33|temp\(1) & ((\inst9|t_0\(1) & (!\inst9|Add0~1\)) # (!\inst9|t_0\(1) & (\inst9|Add0~1\ & VCC)))) # (!\inst33|temp\(1) & ((\inst9|t_0\(1) & ((\inst9|Add0~1\) # (GND))) # (!\inst9|t_0\(1) & (!\inst9|Add0~1\))))
-- \inst9|Add0~3\ = CARRY((\inst33|temp\(1) & (\inst9|t_0\(1) & !\inst9|Add0~1\)) # (!\inst33|temp\(1) & ((\inst9|t_0\(1)) # (!\inst9|Add0~1\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(1),
	datab => \inst9|t_0\(1),
	datad => VCC,
	cin => \inst9|Add0~1\,
	combout => \inst9|Add0~2_combout\,
	cout => \inst9|Add0~3\);

-- Location: LCCOMB_X20_Y31_N4
\inst9|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~4_combout\ = ((\inst33|temp\(2) $ (\inst9|t_0\(2) $ (\inst9|Add0~3\)))) # (GND)
-- \inst9|Add0~5\ = CARRY((\inst33|temp\(2) & ((!\inst9|Add0~3\) # (!\inst9|t_0\(2)))) # (!\inst33|temp\(2) & (!\inst9|t_0\(2) & !\inst9|Add0~3\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(2),
	datab => \inst9|t_0\(2),
	datad => VCC,
	cin => \inst9|Add0~3\,
	combout => \inst9|Add0~4_combout\,
	cout => \inst9|Add0~5\);

-- Location: LCCOMB_X20_Y31_N6
\inst9|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~6_combout\ = (\inst9|t_0\(3) & ((\inst33|temp\(3) & (!\inst9|Add0~5\)) # (!\inst33|temp\(3) & ((\inst9|Add0~5\) # (GND))))) # (!\inst9|t_0\(3) & ((\inst33|temp\(3) & (\inst9|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst9|Add0~5\))))
-- \inst9|Add0~7\ = CARRY((\inst9|t_0\(3) & ((!\inst9|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst9|t_0\(3) & (!\inst33|temp\(3) & !\inst9|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst9|Add0~5\,
	combout => \inst9|Add0~6_combout\,
	cout => \inst9|Add0~7\);

-- Location: LCCOMB_X20_Y31_N8
\inst9|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~8_combout\ = ((\inst33|temp\(4) $ (\inst9|t_0\(4) $ (\inst9|Add0~7\)))) # (GND)
-- \inst9|Add0~9\ = CARRY((\inst33|temp\(4) & ((!\inst9|Add0~7\) # (!\inst9|t_0\(4)))) # (!\inst33|temp\(4) & (!\inst9|t_0\(4) & !\inst9|Add0~7\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(4),
	datab => \inst9|t_0\(4),
	datad => VCC,
	cin => \inst9|Add0~7\,
	combout => \inst9|Add0~8_combout\,
	cout => \inst9|Add0~9\);

-- Location: LCCOMB_X20_Y31_N10
\inst9|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~10_combout\ = (\inst9|t_0\(5) & ((\inst33|temp\(5) & (!\inst9|Add0~9\)) # (!\inst33|temp\(5) & ((\inst9|Add0~9\) # (GND))))) # (!\inst9|t_0\(5) & ((\inst33|temp\(5) & (\inst9|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst9|Add0~9\))))
-- \inst9|Add0~11\ = CARRY((\inst9|t_0\(5) & ((!\inst9|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst9|t_0\(5) & (!\inst33|temp\(5) & !\inst9|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst9|Add0~9\,
	combout => \inst9|Add0~10_combout\,
	cout => \inst9|Add0~11\);

-- Location: LCCOMB_X20_Y31_N12
\inst9|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~12_combout\ = ((\inst9|t_0\(6) $ (\inst33|temp\(6) $ (\inst9|Add0~11\)))) # (GND)
-- \inst9|Add0~13\ = CARRY((\inst9|t_0\(6) & (\inst33|temp\(6) & !\inst9|Add0~11\)) # (!\inst9|t_0\(6) & ((\inst33|temp\(6)) # (!\inst9|Add0~11\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(6),
	datab => \inst33|temp\(6),
	datad => VCC,
	cin => \inst9|Add0~11\,
	combout => \inst9|Add0~12_combout\,
	cout => \inst9|Add0~13\);

-- Location: LCCOMB_X20_Y31_N14
\inst9|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~14_combout\ = (\inst33|temp\(7) & ((\inst9|t_0\(7) & (!\inst9|Add0~13\)) # (!\inst9|t_0\(7) & (\inst9|Add0~13\ & VCC)))) # (!\inst33|temp\(7) & ((\inst9|t_0\(7) & ((\inst9|Add0~13\) # (GND))) # (!\inst9|t_0\(7) & (!\inst9|Add0~13\))))
-- \inst9|Add0~15\ = CARRY((\inst33|temp\(7) & (\inst9|t_0\(7) & !\inst9|Add0~13\)) # (!\inst33|temp\(7) & ((\inst9|t_0\(7)) # (!\inst9|Add0~13\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(7),
	datab => \inst9|t_0\(7),
	datad => VCC,
	cin => \inst9|Add0~13\,
	combout => \inst9|Add0~14_combout\,
	cout => \inst9|Add0~15\);

-- Location: LCCOMB_X20_Y31_N16
\inst9|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~16_combout\ = ((\inst33|temp\(8) $ (\inst9|t_0\(8) $ (\inst9|Add0~15\)))) # (GND)
-- \inst9|Add0~17\ = CARRY((\inst33|temp\(8) & ((!\inst9|Add0~15\) # (!\inst9|t_0\(8)))) # (!\inst33|temp\(8) & (!\inst9|t_0\(8) & !\inst9|Add0~15\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(8),
	datab => \inst9|t_0\(8),
	datad => VCC,
	cin => \inst9|Add0~15\,
	combout => \inst9|Add0~16_combout\,
	cout => \inst9|Add0~17\);

-- Location: LCCOMB_X20_Y31_N18
\inst9|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~18_combout\ = (\inst9|t_0\(9) & ((\inst33|temp\(9) & (!\inst9|Add0~17\)) # (!\inst33|temp\(9) & ((\inst9|Add0~17\) # (GND))))) # (!\inst9|t_0\(9) & ((\inst33|temp\(9) & (\inst9|Add0~17\ & VCC)) # (!\inst33|temp\(9) & (!\inst9|Add0~17\))))
-- \inst9|Add0~19\ = CARRY((\inst9|t_0\(9) & ((!\inst9|Add0~17\) # (!\inst33|temp\(9)))) # (!\inst9|t_0\(9) & (!\inst33|temp\(9) & !\inst9|Add0~17\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(9),
	datab => \inst33|temp\(9),
	datad => VCC,
	cin => \inst9|Add0~17\,
	combout => \inst9|Add0~18_combout\,
	cout => \inst9|Add0~19\);

-- Location: LCCOMB_X20_Y31_N20
\inst9|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~20_combout\ = ((\inst33|temp\(10) $ (\inst9|t_0\(10) $ (\inst9|Add0~19\)))) # (GND)
-- \inst9|Add0~21\ = CARRY((\inst33|temp\(10) & ((!\inst9|Add0~19\) # (!\inst9|t_0\(10)))) # (!\inst33|temp\(10) & (!\inst9|t_0\(10) & !\inst9|Add0~19\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(10),
	datab => \inst9|t_0\(10),
	datad => VCC,
	cin => \inst9|Add0~19\,
	combout => \inst9|Add0~20_combout\,
	cout => \inst9|Add0~21\);

-- Location: LCCOMB_X20_Y31_N22
\inst9|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~22_combout\ = (\inst9|t_0\(11) & ((\inst33|temp\(11) & (!\inst9|Add0~21\)) # (!\inst33|temp\(11) & ((\inst9|Add0~21\) # (GND))))) # (!\inst9|t_0\(11) & ((\inst33|temp\(11) & (\inst9|Add0~21\ & VCC)) # (!\inst33|temp\(11) & 
-- (!\inst9|Add0~21\))))
-- \inst9|Add0~23\ = CARRY((\inst9|t_0\(11) & ((!\inst9|Add0~21\) # (!\inst33|temp\(11)))) # (!\inst9|t_0\(11) & (!\inst33|temp\(11) & !\inst9|Add0~21\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(11),
	datab => \inst33|temp\(11),
	datad => VCC,
	cin => \inst9|Add0~21\,
	combout => \inst9|Add0~22_combout\,
	cout => \inst9|Add0~23\);

-- Location: LCCOMB_X20_Y31_N24
\inst9|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~24_combout\ = ((\inst9|t_0\(12) $ (\inst33|temp\(12) $ (\inst9|Add0~23\)))) # (GND)
-- \inst9|Add0~25\ = CARRY((\inst9|t_0\(12) & (\inst33|temp\(12) & !\inst9|Add0~23\)) # (!\inst9|t_0\(12) & ((\inst33|temp\(12)) # (!\inst9|Add0~23\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(12),
	datab => \inst33|temp\(12),
	datad => VCC,
	cin => \inst9|Add0~23\,
	combout => \inst9|Add0~24_combout\,
	cout => \inst9|Add0~25\);

-- Location: LCCOMB_X20_Y31_N26
\inst9|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~26_combout\ = (\inst9|t_0\(13) & ((\inst33|temp\(13) & (!\inst9|Add0~25\)) # (!\inst33|temp\(13) & ((\inst9|Add0~25\) # (GND))))) # (!\inst9|t_0\(13) & ((\inst33|temp\(13) & (\inst9|Add0~25\ & VCC)) # (!\inst33|temp\(13) & 
-- (!\inst9|Add0~25\))))
-- \inst9|Add0~27\ = CARRY((\inst9|t_0\(13) & ((!\inst9|Add0~25\) # (!\inst33|temp\(13)))) # (!\inst9|t_0\(13) & (!\inst33|temp\(13) & !\inst9|Add0~25\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(13),
	datab => \inst33|temp\(13),
	datad => VCC,
	cin => \inst9|Add0~25\,
	combout => \inst9|Add0~26_combout\,
	cout => \inst9|Add0~27\);

-- Location: LCCOMB_X20_Y31_N28
\inst9|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst9|t_0\(14) $ (\inst9|Add0~27\)))) # (GND)
-- \inst9|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst9|Add0~27\) # (!\inst9|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst9|t_0\(14) & !\inst9|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst9|t_0\(14),
	datad => VCC,
	cin => \inst9|Add0~27\,
	combout => \inst9|Add0~28_combout\,
	cout => \inst9|Add0~29\);

-- Location: LCCOMB_X20_Y31_N30
\inst9|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~30_combout\ = (\inst33|temp\(15) & ((\inst9|t_0\(15) & (!\inst9|Add0~29\)) # (!\inst9|t_0\(15) & (\inst9|Add0~29\ & VCC)))) # (!\inst33|temp\(15) & ((\inst9|t_0\(15) & ((\inst9|Add0~29\) # (GND))) # (!\inst9|t_0\(15) & (!\inst9|Add0~29\))))
-- \inst9|Add0~31\ = CARRY((\inst33|temp\(15) & (\inst9|t_0\(15) & !\inst9|Add0~29\)) # (!\inst33|temp\(15) & ((\inst9|t_0\(15)) # (!\inst9|Add0~29\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(15),
	datab => \inst9|t_0\(15),
	datad => VCC,
	cin => \inst9|Add0~29\,
	combout => \inst9|Add0~30_combout\,
	cout => \inst9|Add0~31\);

-- Location: LCCOMB_X20_Y30_N0
\inst9|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~32_combout\ = ((\inst9|t_0\(16) $ (\inst33|temp\(16) $ (\inst9|Add0~31\)))) # (GND)
-- \inst9|Add0~33\ = CARRY((\inst9|t_0\(16) & (\inst33|temp\(16) & !\inst9|Add0~31\)) # (!\inst9|t_0\(16) & ((\inst33|temp\(16)) # (!\inst9|Add0~31\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(16),
	datab => \inst33|temp\(16),
	datad => VCC,
	cin => \inst9|Add0~31\,
	combout => \inst9|Add0~32_combout\,
	cout => \inst9|Add0~33\);

-- Location: LCCOMB_X20_Y30_N2
\inst9|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~34_combout\ = (\inst9|t_0\(17) & ((\inst33|temp\(17) & (!\inst9|Add0~33\)) # (!\inst33|temp\(17) & ((\inst9|Add0~33\) # (GND))))) # (!\inst9|t_0\(17) & ((\inst33|temp\(17) & (\inst9|Add0~33\ & VCC)) # (!\inst33|temp\(17) & 
-- (!\inst9|Add0~33\))))
-- \inst9|Add0~35\ = CARRY((\inst9|t_0\(17) & ((!\inst9|Add0~33\) # (!\inst33|temp\(17)))) # (!\inst9|t_0\(17) & (!\inst33|temp\(17) & !\inst9|Add0~33\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(17),
	datab => \inst33|temp\(17),
	datad => VCC,
	cin => \inst9|Add0~33\,
	combout => \inst9|Add0~34_combout\,
	cout => \inst9|Add0~35\);

-- Location: LCCOMB_X20_Y30_N4
\inst9|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~36_combout\ = ((\inst33|temp\(18) $ (\inst9|t_0\(18) $ (\inst9|Add0~35\)))) # (GND)
-- \inst9|Add0~37\ = CARRY((\inst33|temp\(18) & ((!\inst9|Add0~35\) # (!\inst9|t_0\(18)))) # (!\inst33|temp\(18) & (!\inst9|t_0\(18) & !\inst9|Add0~35\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(18),
	datab => \inst9|t_0\(18),
	datad => VCC,
	cin => \inst9|Add0~35\,
	combout => \inst9|Add0~36_combout\,
	cout => \inst9|Add0~37\);

-- Location: LCCOMB_X20_Y30_N6
\inst9|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~38_combout\ = (\inst9|t_0\(19) & ((\inst33|temp\(19) & (!\inst9|Add0~37\)) # (!\inst33|temp\(19) & ((\inst9|Add0~37\) # (GND))))) # (!\inst9|t_0\(19) & ((\inst33|temp\(19) & (\inst9|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst9|Add0~37\))))
-- \inst9|Add0~39\ = CARRY((\inst9|t_0\(19) & ((!\inst9|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst9|t_0\(19) & (!\inst33|temp\(19) & !\inst9|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst9|Add0~37\,
	combout => \inst9|Add0~38_combout\,
	cout => \inst9|Add0~39\);

-- Location: LCCOMB_X20_Y30_N8
\inst9|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~40_combout\ = ((\inst9|t_0\(20) $ (\inst33|temp\(20) $ (\inst9|Add0~39\)))) # (GND)
-- \inst9|Add0~41\ = CARRY((\inst9|t_0\(20) & (\inst33|temp\(20) & !\inst9|Add0~39\)) # (!\inst9|t_0\(20) & ((\inst33|temp\(20)) # (!\inst9|Add0~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(20),
	datab => \inst33|temp\(20),
	datad => VCC,
	cin => \inst9|Add0~39\,
	combout => \inst9|Add0~40_combout\,
	cout => \inst9|Add0~41\);

-- Location: LCCOMB_X20_Y30_N10
\inst9|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~42_combout\ = (\inst9|t_0\(21) & ((\inst33|temp\(21) & (!\inst9|Add0~41\)) # (!\inst33|temp\(21) & ((\inst9|Add0~41\) # (GND))))) # (!\inst9|t_0\(21) & ((\inst33|temp\(21) & (\inst9|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst9|Add0~41\))))
-- \inst9|Add0~43\ = CARRY((\inst9|t_0\(21) & ((!\inst9|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst9|t_0\(21) & (!\inst33|temp\(21) & !\inst9|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst9|Add0~41\,
	combout => \inst9|Add0~42_combout\,
	cout => \inst9|Add0~43\);

-- Location: LCCOMB_X20_Y30_N12
\inst9|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~44_combout\ = ((\inst33|temp\(22) $ (\inst9|t_0\(22) $ (\inst9|Add0~43\)))) # (GND)
-- \inst9|Add0~45\ = CARRY((\inst33|temp\(22) & ((!\inst9|Add0~43\) # (!\inst9|t_0\(22)))) # (!\inst33|temp\(22) & (!\inst9|t_0\(22) & !\inst9|Add0~43\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(22),
	datab => \inst9|t_0\(22),
	datad => VCC,
	cin => \inst9|Add0~43\,
	combout => \inst9|Add0~44_combout\,
	cout => \inst9|Add0~45\);

-- Location: LCCOMB_X20_Y30_N14
\inst9|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~46_combout\ = (\inst33|temp\(23) & ((\inst9|t_0\(23) & (!\inst9|Add0~45\)) # (!\inst9|t_0\(23) & (\inst9|Add0~45\ & VCC)))) # (!\inst33|temp\(23) & ((\inst9|t_0\(23) & ((\inst9|Add0~45\) # (GND))) # (!\inst9|t_0\(23) & (!\inst9|Add0~45\))))
-- \inst9|Add0~47\ = CARRY((\inst33|temp\(23) & (\inst9|t_0\(23) & !\inst9|Add0~45\)) # (!\inst33|temp\(23) & ((\inst9|t_0\(23)) # (!\inst9|Add0~45\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(23),
	datab => \inst9|t_0\(23),
	datad => VCC,
	cin => \inst9|Add0~45\,
	combout => \inst9|Add0~46_combout\,
	cout => \inst9|Add0~47\);

-- Location: LCCOMB_X20_Y30_N16
\inst9|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~48_combout\ = ((\inst9|t_0\(24) $ (\inst33|temp\(24) $ (\inst9|Add0~47\)))) # (GND)
-- \inst9|Add0~49\ = CARRY((\inst9|t_0\(24) & (\inst33|temp\(24) & !\inst9|Add0~47\)) # (!\inst9|t_0\(24) & ((\inst33|temp\(24)) # (!\inst9|Add0~47\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(24),
	datab => \inst33|temp\(24),
	datad => VCC,
	cin => \inst9|Add0~47\,
	combout => \inst9|Add0~48_combout\,
	cout => \inst9|Add0~49\);

-- Location: LCCOMB_X20_Y30_N18
\inst9|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst9|t_0\(25) & (!\inst9|Add0~49\)) # (!\inst9|t_0\(25) & (\inst9|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst9|t_0\(25) & ((\inst9|Add0~49\) # (GND))) # (!\inst9|t_0\(25) & (!\inst9|Add0~49\))))
-- \inst9|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst9|t_0\(25) & !\inst9|Add0~49\)) # (!\inst33|temp\(25) & ((\inst9|t_0\(25)) # (!\inst9|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst9|t_0\(25),
	datad => VCC,
	cin => \inst9|Add0~49\,
	combout => \inst9|Add0~50_combout\,
	cout => \inst9|Add0~51\);

-- Location: LCCOMB_X20_Y30_N20
\inst9|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~52_combout\ = ((\inst33|temp\(26) $ (\inst9|t_0\(26) $ (\inst9|Add0~51\)))) # (GND)
-- \inst9|Add0~53\ = CARRY((\inst33|temp\(26) & ((!\inst9|Add0~51\) # (!\inst9|t_0\(26)))) # (!\inst33|temp\(26) & (!\inst9|t_0\(26) & !\inst9|Add0~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(26),
	datab => \inst9|t_0\(26),
	datad => VCC,
	cin => \inst9|Add0~51\,
	combout => \inst9|Add0~52_combout\,
	cout => \inst9|Add0~53\);

-- Location: LCCOMB_X20_Y30_N22
\inst9|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~54_combout\ = (\inst33|temp\(27) & ((\inst9|t_0\(27) & (!\inst9|Add0~53\)) # (!\inst9|t_0\(27) & (\inst9|Add0~53\ & VCC)))) # (!\inst33|temp\(27) & ((\inst9|t_0\(27) & ((\inst9|Add0~53\) # (GND))) # (!\inst9|t_0\(27) & (!\inst9|Add0~53\))))
-- \inst9|Add0~55\ = CARRY((\inst33|temp\(27) & (\inst9|t_0\(27) & !\inst9|Add0~53\)) # (!\inst33|temp\(27) & ((\inst9|t_0\(27)) # (!\inst9|Add0~53\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(27),
	datab => \inst9|t_0\(27),
	datad => VCC,
	cin => \inst9|Add0~53\,
	combout => \inst9|Add0~54_combout\,
	cout => \inst9|Add0~55\);

-- Location: LCCOMB_X20_Y30_N24
\inst9|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~56_combout\ = ((\inst33|temp\(28) $ (\inst9|t_0\(28) $ (\inst9|Add0~55\)))) # (GND)
-- \inst9|Add0~57\ = CARRY((\inst33|temp\(28) & ((!\inst9|Add0~55\) # (!\inst9|t_0\(28)))) # (!\inst33|temp\(28) & (!\inst9|t_0\(28) & !\inst9|Add0~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(28),
	datab => \inst9|t_0\(28),
	datad => VCC,
	cin => \inst9|Add0~55\,
	combout => \inst9|Add0~56_combout\,
	cout => \inst9|Add0~57\);

-- Location: FF_X20_Y30_N29
\inst9|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(30));

-- Location: FF_X20_Y30_N27
\inst9|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(29));

-- Location: LCCOMB_X20_Y30_N26
\inst9|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~58_combout\ = (\inst9|t_0\(29) & ((\inst33|temp\(29) & (!\inst9|Add0~57\)) # (!\inst33|temp\(29) & ((\inst9|Add0~57\) # (GND))))) # (!\inst9|t_0\(29) & ((\inst33|temp\(29) & (\inst9|Add0~57\ & VCC)) # (!\inst33|temp\(29) & 
-- (!\inst9|Add0~57\))))
-- \inst9|Add0~59\ = CARRY((\inst9|t_0\(29) & ((!\inst9|Add0~57\) # (!\inst33|temp\(29)))) # (!\inst9|t_0\(29) & (!\inst33|temp\(29) & !\inst9|Add0~57\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|t_0\(29),
	datab => \inst33|temp\(29),
	datad => VCC,
	cin => \inst9|Add0~57\,
	combout => \inst9|Add0~58_combout\,
	cout => \inst9|Add0~59\);

-- Location: LCCOMB_X20_Y30_N28
\inst9|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst9|t_0\(30) $ (\inst9|Add0~59\)))) # (GND)
-- \inst9|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst9|Add0~59\) # (!\inst9|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst9|t_0\(30) & !\inst9|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst9|t_0\(30),
	datad => VCC,
	cin => \inst9|Add0~59\,
	combout => \inst9|Add0~60_combout\,
	cout => \inst9|Add0~61\);

-- Location: LCCOMB_X19_Y30_N2
\inst9|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~6_combout\ = (!\inst9|Add0~56_combout\ & (!\inst9|Add0~60_combout\ & (!\inst9|Add0~58_combout\ & !\inst9|Add0~54_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|Add0~56_combout\,
	datab => \inst9|Add0~60_combout\,
	datac => \inst9|Add0~58_combout\,
	datad => \inst9|Add0~54_combout\,
	combout => \inst9|data_available~6_combout\);

-- Location: FF_X20_Y30_N31
\inst9|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor3_signal~input_o\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|t_0\(31));

-- Location: LCCOMB_X20_Y30_N30
\inst9|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|Add0~62_combout\ = \inst33|temp\(31) $ (\inst9|Add0~61\ $ (!\inst9|t_0\(31)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110011000011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst33|temp\(31),
	datad => \inst9|t_0\(31),
	cin => \inst9|Add0~61\,
	combout => \inst9|Add0~62_combout\);

-- Location: LCCOMB_X19_Y31_N10
\inst9|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~5_combout\ = (!\inst9|Add0~50_combout\ & (!\inst9|Add0~52_combout\ & (!\inst9|Add0~46_combout\ & !\inst9|Add0~48_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|Add0~50_combout\,
	datab => \inst9|Add0~52_combout\,
	datac => \inst9|Add0~46_combout\,
	datad => \inst9|Add0~48_combout\,
	combout => \inst9|data_available~5_combout\);

-- Location: LCCOMB_X19_Y31_N12
\inst9|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~7_combout\ = (\inst9|data_available~6_combout\ & (!\inst9|Add0~62_combout\ & \inst9|data_available~5_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000101000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|data_available~6_combout\,
	datac => \inst9|Add0~62_combout\,
	datad => \inst9|data_available~5_combout\,
	combout => \inst9|data_available~7_combout\);

-- Location: LCCOMB_X19_Y31_N22
\inst9|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~8_combout\ = (\inst9|Add0~4_combout\) # ((\inst9|Add0~0_combout\ & \inst9|Add0~2_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|Add0~4_combout\,
	datac => \inst9|Add0~0_combout\,
	datad => \inst9|Add0~2_combout\,
	combout => \inst9|data_available~8_combout\);

-- Location: LCCOMB_X19_Y31_N16
\inst9|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~9_combout\ = (\inst9|Add0~12_combout\ & ((\inst9|Add0~8_combout\) # (\inst9|Add0~6_combout\))) # (!\inst9|Add0~12_combout\ & (\inst9|Add0~8_combout\ & \inst9|Add0~6_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst9|Add0~12_combout\,
	datac => \inst9|Add0~8_combout\,
	datad => \inst9|Add0~6_combout\,
	combout => \inst9|data_available~9_combout\);

-- Location: LCCOMB_X19_Y31_N18
\inst9|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~10_combout\ = (\inst9|Add0~12_combout\ & (((!\inst9|data_available~8_combout\ & !\inst9|data_available~9_combout\)) # (!\inst9|Add0~10_combout\))) # (!\inst9|Add0~12_combout\ & (\inst9|data_available~8_combout\ & 
-- (\inst9|Add0~10_combout\ & \inst9|data_available~9_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010110001001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|data_available~8_combout\,
	datab => \inst9|Add0~12_combout\,
	datac => \inst9|Add0~10_combout\,
	datad => \inst9|data_available~9_combout\,
	combout => \inst9|data_available~10_combout\);

-- Location: LCCOMB_X19_Y31_N26
\inst9|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~0_combout\ = (!\inst9|Add0~20_combout\ & (!\inst9|Add0~18_combout\ & (!\inst9|Add0~14_combout\ & !\inst9|Add0~16_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|Add0~20_combout\,
	datab => \inst9|Add0~18_combout\,
	datac => \inst9|Add0~14_combout\,
	datad => \inst9|Add0~16_combout\,
	combout => \inst9|data_available~0_combout\);

-- Location: LCCOMB_X19_Y30_N0
\inst9|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~3_combout\ = (!\inst9|Add0~38_combout\ & (!\inst9|Add0~40_combout\ & (!\inst9|Add0~42_combout\ & !\inst9|Add0~44_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|Add0~38_combout\,
	datab => \inst9|Add0~40_combout\,
	datac => \inst9|Add0~42_combout\,
	datad => \inst9|Add0~44_combout\,
	combout => \inst9|data_available~3_combout\);

-- Location: LCCOMB_X19_Y31_N4
\inst9|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~1_combout\ = (!\inst9|Add0~22_combout\ & (!\inst9|Add0~28_combout\ & (!\inst9|Add0~26_combout\ & !\inst9|Add0~24_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|Add0~22_combout\,
	datab => \inst9|Add0~28_combout\,
	datac => \inst9|Add0~26_combout\,
	datad => \inst9|Add0~24_combout\,
	combout => \inst9|data_available~1_combout\);

-- Location: LCCOMB_X19_Y31_N6
\inst9|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~2_combout\ = (!\inst9|Add0~34_combout\ & (!\inst9|Add0~32_combout\ & (!\inst9|Add0~30_combout\ & !\inst9|Add0~36_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|Add0~34_combout\,
	datab => \inst9|Add0~32_combout\,
	datac => \inst9|Add0~30_combout\,
	datad => \inst9|Add0~36_combout\,
	combout => \inst9|data_available~2_combout\);

-- Location: LCCOMB_X19_Y31_N24
\inst9|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~4_combout\ = (\inst9|data_available~0_combout\ & (\inst9|data_available~3_combout\ & (\inst9|data_available~1_combout\ & \inst9|data_available~2_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|data_available~0_combout\,
	datab => \inst9|data_available~3_combout\,
	datac => \inst9|data_available~1_combout\,
	datad => \inst9|data_available~2_combout\,
	combout => \inst9|data_available~4_combout\);

-- Location: LCCOMB_X19_Y31_N8
\inst9|data_available~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst9|data_available~11_combout\ = (\inst9|data_available~7_combout\ & (\inst9|data_available~10_combout\ & \inst9|data_available~4_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000100000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst9|data_available~7_combout\,
	datab => \inst9|data_available~10_combout\,
	datad => \inst9|data_available~4_combout\,
	combout => \inst9|data_available~11_combout\);

-- Location: FF_X19_Y31_N9
\inst9|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor3_signal~input_o\,
	d => \inst9|data_available~11_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst9|data_available~q\);

-- Location: LCCOMB_X26_Y26_N24
\inst20|cur_value~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst20|cur_value~feeder_combout\ = \inst9|data_available~q\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst9|data_available~q\,
	combout => \inst20|cur_value~feeder_combout\);

-- Location: FF_X26_Y26_N25
\inst20|cur_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst20|cur_value~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst20|cur_value~q\);

-- Location: FF_X26_Y26_N27
\inst20|last_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst20|cur_value~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst20|last_value~q\);

-- Location: LCCOMB_X26_Y26_N26
\inst20|level_sig\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst20|level_sig~combout\ = (!\inst20|last_value~q\ & \inst20|cur_value~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst20|last_value~q\,
	datad => \inst20|cur_value~q\,
	combout => \inst20|level_sig~combout\);

-- Location: IOIBUF_X16_Y34_N1
\sensor1_signal~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_sensor1_signal,
	o => \sensor1_signal~input_o\);

-- Location: CLKCTRL_G11
\sensor1_signal~inputclkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \sensor1_signal~inputclkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \sensor1_signal~inputclkctrl_outclk\);

-- Location: FF_X21_Y22_N9
\inst7|t_0[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(4),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(4));

-- Location: FF_X21_Y22_N7
\inst7|t_0[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(3),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(3));

-- Location: FF_X21_Y22_N5
\inst7|t_0[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(2),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(2));

-- Location: FF_X21_Y22_N3
\inst7|t_0[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(1),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(1));

-- Location: FF_X21_Y22_N1
\inst7|t_0[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(0),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(0));

-- Location: LCCOMB_X21_Y22_N0
\inst7|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~0_combout\ = (\inst7|t_0\(0) & (\inst33|temp\(0) $ (VCC))) # (!\inst7|t_0\(0) & ((\inst33|temp\(0)) # (GND)))
-- \inst7|Add0~1\ = CARRY((\inst33|temp\(0)) # (!\inst7|t_0\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011011011101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(0),
	datab => \inst33|temp\(0),
	datad => VCC,
	combout => \inst7|Add0~0_combout\,
	cout => \inst7|Add0~1\);

-- Location: LCCOMB_X21_Y22_N2
\inst7|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~2_combout\ = (\inst33|temp\(1) & ((\inst7|t_0\(1) & (!\inst7|Add0~1\)) # (!\inst7|t_0\(1) & (\inst7|Add0~1\ & VCC)))) # (!\inst33|temp\(1) & ((\inst7|t_0\(1) & ((\inst7|Add0~1\) # (GND))) # (!\inst7|t_0\(1) & (!\inst7|Add0~1\))))
-- \inst7|Add0~3\ = CARRY((\inst33|temp\(1) & (\inst7|t_0\(1) & !\inst7|Add0~1\)) # (!\inst33|temp\(1) & ((\inst7|t_0\(1)) # (!\inst7|Add0~1\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(1),
	datab => \inst7|t_0\(1),
	datad => VCC,
	cin => \inst7|Add0~1\,
	combout => \inst7|Add0~2_combout\,
	cout => \inst7|Add0~3\);

-- Location: LCCOMB_X21_Y22_N4
\inst7|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~4_combout\ = ((\inst33|temp\(2) $ (\inst7|t_0\(2) $ (\inst7|Add0~3\)))) # (GND)
-- \inst7|Add0~5\ = CARRY((\inst33|temp\(2) & ((!\inst7|Add0~3\) # (!\inst7|t_0\(2)))) # (!\inst33|temp\(2) & (!\inst7|t_0\(2) & !\inst7|Add0~3\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(2),
	datab => \inst7|t_0\(2),
	datad => VCC,
	cin => \inst7|Add0~3\,
	combout => \inst7|Add0~4_combout\,
	cout => \inst7|Add0~5\);

-- Location: LCCOMB_X21_Y22_N6
\inst7|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~6_combout\ = (\inst7|t_0\(3) & ((\inst33|temp\(3) & (!\inst7|Add0~5\)) # (!\inst33|temp\(3) & ((\inst7|Add0~5\) # (GND))))) # (!\inst7|t_0\(3) & ((\inst33|temp\(3) & (\inst7|Add0~5\ & VCC)) # (!\inst33|temp\(3) & (!\inst7|Add0~5\))))
-- \inst7|Add0~7\ = CARRY((\inst7|t_0\(3) & ((!\inst7|Add0~5\) # (!\inst33|temp\(3)))) # (!\inst7|t_0\(3) & (!\inst33|temp\(3) & !\inst7|Add0~5\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(3),
	datab => \inst33|temp\(3),
	datad => VCC,
	cin => \inst7|Add0~5\,
	combout => \inst7|Add0~6_combout\,
	cout => \inst7|Add0~7\);

-- Location: LCCOMB_X21_Y22_N8
\inst7|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~8_combout\ = ((\inst33|temp\(4) $ (\inst7|t_0\(4) $ (\inst7|Add0~7\)))) # (GND)
-- \inst7|Add0~9\ = CARRY((\inst33|temp\(4) & ((!\inst7|Add0~7\) # (!\inst7|t_0\(4)))) # (!\inst33|temp\(4) & (!\inst7|t_0\(4) & !\inst7|Add0~7\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(4),
	datab => \inst7|t_0\(4),
	datad => VCC,
	cin => \inst7|Add0~7\,
	combout => \inst7|Add0~8_combout\,
	cout => \inst7|Add0~9\);

-- Location: FF_X21_Y22_N13
\inst7|t_0[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(6),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(6));

-- Location: FF_X21_Y22_N11
\inst7|t_0[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(5),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(5));

-- Location: LCCOMB_X21_Y22_N10
\inst7|Add0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~10_combout\ = (\inst7|t_0\(5) & ((\inst33|temp\(5) & (!\inst7|Add0~9\)) # (!\inst33|temp\(5) & ((\inst7|Add0~9\) # (GND))))) # (!\inst7|t_0\(5) & ((\inst33|temp\(5) & (\inst7|Add0~9\ & VCC)) # (!\inst33|temp\(5) & (!\inst7|Add0~9\))))
-- \inst7|Add0~11\ = CARRY((\inst7|t_0\(5) & ((!\inst7|Add0~9\) # (!\inst33|temp\(5)))) # (!\inst7|t_0\(5) & (!\inst33|temp\(5) & !\inst7|Add0~9\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(5),
	datab => \inst33|temp\(5),
	datad => VCC,
	cin => \inst7|Add0~9\,
	combout => \inst7|Add0~10_combout\,
	cout => \inst7|Add0~11\);

-- Location: LCCOMB_X21_Y22_N12
\inst7|Add0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~12_combout\ = ((\inst7|t_0\(6) $ (\inst33|temp\(6) $ (\inst7|Add0~11\)))) # (GND)
-- \inst7|Add0~13\ = CARRY((\inst7|t_0\(6) & (\inst33|temp\(6) & !\inst7|Add0~11\)) # (!\inst7|t_0\(6) & ((\inst33|temp\(6)) # (!\inst7|Add0~11\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(6),
	datab => \inst33|temp\(6),
	datad => VCC,
	cin => \inst7|Add0~11\,
	combout => \inst7|Add0~12_combout\,
	cout => \inst7|Add0~13\);

-- Location: LCCOMB_X21_Y19_N18
\inst7|data_available~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~8_combout\ = (\inst7|Add0~4_combout\) # ((\inst7|Add0~2_combout\ & \inst7|Add0~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~4_combout\,
	datac => \inst7|Add0~2_combout\,
	datad => \inst7|Add0~0_combout\,
	combout => \inst7|data_available~8_combout\);

-- Location: LCCOMB_X21_Y19_N12
\inst7|data_available~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~9_combout\ = (\inst7|Add0~8_combout\ & ((\inst7|Add0~12_combout\) # ((\inst7|Add0~6_combout\ & \inst7|data_available~8_combout\)))) # (!\inst7|Add0~8_combout\ & (\inst7|Add0~12_combout\ & ((\inst7|Add0~6_combout\) # 
-- (\inst7|data_available~8_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110110011001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~8_combout\,
	datab => \inst7|Add0~12_combout\,
	datac => \inst7|Add0~6_combout\,
	datad => \inst7|data_available~8_combout\,
	combout => \inst7|data_available~9_combout\);

-- Location: FF_X21_Y21_N3
\inst7|t_0[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(17),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(17));

-- Location: FF_X21_Y21_N1
\inst7|t_0[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(16),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(16));

-- Location: FF_X21_Y22_N31
\inst7|t_0[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(15),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(15));

-- Location: FF_X21_Y22_N29
\inst7|t_0[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(14),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(14));

-- Location: FF_X21_Y22_N27
\inst7|t_0[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(13),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(13));

-- Location: FF_X21_Y22_N25
\inst7|t_0[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(12),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(12));

-- Location: FF_X21_Y22_N23
\inst7|t_0[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(11),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(11));

-- Location: FF_X21_Y22_N21
\inst7|t_0[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(10),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(10));

-- Location: FF_X21_Y22_N19
\inst7|t_0[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(9),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(9));

-- Location: FF_X21_Y22_N17
\inst7|t_0[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(8),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(8));

-- Location: FF_X21_Y22_N15
\inst7|t_0[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(7),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(7));

-- Location: LCCOMB_X21_Y22_N14
\inst7|Add0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~14_combout\ = (\inst33|temp\(7) & ((\inst7|t_0\(7) & (!\inst7|Add0~13\)) # (!\inst7|t_0\(7) & (\inst7|Add0~13\ & VCC)))) # (!\inst33|temp\(7) & ((\inst7|t_0\(7) & ((\inst7|Add0~13\) # (GND))) # (!\inst7|t_0\(7) & (!\inst7|Add0~13\))))
-- \inst7|Add0~15\ = CARRY((\inst33|temp\(7) & (\inst7|t_0\(7) & !\inst7|Add0~13\)) # (!\inst33|temp\(7) & ((\inst7|t_0\(7)) # (!\inst7|Add0~13\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(7),
	datab => \inst7|t_0\(7),
	datad => VCC,
	cin => \inst7|Add0~13\,
	combout => \inst7|Add0~14_combout\,
	cout => \inst7|Add0~15\);

-- Location: LCCOMB_X21_Y22_N16
\inst7|Add0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~16_combout\ = ((\inst33|temp\(8) $ (\inst7|t_0\(8) $ (\inst7|Add0~15\)))) # (GND)
-- \inst7|Add0~17\ = CARRY((\inst33|temp\(8) & ((!\inst7|Add0~15\) # (!\inst7|t_0\(8)))) # (!\inst33|temp\(8) & (!\inst7|t_0\(8) & !\inst7|Add0~15\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(8),
	datab => \inst7|t_0\(8),
	datad => VCC,
	cin => \inst7|Add0~15\,
	combout => \inst7|Add0~16_combout\,
	cout => \inst7|Add0~17\);

-- Location: LCCOMB_X21_Y22_N18
\inst7|Add0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~18_combout\ = (\inst33|temp\(9) & ((\inst7|t_0\(9) & (!\inst7|Add0~17\)) # (!\inst7|t_0\(9) & (\inst7|Add0~17\ & VCC)))) # (!\inst33|temp\(9) & ((\inst7|t_0\(9) & ((\inst7|Add0~17\) # (GND))) # (!\inst7|t_0\(9) & (!\inst7|Add0~17\))))
-- \inst7|Add0~19\ = CARRY((\inst33|temp\(9) & (\inst7|t_0\(9) & !\inst7|Add0~17\)) # (!\inst33|temp\(9) & ((\inst7|t_0\(9)) # (!\inst7|Add0~17\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(9),
	datab => \inst7|t_0\(9),
	datad => VCC,
	cin => \inst7|Add0~17\,
	combout => \inst7|Add0~18_combout\,
	cout => \inst7|Add0~19\);

-- Location: LCCOMB_X21_Y22_N20
\inst7|Add0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~20_combout\ = ((\inst33|temp\(10) $ (\inst7|t_0\(10) $ (\inst7|Add0~19\)))) # (GND)
-- \inst7|Add0~21\ = CARRY((\inst33|temp\(10) & ((!\inst7|Add0~19\) # (!\inst7|t_0\(10)))) # (!\inst33|temp\(10) & (!\inst7|t_0\(10) & !\inst7|Add0~19\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(10),
	datab => \inst7|t_0\(10),
	datad => VCC,
	cin => \inst7|Add0~19\,
	combout => \inst7|Add0~20_combout\,
	cout => \inst7|Add0~21\);

-- Location: LCCOMB_X21_Y22_N22
\inst7|Add0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~22_combout\ = (\inst7|t_0\(11) & ((\inst33|temp\(11) & (!\inst7|Add0~21\)) # (!\inst33|temp\(11) & ((\inst7|Add0~21\) # (GND))))) # (!\inst7|t_0\(11) & ((\inst33|temp\(11) & (\inst7|Add0~21\ & VCC)) # (!\inst33|temp\(11) & 
-- (!\inst7|Add0~21\))))
-- \inst7|Add0~23\ = CARRY((\inst7|t_0\(11) & ((!\inst7|Add0~21\) # (!\inst33|temp\(11)))) # (!\inst7|t_0\(11) & (!\inst33|temp\(11) & !\inst7|Add0~21\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(11),
	datab => \inst33|temp\(11),
	datad => VCC,
	cin => \inst7|Add0~21\,
	combout => \inst7|Add0~22_combout\,
	cout => \inst7|Add0~23\);

-- Location: LCCOMB_X21_Y22_N24
\inst7|Add0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~24_combout\ = ((\inst33|temp\(12) $ (\inst7|t_0\(12) $ (\inst7|Add0~23\)))) # (GND)
-- \inst7|Add0~25\ = CARRY((\inst33|temp\(12) & ((!\inst7|Add0~23\) # (!\inst7|t_0\(12)))) # (!\inst33|temp\(12) & (!\inst7|t_0\(12) & !\inst7|Add0~23\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(12),
	datab => \inst7|t_0\(12),
	datad => VCC,
	cin => \inst7|Add0~23\,
	combout => \inst7|Add0~24_combout\,
	cout => \inst7|Add0~25\);

-- Location: LCCOMB_X21_Y22_N26
\inst7|Add0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~26_combout\ = (\inst33|temp\(13) & ((\inst7|t_0\(13) & (!\inst7|Add0~25\)) # (!\inst7|t_0\(13) & (\inst7|Add0~25\ & VCC)))) # (!\inst33|temp\(13) & ((\inst7|t_0\(13) & ((\inst7|Add0~25\) # (GND))) # (!\inst7|t_0\(13) & (!\inst7|Add0~25\))))
-- \inst7|Add0~27\ = CARRY((\inst33|temp\(13) & (\inst7|t_0\(13) & !\inst7|Add0~25\)) # (!\inst33|temp\(13) & ((\inst7|t_0\(13)) # (!\inst7|Add0~25\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(13),
	datab => \inst7|t_0\(13),
	datad => VCC,
	cin => \inst7|Add0~25\,
	combout => \inst7|Add0~26_combout\,
	cout => \inst7|Add0~27\);

-- Location: LCCOMB_X21_Y22_N28
\inst7|Add0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~28_combout\ = ((\inst33|temp\(14) $ (\inst7|t_0\(14) $ (\inst7|Add0~27\)))) # (GND)
-- \inst7|Add0~29\ = CARRY((\inst33|temp\(14) & ((!\inst7|Add0~27\) # (!\inst7|t_0\(14)))) # (!\inst33|temp\(14) & (!\inst7|t_0\(14) & !\inst7|Add0~27\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(14),
	datab => \inst7|t_0\(14),
	datad => VCC,
	cin => \inst7|Add0~27\,
	combout => \inst7|Add0~28_combout\,
	cout => \inst7|Add0~29\);

-- Location: LCCOMB_X21_Y22_N30
\inst7|Add0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~30_combout\ = (\inst7|t_0\(15) & ((\inst33|temp\(15) & (!\inst7|Add0~29\)) # (!\inst33|temp\(15) & ((\inst7|Add0~29\) # (GND))))) # (!\inst7|t_0\(15) & ((\inst33|temp\(15) & (\inst7|Add0~29\ & VCC)) # (!\inst33|temp\(15) & 
-- (!\inst7|Add0~29\))))
-- \inst7|Add0~31\ = CARRY((\inst7|t_0\(15) & ((!\inst7|Add0~29\) # (!\inst33|temp\(15)))) # (!\inst7|t_0\(15) & (!\inst33|temp\(15) & !\inst7|Add0~29\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(15),
	datab => \inst33|temp\(15),
	datad => VCC,
	cin => \inst7|Add0~29\,
	combout => \inst7|Add0~30_combout\,
	cout => \inst7|Add0~31\);

-- Location: LCCOMB_X21_Y21_N0
\inst7|Add0~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~32_combout\ = ((\inst33|temp\(16) $ (\inst7|t_0\(16) $ (\inst7|Add0~31\)))) # (GND)
-- \inst7|Add0~33\ = CARRY((\inst33|temp\(16) & ((!\inst7|Add0~31\) # (!\inst7|t_0\(16)))) # (!\inst33|temp\(16) & (!\inst7|t_0\(16) & !\inst7|Add0~31\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(16),
	datab => \inst7|t_0\(16),
	datad => VCC,
	cin => \inst7|Add0~31\,
	combout => \inst7|Add0~32_combout\,
	cout => \inst7|Add0~33\);

-- Location: LCCOMB_X21_Y21_N2
\inst7|Add0~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~34_combout\ = (\inst33|temp\(17) & ((\inst7|t_0\(17) & (!\inst7|Add0~33\)) # (!\inst7|t_0\(17) & (\inst7|Add0~33\ & VCC)))) # (!\inst33|temp\(17) & ((\inst7|t_0\(17) & ((\inst7|Add0~33\) # (GND))) # (!\inst7|t_0\(17) & (!\inst7|Add0~33\))))
-- \inst7|Add0~35\ = CARRY((\inst33|temp\(17) & (\inst7|t_0\(17) & !\inst7|Add0~33\)) # (!\inst33|temp\(17) & ((\inst7|t_0\(17)) # (!\inst7|Add0~33\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(17),
	datab => \inst7|t_0\(17),
	datad => VCC,
	cin => \inst7|Add0~33\,
	combout => \inst7|Add0~34_combout\,
	cout => \inst7|Add0~35\);

-- Location: FF_X21_Y21_N5
\inst7|t_0[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(18),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(18));

-- Location: LCCOMB_X21_Y21_N4
\inst7|Add0~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~36_combout\ = ((\inst7|t_0\(18) $ (\inst33|temp\(18) $ (\inst7|Add0~35\)))) # (GND)
-- \inst7|Add0~37\ = CARRY((\inst7|t_0\(18) & (\inst33|temp\(18) & !\inst7|Add0~35\)) # (!\inst7|t_0\(18) & ((\inst33|temp\(18)) # (!\inst7|Add0~35\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(18),
	datab => \inst33|temp\(18),
	datad => VCC,
	cin => \inst7|Add0~35\,
	combout => \inst7|Add0~36_combout\,
	cout => \inst7|Add0~37\);

-- Location: LCCOMB_X21_Y19_N6
\inst7|data_available~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~2_combout\ = (!\inst7|Add0~34_combout\ & (!\inst7|Add0~32_combout\ & (!\inst7|Add0~30_combout\ & !\inst7|Add0~36_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~34_combout\,
	datab => \inst7|Add0~32_combout\,
	datac => \inst7|Add0~30_combout\,
	datad => \inst7|Add0~36_combout\,
	combout => \inst7|data_available~2_combout\);

-- Location: FF_X21_Y21_N7
\inst7|t_0[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(19),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(19));

-- Location: LCCOMB_X21_Y21_N6
\inst7|Add0~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~38_combout\ = (\inst7|t_0\(19) & ((\inst33|temp\(19) & (!\inst7|Add0~37\)) # (!\inst33|temp\(19) & ((\inst7|Add0~37\) # (GND))))) # (!\inst7|t_0\(19) & ((\inst33|temp\(19) & (\inst7|Add0~37\ & VCC)) # (!\inst33|temp\(19) & 
-- (!\inst7|Add0~37\))))
-- \inst7|Add0~39\ = CARRY((\inst7|t_0\(19) & ((!\inst7|Add0~37\) # (!\inst33|temp\(19)))) # (!\inst7|t_0\(19) & (!\inst33|temp\(19) & !\inst7|Add0~37\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(19),
	datab => \inst33|temp\(19),
	datad => VCC,
	cin => \inst7|Add0~37\,
	combout => \inst7|Add0~38_combout\,
	cout => \inst7|Add0~39\);

-- Location: FF_X21_Y21_N11
\inst7|t_0[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(21),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(21));

-- Location: FF_X21_Y21_N9
\inst7|t_0[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(20),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(20));

-- Location: LCCOMB_X21_Y21_N8
\inst7|Add0~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~40_combout\ = ((\inst33|temp\(20) $ (\inst7|t_0\(20) $ (\inst7|Add0~39\)))) # (GND)
-- \inst7|Add0~41\ = CARRY((\inst33|temp\(20) & ((!\inst7|Add0~39\) # (!\inst7|t_0\(20)))) # (!\inst33|temp\(20) & (!\inst7|t_0\(20) & !\inst7|Add0~39\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(20),
	datab => \inst7|t_0\(20),
	datad => VCC,
	cin => \inst7|Add0~39\,
	combout => \inst7|Add0~40_combout\,
	cout => \inst7|Add0~41\);

-- Location: LCCOMB_X21_Y21_N10
\inst7|Add0~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~42_combout\ = (\inst7|t_0\(21) & ((\inst33|temp\(21) & (!\inst7|Add0~41\)) # (!\inst33|temp\(21) & ((\inst7|Add0~41\) # (GND))))) # (!\inst7|t_0\(21) & ((\inst33|temp\(21) & (\inst7|Add0~41\ & VCC)) # (!\inst33|temp\(21) & 
-- (!\inst7|Add0~41\))))
-- \inst7|Add0~43\ = CARRY((\inst7|t_0\(21) & ((!\inst7|Add0~41\) # (!\inst33|temp\(21)))) # (!\inst7|t_0\(21) & (!\inst33|temp\(21) & !\inst7|Add0~41\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(21),
	datab => \inst33|temp\(21),
	datad => VCC,
	cin => \inst7|Add0~41\,
	combout => \inst7|Add0~42_combout\,
	cout => \inst7|Add0~43\);

-- Location: FF_X21_Y21_N13
\inst7|t_0[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(22),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(22));

-- Location: LCCOMB_X21_Y21_N12
\inst7|Add0~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~44_combout\ = ((\inst7|t_0\(22) $ (\inst33|temp\(22) $ (\inst7|Add0~43\)))) # (GND)
-- \inst7|Add0~45\ = CARRY((\inst7|t_0\(22) & (\inst33|temp\(22) & !\inst7|Add0~43\)) # (!\inst7|t_0\(22) & ((\inst33|temp\(22)) # (!\inst7|Add0~43\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(22),
	datab => \inst33|temp\(22),
	datad => VCC,
	cin => \inst7|Add0~43\,
	combout => \inst7|Add0~44_combout\,
	cout => \inst7|Add0~45\);

-- Location: LCCOMB_X21_Y19_N16
\inst7|data_available~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~3_combout\ = (!\inst7|Add0~38_combout\ & (!\inst7|Add0~42_combout\ & (!\inst7|Add0~40_combout\ & !\inst7|Add0~44_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~38_combout\,
	datab => \inst7|Add0~42_combout\,
	datac => \inst7|Add0~40_combout\,
	datad => \inst7|Add0~44_combout\,
	combout => \inst7|data_available~3_combout\);

-- Location: LCCOMB_X21_Y19_N4
\inst7|data_available~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~1_combout\ = (!\inst7|Add0~22_combout\ & (!\inst7|Add0~24_combout\ & (!\inst7|Add0~26_combout\ & !\inst7|Add0~28_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~22_combout\,
	datab => \inst7|Add0~24_combout\,
	datac => \inst7|Add0~26_combout\,
	datad => \inst7|Add0~28_combout\,
	combout => \inst7|data_available~1_combout\);

-- Location: LCCOMB_X21_Y19_N10
\inst7|data_available~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~0_combout\ = (!\inst7|Add0~18_combout\ & (!\inst7|Add0~16_combout\ & (!\inst7|Add0~20_combout\ & !\inst7|Add0~14_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~18_combout\,
	datab => \inst7|Add0~16_combout\,
	datac => \inst7|Add0~20_combout\,
	datad => \inst7|Add0~14_combout\,
	combout => \inst7|data_available~0_combout\);

-- Location: LCCOMB_X21_Y19_N26
\inst7|data_available~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~4_combout\ = (\inst7|data_available~2_combout\ & (\inst7|data_available~3_combout\ & (\inst7|data_available~1_combout\ & \inst7|data_available~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|data_available~2_combout\,
	datab => \inst7|data_available~3_combout\,
	datac => \inst7|data_available~1_combout\,
	datad => \inst7|data_available~0_combout\,
	combout => \inst7|data_available~4_combout\);

-- Location: FF_X21_Y21_N21
\inst7|t_0[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(26),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(26));

-- Location: FF_X21_Y21_N19
\inst7|t_0[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(25),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(25));

-- Location: FF_X21_Y21_N17
\inst7|t_0[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(24),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(24));

-- Location: FF_X21_Y21_N15
\inst7|t_0[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(23),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(23));

-- Location: LCCOMB_X21_Y21_N14
\inst7|Add0~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~46_combout\ = (\inst7|t_0\(23) & ((\inst33|temp\(23) & (!\inst7|Add0~45\)) # (!\inst33|temp\(23) & ((\inst7|Add0~45\) # (GND))))) # (!\inst7|t_0\(23) & ((\inst33|temp\(23) & (\inst7|Add0~45\ & VCC)) # (!\inst33|temp\(23) & 
-- (!\inst7|Add0~45\))))
-- \inst7|Add0~47\ = CARRY((\inst7|t_0\(23) & ((!\inst7|Add0~45\) # (!\inst33|temp\(23)))) # (!\inst7|t_0\(23) & (!\inst33|temp\(23) & !\inst7|Add0~45\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(23),
	datab => \inst33|temp\(23),
	datad => VCC,
	cin => \inst7|Add0~45\,
	combout => \inst7|Add0~46_combout\,
	cout => \inst7|Add0~47\);

-- Location: LCCOMB_X21_Y21_N16
\inst7|Add0~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~48_combout\ = ((\inst33|temp\(24) $ (\inst7|t_0\(24) $ (\inst7|Add0~47\)))) # (GND)
-- \inst7|Add0~49\ = CARRY((\inst33|temp\(24) & ((!\inst7|Add0~47\) # (!\inst7|t_0\(24)))) # (!\inst33|temp\(24) & (!\inst7|t_0\(24) & !\inst7|Add0~47\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(24),
	datab => \inst7|t_0\(24),
	datad => VCC,
	cin => \inst7|Add0~47\,
	combout => \inst7|Add0~48_combout\,
	cout => \inst7|Add0~49\);

-- Location: LCCOMB_X21_Y21_N18
\inst7|Add0~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~50_combout\ = (\inst33|temp\(25) & ((\inst7|t_0\(25) & (!\inst7|Add0~49\)) # (!\inst7|t_0\(25) & (\inst7|Add0~49\ & VCC)))) # (!\inst33|temp\(25) & ((\inst7|t_0\(25) & ((\inst7|Add0~49\) # (GND))) # (!\inst7|t_0\(25) & (!\inst7|Add0~49\))))
-- \inst7|Add0~51\ = CARRY((\inst33|temp\(25) & (\inst7|t_0\(25) & !\inst7|Add0~49\)) # (!\inst33|temp\(25) & ((\inst7|t_0\(25)) # (!\inst7|Add0~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(25),
	datab => \inst7|t_0\(25),
	datad => VCC,
	cin => \inst7|Add0~49\,
	combout => \inst7|Add0~50_combout\,
	cout => \inst7|Add0~51\);

-- Location: LCCOMB_X21_Y21_N20
\inst7|Add0~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~52_combout\ = ((\inst33|temp\(26) $ (\inst7|t_0\(26) $ (\inst7|Add0~51\)))) # (GND)
-- \inst7|Add0~53\ = CARRY((\inst33|temp\(26) & ((!\inst7|Add0~51\) # (!\inst7|t_0\(26)))) # (!\inst33|temp\(26) & (!\inst7|t_0\(26) & !\inst7|Add0~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(26),
	datab => \inst7|t_0\(26),
	datad => VCC,
	cin => \inst7|Add0~51\,
	combout => \inst7|Add0~52_combout\,
	cout => \inst7|Add0~53\);

-- Location: LCCOMB_X21_Y19_N28
\inst7|data_available~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~5_combout\ = (!\inst7|Add0~52_combout\ & (!\inst7|Add0~48_combout\ & (!\inst7|Add0~50_combout\ & !\inst7|Add0~46_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~52_combout\,
	datab => \inst7|Add0~48_combout\,
	datac => \inst7|Add0~50_combout\,
	datad => \inst7|Add0~46_combout\,
	combout => \inst7|data_available~5_combout\);

-- Location: FF_X21_Y21_N25
\inst7|t_0[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(28),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(28));

-- Location: FF_X21_Y21_N23
\inst7|t_0[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(27),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(27));

-- Location: LCCOMB_X21_Y21_N22
\inst7|Add0~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~54_combout\ = (\inst33|temp\(27) & ((\inst7|t_0\(27) & (!\inst7|Add0~53\)) # (!\inst7|t_0\(27) & (\inst7|Add0~53\ & VCC)))) # (!\inst33|temp\(27) & ((\inst7|t_0\(27) & ((\inst7|Add0~53\) # (GND))) # (!\inst7|t_0\(27) & (!\inst7|Add0~53\))))
-- \inst7|Add0~55\ = CARRY((\inst33|temp\(27) & (\inst7|t_0\(27) & !\inst7|Add0~53\)) # (!\inst33|temp\(27) & ((\inst7|t_0\(27)) # (!\inst7|Add0~53\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(27),
	datab => \inst7|t_0\(27),
	datad => VCC,
	cin => \inst7|Add0~53\,
	combout => \inst7|Add0~54_combout\,
	cout => \inst7|Add0~55\);

-- Location: LCCOMB_X21_Y21_N24
\inst7|Add0~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~56_combout\ = ((\inst33|temp\(28) $ (\inst7|t_0\(28) $ (\inst7|Add0~55\)))) # (GND)
-- \inst7|Add0~57\ = CARRY((\inst33|temp\(28) & ((!\inst7|Add0~55\) # (!\inst7|t_0\(28)))) # (!\inst33|temp\(28) & (!\inst7|t_0\(28) & !\inst7|Add0~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(28),
	datab => \inst7|t_0\(28),
	datad => VCC,
	cin => \inst7|Add0~55\,
	combout => \inst7|Add0~56_combout\,
	cout => \inst7|Add0~57\);

-- Location: FF_X21_Y21_N27
\inst7|t_0[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(29),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(29));

-- Location: LCCOMB_X21_Y21_N26
\inst7|Add0~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~58_combout\ = (\inst7|t_0\(29) & ((\inst33|temp\(29) & (!\inst7|Add0~57\)) # (!\inst33|temp\(29) & ((\inst7|Add0~57\) # (GND))))) # (!\inst7|t_0\(29) & ((\inst33|temp\(29) & (\inst7|Add0~57\ & VCC)) # (!\inst33|temp\(29) & 
-- (!\inst7|Add0~57\))))
-- \inst7|Add0~59\ = CARRY((\inst7|t_0\(29) & ((!\inst7|Add0~57\) # (!\inst33|temp\(29)))) # (!\inst7|t_0\(29) & (!\inst33|temp\(29) & !\inst7|Add0~57\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(29),
	datab => \inst33|temp\(29),
	datad => VCC,
	cin => \inst7|Add0~57\,
	combout => \inst7|Add0~58_combout\,
	cout => \inst7|Add0~59\);

-- Location: FF_X21_Y21_N29
\inst7|t_0[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(30),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(30));

-- Location: LCCOMB_X21_Y21_N28
\inst7|Add0~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~60_combout\ = ((\inst33|temp\(30) $ (\inst7|t_0\(30) $ (\inst7|Add0~59\)))) # (GND)
-- \inst7|Add0~61\ = CARRY((\inst33|temp\(30) & ((!\inst7|Add0~59\) # (!\inst7|t_0\(30)))) # (!\inst33|temp\(30) & (!\inst7|t_0\(30) & !\inst7|Add0~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst33|temp\(30),
	datab => \inst7|t_0\(30),
	datad => VCC,
	cin => \inst7|Add0~59\,
	combout => \inst7|Add0~60_combout\,
	cout => \inst7|Add0~61\);

-- Location: LCCOMB_X21_Y19_N22
\inst7|data_available~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~6_combout\ = (!\inst7|Add0~56_combout\ & (!\inst7|Add0~58_combout\ & (!\inst7|Add0~60_combout\ & !\inst7|Add0~54_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|Add0~56_combout\,
	datab => \inst7|Add0~58_combout\,
	datac => \inst7|Add0~60_combout\,
	datad => \inst7|Add0~54_combout\,
	combout => \inst7|data_available~6_combout\);

-- Location: FF_X21_Y21_N31
\inst7|t_0[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \sensor1_signal~inputclkctrl_outclk\,
	asdata => \inst33|temp\(31),
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|t_0\(31));

-- Location: LCCOMB_X21_Y21_N30
\inst7|Add0~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|Add0~62_combout\ = \inst7|t_0\(31) $ (\inst33|temp\(31) $ (!\inst7|Add0~61\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101101001",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|t_0\(31),
	datab => \inst33|temp\(31),
	cin => \inst7|Add0~61\,
	combout => \inst7|Add0~62_combout\);

-- Location: LCCOMB_X21_Y19_N0
\inst7|data_available~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~7_combout\ = (\inst7|data_available~4_combout\ & (\inst7|data_available~5_combout\ & (\inst7|data_available~6_combout\ & !\inst7|Add0~62_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000010000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|data_available~4_combout\,
	datab => \inst7|data_available~5_combout\,
	datac => \inst7|data_available~6_combout\,
	datad => \inst7|Add0~62_combout\,
	combout => \inst7|data_available~7_combout\);

-- Location: LCCOMB_X21_Y19_N24
\inst7|data_available~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst7|data_available~10_combout\ = (\inst7|data_available~7_combout\ & (\inst7|Add0~12_combout\ $ (((\inst7|data_available~9_combout\ & \inst7|Add0~10_combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100100011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|data_available~9_combout\,
	datab => \inst7|data_available~7_combout\,
	datac => \inst7|Add0~12_combout\,
	datad => \inst7|Add0~10_combout\,
	combout => \inst7|data_available~10_combout\);

-- Location: FF_X21_Y19_N25
\inst7|data_available\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_sensor1_signal~inputclkctrl_outclk\,
	d => \inst7|data_available~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst7|data_available~q\);

-- Location: LCCOMB_X26_Y26_N8
\inst18|cur_value~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst18|cur_value~feeder_combout\ = \inst7|data_available~q\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst7|data_available~q\,
	combout => \inst18|cur_value~feeder_combout\);

-- Location: FF_X26_Y26_N9
\inst18|cur_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst18|cur_value~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst18|cur_value~q\);

-- Location: FF_X26_Y26_N11
\inst18|last_value\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst18|cur_value~q\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst18|last_value~q\);

-- Location: LCCOMB_X26_Y26_N10
\inst18|level_sig\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst18|level_sig~combout\ = (\inst18|cur_value~q\ & !\inst18|last_value~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000110000001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst18|cur_value~q\,
	datac => \inst18|last_value~q\,
	combout => \inst18|level_sig~combout\);

-- Location: LCCOMB_X26_Y26_N14
\inst16|WideOr0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|WideOr0~0_combout\ = (\inst19|level_sig~combout\ & (!\inst6|data_available~q\ & (!\inst20|level_sig~combout\ & !\inst18|level_sig~combout\))) # (!\inst19|level_sig~combout\ & ((\inst6|data_available~q\ & (!\inst20|level_sig~combout\ & 
-- !\inst18|level_sig~combout\)) # (!\inst6|data_available~q\ & (\inst20|level_sig~combout\ $ (\inst18|level_sig~combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000100010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst19|level_sig~combout\,
	datab => \inst6|data_available~q\,
	datac => \inst20|level_sig~combout\,
	datad => \inst18|level_sig~combout\,
	combout => \inst16|WideOr0~0_combout\);

-- Location: LCCOMB_X23_Y27_N16
\inst16|write~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|write~2_combout\ = (!\inst24|level_sig~combout\ & (!\inst23|level_sig~combout\ & (\inst16|write~0_combout\ & \inst16|WideOr0~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst24|level_sig~combout\,
	datab => \inst23|level_sig~combout\,
	datac => \inst16|write~0_combout\,
	datad => \inst16|WideOr0~0_combout\,
	combout => \inst16|write~2_combout\);

-- Location: LCCOMB_X26_Y26_N12
\inst16|write~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|write~1_combout\ = (!\inst19|level_sig~combout\ & (!\inst6|data_available~q\ & (!\inst20|level_sig~combout\ & !\inst18|level_sig~combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst19|level_sig~combout\,
	datab => \inst6|data_available~q\,
	datac => \inst20|level_sig~combout\,
	datad => \inst18|level_sig~combout\,
	combout => \inst16|write~1_combout\);

-- Location: LCCOMB_X23_Y27_N30
\inst16|LessThan0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|LessThan0~0_combout\ = (\inst24|level_sig~combout\) # ((\inst23|level_sig~combout\) # ((!\inst16|write~1_combout\) # (!\inst16|write~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110111111111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst24|level_sig~combout\,
	datab => \inst23|level_sig~combout\,
	datac => \inst16|write~0_combout\,
	datad => \inst16|write~1_combout\,
	combout => \inst16|LessThan0~0_combout\);

-- Location: LCCOMB_X23_Y27_N4
\inst16|write~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|write~4_combout\ = (\inst16|LessThan0~0_combout\ & ((\inst16|write~2_combout\) # ((\inst16|write~3_combout\ & \inst16|write~1_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110000011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|write~3_combout\,
	datab => \inst16|write~2_combout\,
	datac => \inst16|LessThan0~0_combout\,
	datad => \inst16|write~1_combout\,
	combout => \inst16|write~4_combout\);

-- Location: FF_X23_Y23_N1
\inst16|write\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|write~4_combout\,
	sload => VCC,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|write~q\);

-- Location: LCCOMB_X24_Y22_N10
\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~0_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0_combout\ & (!\inst27~0_combout\ & ((!\inst16|write~q\) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000001001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_1~3_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_0~0_combout\,
	datac => \inst16|write~q\,
	datad => \inst27~0_combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~0_combout\);

-- Location: FF_X24_Y22_N11
\inst3|scfifo_component|auto_generated|dpfifo|empty_dff\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\);

-- Location: LCCOMB_X27_Y22_N4
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\ = (\inst27~0_combout\) # (\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ $ (((\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & 
-- \inst11|fifo_read~q\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111101101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datac => \inst11|fifo_read~q\,
	datad => \inst27~0_combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\);

-- Location: FF_X28_Y22_N5
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0));

-- Location: LCCOMB_X28_Y22_N6
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~COUT\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1) $ (((\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\) # (VCC))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~COUT\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1)) # (GND))))
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ $ (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110001101111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita0~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~COUT\);

-- Location: FF_X28_Y22_N7
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita1~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1));

-- Location: FF_X28_Y22_N9
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita2~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2));

-- Location: LCCOMB_X28_Y22_N26
\inst3|scfifo_component|auto_generated|dpfifo|full_dff~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|full_dff~3_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2) & \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(2),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(3),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~3_combout\);

-- Location: LCCOMB_X24_Y22_N4
\inst3|scfifo_component|auto_generated|dpfifo|full_dff~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|full_dff~0_combout\ = ((\inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\) # ((\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & \inst11|fifo_read~q\))) # (!\button0~input_o\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101111110011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datab => \button0~input_o\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	datad => \inst11|fifo_read~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~0_combout\);

-- Location: LCCOMB_X28_Y22_N0
\inst3|scfifo_component|auto_generated|dpfifo|full_dff~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|full_dff~2_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4) & (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6) & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5) & \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~2_combout\);

-- Location: LCCOMB_X25_Y22_N8
\inst3|scfifo_component|auto_generated|dpfifo|full_dff~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|full_dff~1_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1) & (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8) & (\inst16|write~q\ & 
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8),
	datac => \inst16|write~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(0),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~1_combout\);

-- Location: LCCOMB_X24_Y22_N16
\inst3|scfifo_component|auto_generated|dpfifo|full_dff~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|full_dff~4_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|full_dff~3_combout\ & (!\inst3|scfifo_component|auto_generated|dpfifo|full_dff~0_combout\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|full_dff~2_combout\ & \inst3|scfifo_component|auto_generated|dpfifo|full_dff~1_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~3_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~2_combout\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~1_combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~4_combout\);

-- Location: FF_X24_Y22_N17
\inst3|scfifo_component|auto_generated|dpfifo|full_dff\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\);

-- Location: LCCOMB_X23_Y23_N0
\inst3|scfifo_component|auto_generated|dpfifo|valid_wreq\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\ = (!\inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\ & \inst16|write~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101000001010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	datac => \inst16|write~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\);

-- Location: FF_X28_Y22_N13
\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_comb_bita4~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|_~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4));

-- Location: LCCOMB_X28_Y22_N28
\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6_combout\ = (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6) & (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7) & 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5) & !\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(6),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(7),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(5),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(8),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6_combout\);

-- Location: LCCOMB_X30_Y22_N12
\inst11|always0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|always0~0_combout\ = (!\inst11|LessThan0~1_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4)) # ((\inst11|LessThan2~0_combout\) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000011101111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4),
	datab => \inst11|LessThan2~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6_combout\,
	datad => \inst11|LessThan0~1_combout\,
	combout => \inst11|always0~0_combout\);

-- Location: LCCOMB_X31_Y22_N24
\inst11|wren~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|wren~0_combout\ = (\inst11|wren~q\ & ((\inst11|write_ack_prev~q\) # (!\inst2|wr_ack_reg~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|write_ack_prev~q\,
	datac => \inst2|wr_ack_reg~q\,
	datad => \inst11|wren~q\,
	combout => \inst11|wren~0_combout\);

-- Location: LCCOMB_X31_Y22_N26
\inst11|wren~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|wren~1_combout\ = (\inst11|always0~0_combout\) # ((\inst11|wren~0_combout\) # ((\inst11|LessThan0~1_combout\ & !\inst2|di_req_o_reg~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111110101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|always0~0_combout\,
	datab => \inst11|LessThan0~1_combout\,
	datac => \inst2|di_req_o_reg~q\,
	datad => \inst11|wren~0_combout\,
	combout => \inst11|wren~1_combout\);

-- Location: FF_X31_Y22_N27
\inst11|wren\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|wren~1_combout\,
	clrn => \button0~input_o\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|wren~q\);

-- Location: LCCOMB_X31_Y22_N8
\inst2|wren~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|wren~0_combout\ = (\inst11|wren~q\) # ((!\inst2|wr_ack_reg~q\ & \inst2|wren~q\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111101010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wr_ack_reg~q\,
	datac => \inst2|wren~q\,
	datad => \inst11|wren~q\,
	combout => \inst2|wren~0_combout\);

-- Location: FF_X31_Y22_N9
\inst2|wren\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|wren~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|wren~q\);

-- Location: LCCOMB_X32_Y22_N28
\inst2|Equal4~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Equal4~0_combout\ = (!\inst2|state_reg\(3) & (!\inst2|state_reg\(0) & (!\inst2|state_reg\(2) & !\inst2|state_reg\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|state_reg\(0),
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|Equal4~0_combout\);

-- Location: LCCOMB_X23_Y26_N24
\inst16|counter[0]~93\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[0]~93_combout\ = \inst16|counter\(0) $ (\inst16|LessThan0~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst16|counter\(0),
	datad => \inst16|LessThan0~0_combout\,
	combout => \inst16|counter[0]~93_combout\);

-- Location: FF_X23_Y26_N25
\inst16|counter[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[0]~93_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(0));

-- Location: LCCOMB_X24_Y24_N2
\inst16|counter[1]~31\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[1]~31_combout\ = (\inst16|counter\(0) & (\inst16|counter\(1) $ (VCC))) # (!\inst16|counter\(0) & (\inst16|counter\(1) & VCC))
-- \inst16|counter[1]~32\ = CARRY((\inst16|counter\(0) & \inst16|counter\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(0),
	datab => \inst16|counter\(1),
	datad => VCC,
	combout => \inst16|counter[1]~31_combout\,
	cout => \inst16|counter[1]~32\);

-- Location: FF_X24_Y24_N3
\inst16|counter[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[1]~31_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(1));

-- Location: LCCOMB_X24_Y24_N4
\inst16|counter[2]~33\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[2]~33_combout\ = (\inst16|counter\(2) & (!\inst16|counter[1]~32\)) # (!\inst16|counter\(2) & ((\inst16|counter[1]~32\) # (GND)))
-- \inst16|counter[2]~34\ = CARRY((!\inst16|counter[1]~32\) # (!\inst16|counter\(2)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(2),
	datad => VCC,
	cin => \inst16|counter[1]~32\,
	combout => \inst16|counter[2]~33_combout\,
	cout => \inst16|counter[2]~34\);

-- Location: FF_X24_Y24_N5
\inst16|counter[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[2]~33_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(2));

-- Location: LCCOMB_X24_Y24_N6
\inst16|counter[3]~35\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[3]~35_combout\ = (\inst16|counter\(3) & (\inst16|counter[2]~34\ $ (GND))) # (!\inst16|counter\(3) & (!\inst16|counter[2]~34\ & VCC))
-- \inst16|counter[3]~36\ = CARRY((\inst16|counter\(3) & !\inst16|counter[2]~34\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(3),
	datad => VCC,
	cin => \inst16|counter[2]~34\,
	combout => \inst16|counter[3]~35_combout\,
	cout => \inst16|counter[3]~36\);

-- Location: FF_X24_Y24_N7
\inst16|counter[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[3]~35_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(3));

-- Location: LCCOMB_X23_Y23_N30
\inst16|sensor_value_out[3]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[3]~feeder_combout\ = \inst16|counter\(3)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(3),
	combout => \inst16|sensor_value_out[3]~feeder_combout\);

-- Location: FF_X23_Y23_N31
\inst16|sensor_value_out[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[3]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(3));

-- Location: LCCOMB_X25_Y22_N10
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~combout\ = \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(0) $ (VCC)
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~COUT\ = CARRY(\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010110101010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(0),
	datad => VCC,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~COUT\);

-- Location: LCCOMB_X25_Y22_N2
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\) # ((\inst16|write~q\) # (!\button0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101011111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	datac => \inst16|write~q\,
	datad => \button0~input_o\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\);

-- Location: FF_X25_Y22_N11
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(0));

-- Location: LCCOMB_X25_Y22_N12
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(1) & (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~COUT\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(1) & ((\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~COUT\) # (GND)))
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~COUT\) # (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(1),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita0~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~COUT\);

-- Location: FF_X25_Y22_N13
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(1));

-- Location: LCCOMB_X25_Y22_N14
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(2) & (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~COUT\ $ (GND))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(2) & (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~COUT\ & VCC))
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(2) & !\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(2),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita1~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~COUT\);

-- Location: FF_X25_Y22_N15
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(2));

-- Location: LCCOMB_X25_Y22_N16
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(3) & (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~COUT\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(3) & ((\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~COUT\) # (GND)))
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~COUT\) # (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(3),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita2~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~COUT\);

-- Location: FF_X25_Y22_N17
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(3));

-- Location: LCCOMB_X25_Y22_N18
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(4) & (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~COUT\ $ (GND))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(4) & (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~COUT\ & VCC))
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(4) & !\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(4),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita3~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~COUT\);

-- Location: FF_X25_Y22_N19
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(4));

-- Location: LCCOMB_X25_Y22_N20
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(5) & (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~COUT\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(5) & ((\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~COUT\) # (GND)))
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~COUT\) # (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(5)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(5),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita4~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~COUT\);

-- Location: FF_X25_Y22_N21
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(5));

-- Location: LCCOMB_X25_Y22_N22
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(6) & (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~COUT\ $ (GND))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(6) & (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~COUT\ & VCC))
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(6) & !\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(6),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita5~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~COUT\);

-- Location: FF_X25_Y22_N23
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(6));

-- Location: LCCOMB_X25_Y22_N24
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(7) & (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~COUT\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(7) & ((\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~COUT\) # (GND)))
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~COUT\) # (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(7)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(7),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita6~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~COUT\);

-- Location: FF_X25_Y22_N25
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(7));

-- Location: LCCOMB_X25_Y22_N26
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita8~combout\ = \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~COUT\ $ (!\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(8))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datad => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(8),
	cin => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita7~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita8~combout\);

-- Location: FF_X25_Y22_N27
\inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_comb_bita8~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|_~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|wr_ptr|counter_reg_bit\(8));

-- Location: LCCOMB_X24_Y22_N8
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~0_combout\ = (!\inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\ & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\ & \button0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000010100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\,
	datad => \button0~input_o\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~0_combout\);

-- Location: FF_X24_Y22_N9
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|full_dff~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\);

-- Location: LCCOMB_X24_Y22_N22
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[0]~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[0]~0_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(0))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001000100110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\,
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(0),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[0]~0_combout\);

-- Location: FF_X24_Y22_N23
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[0]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(0));

-- Location: LCCOMB_X24_Y22_N2
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[0]~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[0]~0_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\)) # (!\inst11|fifo_read~q\ 
-- & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(0)))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0111001011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(0),
	datad => \inst11|fifo_read~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[0]~0_combout\);

-- Location: LCCOMB_X26_Y22_N2
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~combout\ = \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0) $ (VCC)
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~COUT\ = CARRY(\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001111001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0),
	datad => VCC,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~COUT\);

-- Location: LCCOMB_X26_Y22_N22
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\ = (\inst27~0_combout\) # ((\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (\inst11|fifo_read~q\ & !\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110011101100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datab => \inst27~0_combout\,
	datac => \inst11|fifo_read~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_lsb~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\);

-- Location: FF_X26_Y22_N3
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0));

-- Location: LCCOMB_X27_Y22_N10
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[1]~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[1]~1_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(1))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010001000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0),
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(1),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[1]~1_combout\);

-- Location: FF_X27_Y22_N11
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[1]~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(1));

-- Location: LCCOMB_X27_Y22_N20
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[1]~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[1]~1_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0))) # 
-- (!\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(1)))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011111110000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(0),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datac => \inst11|fifo_read~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(1),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[1]~1_combout\);

-- Location: LCCOMB_X26_Y22_N4
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1) & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~COUT\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1) & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~COUT\) # (GND)))
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~COUT\) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita0~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~COUT\);

-- Location: FF_X26_Y22_N5
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1));

-- Location: LCCOMB_X27_Y22_N6
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[2]~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[2]~2_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(2))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100010001010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst27~0_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(2),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[2]~2_combout\);

-- Location: FF_X27_Y22_N7
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[2]~2_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(2));

-- Location: LCCOMB_X27_Y22_N24
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[2]~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[2]~2_combout\ = (\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1)))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(2))))) # (!\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(2)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100101010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(2),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(1),
	datac => \inst11|fifo_read~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[2]~2_combout\);

-- Location: LCCOMB_X26_Y22_N6
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2) & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~COUT\ $ 
-- (GND))) # (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2) & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~COUT\ & VCC))
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2) & !\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita1~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~COUT\);

-- Location: FF_X26_Y22_N7
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2));

-- Location: LCCOMB_X26_Y22_N0
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[3]~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[3]~3_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2)))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(3)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001000010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(3),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[3]~3_combout\);

-- Location: FF_X26_Y22_N1
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[3]~3_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(3));

-- Location: LCCOMB_X26_Y22_N26
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[3]~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[3]~3_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2)))) # 
-- (!\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(3))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110110001001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(3),
	datac => \inst11|fifo_read~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(2),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[3]~3_combout\);

-- Location: LCCOMB_X26_Y22_N8
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3) & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~COUT\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3) & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~COUT\) # (GND)))
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~COUT\) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita2~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~COUT\);

-- Location: FF_X26_Y22_N9
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3));

-- Location: LCCOMB_X27_Y22_N2
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[4]~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[4]~4_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(4))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010001000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3),
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(4),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[4]~4_combout\);

-- Location: FF_X27_Y22_N3
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[4]~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(4));

-- Location: LCCOMB_X26_Y22_N28
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[4]~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[4]~4_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3))) # 
-- (!\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(4)))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111011110000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datab => \inst11|fifo_read~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(3),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(4),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[4]~4_combout\);

-- Location: LCCOMB_X26_Y22_N10
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4) & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~COUT\ $ 
-- (GND))) # (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4) & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~COUT\ & VCC))
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4) & !\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita3~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~COUT\);

-- Location: FF_X26_Y22_N11
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4));

-- Location: LCCOMB_X26_Y22_N30
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[5]~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[5]~5_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4)))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(5)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001000010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(5),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[5]~5_combout\);

-- Location: FF_X26_Y22_N31
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[5]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(5));

-- Location: LCCOMB_X26_Y22_N24
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[5]~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[5]~5_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4)))) # 
-- (!\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(5))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(5)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111100001110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datab => \inst11|fifo_read~q\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(5),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(4),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[5]~5_combout\);

-- Location: LCCOMB_X26_Y22_N12
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5) & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~COUT\)) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5) & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~COUT\) # (GND)))
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~COUT\ = CARRY((!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~COUT\) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita4~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~COUT\);

-- Location: FF_X26_Y22_N13
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5));

-- Location: LCCOMB_X27_Y22_N28
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[6]~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[6]~6_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(6))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010001000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5),
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(6),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[6]~6_combout\);

-- Location: FF_X27_Y22_N29
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[6]~6_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(6));

-- Location: LCCOMB_X27_Y22_N14
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[6]~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[6]~6_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5))) # 
-- (!\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(6)))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(6)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011111110000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(5),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datac => \inst11|fifo_read~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(6),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[6]~6_combout\);

-- Location: LCCOMB_X26_Y22_N14
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6) & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~COUT\ $ 
-- (GND))) # (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6) & (!\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~COUT\ & VCC))
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~COUT\ = CARRY((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6) & !\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~COUT\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6),
	datad => VCC,
	cin => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita5~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~combout\,
	cout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~COUT\);

-- Location: FF_X26_Y22_N15
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6));

-- Location: LCCOMB_X27_Y22_N0
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[7]~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[7]~7_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(7))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010001000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6),
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(7),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[7]~7_combout\);

-- Location: FF_X27_Y22_N1
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[7]~7_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(7));

-- Location: LCCOMB_X27_Y22_N26
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[7]~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[7]~7_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6))) # 
-- (!\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(7)))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (((\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(7)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011111110000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(6),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datac => \inst11|fifo_read~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(7),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[7]~7_combout\);

-- Location: LCCOMB_X26_Y22_N16
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita7~combout\ = \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~COUT\ $ (\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(7))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(7),
	cin => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita6~COUT\,
	combout => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita7~combout\);

-- Location: FF_X26_Y22_N17
\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_comb_bita7~combout\,
	asdata => \~GND~combout\,
	sload => \inst27~0_combout\,
	ena => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|_~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(7));

-- Location: LCCOMB_X26_Y22_N18
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[8]~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[8]~8_combout\ = (!\inst27~0_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(7)))) # 
-- (!\inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(8)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001000010000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|valid_rreq~combout\,
	datab => \inst27~0_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(8),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(7),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[8]~8_combout\);

-- Location: FF_X26_Y22_N19
\inst3|scfifo_component|auto_generated|dpfifo|low_addressa[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa[8]~8_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(8));

-- Location: LCCOMB_X26_Y22_N20
\inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[8]~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[8]~8_combout\ = (\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & ((\inst11|fifo_read~q\ & ((\inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(7)))) # 
-- (!\inst11|fifo_read~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(8))))) # (!\inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\ & (\inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(8)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110110001001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|empty_dff~q\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|low_addressa\(8),
	datac => \inst11|fifo_read~q\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|rd_ptr_msb|counter_reg_bit\(7),
	combout => \inst3|scfifo_component|auto_generated|dpfifo|ram_read_address[8]~8_combout\);

-- Location: LCCOMB_X24_Y24_N8
\inst16|counter[4]~37\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[4]~37_combout\ = (\inst16|counter\(4) & (!\inst16|counter[3]~36\)) # (!\inst16|counter\(4) & ((\inst16|counter[3]~36\) # (GND)))
-- \inst16|counter[4]~38\ = CARRY((!\inst16|counter[3]~36\) # (!\inst16|counter\(4)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(4),
	datad => VCC,
	cin => \inst16|counter[3]~36\,
	combout => \inst16|counter[4]~37_combout\,
	cout => \inst16|counter[4]~38\);

-- Location: FF_X24_Y24_N9
\inst16|counter[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[4]~37_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(4));

-- Location: LCCOMB_X23_Y23_N14
\inst16|sensor_value_out[4]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[4]~feeder_combout\ = \inst16|counter\(4)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(4),
	combout => \inst16|sensor_value_out[4]~feeder_combout\);

-- Location: FF_X23_Y23_N15
\inst16|sensor_value_out[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[4]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(4));

-- Location: LCCOMB_X24_Y24_N10
\inst16|counter[5]~39\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[5]~39_combout\ = (\inst16|counter\(5) & (\inst16|counter[4]~38\ $ (GND))) # (!\inst16|counter\(5) & (!\inst16|counter[4]~38\ & VCC))
-- \inst16|counter[5]~40\ = CARRY((\inst16|counter\(5) & !\inst16|counter[4]~38\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(5),
	datad => VCC,
	cin => \inst16|counter[4]~38\,
	combout => \inst16|counter[5]~39_combout\,
	cout => \inst16|counter[5]~40\);

-- Location: FF_X24_Y24_N11
\inst16|counter[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[5]~39_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(5));

-- Location: LCCOMB_X23_Y23_N16
\inst16|sensor_value_out[5]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[5]~feeder_combout\ = \inst16|counter\(5)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(5),
	combout => \inst16|sensor_value_out[5]~feeder_combout\);

-- Location: FF_X23_Y23_N17
\inst16|sensor_value_out[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[5]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(5));

-- Location: LCCOMB_X24_Y24_N12
\inst16|counter[6]~41\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[6]~41_combout\ = (\inst16|counter\(6) & (!\inst16|counter[5]~40\)) # (!\inst16|counter\(6) & ((\inst16|counter[5]~40\) # (GND)))
-- \inst16|counter[6]~42\ = CARRY((!\inst16|counter[5]~40\) # (!\inst16|counter\(6)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(6),
	datad => VCC,
	cin => \inst16|counter[5]~40\,
	combout => \inst16|counter[6]~41_combout\,
	cout => \inst16|counter[6]~42\);

-- Location: FF_X24_Y24_N13
\inst16|counter[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[6]~41_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(6));

-- Location: LCCOMB_X23_Y23_N22
\inst16|sensor_value_out[6]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[6]~feeder_combout\ = \inst16|counter\(6)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(6),
	combout => \inst16|sensor_value_out[6]~feeder_combout\);

-- Location: FF_X23_Y23_N23
\inst16|sensor_value_out[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[6]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(6));

-- Location: LCCOMB_X24_Y24_N14
\inst16|counter[7]~43\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[7]~43_combout\ = (\inst16|counter\(7) & (\inst16|counter[6]~42\ $ (GND))) # (!\inst16|counter\(7) & (!\inst16|counter[6]~42\ & VCC))
-- \inst16|counter[7]~44\ = CARRY((\inst16|counter\(7) & !\inst16|counter[6]~42\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(7),
	datad => VCC,
	cin => \inst16|counter[6]~42\,
	combout => \inst16|counter[7]~43_combout\,
	cout => \inst16|counter[7]~44\);

-- Location: FF_X24_Y24_N15
\inst16|counter[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[7]~43_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(7));

-- Location: FF_X23_Y24_N19
\inst16|sensor_value_out[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(7),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(7));

-- Location: LCCOMB_X24_Y24_N16
\inst16|counter[8]~45\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[8]~45_combout\ = (\inst16|counter\(8) & (!\inst16|counter[7]~44\)) # (!\inst16|counter\(8) & ((\inst16|counter[7]~44\) # (GND)))
-- \inst16|counter[8]~46\ = CARRY((!\inst16|counter[7]~44\) # (!\inst16|counter\(8)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(8),
	datad => VCC,
	cin => \inst16|counter[7]~44\,
	combout => \inst16|counter[8]~45_combout\,
	cout => \inst16|counter[8]~46\);

-- Location: FF_X24_Y24_N17
\inst16|counter[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[8]~45_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(8));

-- Location: LCCOMB_X24_Y24_N18
\inst16|counter[9]~47\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[9]~47_combout\ = (\inst16|counter\(9) & (\inst16|counter[8]~46\ $ (GND))) # (!\inst16|counter\(9) & (!\inst16|counter[8]~46\ & VCC))
-- \inst16|counter[9]~48\ = CARRY((\inst16|counter\(9) & !\inst16|counter[8]~46\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(9),
	datad => VCC,
	cin => \inst16|counter[8]~46\,
	combout => \inst16|counter[9]~47_combout\,
	cout => \inst16|counter[9]~48\);

-- Location: FF_X24_Y24_N19
\inst16|counter[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[9]~47_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(9));

-- Location: LCCOMB_X24_Y24_N20
\inst16|counter[10]~49\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[10]~49_combout\ = (\inst16|counter\(10) & (!\inst16|counter[9]~48\)) # (!\inst16|counter\(10) & ((\inst16|counter[9]~48\) # (GND)))
-- \inst16|counter[10]~50\ = CARRY((!\inst16|counter[9]~48\) # (!\inst16|counter\(10)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(10),
	datad => VCC,
	cin => \inst16|counter[9]~48\,
	combout => \inst16|counter[10]~49_combout\,
	cout => \inst16|counter[10]~50\);

-- Location: FF_X24_Y24_N21
\inst16|counter[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[10]~49_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(10));

-- Location: LCCOMB_X24_Y24_N22
\inst16|counter[11]~51\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[11]~51_combout\ = (\inst16|counter\(11) & (\inst16|counter[10]~50\ $ (GND))) # (!\inst16|counter\(11) & (!\inst16|counter[10]~50\ & VCC))
-- \inst16|counter[11]~52\ = CARRY((\inst16|counter\(11) & !\inst16|counter[10]~50\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(11),
	datad => VCC,
	cin => \inst16|counter[10]~50\,
	combout => \inst16|counter[11]~51_combout\,
	cout => \inst16|counter[11]~52\);

-- Location: FF_X24_Y24_N23
\inst16|counter[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[11]~51_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(11));

-- Location: LCCOMB_X24_Y24_N24
\inst16|counter[12]~53\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[12]~53_combout\ = (\inst16|counter\(12) & (!\inst16|counter[11]~52\)) # (!\inst16|counter\(12) & ((\inst16|counter[11]~52\) # (GND)))
-- \inst16|counter[12]~54\ = CARRY((!\inst16|counter[11]~52\) # (!\inst16|counter\(12)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(12),
	datad => VCC,
	cin => \inst16|counter[11]~52\,
	combout => \inst16|counter[12]~53_combout\,
	cout => \inst16|counter[12]~54\);

-- Location: FF_X24_Y24_N25
\inst16|counter[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[12]~53_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(12));

-- Location: LCCOMB_X23_Y23_N20
\inst16|sensor_value_out[12]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[12]~feeder_combout\ = \inst16|counter\(12)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(12),
	combout => \inst16|sensor_value_out[12]~feeder_combout\);

-- Location: FF_X23_Y23_N21
\inst16|sensor_value_out[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[12]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(12));

-- Location: LCCOMB_X24_Y24_N26
\inst16|counter[13]~55\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[13]~55_combout\ = (\inst16|counter\(13) & (\inst16|counter[12]~54\ $ (GND))) # (!\inst16|counter\(13) & (!\inst16|counter[12]~54\ & VCC))
-- \inst16|counter[13]~56\ = CARRY((\inst16|counter\(13) & !\inst16|counter[12]~54\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(13),
	datad => VCC,
	cin => \inst16|counter[12]~54\,
	combout => \inst16|counter[13]~55_combout\,
	cout => \inst16|counter[13]~56\);

-- Location: FF_X24_Y24_N27
\inst16|counter[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[13]~55_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(13));

-- Location: LCCOMB_X23_Y23_N4
\inst16|sensor_value_out[13]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[13]~feeder_combout\ = \inst16|counter\(13)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(13),
	combout => \inst16|sensor_value_out[13]~feeder_combout\);

-- Location: FF_X23_Y23_N5
\inst16|sensor_value_out[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[13]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(13));

-- Location: LCCOMB_X24_Y24_N28
\inst16|counter[14]~57\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[14]~57_combout\ = (\inst16|counter\(14) & (!\inst16|counter[13]~56\)) # (!\inst16|counter\(14) & ((\inst16|counter[13]~56\) # (GND)))
-- \inst16|counter[14]~58\ = CARRY((!\inst16|counter[13]~56\) # (!\inst16|counter\(14)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(14),
	datad => VCC,
	cin => \inst16|counter[13]~56\,
	combout => \inst16|counter[14]~57_combout\,
	cout => \inst16|counter[14]~58\);

-- Location: FF_X24_Y24_N29
\inst16|counter[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[14]~57_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(14));

-- Location: FF_X23_Y23_N29
\inst16|sensor_value_out[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(14),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(14));

-- Location: LCCOMB_X24_Y24_N30
\inst16|counter[15]~59\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[15]~59_combout\ = (\inst16|counter\(15) & (\inst16|counter[14]~58\ $ (GND))) # (!\inst16|counter\(15) & (!\inst16|counter[14]~58\ & VCC))
-- \inst16|counter[15]~60\ = CARRY((\inst16|counter\(15) & !\inst16|counter[14]~58\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(15),
	datad => VCC,
	cin => \inst16|counter[14]~58\,
	combout => \inst16|counter[15]~59_combout\,
	cout => \inst16|counter[15]~60\);

-- Location: FF_X24_Y24_N31
\inst16|counter[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[15]~59_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(15));

-- Location: FF_X23_Y23_N13
\inst16|sensor_value_out[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(15),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(15));

-- Location: LCCOMB_X24_Y23_N0
\inst16|counter[16]~61\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[16]~61_combout\ = (\inst16|counter\(16) & (!\inst16|counter[15]~60\)) # (!\inst16|counter\(16) & ((\inst16|counter[15]~60\) # (GND)))
-- \inst16|counter[16]~62\ = CARRY((!\inst16|counter[15]~60\) # (!\inst16|counter\(16)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(16),
	datad => VCC,
	cin => \inst16|counter[15]~60\,
	combout => \inst16|counter[16]~61_combout\,
	cout => \inst16|counter[16]~62\);

-- Location: FF_X24_Y23_N1
\inst16|counter[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[16]~61_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(16));

-- Location: LCCOMB_X24_Y23_N2
\inst16|counter[17]~63\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[17]~63_combout\ = (\inst16|counter\(17) & (\inst16|counter[16]~62\ $ (GND))) # (!\inst16|counter\(17) & (!\inst16|counter[16]~62\ & VCC))
-- \inst16|counter[17]~64\ = CARRY((\inst16|counter\(17) & !\inst16|counter[16]~62\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(17),
	datad => VCC,
	cin => \inst16|counter[16]~62\,
	combout => \inst16|counter[17]~63_combout\,
	cout => \inst16|counter[17]~64\);

-- Location: FF_X24_Y23_N3
\inst16|counter[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[17]~63_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(17));

-- Location: LCCOMB_X24_Y23_N4
\inst16|counter[18]~65\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[18]~65_combout\ = (\inst16|counter\(18) & (!\inst16|counter[17]~64\)) # (!\inst16|counter\(18) & ((\inst16|counter[17]~64\) # (GND)))
-- \inst16|counter[18]~66\ = CARRY((!\inst16|counter[17]~64\) # (!\inst16|counter\(18)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(18),
	datad => VCC,
	cin => \inst16|counter[17]~64\,
	combout => \inst16|counter[18]~65_combout\,
	cout => \inst16|counter[18]~66\);

-- Location: FF_X24_Y23_N5
\inst16|counter[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[18]~65_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(18));

-- Location: LCCOMB_X24_Y23_N6
\inst16|counter[19]~67\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[19]~67_combout\ = (\inst16|counter\(19) & (\inst16|counter[18]~66\ $ (GND))) # (!\inst16|counter\(19) & (!\inst16|counter[18]~66\ & VCC))
-- \inst16|counter[19]~68\ = CARRY((\inst16|counter\(19) & !\inst16|counter[18]~66\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(19),
	datad => VCC,
	cin => \inst16|counter[18]~66\,
	combout => \inst16|counter[19]~67_combout\,
	cout => \inst16|counter[19]~68\);

-- Location: FF_X24_Y23_N7
\inst16|counter[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[19]~67_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(19));

-- Location: LCCOMB_X24_Y23_N8
\inst16|counter[20]~69\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[20]~69_combout\ = (\inst16|counter\(20) & (!\inst16|counter[19]~68\)) # (!\inst16|counter\(20) & ((\inst16|counter[19]~68\) # (GND)))
-- \inst16|counter[20]~70\ = CARRY((!\inst16|counter[19]~68\) # (!\inst16|counter\(20)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(20),
	datad => VCC,
	cin => \inst16|counter[19]~68\,
	combout => \inst16|counter[20]~69_combout\,
	cout => \inst16|counter[20]~70\);

-- Location: FF_X24_Y23_N9
\inst16|counter[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[20]~69_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(20));

-- Location: FF_X23_Y23_N19
\inst16|sensor_value_out[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(20),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(20));

-- Location: LCCOMB_X24_Y23_N10
\inst16|counter[21]~71\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[21]~71_combout\ = (\inst16|counter\(21) & (\inst16|counter[20]~70\ $ (GND))) # (!\inst16|counter\(21) & (!\inst16|counter[20]~70\ & VCC))
-- \inst16|counter[21]~72\ = CARRY((\inst16|counter\(21) & !\inst16|counter[20]~70\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(21),
	datad => VCC,
	cin => \inst16|counter[20]~70\,
	combout => \inst16|counter[21]~71_combout\,
	cout => \inst16|counter[21]~72\);

-- Location: FF_X24_Y23_N11
\inst16|counter[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[21]~71_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(21));

-- Location: FF_X23_Y23_N27
\inst16|sensor_value_out[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(21),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(21));

-- Location: LCCOMB_X24_Y23_N12
\inst16|counter[22]~73\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[22]~73_combout\ = (\inst16|counter\(22) & (!\inst16|counter[21]~72\)) # (!\inst16|counter\(22) & ((\inst16|counter[21]~72\) # (GND)))
-- \inst16|counter[22]~74\ = CARRY((!\inst16|counter[21]~72\) # (!\inst16|counter\(22)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(22),
	datad => VCC,
	cin => \inst16|counter[21]~72\,
	combout => \inst16|counter[22]~73_combout\,
	cout => \inst16|counter[22]~74\);

-- Location: FF_X24_Y23_N13
\inst16|counter[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[22]~73_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(22));

-- Location: FF_X23_Y23_N11
\inst16|sensor_value_out[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(22),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(22));

-- Location: LCCOMB_X24_Y23_N14
\inst16|counter[23]~75\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[23]~75_combout\ = (\inst16|counter\(23) & (\inst16|counter[22]~74\ $ (GND))) # (!\inst16|counter\(23) & (!\inst16|counter[22]~74\ & VCC))
-- \inst16|counter[23]~76\ = CARRY((\inst16|counter\(23) & !\inst16|counter[22]~74\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(23),
	datad => VCC,
	cin => \inst16|counter[22]~74\,
	combout => \inst16|counter[23]~75_combout\,
	cout => \inst16|counter[23]~76\);

-- Location: FF_X24_Y23_N15
\inst16|counter[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[23]~75_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(23));

-- Location: LCCOMB_X23_Y23_N2
\inst16|sensor_value_out[23]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[23]~feeder_combout\ = \inst16|counter\(23)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(23),
	combout => \inst16|sensor_value_out[23]~feeder_combout\);

-- Location: FF_X23_Y23_N3
\inst16|sensor_value_out[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[23]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(23));

-- Location: LCCOMB_X24_Y23_N16
\inst16|counter[24]~77\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[24]~77_combout\ = (\inst16|counter\(24) & (!\inst16|counter[23]~76\)) # (!\inst16|counter\(24) & ((\inst16|counter[23]~76\) # (GND)))
-- \inst16|counter[24]~78\ = CARRY((!\inst16|counter[23]~76\) # (!\inst16|counter\(24)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(24),
	datad => VCC,
	cin => \inst16|counter[23]~76\,
	combout => \inst16|counter[24]~77_combout\,
	cout => \inst16|counter[24]~78\);

-- Location: FF_X24_Y23_N17
\inst16|counter[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[24]~77_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(24));

-- Location: LCCOMB_X24_Y23_N18
\inst16|counter[25]~79\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[25]~79_combout\ = (\inst16|counter\(25) & (\inst16|counter[24]~78\ $ (GND))) # (!\inst16|counter\(25) & (!\inst16|counter[24]~78\ & VCC))
-- \inst16|counter[25]~80\ = CARRY((\inst16|counter\(25) & !\inst16|counter[24]~78\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(25),
	datad => VCC,
	cin => \inst16|counter[24]~78\,
	combout => \inst16|counter[25]~79_combout\,
	cout => \inst16|counter[25]~80\);

-- Location: FF_X24_Y23_N19
\inst16|counter[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[25]~79_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(25));

-- Location: LCCOMB_X24_Y23_N20
\inst16|counter[26]~81\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[26]~81_combout\ = (\inst16|counter\(26) & (!\inst16|counter[25]~80\)) # (!\inst16|counter\(26) & ((\inst16|counter[25]~80\) # (GND)))
-- \inst16|counter[26]~82\ = CARRY((!\inst16|counter[25]~80\) # (!\inst16|counter\(26)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(26),
	datad => VCC,
	cin => \inst16|counter[25]~80\,
	combout => \inst16|counter[26]~81_combout\,
	cout => \inst16|counter[26]~82\);

-- Location: FF_X24_Y23_N21
\inst16|counter[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[26]~81_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(26));

-- Location: LCCOMB_X24_Y23_N22
\inst16|counter[27]~83\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[27]~83_combout\ = (\inst16|counter\(27) & (\inst16|counter[26]~82\ $ (GND))) # (!\inst16|counter\(27) & (!\inst16|counter[26]~82\ & VCC))
-- \inst16|counter[27]~84\ = CARRY((\inst16|counter\(27) & !\inst16|counter[26]~82\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(27),
	datad => VCC,
	cin => \inst16|counter[26]~82\,
	combout => \inst16|counter[27]~83_combout\,
	cout => \inst16|counter[27]~84\);

-- Location: FF_X24_Y23_N23
\inst16|counter[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[27]~83_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(27));

-- Location: LCCOMB_X23_Y24_N20
\inst16|sensor_value_out[27]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[27]~feeder_combout\ = \inst16|counter\(27)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(27),
	combout => \inst16|sensor_value_out[27]~feeder_combout\);

-- Location: FF_X23_Y24_N21
\inst16|sensor_value_out[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[27]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(27));

-- Location: LCCOMB_X24_Y23_N24
\inst16|counter[28]~85\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[28]~85_combout\ = (\inst16|counter\(28) & (!\inst16|counter[27]~84\)) # (!\inst16|counter\(28) & ((\inst16|counter[27]~84\) # (GND)))
-- \inst16|counter[28]~86\ = CARRY((!\inst16|counter[27]~84\) # (!\inst16|counter\(28)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(28),
	datad => VCC,
	cin => \inst16|counter[27]~84\,
	combout => \inst16|counter[28]~85_combout\,
	cout => \inst16|counter[28]~86\);

-- Location: FF_X24_Y23_N25
\inst16|counter[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[28]~85_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(28));

-- Location: LCCOMB_X23_Y23_N24
\inst16|sensor_value_out[28]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[28]~feeder_combout\ = \inst16|counter\(28)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(28),
	combout => \inst16|sensor_value_out[28]~feeder_combout\);

-- Location: FF_X23_Y23_N25
\inst16|sensor_value_out[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[28]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(28));

-- Location: LCCOMB_X24_Y23_N26
\inst16|counter[29]~87\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[29]~87_combout\ = (\inst16|counter\(29) & (\inst16|counter[28]~86\ $ (GND))) # (!\inst16|counter\(29) & (!\inst16|counter[28]~86\ & VCC))
-- \inst16|counter[29]~88\ = CARRY((\inst16|counter\(29) & !\inst16|counter[28]~86\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(29),
	datad => VCC,
	cin => \inst16|counter[28]~86\,
	combout => \inst16|counter[29]~87_combout\,
	cout => \inst16|counter[29]~88\);

-- Location: FF_X24_Y23_N27
\inst16|counter[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[29]~87_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(29));

-- Location: LCCOMB_X23_Y23_N6
\inst16|sensor_value_out[29]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[29]~feeder_combout\ = \inst16|counter\(29)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(29),
	combout => \inst16|sensor_value_out[29]~feeder_combout\);

-- Location: FF_X23_Y23_N7
\inst16|sensor_value_out[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[29]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(29));

-- Location: LCCOMB_X24_Y23_N28
\inst16|counter[30]~89\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[30]~89_combout\ = (\inst16|counter\(30) & (!\inst16|counter[29]~88\)) # (!\inst16|counter\(30) & ((\inst16|counter[29]~88\) # (GND)))
-- \inst16|counter[30]~90\ = CARRY((!\inst16|counter[29]~88\) # (!\inst16|counter\(30)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst16|counter\(30),
	datad => VCC,
	cin => \inst16|counter[29]~88\,
	combout => \inst16|counter[30]~89_combout\,
	cout => \inst16|counter[30]~90\);

-- Location: FF_X24_Y23_N29
\inst16|counter[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[30]~89_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(30));

-- Location: LCCOMB_X23_Y23_N8
\inst16|sensor_value_out[30]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[30]~feeder_combout\ = \inst16|counter\(30)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(30),
	combout => \inst16|sensor_value_out[30]~feeder_combout\);

-- Location: FF_X23_Y23_N9
\inst16|sensor_value_out[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[30]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(30));

-- Location: LCCOMB_X24_Y23_N30
\inst16|counter[31]~91\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|counter[31]~91_combout\ = \inst16|counter\(31) $ (!\inst16|counter[30]~90\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst16|counter\(31),
	cin => \inst16|counter[30]~90\,
	combout => \inst16|counter[31]~91_combout\);

-- Location: FF_X24_Y23_N31
\inst16|counter[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|counter[31]~91_combout\,
	ena => \inst16|LessThan0~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|counter\(31));

-- Location: FF_X23_Y24_N17
\inst16|sensor_value_out[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(31),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(31));

-- Location: M9K_X22_Y23_N0
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3\ : cycloneive_ram_block
-- pragma translate_off
GENERIC MAP (
	clk0_core_clock_enable => "ena0",
	data_interleave_offset_in_bits => 1,
	data_interleave_width_in_bits => 1,
	logical_ram_name => "fifo:inst3|scfifo:scfifo_component|scfifo_c341:auto_generated|a_dpfifo_vq31:dpfifo|altsyncram_5tb1:FIFOram|ALTSYNCRAM",
	mixed_port_feed_through_mode => "dont_care",
	operation_mode => "dual_port",
	port_a_address_clear => "none",
	port_a_address_width => 9,
	port_a_byte_enable_clock => "none",
	port_a_data_out_clear => "none",
	port_a_data_out_clock => "none",
	port_a_data_width => 18,
	port_a_first_address => 0,
	port_a_first_bit_number => 3,
	port_a_last_address => 511,
	port_a_logical_ram_depth => 512,
	port_a_logical_ram_width => 32,
	port_a_read_during_write_mode => "new_data_with_nbe_read",
	port_b_address_clear => "none",
	port_b_address_clock => "clock1",
	port_b_address_width => 9,
	port_b_data_out_clear => "none",
	port_b_data_out_clock => "none",
	port_b_data_width => 18,
	port_b_first_address => 0,
	port_b_first_bit_number => 3,
	port_b_last_address => 511,
	port_b_logical_ram_depth => 512,
	port_b_logical_ram_width => 32,
	port_b_read_during_write_mode => "new_data_with_nbe_read",
	port_b_read_enable_clock => "clock1",
	ram_block_type => "M9K")
-- pragma translate_on
PORT MAP (
	portawe => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	portbre => VCC,
	clk0 => \clock_50~inputclkctrl_outclk\,
	clk1 => \clock_50~inputclkctrl_outclk\,
	ena0 => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	portadatain => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTADATAIN_bus\,
	portaaddr => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTAADDR_bus\,
	portbaddr => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBADDR_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	portbdataout => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a3_PORTBDATAOUT_bus\);

-- Location: LCCOMB_X25_Y23_N26
\inst11|Byte~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~1_combout\ = (\inst11|numberOfBytesTransmitted\(1) & (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(23) & ((!\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|numberOfBytesTransmitted\(1) & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(7)) # (\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010111011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(23),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(7),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~1_combout\);

-- Location: LCCOMB_X25_Y23_N12
\inst11|Byte~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~2_combout\ = (\inst11|Byte~1_combout\ & (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(15)) # (!\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|Byte~1_combout\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(31) & ((\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110010010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte~1_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(31),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(15),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~2_combout\);

-- Location: LCCOMB_X30_Y22_N22
\inst11|Byte[3]~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte[3]~0_combout\ = (\inst11|LessThan1~0_combout\ & ((\inst11|LessThan0~1_combout\) # ((\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\ & !\inst11|LessThan2~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100111000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~5_combout\,
	datab => \inst11|LessThan0~1_combout\,
	datac => \inst11|LessThan2~0_combout\,
	datad => \inst11|LessThan1~0_combout\,
	combout => \inst11|Byte[3]~0_combout\);

-- Location: LCCOMB_X25_Y23_N8
\inst11|Byte~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~3_combout\ = (\inst11|Byte~2_combout\ & \inst11|Byte[3]~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte~2_combout\,
	datac => \inst11|Byte[3]~0_combout\,
	combout => \inst11|Byte~3_combout\);

-- Location: LCCOMB_X30_Y22_N24
\inst11|Byte[7]~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte[7]~4_combout\ = ((\inst11|LessThan2~0_combout\) # (\inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4))) # (!\inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111011101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|usedw_will_be_2~6_combout\,
	datab => \inst11|LessThan2~0_combout\,
	datad => \inst3|scfifo_component|auto_generated|dpfifo|usedw_counter|counter_reg_bit\(4),
	combout => \inst11|Byte[7]~4_combout\);

-- Location: LCCOMB_X30_Y22_N10
\inst11|Byte[7]~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte[7]~5_combout\ = (\button0~input_o\ & ((\inst11|LessThan0~1_combout\ & (!\inst2|di_req_o_reg~q\)) # (!\inst11|LessThan0~1_combout\ & ((\inst11|Byte[7]~4_combout\)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101110000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|di_req_o_reg~q\,
	datab => \inst11|Byte[7]~4_combout\,
	datac => \inst11|LessThan0~1_combout\,
	datad => \button0~input_o\,
	combout => \inst11|Byte[7]~5_combout\);

-- Location: FF_X25_Y23_N9
\inst11|Byte[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~3_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(7));

-- Location: FF_X32_Y22_N9
\inst2|di_reg[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst11|Byte\(7),
	sload => VCC,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(7));

-- Location: LCCOMB_X32_Y22_N10
\inst2|Equal3~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Equal3~0_combout\ = (!\inst2|state_reg\(3) & (!\inst2|state_reg\(2) & !\inst2|state_reg\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|Equal3~0_combout\);

-- Location: LCCOMB_X32_Y22_N8
\inst2|Selector6~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector6~0_combout\ = (\inst2|Equal3~0_combout\ & ((\inst2|wren~q\ & ((\inst2|di_reg\(7)))) # (!\inst2|wren~q\ & (\inst2|sh_reg\(7)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110010000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wren~q\,
	datab => \inst2|sh_reg\(7),
	datac => \inst2|di_reg\(7),
	datad => \inst2|Equal3~0_combout\,
	combout => \inst2|Selector6~0_combout\);

-- Location: LCCOMB_X32_Y22_N14
\inst2|Selector6~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector6~1_combout\ = (\inst2|state_reg\(3) & (\inst2|Selector4~0_combout\ & (\inst2|state_reg\(0) & \inst2|sh_reg\(7))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|Selector4~0_combout\,
	datac => \inst2|state_reg\(0),
	datad => \inst2|sh_reg\(7),
	combout => \inst2|Selector6~1_combout\);

-- Location: LCCOMB_X21_Y23_N22
\inst11|Byte~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~12_combout\ = (\inst11|numberOfBytesTransmitted\(1) & ((\inst11|numberOfBytesTransmitted\(0) & (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(28))) # (!\inst11|numberOfBytesTransmitted\(0) & 
-- ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(20)))))) # (!\inst11|numberOfBytesTransmitted\(1) & (((\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1101110110100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(28),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(20),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~12_combout\);

-- Location: LCCOMB_X21_Y23_N8
\inst11|Byte~13\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~13_combout\ = (\inst11|Byte~12_combout\ & (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(12)) # (\inst11|numberOfBytesTransmitted\(1))))) # (!\inst11|Byte~12_combout\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(4) & ((!\inst11|numberOfBytesTransmitted\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101011100100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte~12_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(4),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(12),
	datad => \inst11|numberOfBytesTransmitted\(1),
	combout => \inst11|Byte~13_combout\);

-- Location: LCCOMB_X21_Y23_N26
\inst11|Byte~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~14_combout\ = (\inst11|Byte[3]~0_combout\ & \inst11|Byte~13_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte[3]~0_combout\,
	datac => \inst11|Byte~13_combout\,
	combout => \inst11|Byte~14_combout\);

-- Location: FF_X21_Y23_N27
\inst11|Byte[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~14_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(4));

-- Location: FF_X21_Y23_N17
\inst2|di_reg[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst11|Byte\(4),
	sload => VCC,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(4));

-- Location: FF_X23_Y24_N11
\inst16|sensor_value_out[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(0),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(0));

-- Location: LCCOMB_X23_Y24_N28
\inst16|sensor_value_out[1]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[1]~feeder_combout\ = \inst16|counter\(1)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(1),
	combout => \inst16|sensor_value_out[1]~feeder_combout\);

-- Location: FF_X23_Y24_N29
\inst16|sensor_value_out[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[1]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(1));

-- Location: LCCOMB_X23_Y24_N2
\inst16|sensor_value_out[2]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[2]~feeder_combout\ = \inst16|counter\(2)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(2),
	combout => \inst16|sensor_value_out[2]~feeder_combout\);

-- Location: FF_X23_Y24_N3
\inst16|sensor_value_out[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[2]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(2));

-- Location: LCCOMB_X23_Y24_N22
\inst16|sensor_value_out[8]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[8]~feeder_combout\ = \inst16|counter\(8)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(8),
	combout => \inst16|sensor_value_out[8]~feeder_combout\);

-- Location: FF_X23_Y24_N23
\inst16|sensor_value_out[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[8]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(8));

-- Location: LCCOMB_X23_Y24_N8
\inst16|sensor_value_out[9]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[9]~feeder_combout\ = \inst16|counter\(9)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(9),
	combout => \inst16|sensor_value_out[9]~feeder_combout\);

-- Location: FF_X23_Y24_N9
\inst16|sensor_value_out[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[9]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(9));

-- Location: FF_X23_Y24_N25
\inst16|sensor_value_out[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(10),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(10));

-- Location: LCCOMB_X23_Y24_N0
\inst16|sensor_value_out[11]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[11]~feeder_combout\ = \inst16|counter\(11)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(11),
	combout => \inst16|sensor_value_out[11]~feeder_combout\);

-- Location: FF_X23_Y24_N1
\inst16|sensor_value_out[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[11]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(11));

-- Location: FF_X23_Y24_N13
\inst16|sensor_value_out[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(16),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(16));

-- Location: FF_X23_Y24_N7
\inst16|sensor_value_out[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(17),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(17));

-- Location: FF_X23_Y24_N31
\inst16|sensor_value_out[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(18),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(18));

-- Location: FF_X23_Y24_N15
\inst16|sensor_value_out[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(19),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(19));

-- Location: FF_X21_Y24_N3
\inst16|sensor_value_out[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst16|counter\(24),
	sload => VCC,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(24));

-- Location: LCCOMB_X23_Y24_N26
\inst16|sensor_value_out[25]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[25]~feeder_combout\ = \inst16|counter\(25)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(25),
	combout => \inst16|sensor_value_out[25]~feeder_combout\);

-- Location: FF_X23_Y24_N27
\inst16|sensor_value_out[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[25]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(25));

-- Location: LCCOMB_X23_Y24_N4
\inst16|sensor_value_out[26]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst16|sensor_value_out[26]~feeder_combout\ = \inst16|counter\(26)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst16|counter\(26),
	combout => \inst16|sensor_value_out[26]~feeder_combout\);

-- Location: FF_X23_Y24_N5
\inst16|sensor_value_out[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst16|sensor_value_out[26]~feeder_combout\,
	ena => \inst16|write~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst16|sensor_value_out\(26));

-- Location: M9K_X22_Y24_N0
\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0\ : cycloneive_ram_block
-- pragma translate_off
GENERIC MAP (
	clk0_core_clock_enable => "ena0",
	data_interleave_offset_in_bits => 1,
	data_interleave_width_in_bits => 1,
	logical_ram_name => "fifo:inst3|scfifo:scfifo_component|scfifo_c341:auto_generated|a_dpfifo_vq31:dpfifo|altsyncram_5tb1:FIFOram|ALTSYNCRAM",
	mixed_port_feed_through_mode => "dont_care",
	operation_mode => "dual_port",
	port_a_address_clear => "none",
	port_a_address_width => 9,
	port_a_byte_enable_clock => "none",
	port_a_data_out_clear => "none",
	port_a_data_out_clock => "none",
	port_a_data_width => 18,
	port_a_first_address => 0,
	port_a_first_bit_number => 0,
	port_a_last_address => 511,
	port_a_logical_ram_depth => 512,
	port_a_logical_ram_width => 32,
	port_a_read_during_write_mode => "new_data_with_nbe_read",
	port_b_address_clear => "none",
	port_b_address_clock => "clock1",
	port_b_address_width => 9,
	port_b_data_out_clear => "none",
	port_b_data_out_clock => "none",
	port_b_data_width => 18,
	port_b_first_address => 0,
	port_b_first_bit_number => 0,
	port_b_last_address => 511,
	port_b_logical_ram_depth => 512,
	port_b_logical_ram_width => 32,
	port_b_read_during_write_mode => "new_data_with_nbe_read",
	port_b_read_enable_clock => "clock1",
	ram_block_type => "M9K")
-- pragma translate_on
PORT MAP (
	portawe => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	portbre => VCC,
	clk0 => \clock_50~inputclkctrl_outclk\,
	clk1 => \clock_50~inputclkctrl_outclk\,
	ena0 => \inst3|scfifo_component|auto_generated|dpfifo|valid_wreq~combout\,
	portadatain => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTADATAIN_bus\,
	portaaddr => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTAADDR_bus\,
	portbaddr => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBADDR_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	portbdataout => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|ram_block1a0_PORTBDATAOUT_bus\);

-- Location: LCCOMB_X21_Y24_N8
\inst11|Byte~21\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~21_combout\ = (\inst11|numberOfBytesTransmitted\(1) & (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(17) & ((!\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|numberOfBytesTransmitted\(1) & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(1)) # (\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010111011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(17),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(1),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~21_combout\);

-- Location: LCCOMB_X25_Y23_N0
\inst11|Byte~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~22_combout\ = (\inst11|Byte~21_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(9)) # ((!\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|Byte~21_combout\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(25) & \inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010110011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(9),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(25),
	datac => \inst11|Byte~21_combout\,
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~22_combout\);

-- Location: LCCOMB_X25_Y23_N6
\inst11|Byte~23\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~23_combout\ = (\inst11|always0~0_combout\) # ((\inst11|Byte~22_combout\ & \inst11|Byte[3]~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst11|Byte~22_combout\,
	datac => \inst11|Byte[3]~0_combout\,
	datad => \inst11|always0~0_combout\,
	combout => \inst11|Byte~23_combout\);

-- Location: FF_X25_Y23_N7
\inst11|Byte[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~23_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(1));

-- Location: LCCOMB_X25_Y23_N28
\inst2|di_reg[1]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_reg[1]~feeder_combout\ = \inst11|Byte\(1)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst11|Byte\(1),
	combout => \inst2|di_reg[1]~feeder_combout\);

-- Location: FF_X25_Y23_N29
\inst2|di_reg[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_reg[1]~feeder_combout\,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(1));

-- Location: LCCOMB_X21_Y23_N12
\inst11|Byte~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~24_combout\ = (\inst11|numberOfBytesTransmitted\(1) & ((\inst11|numberOfBytesTransmitted\(0) & ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(24)))) # (!\inst11|numberOfBytesTransmitted\(0) & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(16))))) # (!\inst11|numberOfBytesTransmitted\(1) & (((\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111010110001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(16),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(24),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~24_combout\);

-- Location: LCCOMB_X21_Y23_N30
\inst11|Byte~25\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~25_combout\ = (\inst11|Byte~24_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(8)) # ((\inst11|numberOfBytesTransmitted\(1))))) # (!\inst11|Byte~24_combout\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(0) & !\inst11|numberOfBytesTransmitted\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101011011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte~24_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(8),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(0),
	datad => \inst11|numberOfBytesTransmitted\(1),
	combout => \inst11|Byte~25_combout\);

-- Location: LCCOMB_X21_Y23_N10
\inst11|Byte~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~26_combout\ = (\inst11|Byte[3]~0_combout\ & \inst11|Byte~25_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte[3]~0_combout\,
	datac => \inst11|Byte~25_combout\,
	combout => \inst11|Byte~26_combout\);

-- Location: FF_X21_Y23_N11
\inst11|Byte[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~26_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(0));

-- Location: LCCOMB_X21_Y23_N0
\inst2|di_reg[0]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_reg[0]~feeder_combout\ = \inst11|Byte\(0)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst11|Byte\(0),
	combout => \inst2|di_reg[0]~feeder_combout\);

-- Location: FF_X21_Y23_N1
\inst2|di_reg[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_reg[0]~feeder_combout\,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(0));

-- Location: LCCOMB_X28_Y23_N18
\inst2|core_ce~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|core_ce~0_combout\ = (!\inst2|core_n_clk~q\ & !\inst2|spi_2x_ce~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000110011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|core_n_clk~q\,
	datad => \inst2|spi_2x_ce~q\,
	combout => \inst2|core_ce~0_combout\);

-- Location: FF_X28_Y23_N19
\inst2|core_ce\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|core_ce~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|core_ce~q\);

-- Location: IOIBUF_X40_Y34_N8
\spi_miso~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_spi_miso,
	o => \spi_miso~input_o\);

-- Location: LCCOMB_X28_Y23_N0
\inst2|rx_bit_reg~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|rx_bit_reg~0_combout\ = (\inst2|core_ce~q\ & ((\spi_miso~input_o\))) # (!\inst2|core_ce~q\ & (\inst2|rx_bit_reg~q\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|core_ce~q\,
	datac => \inst2|rx_bit_reg~q\,
	datad => \spi_miso~input_o\,
	combout => \inst2|rx_bit_reg~0_combout\);

-- Location: FF_X28_Y23_N1
\inst2|rx_bit_reg\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|rx_bit_reg~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|rx_bit_reg~q\);

-- Location: LCCOMB_X32_Y22_N0
\inst2|WideOr1~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|WideOr1~0_combout\ = (\inst2|state_reg\(3) & ((\inst2|state_reg\(0)) # ((\inst2|state_reg\(2)) # (\inst2|state_reg\(1))))) # (!\inst2|state_reg\(3) & (((!\inst2|state_reg\(2) & !\inst2|state_reg\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101010101101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|state_reg\(0),
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|WideOr1~0_combout\);

-- Location: LCCOMB_X28_Y23_N22
\inst2|Selector13~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector13~0_combout\ = (\inst2|WideOr1~0_combout\ & (\inst2|di_reg\(0))) # (!\inst2|WideOr1~0_combout\ & ((\inst2|rx_bit_reg~q\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|di_reg\(0),
	datab => \inst2|rx_bit_reg~q\,
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector13~0_combout\);

-- Location: LCCOMB_X32_Y22_N2
\inst2|sh_reg[6]~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|sh_reg[6]~0_combout\ = (!\inst2|Equal1~0_combout\ & (!\inst2|core_n_ce~q\ & ((\inst2|wren~q\) # (!\inst2|Equal3~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000001000000011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wren~q\,
	datab => \inst2|Equal1~0_combout\,
	datac => \inst2|core_n_ce~q\,
	datad => \inst2|Equal3~0_combout\,
	combout => \inst2|sh_reg[6]~0_combout\);

-- Location: FF_X28_Y23_N23
\inst2|sh_reg[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector13~0_combout\,
	ena => \inst2|sh_reg[6]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(0));

-- Location: LCCOMB_X28_Y23_N28
\inst2|Selector12~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector12~0_combout\ = (\inst2|WideOr1~0_combout\ & (\inst2|di_reg\(1))) # (!\inst2|WideOr1~0_combout\ & ((\inst2|sh_reg\(0))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|di_reg\(1),
	datac => \inst2|sh_reg\(0),
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector12~0_combout\);

-- Location: FF_X28_Y23_N29
\inst2|sh_reg[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector12~0_combout\,
	ena => \inst2|sh_reg[6]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(1));

-- Location: LCCOMB_X21_Y23_N28
\inst11|Byte~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~18_combout\ = (\inst11|numberOfBytesTransmitted\(0) & ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(26)) # ((!\inst11|numberOfBytesTransmitted\(1))))) # (!\inst11|numberOfBytesTransmitted\(0) & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(18) & \inst11|numberOfBytesTransmitted\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1101100010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(0),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(26),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(18),
	datad => \inst11|numberOfBytesTransmitted\(1),
	combout => \inst11|Byte~18_combout\);

-- Location: LCCOMB_X21_Y23_N6
\inst11|Byte~19\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~19_combout\ = (\inst11|Byte~18_combout\ & (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(10)) # (\inst11|numberOfBytesTransmitted\(1))))) # (!\inst11|Byte~18_combout\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(2) & ((!\inst11|numberOfBytesTransmitted\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110011100010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(2),
	datab => \inst11|Byte~18_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(10),
	datad => \inst11|numberOfBytesTransmitted\(1),
	combout => \inst11|Byte~19_combout\);

-- Location: LCCOMB_X21_Y23_N18
\inst11|Byte~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~20_combout\ = (\inst11|Byte~19_combout\ & \inst11|Byte[3]~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte~19_combout\,
	datac => \inst11|Byte[3]~0_combout\,
	combout => \inst11|Byte~20_combout\);

-- Location: FF_X21_Y23_N19
\inst11|Byte[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~20_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(2));

-- Location: LCCOMB_X21_Y23_N4
\inst2|di_reg[2]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_reg[2]~feeder_combout\ = \inst11|Byte\(2)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst11|Byte\(2),
	combout => \inst2|di_reg[2]~feeder_combout\);

-- Location: FF_X21_Y23_N5
\inst2|di_reg[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_reg[2]~feeder_combout\,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(2));

-- Location: LCCOMB_X28_Y23_N26
\inst2|Selector11~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector11~0_combout\ = (\inst2|WideOr1~0_combout\ & ((\inst2|di_reg\(2)))) # (!\inst2|WideOr1~0_combout\ & (\inst2|sh_reg\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|sh_reg\(1),
	datac => \inst2|di_reg\(2),
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector11~0_combout\);

-- Location: FF_X28_Y23_N27
\inst2|sh_reg[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector11~0_combout\,
	ena => \inst2|sh_reg[6]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(2));

-- Location: LCCOMB_X21_Y23_N14
\inst11|Byte~15\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~15_combout\ = (\inst11|numberOfBytesTransmitted\(1) & (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(19) & !\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|numberOfBytesTransmitted\(1) & 
-- ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(3)) # ((\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010111100100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(3),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(19),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~15_combout\);

-- Location: LCCOMB_X21_Y23_N24
\inst11|Byte~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~16_combout\ = (\inst11|Byte~15_combout\ & (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(11)) # (!\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|Byte~15_combout\ & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(27) & ((\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110010010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte~15_combout\,
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(27),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(11),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~16_combout\);

-- Location: LCCOMB_X21_Y23_N2
\inst11|Byte~17\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~17_combout\ = (\inst11|Byte~16_combout\ & \inst11|Byte[3]~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100000011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst11|Byte~16_combout\,
	datac => \inst11|Byte[3]~0_combout\,
	combout => \inst11|Byte~17_combout\);

-- Location: FF_X21_Y23_N3
\inst11|Byte[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~17_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(3));

-- Location: LCCOMB_X21_Y23_N20
\inst2|di_reg[3]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_reg[3]~feeder_combout\ = \inst11|Byte\(3)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst11|Byte\(3),
	combout => \inst2|di_reg[3]~feeder_combout\);

-- Location: FF_X21_Y23_N21
\inst2|di_reg[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_reg[3]~feeder_combout\,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(3));

-- Location: LCCOMB_X28_Y23_N8
\inst2|Selector10~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector10~0_combout\ = (\inst2|WideOr1~0_combout\ & ((\inst2|di_reg\(3)))) # (!\inst2|WideOr1~0_combout\ & (\inst2|sh_reg\(2)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|sh_reg\(2),
	datac => \inst2|di_reg\(3),
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector10~0_combout\);

-- Location: FF_X28_Y23_N9
\inst2|sh_reg[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector10~0_combout\,
	ena => \inst2|sh_reg[6]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(3));

-- Location: LCCOMB_X28_Y23_N24
\inst2|Selector9~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector9~0_combout\ = (\inst2|WideOr1~0_combout\ & (\inst2|di_reg\(4))) # (!\inst2|WideOr1~0_combout\ & ((\inst2|sh_reg\(3))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|di_reg\(4),
	datac => \inst2|sh_reg\(3),
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector9~0_combout\);

-- Location: FF_X28_Y23_N25
\inst2|sh_reg[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector9~0_combout\,
	ena => \inst2|sh_reg[6]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(4));

-- Location: LCCOMB_X25_Y23_N16
\inst11|Byte~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~9_combout\ = (\inst11|numberOfBytesTransmitted\(1) & (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(21) & !\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|numberOfBytesTransmitted\(1) & 
-- ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(5)) # ((\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010111100100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(5),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(21),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~9_combout\);

-- Location: LCCOMB_X25_Y23_N10
\inst11|Byte~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~10_combout\ = (\inst11|Byte~9_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(13)) # ((!\inst11|numberOfBytesTransmitted\(0))))) # (!\inst11|Byte~9_combout\ & 
-- (((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(29) & \inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011100011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(13),
	datab => \inst11|Byte~9_combout\,
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(29),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~10_combout\);

-- Location: LCCOMB_X25_Y23_N30
\inst11|Byte~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~11_combout\ = (\inst11|Byte~10_combout\ & \inst11|Byte[3]~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|Byte~10_combout\,
	datac => \inst11|Byte[3]~0_combout\,
	combout => \inst11|Byte~11_combout\);

-- Location: FF_X25_Y23_N31
\inst11|Byte[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~11_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(5));

-- Location: FF_X25_Y23_N25
\inst2|di_reg[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	asdata => \inst11|Byte\(5),
	sload => VCC,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(5));

-- Location: LCCOMB_X28_Y23_N12
\inst2|Selector8~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector8~0_combout\ = (\inst2|WideOr1~0_combout\ & ((\inst2|di_reg\(5)))) # (!\inst2|WideOr1~0_combout\ & (\inst2|sh_reg\(4)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|sh_reg\(4),
	datac => \inst2|di_reg\(5),
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector8~0_combout\);

-- Location: FF_X28_Y23_N13
\inst2|sh_reg[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector8~0_combout\,
	ena => \inst2|sh_reg[6]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(5));

-- Location: LCCOMB_X25_Y23_N18
\inst11|Byte~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~6_combout\ = (\inst11|numberOfBytesTransmitted\(1) & ((\inst11|numberOfBytesTransmitted\(0) & ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(30)))) # (!\inst11|numberOfBytesTransmitted\(0) & 
-- (\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(22))))) # (!\inst11|numberOfBytesTransmitted\(1) & (((\inst11|numberOfBytesTransmitted\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111010110001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst11|numberOfBytesTransmitted\(1),
	datab => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(22),
	datac => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(30),
	datad => \inst11|numberOfBytesTransmitted\(0),
	combout => \inst11|Byte~6_combout\);

-- Location: LCCOMB_X25_Y23_N4
\inst11|Byte~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~7_combout\ = (\inst11|Byte~6_combout\ & ((\inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(14)) # ((\inst11|numberOfBytesTransmitted\(1))))) # (!\inst11|Byte~6_combout\ & (((!\inst11|numberOfBytesTransmitted\(1) & 
-- \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(6)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100101111001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(14),
	datab => \inst11|Byte~6_combout\,
	datac => \inst11|numberOfBytesTransmitted\(1),
	datad => \inst3|scfifo_component|auto_generated|dpfifo|FIFOram|q_b\(6),
	combout => \inst11|Byte~7_combout\);

-- Location: LCCOMB_X25_Y23_N14
\inst11|Byte~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst11|Byte~8_combout\ = (\inst11|Byte[3]~0_combout\ & \inst11|Byte~7_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100000011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst11|Byte[3]~0_combout\,
	datac => \inst11|Byte~7_combout\,
	combout => \inst11|Byte~8_combout\);

-- Location: FF_X25_Y23_N15
\inst11|Byte[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst11|Byte~8_combout\,
	ena => \inst11|Byte[7]~5_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst11|Byte\(6));

-- Location: LCCOMB_X26_Y23_N24
\inst2|di_reg[6]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|di_reg[6]~feeder_combout\ = \inst11|Byte\(6)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \inst11|Byte\(6),
	combout => \inst2|di_reg[6]~feeder_combout\);

-- Location: FF_X26_Y23_N25
\inst2|di_reg[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|di_reg[6]~feeder_combout\,
	ena => \inst11|wren~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|di_reg\(6));

-- Location: LCCOMB_X28_Y23_N16
\inst2|Selector7~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector7~0_combout\ = (\inst2|WideOr1~0_combout\ & ((\inst2|di_reg\(6)))) # (!\inst2|WideOr1~0_combout\ & (\inst2|sh_reg\(5)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000010101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|sh_reg\(5),
	datac => \inst2|di_reg\(6),
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector7~0_combout\);

-- Location: FF_X28_Y23_N17
\inst2|sh_reg[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector7~0_combout\,
	ena => \inst2|sh_reg[6]~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(6));

-- Location: LCCOMB_X32_Y22_N18
\inst2|Selector6~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector6~2_combout\ = (\inst2|Selector6~0_combout\) # ((\inst2|Selector6~1_combout\) # ((\inst2|sh_reg\(6) & !\inst2|WideOr1~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110111011111110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Selector6~0_combout\,
	datab => \inst2|Selector6~1_combout\,
	datac => \inst2|sh_reg\(6),
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector6~2_combout\);

-- Location: FF_X32_Y22_N19
\inst2|sh_reg[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector6~2_combout\,
	ena => \inst2|ALT_INV_core_n_ce~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sh_reg\(7));

-- Location: LCCOMB_X32_Y22_N6
\inst2|spi_mosi_o~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|spi_mosi_o~0_combout\ = (\inst2|wren~q\ & ((\inst2|Equal4~0_combout\ & (\inst2|di_reg\(7))) # (!\inst2|Equal4~0_combout\ & ((\inst2|sh_reg\(7)))))) # (!\inst2|wren~q\ & (((\inst2|sh_reg\(7)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111011110000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wren~q\,
	datab => \inst2|Equal4~0_combout\,
	datac => \inst2|di_reg\(7),
	datad => \inst2|sh_reg\(7),
	combout => \inst2|spi_mosi_o~0_combout\);

-- Location: LCCOMB_X32_Y22_N12
\inst2|Selector1~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector1~0_combout\ = (\inst2|wren~q\ & (\inst2|state_reg\(0) & \inst2|Equal3~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wren~q\,
	datac => \inst2|state_reg\(0),
	datad => \inst2|Equal3~0_combout\,
	combout => \inst2|Selector1~0_combout\);

-- Location: LCCOMB_X32_Y22_N30
\inst2|Selector1~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector1~1_combout\ = (\inst2|Selector1~0_combout\) # ((\inst2|Equal1~0_combout\) # ((\inst2|sck_ena_reg~q\ & !\inst2|WideOr1~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110111011111110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Selector1~0_combout\,
	datab => \inst2|Equal1~0_combout\,
	datac => \inst2|sck_ena_reg~q\,
	datad => \inst2|WideOr1~0_combout\,
	combout => \inst2|Selector1~1_combout\);

-- Location: FF_X32_Y22_N31
\inst2|sck_ena_reg\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector1~1_combout\,
	ena => \inst2|ALT_INV_core_n_ce~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|sck_ena_reg~q\);

-- Location: LCCOMB_X31_Y23_N8
\inst2|spi_clk_reg~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|spi_clk_reg~0_combout\ = (\inst2|core_n_clk~q\ & \inst2|sck_ena_reg~q\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|core_n_clk~q\,
	datad => \inst2|sck_ena_reg~q\,
	combout => \inst2|spi_clk_reg~0_combout\);

-- Location: FF_X31_Y23_N9
\inst2|spi_clk_reg\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|spi_clk_reg~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|spi_clk_reg~q\);

-- Location: LCCOMB_X32_Y22_N16
\inst2|WideOr0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|WideOr0~0_combout\ = \inst2|state_reg\(3) $ (((\inst2|state_reg\(0)) # ((\inst2|state_reg\(2)) # (\inst2|state_reg\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010101010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|state_reg\(3),
	datab => \inst2|state_reg\(0),
	datac => \inst2|state_reg\(2),
	datad => \inst2|state_reg\(1),
	combout => \inst2|WideOr0~0_combout\);

-- Location: LCCOMB_X32_Y22_N24
\inst2|Selector0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Selector0~0_combout\ = (\inst2|wren~q\ & (((\inst2|ssel_ena_reg~q\)) # (!\inst2|WideOr0~0_combout\))) # (!\inst2|wren~q\ & (!\inst2|Equal4~0_combout\ & ((\inst2|ssel_ena_reg~q\) # (!\inst2|WideOr0~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010001011110011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|wren~q\,
	datab => \inst2|WideOr0~0_combout\,
	datac => \inst2|ssel_ena_reg~q\,
	datad => \inst2|Equal4~0_combout\,
	combout => \inst2|Selector0~0_combout\);

-- Location: FF_X32_Y22_N25
\inst2|ssel_ena_reg\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \clock_50~inputclkctrl_outclk\,
	d => \inst2|Selector0~0_combout\,
	ena => \inst2|ALT_INV_core_n_ce~q\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|ssel_ena_reg~q\);

-- Location: LCCOMB_X43_Y33_N0
and1 : cycloneive_lcell_comb
-- Equation(s):
-- \and1~combout\ = (!\inst2|ssel_ena_reg~q\ & \button0~input_o\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|ssel_ena_reg~q\,
	datad => \button0~input_o\,
	combout => \and1~combout\);

-- Location: IOIBUF_X43_Y34_N15
\MPU_6050_interrupt_in~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_MPU_6050_interrupt_in,
	o => \MPU_6050_interrupt_in~input_o\);

-- Location: LCCOMB_X43_Y33_N10
and2 : cycloneive_lcell_comb
-- Equation(s):
-- \and2~combout\ = (\MPU_6050_interrupt_in~input_o\ & \button0~input_o\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \MPU_6050_interrupt_in~input_o\,
	datad => \button0~input_o\,
	combout => \and2~combout\);

-- Location: IOIBUF_X0_Y16_N8
\button1~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_button1,
	o => \button1~input_o\);
END structure;


