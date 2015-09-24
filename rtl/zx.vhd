-------------------------------------------------------------------[22.06.2015]
-- U16-ZX128K Version 2.0
-- DEVBOARD ReVerSE-U16
-------------------------------------------------------------------------------
-- Engineer: 	MVV
--
-- 07.06.2015	Initial release
--		CPU: T80@3.5MHz
--		RAM: M9K 56K (32K ROM + 8K ROM + 16K RAM)
--		SRAM: 128K
--		Video: HDMI 640x480@60Hz(ZX-Spectrum screen x2 = H:32+256+32; V=24+192+24)
--		Int: 60Hz (h_sync_on and v_sync_on)
--		Sound: Stereo (Delta-sigma) AY3-8910 + Beeper
--		Keyboard: USB HID Keyboard (F4=CPU Reset, F5=NMI, ScrollLock=Hard Reset)
--		DivMMC: 512K (Press Space+F5+F4 to initial, F5=Go to ESXDOS)
-------------------------------------------------------------------------------
-- github.com/mvvproject/ReVerSE-U16
--
-- Copyright (c) 2015 MVV
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- * Redistributions of source code must retain the above copyright notice,
--   this list of conditions and the following disclaimer.
--
-- * Redistributions in synthesized form must reproduce the above copyright
--   notice, this list of conditions and the following disclaimer in the
--   documentation and/or other materials provided with the distribution.
--
-- * Neither the name of the author nor the names of other contributors may
--   be used to endorse or promote products derived from this software without
--   specific prior written agreement from the author.
--
-- * License is granted for non-commercial use only.  A fee may not be charged
--   for redistributions as source code or in synthesized/hardware form without 
--   specific prior written agreement from the author.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.

-- Adapted By Ynicky for Marsohod3

library IEEE; 
use IEEE.std_logic_1164.all; 
use IEEE.std_logic_unsigned.all;
use IEEE.numeric_std.all; 

entity zx is
port (
	-- Clock (100MHz)
	CLK_100MHZ	: in std_logic;
	NRESET	: in std_logic;
	-- HDMI
	HDMI_D0		: out std_logic;
	HDMI_D0N		: out std_logic;
	HDMI_D1		: out std_logic;
	HDMI_D1N		: out std_logic;
	HDMI_D2		: out std_logic;
	HDMI_D2N		: out std_logic;
	HDMI_CLK		: out std_logic;
	HDMI_CLKN	: out std_logic;
	-- PS/2
	PS2_KBCLK	: inout std_logic;
	PS2_KBDAT	: inout std_logic;		
	-- SD
	SD_SO		: in std_logic;
	SD_CLK		: out std_logic;
	SD_SI		: out std_logic;
	SD_NCS		: out std_logic;
	-- Audio (Delta-sigma)
	DAC_OUT_L	: out std_logic;
	DAC_OUT_R	: out std_logic);
end zx;

architecture rtl of zx is
-- HDMI
signal sHDMI_D0	: std_logic;
signal sHDMI_D0N	: std_logic;
signal sHDMI_D1	: std_logic;
signal sHDMI_D1N	: std_logic;
signal sHDMI_D2	: std_logic;
signal sHDMI_D2N	: std_logic;
signal sHDMI_CLK	: std_logic;
signal sHDMI_CLKN	: std_logic;
-- CPU
signal cpu_reset	: std_logic;
signal cpu_addr	: std_logic_vector(15 downto 0);
signal cpu_data_o	: std_logic_vector(7 downto 0);
signal cpu_data_i	: std_logic_vector(7 downto 0);
signal cpu_mreq	: std_logic;
signal cpu_iorq	: std_logic;
signal cpu_wr		: std_logic;
signal cpu_rd		: std_logic;
signal cpu_int		: std_logic;
signal cpu_m1		: std_logic;
signal cpu_nmi		: std_logic;
-- Memory
signal rom0_data_o	: std_logic_vector(7 downto 0);
signal rom1_data_o	: std_logic_vector(7 downto 0);
signal ram_addr		: std_logic_vector(11 downto 0);
signal mux				: std_logic_vector(3 downto 0);
-- Port
signal port_xxfe_reg	: std_logic_vector(7 downto 0);
signal port_7ffd_reg	: std_logic_vector(7 downto 0);
-- PS/2 Keyboard
signal kb_do_bus	: std_logic_vector(4 downto 0);
signal kb_f_bus	: std_logic_vector(12 downto 1);
signal kb_joy_bus	: std_logic_vector(4 downto 0);
-- Video
signal vga_addr	: std_logic_vector(12 downto 0);
signal vga_data	: std_logic_vector(7 downto 0);
signal vga_wr		: std_logic;
signal vga_hsync	: std_logic;
signal vga_vsync	: std_logic;
signal vga_blank	: std_logic;
signal vga_rgb		: std_logic_vector(5 downto 0);
signal vga_int		: std_logic;
---signal vga_hcnt	: std_logic_vector(9 downto 0);
signal vram_wr		: std_logic;
signal vram_scr	: std_logic;
-- Clock
signal clk_bus		: std_logic;
signal clk_vga		: std_logic;
signal clk_hdmi	: std_logic;
signal clk_cpu		: std_logic;
---signal clk_divmmc	: std_logic;
signal clk_ssg		: std_logic;
-- System
signal reset		: std_logic;
signal areset		: std_logic;
---signal key_reset	: std_logic;
---signal locked0		: std_logic;
signal locked1		: std_logic;
signal selector	: std_logic_vector(3 downto 0);
signal key_f		: std_logic_vector(12 downto 1);
signal key		: std_logic_vector(12 downto 1) := "000000000000";
signal inta		: std_logic;
-- SRAM
signal sram_wr		: std_logic;
---signal sram_rd		: std_logic;
signal sram_data_o	: std_logic_vector(7 downto 0);
signal sramdnn_data_o	: std_logic_vector(7 downto 0);
signal sramdn_data_o	: std_logic_vector(7 downto 0);
signal sramup_data_o	: std_logic_vector(7 downto 0);
-- DivMMC
signal divmmc_data_o	: std_logic_vector(7 downto 0);
signal divmmc_e3reg	: std_logic_vector(7 downto 0);
signal divmmc_amap	: std_logic;
-- SSG
signal ssg_bdir		: std_logic;
signal ssg_bc			: std_logic;
signal ssg_data_o		: std_logic_vector(7 downto 0);
signal ssg_ch_a		: std_logic_vector(7 downto 0);
signal ssg_ch_b		: std_logic_vector(7 downto 0);
signal ssg_ch_c		: std_logic_vector(7 downto 0);
signal dac_left		: std_logic_vector(8 downto 0);
signal dac_right		: std_logic_vector(8 downto 0);
-- Tape input from ADC
signal tape_in		: std_logic;
signal adc_d		: std_logic_vector(11 downto 0);

COMPONENT fiftyfivenm_adcblock_top_wrapper
GENERIC (analog_input_pin_mask : INTEGER;
			clkdiv : INTEGER;
			device_partname_fivechar_prefix : STRING;
			hard_pwd : INTEGER;
			is_this_first_or_second_adc : INTEGER;
			prescalar : INTEGER;
			refsel : INTEGER;
			tsclkdiv : INTEGER;
			tsclksel : INTEGER
			);
	PORT(soc : IN STD_LOGIC;
		 usr_pwd : IN STD_LOGIC;
		 tsen : IN STD_LOGIC;
		 clkin_from_pll_c0 : IN STD_LOGIC;
		 chsel : IN STD_LOGIC_VECTOR(4 DOWNTO 0);
		 eoc : OUT STD_LOGIC;
		 clkout_adccore : OUT STD_LOGIC;
		 dout : OUT STD_LOGIC_VECTOR(11 DOWNTO 0)
	);
END COMPONENT;

begin

-- ADC
U0 : fiftyfivenm_adcblock_top_wrapper
GENERIC MAP(analog_input_pin_mask => 0,
			clkdiv => 1, --- 2 MHz
			device_partname_fivechar_prefix => "10M50",
			hard_pwd => 0,
			is_this_first_or_second_adc => 1,
			prescalar => 0,
			refsel => 0,
			tsclkdiv => 0,
			tsclksel => 0
			)
PORT MAP(soc => '1',
		 usr_pwd => '0',
		 tsen => '1',
		 clkin_from_pll_c0 => clk_ssg,
		 chsel => "00000",
		 dout => adc_d);

U1: entity work.altpll1
port map (
	areset		=> areset,
	locked		=> locked1,
	inclk0		=> CLK_100MHZ,	-- 100.0 MHz
	c0		=> clk_hdmi,	-- 125.0 MHz
	c1		=> clk_bus,		-- 100.0 MHz
	c2		=> clk_vga,		-- 25.0 MHz
	c3		=> clk_cpu,		-- 4.0 MHz
	c4		=> clk_ssg);	-- 2.0 MHz

-- ROM 32K
U2: entity work.rom0
port map (
	address	=> port_7ffd_reg(4) & cpu_addr(13 downto 0),
	clock		=> clk_bus,
	q	 		=> rom0_data_o);

-- Video RAM 16K
U3: entity work.ram
port map (
	address_a	=> vram_scr & cpu_addr(12 downto 0),
	address_b	=> port_7ffd_reg(3) & vga_addr,
	clock_a		=> clk_bus,
	clock_b		=> clk_vga,
	data_a	 	=> cpu_data_o,
	data_b	 	=> (others => '0'),
	wren_a	 	=> vram_wr,
	wren_b	 	=> '0',
	q_a	 		=> open,
	q_b	 		=> vga_data);
	
-- CPU
U4: entity work.T80CPU
port map (
	RESET_N_I	=> cpu_reset,
	CLK_N_I		=> clk_cpu,
	CLKEN_I		=> '1',
	WAIT_N_I		=> '1',
	INT_N_I		=> cpu_int,
	NMI_N_I		=> cpu_nmi,
	BUSRQ_N_I	=> '1',
	DATA_I		=> cpu_data_i,
	DATA_O		=> cpu_data_o,
	ADDR_O		=> cpu_addr,
	M1_N_O		=> cpu_m1,
	MREQ_N_O		=> cpu_mreq,
	IORQ_N_O		=> cpu_iorq,
	RD_N_O		=> cpu_rd,
	WR_N_O		=> cpu_wr,
	RFSH_N_O		=> open,
	HALT_N_O		=> open,
	BUSAK_N_O	=> open);
	
-- Video
U5: entity work.vga
port map (
	CLK_I			=> clk_vga,
	DATA_I		=> vga_data,
	BORDER_I		=> port_xxfe_reg(2 downto 0),	-- Биты D0..D2 порта xxFE определяют цвет бордюра
	INT_O			=> vga_int,
	ADDR_O		=> vga_addr,
	BLANK_O		=> vga_blank,
	RGB_O			=> vga_rgb,	-- RRGGBB
---	HCNT_O		=> vga_hcnt,
	HSYNC_O		=> vga_hsync,
	VSYNC_O		=> vga_vsync);
	
-- Keyboard
U6: entity work.keyboard
port map(
	CLK			=> clk_bus,
	RESET			=> reset,
	A				=> cpu_addr(15 downto 8),
	KEYB			=> kb_do_bus,
	KEYF			=> kb_f_bus(5 downto 1),
	KEYJOY		=> kb_joy_bus,
	KEYLED		=> "000",
	SCANCODE		=> open,
	PS2_KBCLK	=> PS2_KBCLK,
	PS2_KBDAT	=> PS2_KBDAT);
	
-- Delta-Sigma
U7: entity work.dac
port map (
	CLK_I  		=> clk_bus,
	RESET_I		=> reset,
	DAC_DATA_I	=> dac_left,
	DAC_O			=> DAC_OUT_L);

-- Delta-Sigma
U8: entity work.dac
port map (
	CLK_I			=> clk_bus,
	RESET_I		=> reset,
	DAC_DATA_I	=> dac_right,
	DAC_O			=> DAC_OUT_R);

-- HDMI
U9: entity work.hdmi
port map(
	CLK_DVI_I	=> clk_hdmi,
	CLK_PIXEL_I	=> clk_vga,
	R_I		=> vga_rgb(5 downto 4) & vga_rgb(5 downto 4) & vga_rgb(5 downto 4) & vga_rgb(5 downto 4),
	G_I		=> vga_rgb(3 downto 2) & vga_rgb(3 downto 2) & vga_rgb(3 downto 2) & vga_rgb(3 downto 2),
	B_I		=> vga_rgb(1 downto 0) & vga_rgb(1 downto 0) & vga_rgb(1 downto 0) & vga_rgb(1 downto 0),
	BLANK_I		=> vga_blank,
	HSYNC_I		=> vga_hsync,
	VSYNC_I		=> vga_vsync,
	HDMI_D0_O	=> sHDMI_D0,
	HDMI_D0N_O	=> sHDMI_D0N,
	HDMI_D1_O	=> sHDMI_D1,
	HDMI_D1N_O	=> sHDMI_D1N,
	HDMI_D2_O	=> sHDMI_D2,
	HDMI_D2N_O	=> sHDMI_D2N,
	HDMI_CLK_O	=> sHDMI_CLK,
	HDMI_CLKN_O	=> sHDMI_CLKN);

-- RAM64K
U10: entity work.ram64
port map(
	clock		=> clk_bus,
	address	=> ram_addr(2 downto 0) & cpu_addr(12 downto 0),
	data	   => cpu_data_o,
	q	      => sramdn_data_o,
	wren		=> sram_wr and not ram_addr(3));

-- RAM32K
U100: entity work.ram32
port map(
	clock		=> clk_bus,
	address	=> ram_addr(1 downto 0) & cpu_addr(12 downto 0),
	data	   => cpu_data_o,
	q	      => sramup_data_o,
	wren		=> sram_wr and ram_addr(3));

sram_data_o <= sramdn_data_o when ram_addr(3) = '0' else sramup_data_o;

-- ROM DivMMC 8K
U11: entity work.rom1
port map (
	address	=> cpu_addr(12 downto 0),
	clock		=> clk_bus,
	q	 		=> rom1_data_o);

-- DivMMC
U12: entity work.divMMC
port map (
---	CLK_I		=> clk_divmmc,
	CLK_I			=> clk_vga,
	EN_I			=> port_7ffd_reg(4),
	RESET_I		=> reset,
	ADDR_I		=> cpu_addr,
	DATA_I		=> cpu_data_o,
	DATA_O		=> divmmc_data_o,
	WR_N_I		=> cpu_wr,
	RD_N_I		=> cpu_rd,
	IORQ_N_I		=> cpu_iorq,
	MREQ_N_I		=> cpu_mreq,
	M1_N_I		=> cpu_m1,
	E3REG_O		=> divmmc_e3reg,
	AMAP_O		=> divmmc_amap,
	CS_N_O		=> SD_NCS,
	SCLK_O		=> SD_CLK,
	MOSI_O		=> SD_SI,
	MISO_I		=> SD_SO);

-- SSG
U13: entity work.ay8910
port map (
	CLK_I   		=> clk_ssg,
	EN_I   		=> '1',
	RESET_I 		=> reset,
	BDIR_I  		=> ssg_bdir,
	CS_I    		=> '1',
	BC_I    		=> ssg_bc,
	DATA_I    	=> cpu_data_o,
	DATA_O		=> ssg_data_o,
	CH_A_O		=> ssg_ch_a,
	CH_B_O		=> ssg_ch_b,
	CH_C_O		=> ssg_ch_c);
	
-------------------------------------------------------------------------------
-- Формирование глобальных сигналов

HDMI_D0  <= sHDMI_D0;
HDMI_D0N <= sHDMI_D0N;
HDMI_D1  <= sHDMI_D1;
HDMI_D1N <= sHDMI_D1N;
HDMI_D2  <= sHDMI_D2;
HDMI_D2N <= sHDMI_D2N;
HDMI_CLK  <= sHDMI_CLK;
HDMI_CLKN <= sHDMI_CLKN;

process (clk_bus, inta)
begin
	if (inta = '0') then
		cpu_int <= '1';
	elsif (clk_bus'event and clk_bus = '1') then
		if (vga_int = '1') then cpu_int <= '0'; end if;
	end if;
end process;

kb_f_bus(12 downto 6) <= "0000000";
areset    <= not NRESET;	-- глобальный сброс
---reset     <= areset or key_reset or not locked0 or not locked1;	-- горячий сброс
reset     <= areset or not locked1;	-- горячий сброс
cpu_reset <= not(reset or kb_f_bus(4));	-- CPU сброс
inta      <= cpu_iorq or cpu_m1;	-- INTA
cpu_nmi   <= not(kb_f_bus(5));	-- NMI

-------------------------------------------------------------------------------
-- Video
vram_scr <= '1' when (ram_addr = "000000001110") else '0';
vram_wr  <= '1' when (cpu_mreq = '0' and cpu_wr = '0' and ((ram_addr = "000000001010") or (ram_addr = "000000001110"))) else '0';

-------------------------------------------------------------------------------
-- Регистры
process (reset, clk_bus, cpu_addr, port_7ffd_reg, cpu_wr, cpu_data_o)
begin
	if (reset = '1') then
		port_7ffd_reg <= (others => '0');
	elsif (clk_bus'event and clk_bus = '1') then
		if (cpu_iorq = '0' and cpu_wr = '0' and cpu_addr = X"7FFD" and port_7ffd_reg(5) = '0') then port_7ffd_reg <= cpu_data_o; end if;	-- D7-D6=не используются; D5=запрещение расширенной памяти (48K защёлка); D4=номер страницы ПЗУ(0-BASIC128, 1-BASIC48); D3=выбор отображаемой видеостраницы(0-страница в банке 5, 1 - в банке 7); D2-D0=номер страницы ОЗУ подключенной в верхние 16 КБ памяти (с адреса #C000)
	end if;
end process;

process (clk_bus, cpu_addr, port_xxfe_reg, cpu_wr, cpu_data_o)
begin
	if (clk_bus'event and clk_bus = '1') then
		if (cpu_iorq = '0' and cpu_wr = '0' and cpu_addr(7 downto 0) = X"FE") then port_xxfe_reg <= cpu_data_o; end if;	-- D7-D5=не используются; D4=бипер; D3=MIC; D2-D0=цвет бордюра
	end if;
end process;

-------------------------------------------------------------------------------
-- Функциональные клавиши Fx триггер
process (clk_bus, key, kb_f_bus, key_f)
begin
	if (clk_bus'event and clk_bus = '1') then
		key <= kb_f_bus;
		if (kb_f_bus /= key) then
			key_f <= key_f xor key;
		end if;
	end if;
end process;

-------------------------------------------------------------------------------
-- Шина данных CPU
selector <=	"0000" when (cpu_mreq = '0' and cpu_rd = '0' and cpu_addr(15 downto 14) = "00"  and divmmc_amap = '0' and divmmc_e3reg(7) = '0') else	-- ROM 0000-3FFF
				"0001" when (cpu_mreq = '0' and cpu_rd = '0' and cpu_addr(15 downto 13) = "000" and (divmmc_amap or divmmc_e3reg(7)) /= '0') else	-- ESXDOS ROM 0000-1FFF
				"0010" when (cpu_mreq = '0' and cpu_rd = '0') else	-- SRAM
				"0011" when (cpu_iorq = '0' and cpu_rd = '0' and cpu_addr(7 downto 0) = X"FE") else	-- Клавиатура, порт xxFE
				"0100" when (cpu_iorq = '0' and cpu_rd = '0' and cpu_addr(7 downto 0) = X"1F") else	-- Joystick, порт xx1F
				"0101" when (cpu_iorq = '0' and cpu_rd = '0' and cpu_addr = X"7FFD") else	-- чтение порта 7FFD
				"0110" when (cpu_iorq = '0' and cpu_rd = '0' and cpu_addr(7 downto 0) = X"EB") else	-- DivMMC
				"0111" when (cpu_iorq = '0' and cpu_rd = '0' and cpu_addr = X"FFFD") else	-- TurboSound
				(others => '1');

tape_in <= '1' when adc_d > 2112 else '0' when adc_d < 1984;

process (selector, ssg_data_o, sram_data_o, rom0_data_o, rom1_data_o, kb_do_bus, kb_joy_bus, port_7ffd_reg, divmmc_data_o, tape_in)
begin
	case selector is
		when "0000" => cpu_data_i <= rom0_data_o;	-- ROM
		when "0001" => cpu_data_i <= rom1_data_o;	-- ESXDOS ROM
		when "0010" => cpu_data_i <= sram_data_o;	-- SRAM
---		when "0011" => cpu_data_i <= "111" & kb_do_bus;	-- D7=не используется; D6=EAR; D5=не используется; D4-D0=отображают состояние определённого полуряда клавиатуры
		when "0011" => cpu_data_i <= '1' & tape_in & '1' & kb_do_bus;	-- D7=не используется; D6=EAR; D5=не используется; D4-D0=отображают состояние определённого полуряда клавиатуры
		when "0100" => cpu_data_i <= "000" & kb_joy_bus;	-- D7-D5=0; D4=огонь;  D3=вниз; D2=вверх; D1=вправо; D0=влево
		when "0101" => cpu_data_i <= port_7ffd_reg;
		when "0110" => cpu_data_i <= divmmc_data_o;
		when "0111" => cpu_data_i <= ssg_data_o;
		when others  => cpu_data_i <= (others => '1');
	end case;
end process;

------------------------------------------------------------------------------
-- Селектор
mux <= (divmmc_amap or divmmc_e3reg(7)) & cpu_addr(15 downto 13);

process (mux, port_7ffd_reg, ram_addr, divmmc_e3reg)
begin
	case mux is
		when "1001"        => ram_addr <= "000001" & divmmc_e3reg(5 downto 0);	-- ESXDOS RAM 2000-3FFF
		when "0010"|"1010" => ram_addr <= "000000001010";	-- Seg1 RAM 4000-5FFF
		when "0011"|"1011" => ram_addr <= "000000001011";	-- Seg1 RAM 6000-7FFF
		when "0100"|"1100" => ram_addr <= "000000000100";	-- Seg2 RAM 8000-9FFF
		when "0101"|"1101" => ram_addr <= "000000000101";	-- Seg2 RAM A000-BFFF
		when "0110"|"1110" => ram_addr <= "00000000" & port_7ffd_reg(2 downto 0) & '0';	-- Seg3 RAM C000-DFFF
		when "0111"|"1111" => ram_addr <= "00000000" & port_7ffd_reg(2 downto 0) & '1';	-- Seg3 RAM E000-FFFF
		when others 	=> ram_addr <= "XXXXXXXXXXXX";
	end case;
end process;

-------------------------------------------------------------------------------
-- SRAM
sram_wr <= '1' when (cpu_mreq = '0' and cpu_wr = '0' and (mux(3 downto 1) /= "000" or mux /= "1000")) else '0';
---sram_rd <= '1' when (cpu_mreq = '0' and cpu_rd = '0' and (mux(3 downto 1) /= "000" or mux /= "1000")) else '0';

-------------------------------------------------------------------------------
-- SSG
ssg_bc   <= '1' when (cpu_iorq = '0' and cpu_addr(15 downto 14) = "11" and cpu_addr(1) = '0' and cpu_m1 = '1') else '0';
ssg_bdir <= '1' when (cpu_iorq = '0' and cpu_addr(15) = '1' and cpu_addr(1) = '0' and cpu_m1 = '1' and cpu_wr = '0') else '0';

dac_left  <= ('0' & ssg_ch_a) + ('0' & ssg_ch_b) + ('0' & port_xxfe_reg(4) & "000000");
dac_right <= ('0' & ssg_ch_c) + ('0' & ssg_ch_b) + ('0' & port_xxfe_reg(4) & "000000");

---ecc <= X"00" when port_xxfe_reg(4) = '0' else X"32";

end rtl;
