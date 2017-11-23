-----------------------------------------------------------------------------------------
-- Project : АКПА СП-ФК
-- Company: АО "НТЦ Элинс"
-- Author :Мельников Александр Юрьевич
-- Date : 2017-11-30
-- File : axi_pkg.vhd
-- Design: axi_SpiMaster_v1_0
-----------------------------------------------------------------------------------------
-- Description : package с регистрами axi
-----------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;
use ieee.math_real.all;

package axi_pkg is

	-- Количество регистров AXI
	constant REG_CNT : integer := 6;
	constant OPT_MEM_ADDR_BITS : integer := integer(ceil(log2(real(REG_CNT*4))));
	
	type addr_reg_t is record
		addr_axi : integer range 0 to REG_CNT - 1;
		addr_bit : integer range 0 to 31;
		size     : integer range 1 to 32;
	end record;

	-- Запись '1' - старт передачи данных по SPI
	constant REG_CR         : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 0,
		size     => 1
	);
	-- Готовность принятых данных по SPI
	constant REG_SR_SPI_RDY : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 1,
		size     => 1
	);
	-- регистр данных на передачу пустой
	constant REG_SR_SPI_DRE : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 2,
		size     => 1
	);
	-- Завершение транзакции SPI
	constant REG_SR_SPI_TRC : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 3,
		size     => 1
	);
	-- Разрешение прерывания по "готовности принятых данных"
	constant REG_IR_SPI_RDY : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 4,
		size     => 1
	);
	-- Разрешение прерывания по "регистр данных на передачу пустой"
	constant REG_IR_SPI_DRE : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 5,
		size     => 1
	);
	-- Разрешение прерывания по "Завершение транзакции SPI"
	constant REG_IR_SPI_TRC : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 6,
		size     => 1
	);
	-- Сброс блока SPI (по записи '1')
	constant REG_RST        : addr_reg_t := (
		addr_axi => 0,
		addr_bit => 7,
		size     => 1
	);

	-- Регистр данных для передачи по SPI
	constant REG_WR_DATA      : addr_reg_t := (
		addr_axi => 1,
		addr_bit => 0,
		size     => 32
	);
	-- Регистр принятых данных по SPI
	constant REG_RD_DATA      : addr_reg_t := (
		addr_axi => 2,
		addr_bit => 0,
		size     => 32
	);
	-- Адрес mastera для передачи данных (когда много MOSI 0-31)
	constant REG_WR_ADDR      : addr_reg_t := (
		addr_axi => 3,
		addr_bit => 0,
		size     => 5
	);
	-- Регситр адреса mastera для принятых данных (когда много MISO 0-31)
	constant REG_RD_ADDR      : addr_reg_t := (
		addr_axi => 3,
		addr_bit => 5,
		size     => 5
	);
	-- Маска для установки CS (когда много CS. выставляем '1' по номеру абонента, которому передаем 0-0xffffffff)
	constant REG_CS_ADDR      : addr_reg_t := (
		addr_axi => 4,
		addr_bit => 0,
		size     => 32
	);
	-- Размерность данных SPI (до 32)
	constant REG_DW           : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 0,
		size     => 5
	);
	-- waiting after CS or BUSY before first pulse SCLK 1-16 в полупериодах SCLK
	constant REG_START_WAIT   : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 5,
		size     => 4
	);
	-- waiting after last SCLK pulse before CS >= 0 (При нуле SCLK заканчивается с CS одновременно) 0-16 в полупериодах SCLK
	constant REG_END_WAIT     : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 9,
		size     => 4
	);
	-- Деление s_axi_clk x2 (Для 100Мгц 10 означает 5МГЦ на SCLK) 1-256
	constant REG_CLK_DIV      : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 13,
		size     => 8
	);
	-- Минимальная длина CS между посылками (в полупериодах SCLK) >= 0 
	-- (START_WAIT=1, END_WAIT=0, CS_MIN_WIDTH=0 - непрерывный SCLK) 0-16 в полупериодах SCLK 
	constant REG_CS_MIN_WIDTH : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 21,
		size     => 4
	);
	-- SPI mode selection (mode 0 default)
	-- CPOL = clock polarity, CPHA = clock phase.
	constant REG_CPOL         : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 25,
		size     => 1
	);
	constant REG_CPHA         : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 26,
		size     => 1
	);
	-- Порядок следования бит (1 - старшие вперед, 0 - младшие вперед)
	constant REG_MSB_FIRST    : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 27,
		size     => 1
	);
	
	-- Включение ручного режима управления CS (1 - вкл, 0 - выкл)
	constant REG_EN_CS_MANUAL    : addr_reg_t := (
		addr_axi => 5,
		addr_bit => 28,
		size     => 1
	);
	
	

end package axi_pkg;
