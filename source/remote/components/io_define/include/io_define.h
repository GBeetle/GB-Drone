/*
 * This file is part of GB-Drone project (https://github.com/GBeetle/GB-Drone).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _IO_DEFINE__
#define _IO_DEFINE__

// NRF24
#define NRF24_SPI_MOSI 9
#define NRF24_SPI_MISO 6
#define NRF24_SPI_SCLK 8
#define NRF24_SPI_CS 5
#define NRF24_SPI_CLOCK_SPEED SPI_MASTER_FREQ_10M
#define NRF24_CE 7
#define NRF24_IRQ 10

// TFT
#define TFT_SPI_MOSI 11
#define TFT_SPI_MISO 13
#define TFT_SPI_SCLK 12
#define TFT_SPI_RES  18
#define TFT_SPI_DC   17
#define TFT_SPI_BLK  14
#define TFT_SPI_CLOCK_SPEED SPI_MASTER_FREQ_80M

#define CONFIG_DISP_SPI_MOSI 11
#define CONFIG_DISP_SPI_MISO -1
#define CONFIG_DISP_SPI_CLK  12
#define CONFIG_DISP_PIN_DC   17
#define CONFIG_DISP_PIN_RST  18
#define CONFIG_DISP_PIN_BCKL 14
#define CONFIG_DISP_PIN_BUSY -1
#define CONFIG_DISP_PIN_SDA  -1
#define CONFIG_DISP_PIN_SCL  -1
#define SPI_TFT_CLOCK_SPEED_HZ  SPI_MASTER_FREQ_10M

#define TEST_IMU_IO 17

// SD card
#define SD_CMD  38
#define SD_CLK  39
#define SD_D0   40

// Controller
#define BTN_GPIO_DPAD_UP       28
#define BTN_GPIO_DPAD_DOWN     26
#define BTN_GPIO_DPAD_LEFT     27
#define BTN_GPIO_DPAD_RIGHT    30
#define BTN_GPIO_DPAD_MID      29

#define BTN_GPIO_OP_A          11
#define BTN_GPIO_OP_B          12
#define BTN_GPIO_OP_X          50
#define BTN_GPIO_OP_Y          49

#define BTN_GPIO_OP_MENU       33
#define BTN_GPIO_OP_OPTION     31
#define BTN_GPIO_OP_SELECT     32
#define BTN_GPIO_OP_START      34

#define TOGGLE_SW_1_GPIO       4
#define TOGGLE_SW_2_GPIO       3
#define TOGGLE_SW_3_GPIO       2
#define TOGGLE_SW_4_GPIO       1

#endif /* end of include guard: _IO_DEFINE__ */
