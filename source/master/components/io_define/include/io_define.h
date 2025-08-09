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

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
// MPU SPI IO 11 13 12 10
#define MPU_FSPI_MOSI 11
#define MPU_FSPI_MISO 13
#define MPU_FSPI_SCLK 12
#define MPU_FSPI_CS 10
// up to 1MHz for all registers, and 20MHz for sensor data registers only
#define MPU_SPI_CLOCK_SPEED SPI_MASTER_FREQ_20M

// MPU I2C IO
#define MPU_SDA 18
#define MPU_SCL 8
#define MPU_DMP_INT 9
#define MPU_I2C_CLOCK_SPEED 400000  // range from 100KHz ~ 400KHz
#define MPU_GPIO_INPUT_PIN_SEL  ((1ULL<<MPU_DMP_INT))

// BMP280
#define BMP_I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf
#define BMP_HSPI_MOSI 35
#define BMP_HSPI_MISO 37
#define BMP_HSPI_SCLK 36
#define BMP_HSPI_CS 34
#define BMP_SPI_CLOCK_SPEED SPI_MASTER_FREQ_20M

// BMP I2C IO
#define BMP_SDA 35
#define BMP_SCL 36
#define BMP_I2C_CLOCK_SPEED 400000  // range from 100 KHz ~ 400Hz

// NRF24
#define NRF24_SPI_MOSI 7
#define NRF24_SPI_MISO 16
#define NRF24_SPI_SCLK 6
#define NRF24_SPI_CS 5
#define NRF24_SPI_CLOCK_SPEED SPI_MASTER_FREQ_10M

#define NRF24_CE 4
#define NRF24_INT 15
#define NRF24_GPIO_INPUT_PIN_SEL  ((1ULL<<NRF24_INT))

// TEST
#define TEST_IMU_IO   1
#define TEST_NRF24_IO 2

// MCPWM
#define MCPWM_S0         1
#define MCPWM_S1         2
#define MCPWM_S2        42
#define MCPWM_S3        41

// SPI_FLASH
#define SPI_FLASH_CLK    33
#define SPI_FLASH_CS     32
#define SPI_FLASH_MISO   34
#define SPI_FLASH_MOSI   35
#define SPI_FLASH_HD     30
#define SPI_FLASH_WP     31

// PMW SPI IO 11 13 12 10
#define PMW_FSPI_MOSI 11
#define PMW_FSPI_MISO 13
#define PMW_FSPI_SCLK 12
#define PMW_FSPI_CS   10
// up to 1MHz for all registers, and 20MHz for sensor data registers only
#define PMW_SPI_CLOCK_SPEED SPI_MASTER_FREQ_20M

// SD card
#define SD_CMD  38
#define SD_CLK  39
#define SD_D0   40

// Battery
#define MAX_SDA  47
#define MAX_SCL  48

#define BUZZER_IO  45

#endif /* end of include guard: _IO_DEFINE__ */
