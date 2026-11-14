// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#define MICROPY_HW_BOARD_NAME "Argus5"
#define MICROPY_HW_MCU_NAME "rp2350b"

#define CIRCUITPY_PYSTACK_SIZE 2048

#define MICROPY_HW_NEOPIXEL (&pin_GPIO26)

// I2C0
#define DEFAULT_I2C_BUS_SDA (&pin_GPIO32)
#define DEFAULT_I2C_BUS_SCL (&pin_GPIO33)

// UART1, wired to the GPS (PX1120S)
#define DEFAULT_UART_BUS_TX (&pin_GPIO24)
#define DEFAULT_UART_BUS_RX (&pin_GPIO25)

// SPI0, shared by LoRa, magnetometer, IMU, SD card and backplane
#define DEFAULT_SPI_BUS_SCK  (&pin_GPIO2)
#define DEFAULT_SPI_BUS_MOSI (&pin_GPIO3)
#define DEFAULT_SPI_BUS_MISO (&pin_GPIO4)

// APS6404L 8 MB QSPI PSRAM on XIP_CS1
#define CIRCUITPY_PSRAM_CHIP_SELECT (&pin_GPIO0)
