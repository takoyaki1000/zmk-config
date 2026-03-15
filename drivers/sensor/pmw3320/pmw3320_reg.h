#pragma once

// PMW3320 Registers
#define PMW3320_REG_PRODUCT_ID               0x00
#define PMW3320_REG_REVISION_ID              0x01
#define PMW3320_REG_MOTION                   0x02
#define PMW3320_REG_DELTA_X_L                0x03
#define PMW3320_REG_DELTA_X_H                0x04
#define PMW3320_REG_DELTA_Y_L                0x05
#define PMW3320_REG_DELTA_Y_H                0x06
#define PMW3320_REG_SQUAL                    0x07
#define PMW3320_REG_CONFIG1                  0x0f
#define PMW3320_REG_CONFIG2                  0x10
#define PMW3320_REG_SHUTDOWN                 0x3b

// Power-up sequence
#define PMW3320_REG_POWER_UP_RESET           0x3a
#define PMW3320_RESET_VALUE                  0x5a

// SPI Settings
#define PMW3320_SPI_WRITE_BIT                0x80