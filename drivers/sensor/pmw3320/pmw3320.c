#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include "pmw3320_reg.h"
#define DT_DRV_COMPAT pixart_pmw3320

LOG_MODULE_REGISTER(PMW3320, CONFIG_SENSOR_LOG_LEVEL);

struct pmw3320_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec irq_gpio;
};

struct pmw3320_data {
    const struct device *dev;
    struct k_work work;
};

// SPI Read/Write ヘルパー
static int pmw3320_read(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct pmw3320_config *config = dev->config;
    uint8_t buffer[1] = { reg & ~PMW3320_SPI_WRITE_BIT };
    struct spi_buf tx_buf = { .buf = buffer, .len = 1 };
    struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = val, .len = 1 };
    struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    int ret = spi_transceive_dt(&config->bus, &tx, &rx);
    k_busy_wait(35); // tSRAD: 35us
    return ret;
}

static int pmw3320_write(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct pmw3320_config *config = dev->config;
    uint8_t buffer[2] = { reg | PMW3320_SPI_WRITE_BIT, val };
    struct spi_buf tx_buf = { .buf = buffer, .len = 2 };
    struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    int ret = spi_write_dt(&config->bus, &tx);
    k_busy_wait(180); // tSWR: 180us
    return ret;
}

// モーションデータの読み取りと報告
static void pmw3320_work_handler(struct k_work *work) {
    struct pmw3320_data *data = CONTAINER_OF(work, struct pmw3320_data, work);
    const struct device *dev = data->dev;

    uint8_t motion, xl, xh, yl, yh;
    pmw3320_read(dev, PMW3320_REG_MOTION, &motion);

    if (motion & 0x80) { // Motion bit detected
        pmw3320_read(dev, PMW3320_REG_DELTA_X_L, &xl);
        pmw3320_read(dev, PMW3320_REG_DELTA_X_H, &xh);
        pmw3320_read(dev, PMW3320_REG_DELTA_Y_L, &yl);
        pmw3320_read(dev, PMW3320_REG_DELTA_Y_H, &yh);

        int16_t x = (int16_t)((xh << 8) | xl);
        int16_t y = (int16_t)((yh << 8) | yl);

        // ZMK Input Subsystemへ報告
        input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
    }
}

// 初期化シーケンス
static int pmw3320_init(const struct device *dev) {
    const struct pmw3320_config *config = dev->config;
    struct pmw3320_data *data = dev->data;

    if (!spi_is_ready_dt(&config->bus)) return -ENODEV;

    // Power-up sequence (Datasheet 10.1)
    pmw3320_write(dev, PMW3320_REG_POWER_UP_RESET, PMW3320_RESET_VALUE);
    k_msleep(50); // Wait for reboot

    // Product ID 確認 (0x39 or similar)
    uint8_t pid;
    pmw3320_read(dev, PMW3320_REG_PRODUCT_ID, &pid);
    if (pid != 0x39) {
        LOG_ERR("Failed to detect PMW3320, ID: 0x%02x", pid);
        return -EIO;
    }

    // Work queue initialization (polling or interrupt)
    data->dev = dev;
    k_work_init(&data->work, pmw3320_work_handler);

    // TODO: GPIO割り込み設定 (Motion Pin)
    // 今回は簡略化のためポーリング、またはタイマーでworkをkickする想定
    
    return 0;
}

#define PMW3320_INIT(n) \
    static struct pmw3320_data pmw3320_data_##n; \
    static const struct pmw3320_config pmw3320_config_##n = { \
        .bus = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
        .irq_gpio = GPIO_DT_SPEC_INST_GET_OR(n, irq_gpios, {0}), \
    }; \
    /* POST_KERNEL で初期化し、優先度を低めに設定 */ \
    DEVICE_DT_INST_DEFINE(n, pmw3320_init, NULL, \
                         &pmw3320_data_##n, &pmw3320_config_##n, \
                         POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3320_INIT)