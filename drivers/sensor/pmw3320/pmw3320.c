#define DT_DRV_COMPAT pixart_pmw3320

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include "pmw3320_reg.h"

LOG_MODULE_REGISTER(PMW3320, CONFIG_SENSOR_LOG_LEVEL);

struct pmw3320_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec irq_gpio;
};

struct pmw3320_data {
    const struct device *dev;
    struct k_work work;
    struct k_timer timer;
};

// 3-wire SPI リード
static int pmw3320_read(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct pmw3320_config *config = dev->config;
    uint8_t addr = reg & ~0x80;
    
    struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
    struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf = { .buf = val, .len = 1 };
    struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    int ret = spi_write_dt(&config->bus, &tx);
    if (ret < 0) return ret;
    
    k_busy_wait(35);
    return spi_read_dt(&config->bus, &rx);
}

static int pmw3320_write(const struct device *dev, uint8_t reg, uint8_t val) {
    const struct pmw3320_config *config = dev->config;
    uint8_t buffer[2] = { reg | 0x80, val };
    struct spi_buf tx_buf = { .buf = buffer, .len = 2 };
    struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    int ret = spi_write_dt(&config->bus, &tx);
    k_busy_wait(180);
    return ret;
}

static void pmw3320_work_handler(struct k_work *work) {
    struct pmw3320_data *data = CONTAINER_OF(work, struct pmw3320_data, work);
    const struct device *dev = data->dev;
    uint8_t motion, xl, xh, yl, yh;

    if (pmw3320_read(dev, PMW3320_REG_MOTION, &motion) != 0) return;

    if (motion & 0x80) {
        pmw3320_read(dev, PMW3320_REG_DELTA_X_L, &xl);
        pmw3320_read(dev, PMW3320_REG_DELTA_X_H, &xh);
        pmw3320_read(dev, PMW3320_REG_DELTA_Y_L, &yl);
        pmw3320_read(dev, PMW3320_REG_DELTA_Y_H, &yh);

        int16_t x = (int16_t)((xh << 8) | xl);
        int16_t y = (int16_t)((yh << 8) | yl);

        input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
    }
}

static void pmw3320_timer_handler(struct k_timer *dummy) {
    struct pmw3320_data *data = CONTAINER_OF(dummy, struct pmw3320_data, timer);
    k_work_submit(&data->work);
}

static int pmw3320_init(const struct device *dev) {
    const struct pmw3320_config *config = dev->config;
    struct pmw3320_data *data = dev->data;

    uint8_t pid;
    int ret = pmw3320_read(dev, PMW3320_REG_PRODUCT_ID, &pid);
    LOG_ERR("PMW3320 communication test - RET: %d, PID: 0x%02x", ret, pid);

    if (!device_is_ready(config->bus.bus)) return -ENODEV;

    pmw3320_write(dev, PMW3320_REG_POWER_UP_RESET, PMW3320_RESET_VALUE);
    k_msleep(50);

    data->dev = dev;
    k_work_init(&data->work, pmw3320_work_handler);
    k_timer_init(&data->timer, pmw3320_timer_handler, NULL);
    k_timer_start(&data->timer, K_MSEC(10), K_MSEC(10));

    return 0;
}

#define PMW3320_INIT(n) \
    static struct pmw3320_data pmw3320_data_##n; \
    static const struct pmw3320_config pmw3320_config_##n = { \
        .bus = { \
            .bus = DEVICE_DT_GET(DT_INST_BUS(n)), \
            .config = { \
                .frequency = 2000000, \
                .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB, \
                .slave = DT_INST_REG_ADDR(n), \
            }, \
        }, \
        .irq_gpio = GPIO_DT_SPEC_INST_GET_OR(n, irq_gpios, {0}), \
    }; \
    DEVICE_DT_INST_DEFINE(n, pmw3320_init, NULL, \
                         &pmw3320_data_##n, &pmw3320_config_##n, \
                         POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3320_INIT)