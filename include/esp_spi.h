// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */

#ifndef _ESP_SPI_H_
#define _ESP_SPI_H_

#include "esp.h"

#define HANDSHAKE_PIN           24
#define SPI_IRQ                 gpio_to_irq(HANDSHAKE_PIN)
#define SPI_DATA_READY_PIN      25
#define SPI_DATA_READY_IRQ      gpio_to_irq(SPI_DATA_READY_PIN)
#define SPI_BUF_SIZE            1600

enum spi_flags_e {
	ESP_SPI_BUS_CLAIMED,
	ESP_SPI_BUS_SET,
	ESP_SPI_GPIO_HS_REQUESTED,
	ESP_SPI_GPIO_HS_IRQ_DONE,
	ESP_SPI_GPIO_DR_REQUESTED,
	ESP_SPI_GPIO_DR_IRQ_DONE,
	ESP_SPI_DATAPATH_OPEN,
};

struct esp_spi_context {
	struct esp_adapter          *adapter;
	struct spi_device           *esp_spi_dev;
    struct gpio_desc            *reset_gpio;
    struct gpio_desc            *handshake_gpio;
    struct gpio_desc            *dataready_gpio;
	uint8_t                     handshake_irq;
	uint8_t                     dataready_irq;
	struct sk_buff_head         tx_q[MAX_PRIORITY_QUEUES];
	struct sk_buff_head         rx_q[MAX_PRIORITY_QUEUES];
	struct workqueue_struct     *spi_workqueue;
	struct work_struct          spi_work;
	struct workqueue_struct     *nw_cmd_reinit_workqueue;
	struct work_struct          nw_cmd_reinit_work;
	uint8_t                     spi_clk_mhz;
	uint8_t                     reserved[2];
	unsigned long               spi_flags;
};

enum {
	CLOSE_DATAPATH,
	OPEN_DATAPATH,
};

struct spi_device;
int esp_core_init(struct spi_device *spi);
void esp_core_exit(struct spi_device *spi);
int esp_core5_exit(struct spi_device *spi);
void esp_spi_reset(bool reset);

#endif
