// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"
#include "esp_utils.h"
#include "esp_cfg80211.h"

#define SPI_INITIAL_CLK_MHZ     10
#define TX_MAX_PENDING_COUNT    100
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

extern u32 raw_tp_mode;
uint8_t g_spi_mode = SPI_MODE_0;
static struct sk_buff *read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
static void spi_exit(void);
static int spi_init(void);
static void adjust_spi_clock(u8 spi_clk_mhz);

volatile u8 data_path;
volatile u8 host_sleep;
static struct esp_spi_context spi_context;
static char hardware_type = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
static atomic_t tx_pending;

static struct sk_buff *esp_spi_alloc_skb(u32 len)
{
	struct sk_buff *skb = NULL;
	u32 alloc_len;
	u8 offset;

	alloc_len = max(len, (u32)SPI_BUF_SIZE) + INTERFACE_HEADER_PADDING;

	skb = netdev_alloc_skb(NULL, alloc_len);

	if (skb) {
		/* Align SKB data pointer */
		offset = ((unsigned long)skb->data) & (SKB_DATA_ADDR_ALIGNMENT - 1);

		if (offset)
			skb_reserve(skb, INTERFACE_HEADER_PADDING - offset);
	}

	return skb;
}

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
	.alloc_skb	= esp_spi_alloc_skb,
};

static void open_data_path(void)
{
	atomic_set(&tx_pending, 0);
	msleep(200);
	data_path = OPEN_DATAPATH;
}

static void close_data_path(void)
{
	data_path = CLOSE_DATAPATH;
	msleep(200);
}

static irqreturn_t spi_data_ready_interrupt_handler(int irq, void *dev)
{
	/* ESP peripheral has queued buffer for transmission */
	if (spi_context.spi_workqueue)
		queue_work(spi_context.spi_workqueue, &spi_context.spi_work);

	return IRQ_HANDLED;
 }

static irqreturn_t spi_interrupt_handler(int irq, void *dev)
{
	/* ESP peripheral is ready for next SPI transaction */
	if (spi_context.spi_workqueue)
		queue_work(spi_context.spi_workqueue, &spi_context.spi_work);

	return IRQ_HANDLED;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
	struct esp_spi_context *context;
	struct sk_buff *skb = NULL;

	if (!data_path) {
		return NULL;
	}

	if (!adapter || !adapter->if_context) {
		esp_err("Invalid args\n");
		return NULL;
	}

	context = adapter->if_context;

	if (context->esp_spi_dev) {
		skb = skb_dequeue(&(context->rx_q[PRIO_Q_HIGH]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_MID]));
		if (!skb)
			skb = skb_dequeue(&(context->rx_q[PRIO_Q_LOW]));
	} else {
		esp_err("Invalid args\n");
		return NULL;
	}

	return skb;
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	u32 max_pkt_size = SPI_BUF_SIZE - sizeof(struct esp_payload_header);
	struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;
	struct esp_skb_cb *cb = NULL;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if (skb) {
			dev_kfree_skb(skb);
			skb = NULL;
		}
		return -EINVAL;
	}

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max spi transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	if (!data_path) {
		esp_info("%u datapath closed\n", __LINE__);
		dev_kfree_skb(skb);
		return -EPERM;
	}

	cb = (struct esp_skb_cb *)skb->cb;
	if (cb && cb->priv && (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT)) {
		esp_tx_pause(cb->priv);
		dev_kfree_skb(skb);
		skb = NULL;
		esp_verbose("TX Pause busy");
		if (spi_context.spi_workqueue)
			queue_work(spi_context.spi_workqueue, &spi_context.spi_work);
		return -EBUSY;
	}

	atomic_inc(&tx_pending);

	if (payload_header->if_type == ESP_INTERNAL_IF) {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_HIGH], skb);
	} else if (payload_header->if_type == ESP_HCI_IF) {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_MID], skb);
	} else {
		skb_queue_tail(&spi_context.tx_q[PRIO_Q_LOW], skb);
	}

	if (spi_context.spi_workqueue)
		queue_work(spi_context.spi_workqueue, &spi_context.spi_work);

	return 0;
}

int esp_validate_chipset(struct esp_adapter *adapter, u8 chipset)
{
	int ret = 0;

	switch(chipset) {
	case ESP_FIRMWARE_CHIP_ESP32:
	case ESP_FIRMWARE_CHIP_ESP32S2:
	case ESP_FIRMWARE_CHIP_ESP32S3:
	case ESP_FIRMWARE_CHIP_ESP32C2:
	case ESP_FIRMWARE_CHIP_ESP32C3:
	case ESP_FIRMWARE_CHIP_ESP32C6:
	case ESP_FIRMWARE_CHIP_ESP32C61:
	case ESP_FIRMWARE_CHIP_ESP32C5:
		adapter->chipset = chipset;
		esp_info("Chipset=%s ID=%02x detected over SPI\n", esp_chipname_from_id(chipset), chipset);
		break;
	default:
		esp_err("Unrecognized chipset ID=%02x\n", chipset);
		adapter->chipset = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
		break;
	}

	return ret;
}

int esp_deinit_module(struct esp_adapter *adapter)
{
	/* Second & onward boot-up cleanup:
	 *
	 * SPI is software and not a hardware based module.
	 * When boot-up event is received, we should discard all prior commands,
	 * old messages pending at network and re-initialize everything.
	 */
	uint8_t prio_q_idx, iface_idx;

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&spi_context.tx_q[prio_q_idx]);
	}
	atomic_set(&tx_pending, 0);

	for (iface_idx = 0; iface_idx < ESP_MAX_INTERFACE; iface_idx++) {
		struct esp_wifi_device *priv = adapter->priv[iface_idx];
		esp_mark_scan_done_and_disconnect(priv, true);
	}

	esp_remove_card(adapter);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&spi_context.tx_q[prio_q_idx]);
	}

	return 0;
}

static int process_rx_buf(struct sk_buff *skb)
{
	struct esp_payload_header *header;
	u16 len = 0;
	u16 offset = 0;

	if (!skb)
		return -EINVAL;

	header = (struct esp_payload_header *) skb->data;

	if (header->if_type >= ESP_MAX_IF) {
		return -EINVAL;
	}

	offset = le16_to_cpu(header->offset);
	len = le16_to_cpu(header->len);

	if (len == 0) {
		return -EINVAL;
	}
	if (len > SPI_BUF_SIZE || !ESP_OFFSET_VALID(offset)) {
		esp_err("Drop invalid pkt: len=%d offset=%d\n", len, offset);
		return -EINVAL;
	}

	/* Total length = offset (header + padding) + payload */
	len += offset;

	/* Trim SKB to actual size */
	skb_trim(skb, len);


	if (!data_path) {
		esp_verbose("%u datapath closed\n", __LINE__);
		return -EPERM;
	}

	/* enqueue skb for read_packet to pick it */
	if (header->if_type == ESP_INTERNAL_IF)
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_HIGH], skb);
	else if (header->if_type == ESP_HCI_IF)
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_MID], skb);
	else
		skb_queue_tail(&spi_context.rx_q[PRIO_Q_LOW], skb);

	/* indicate reception of new packet */
	esp_process_new_packet_intr(spi_context.adapter);

	return 0;
}

static void esp_spi_work(struct work_struct *work)
{
	struct spi_transfer trans;
	struct sk_buff *tx_skb = NULL, *rx_skb = NULL;
	struct esp_skb_cb *cb = NULL;
	u8 *rx_buf = NULL;
	int ret = 0;
	volatile int trans_ready, rx_pending;

//	trans_ready = gpio_get_value(HANDSHAKE_PIN);
//	rx_pending = gpio_get_value(SPI_DATA_READY_PIN);
	trans_ready = !gpiod_get_value(spi_context.handshake_gpio);
	rx_pending = !gpiod_get_value(spi_context.dataready_gpio);

	if (!trans_ready) {
		return;
	}

	if (data_path) {
		tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_HIGH]);
		if (!tx_skb)
			tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_MID]);
		if (!tx_skb)
			tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_LOW]);
		if (tx_skb) {
			if (atomic_read(&tx_pending))
				atomic_dec(&tx_pending);

			/* resume network tx queue if bearable load */
			cb = (struct esp_skb_cb *)tx_skb->cb;
			if (cb && cb->priv && atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
				esp_tx_resume(cb->priv);
#if TEST_RAW_TP
				if (raw_tp_mode != 0) {
					esp_raw_tp_queue_resume();
				}
#endif
			}
		}
	}

	if (!rx_pending && !tx_skb) {
		return;
	}

	memset(&trans, 0, sizeof(trans));
	trans.speed_hz = spi_context.spi_clk_mhz * NUMBER_1M;

	/* Setup and execute SPI transaction
	 *	Tx_buf: Check if tx_q has valid buffer for transmission,
	 *		else keep it blank
	 *
	 *	Rx_buf: Allocate memory for incoming data. This will be freed
	 *		immediately if received buffer is invalid.
	 *		If it is a valid buffer, upper layer will free it.
	 * */

	if (tx_skb) {
		if (tx_skb->len < SPI_BUF_SIZE) {
			if (skb_put_padto(tx_skb, SPI_BUF_SIZE)) {
				esp_err("Failed to pad TX buffer to SPI size\n");
				tx_skb = NULL;
				return;
			}
		}

		trans.tx_buf = tx_skb->data;
		esp_hex_dump_verbose("tx: ", trans.tx_buf, 32);
	} else {
		tx_skb = esp_spi_alloc_skb(SPI_BUF_SIZE);
		if (!tx_skb) {
			esp_err("Failed to alloc dummy SPI TX skb\n");
			return;
		}
		trans.tx_buf = skb_put(tx_skb, SPI_BUF_SIZE);
		memset((void *)trans.tx_buf, 0, SPI_BUF_SIZE);
	}

	rx_skb = esp_spi_alloc_skb(SPI_BUF_SIZE);
	if (!rx_skb) {
		esp_err("Failed to alloc SPI RX skb\n");
		dev_kfree_skb(tx_skb);
		return;
	}
	rx_buf = skb_put(rx_skb, SPI_BUF_SIZE);

	memset(rx_buf, 0, SPI_BUF_SIZE);

	trans.rx_buf = rx_buf;
	trans.len = SPI_BUF_SIZE;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0))
	if (hardware_type == ESP_FIRMWARE_CHIP_ESP32) {
		trans.cs_change = 1;
	}
#endif

	ret = spi_sync_transfer(spi_context.esp_spi_dev, &trans, 1);
	if (ret) {
		esp_err("SPI Transaction failed: %d", ret);
		dev_kfree_skb(rx_skb);
		dev_kfree_skb(tx_skb);
	} else {
		/* Free rx_skb if received data is not valid */
		if (process_rx_buf(rx_skb))
			dev_kfree_skb(rx_skb);

		dev_kfree_skb(tx_skb);
	}
}
/*
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
#include <linux/platform_device.h>
static int __spi_controller_match(struct device *dev, const void *data)
{
	struct spi_controller *ctlr;
	const u16 *bus_num = data;

	ctlr = container_of(dev, struct spi_controller, dev);

	if (!ctlr) {
		return 0;
	}

	return ctlr->bus_num == *bus_num;
}

#endif
*/
static int spi_dev_init(int spi_clk_mhz)
{
	int status = 0;

//	struct gpio_desc *handshake_gpio;
//	struct gpio_desc *dataready_gpio;
//	struct gpio_desc *reset_gpio;
//	int handshake_irq;
//	int dataready_irq;

	esp_info("Using SPI MODE %d\n",g_spi_mode);

	set_bit(ESP_SPI_BUS_CLAIMED, &spi_context.spi_flags);
	spi_context.adapter->dev = &spi_context.esp_spi_dev->dev;

	set_bit(ESP_SPI_BUS_SET, &spi_context.spi_flags);

	spi_context.handshake_gpio =
		devm_gpiod_get(&spi_context.esp_spi_dev->dev,
		"handshake",
		GPIOD_IN);
	if (IS_ERR(spi_context.handshake_gpio)) {
		esp_err("Failed to obtain GPIO for Handshake pin, err:%d\n",
		PTR_ERR(spi_context.handshake_gpio));
		return PTR_ERR(spi_context.handshake_gpio);
	}
	set_bit(ESP_SPI_GPIO_HS_REQUESTED, &spi_context.spi_flags);

	spi_context.handshake_irq = gpiod_to_irq(spi_context.handshake_gpio);

	if (spi_context.handshake_irq < 0) {
		esp_err("Failed to obtain IRQ for Handshake pin, err:%d\n",
			spi_context.handshake_irq);
		return spi_context.handshake_irq;
	}

	status = devm_request_irq(&spi_context.esp_spi_dev->dev,
		spi_context.handshake_irq, spi_interrupt_handler,
		IRQF_SHARED | IRQF_TRIGGER_RISING,
		"ESP_SPI",
		spi_context.esp_spi_dev);
	if (status) {
		esp_err("Failed to request IRQ for Handshake pin, err:%d\n", status);
	return status;
	}

	set_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags);

	spi_context.dataready_gpio =
		devm_gpiod_get(&spi_context.esp_spi_dev->dev,
		"dataready",
		GPIOD_IN);
	if (IS_ERR(spi_context.dataready_gpio)) {
		esp_err("Failed to obtain GPIO for DataReady pin, err:%d\n",
		PTR_ERR(spi_context.dataready_gpio));
		return PTR_ERR(spi_context.dataready_gpio);
	}
	set_bit(ESP_SPI_GPIO_DR_REQUESTED, &spi_context.spi_flags);

	spi_context.dataready_irq = gpiod_to_irq(spi_context.dataready_gpio);

	status = devm_request_irq(&spi_context.esp_spi_dev->dev,
                          spi_context.dataready_irq,
                          spi_data_ready_interrupt_handler,
                          IRQF_SHARED | IRQF_TRIGGER_RISING,
                          "ESP_SPI_DATA_READY",
                          spi_context.esp_spi_dev);

	if (spi_context.dataready_irq < 0) {
        	esp_err("Failed to obtain IRQ for Dataready pin, err:%d\n",
                	spi_context.dataready_irq);
        	return spi_context.dataready_irq;
	}

	set_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags);

	spi_context.reset_gpio = devm_gpiod_get(&spi_context.esp_spi_dev->dev,
                            "reset",
                            GPIOD_OUT_HIGH);
	if (IS_ERR(spi_context.reset_gpio)) {
        	esp_err("Failed to obtain reset GPIO, err=%ld\n",
                	PTR_ERR(spi_context.reset_gpio));
        	return PTR_ERR(spi_context.reset_gpio);
	}
	
	open_data_path();

	return 0;
}

void esp_spi_reset(bool reset)
{
        gpiod_set_value_cansleep(spi_context.reset_gpio, !reset);
        msleep(300);
}

static int spi_init(void)
{
	int status = 0;
	uint8_t prio_q_idx = 0;
	struct esp_adapter *adapter;

	spi_context.spi_workqueue = alloc_ordered_workqueue("ESP_SPI_WORK_QUEUE", 0);

	if (!spi_context.spi_workqueue) {
		esp_err("spi workqueue failed to create\n");
		spi_exit();
		return -EFAULT;
	}

	INIT_WORK(&spi_context.spi_work, esp_spi_work);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&spi_context.tx_q[prio_q_idx]);
		skb_queue_head_init(&spi_context.rx_q[prio_q_idx]);
	}

	status = spi_dev_init(spi_context.spi_clk_mhz);
	if (status) {
		spi_exit();
		esp_err("Failed Init SPI device\n");
		return status;
	}

	adapter = spi_context.adapter;
	atomic_set(&adapter->state, ESP_CONTEXT_READY);

	if (!adapter) {
		spi_exit();
		return -EFAULT;
	}

	adapter->dev = &spi_context.esp_spi_dev->dev;

	return status;
}

static void cleanup_spi_gpio(void)
{
	if (test_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags)) {
		clear_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags)) {
		clear_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_DR_REQUESTED, &spi_context.spi_flags)) {
		clear_bit(ESP_SPI_GPIO_DR_REQUESTED, &spi_context.spi_flags);
	}

	if (test_bit(ESP_SPI_GPIO_HS_REQUESTED, &spi_context.spi_flags)) {
		clear_bit(ESP_SPI_GPIO_HS_REQUESTED, &spi_context.spi_flags);
	}
}

static void spi_exit(void)
{
	uint8_t prio_q_idx = 0;
	if (spi_context.adapter)
		atomic_set(&spi_context.adapter->state, ESP_CONTEXT_DISABLED);

	if (test_bit(ESP_SPI_GPIO_HS_IRQ_DONE, &spi_context.spi_flags)) {
		disable_irq(SPI_IRQ);
	}

	if (test_bit(ESP_SPI_GPIO_DR_IRQ_DONE, &spi_context.spi_flags)) {
		disable_irq(SPI_DATA_READY_IRQ);
	}

	close_data_path();
	msleep(200);

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_purge(&spi_context.tx_q[prio_q_idx]);
		skb_queue_purge(&spi_context.rx_q[prio_q_idx]);
	}
	atomic_set(&tx_pending, 0);

	if (spi_context.spi_workqueue) {
		flush_workqueue(spi_context.spi_workqueue);
		destroy_workqueue(spi_context.spi_workqueue);
		spi_context.spi_workqueue = NULL;
	}

	esp_remove_card(spi_context.adapter);

	cleanup_spi_gpio();

	if (spi_context.adapter && spi_context.adapter->hcidev)
		esp_deinit_bt(spi_context.adapter);

	spi_context.adapter->dev = NULL;
	memset(&spi_context, 0, sizeof(spi_context));
}

static void adjust_spi_clock(u8 spi_clk_mhz)
{
	if ((spi_clk_mhz) && (spi_clk_mhz != spi_context.spi_clk_mhz)) {
		esp_info("ESP Reconfigure SPI CLK to %u MHz\n", spi_clk_mhz);
		spi_context.spi_clk_mhz = spi_clk_mhz;
		spi_context.esp_spi_dev->max_speed_hz = spi_clk_mhz * NUMBER_1M;
	}
}

int esp_adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz)
{
	adjust_spi_clock(spi_clk_mhz);

	return 0;
}

int generate_slave_intr(void *context, u8 data)
{
	return 0;
}

int esp_init_interface_layer(struct esp_adapter *adapter, struct spi_device *spi, u32 speed)
{
	if (!adapter)
		return -EINVAL;

	memset(&spi_context, 0, sizeof(spi_context));
	spi_context.esp_spi_dev = spi;
	adapter->if_context = &spi_context;
	adapter->if_ops = &if_ops;
	adapter->if_type = ESP_IF_TYPE_SPI;
	spi_context.adapter = adapter;
	if (speed)
		spi_context.spi_clk_mhz = speed;
	else
		spi_context.spi_clk_mhz = SPI_INITIAL_CLK_MHZ;

	return spi_init();
}

void esp_deinit_interface_layer(void)
{
	spi_exit();
}

static const struct of_device_id esp_spi_of_match[] = {
    { .compatible = "espressif,esp32_spi" },
    { }
};
MODULE_DEVICE_TABLE(of, esp_spi_of_match);

static const struct spi_device_id esp_spi_ids[] = {
    { "esp32_spi", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, esp_spi_ids);

static struct spi_driver esp_spi_driver = {
    .driver = {
        .name = "esp_spi",
        .of_match_table = esp_spi_of_match,
    },
    .probe = esp_core_init,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	.remove = esp_core_exit,
#else
	.remove = esp_core5_exit,
#endif
    .id_table = esp_spi_ids,
};

module_spi_driver(esp_spi_driver);
