/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2026 Skye Green
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#include <stdint.h>
#include "bt_hci.h"
#include "devicedata.h"
#include "esp_bluedroid_hci.h"
#include "esp_bt_main.h"
#include "gpio-ingenic.h"
#include "kernel.h"
#include "panic.h"
#include "thread.h"
#include "uart-x1000.h"
#include "x1000/uart.h"
#include "semaphore.h"
#include "bcm_firmware.h"

#ifdef BT_DEBUG_PCAP
#include "file.h"
#include "mutex.h"
#include "tick.h"

static int hci_log_fd = -1;
static struct mutex hci_log_mutex;

typedef struct {
    uint32_t seconds;
    uint32_t microseconds;
    uint32_t captured_packet_length;
    uint32_t original_packet_length;
    uint32_t direction;
} pcap_header_t;

#define HCI_STACK_SIZE (0x2000 + HCI_RECV_BUF_SIZE + 1024)

static void hci_log_packet(const uint8_t *data, size_t len, uint32_t direction)
{
    if (hci_log_fd == -1)
        return;
    mutex_lock(&hci_log_mutex);
    pcap_header_t header = {
        .seconds = current_tick,
        .microseconds = 0,
        .captured_packet_length = len + 4,
        .original_packet_length = len + 4,
        .direction = direction,
    };
    write(hci_log_fd, &header, 20);
    write(hci_log_fd, data, len);
    fsync(hci_log_fd);
    mutex_unlock(&hci_log_mutex);
}
#else
#define HCI_STACK_SIZE (0x2000 + HCI_RECV_BUF_SIZE)
#endif

void hci_send_uart(uint8_t *data, uint16_t len) {
#ifdef BT_DEBUG_PCAP
    if (hci_log_fd != -1) {
        mutex_lock(&hci_log_mutex);
        pcap_header_t header = {
            .seconds = current_tick,
            .microseconds = 0,
            .captured_packet_length = len + 4,
            .original_packet_length = len + 4,
            .direction = 0x01000000,
        };
        write(hci_log_fd, &header, 20);
        write(hci_log_fd, data, len);
        fsync(hci_log_fd);
        mutex_unlock(&hci_log_mutex);
    }
#endif
    uart_tx(PORT_UART0, data, len);
}

/* Set by bt_hci_disable() to make the receive thread exit */
static volatile bool hci_stop;
static unsigned int hci_thread_id;

/* Receives until data holds `want` bytes. Returns false, dropping the packet,
 * if bt_hci_disable() shuts the UART down in the meantime. */
static bool hci_recv_until(uint8_t *data, size_t *recvd, size_t want)
{
    while (*recvd < want) {
        if (hci_stop)
            return false;
        *recvd += uart_rx(PORT_UART0, data + *recvd, want - *recvd);
    }
    return true;
}

size_t hci_recv_uart(uint8_t *data, size_t len) {
    if (len == 0) {
        panicf("hci recv buffer too small, got %u but wanted at least 1", len);
    }
    size_t recvd = 0;
    if (!hci_recv_until(data, &recvd, 1))
        return 0;
    size_t expected;
    if (data[0] == 0x01) {
        if (len < 4) {
            panicf("hci recv buffer too small, got %u but wanted at least 4", len);
        }
        if (!hci_recv_until(data, &recvd, 4))
            return 0;
        expected = (((size_t) data[3]) & 0xFF) + 4;
    } else if (data[0] == 0x02) {
        if (len < 5) {
            panicf("hci recv buffer too small, got %u but wanted at least 5", len);
        }
        if (!hci_recv_until(data, &recvd, 5))
            return 0;
        /* 16-bit little-endian data length */
        expected = (((size_t) data[3]) | (((size_t) data[4]) << 8)) + 5;
    } else if (data[0] == 0x03) {
        if (len < 4) {
            panicf("hci recv buffer too small, got %u but wanted at least 4", len);
        }
        if (!hci_recv_until(data, &recvd, 4))
            return 0;
        expected = (((size_t) data[3]) & 0xFF) + 4;
    } else if (data[0] == 0x04) {
        if (len < 3) {
            panicf("hci recv buffer too small, got %u but wanted at least 3", len);
        }
        if (!hci_recv_until(data, &recvd, 3))
            return 0;
        expected = (((size_t) data[2]) & 0xFF) + 3;
    } else if (data[0] == 0x05) {
        if (len < 5) {
            panicf("hci recv buffer too small, got %u but wanted at least 5", len);
        }
        if (!hci_recv_until(data, &recvd, 5))
            return 0;
        /* 14-bit little-endian data length */
        expected = ((((size_t) data[3]) | (((size_t) data[4]) << 8)) & 0x3FFF) + 5;
    } else {
        // Unknown frame type.
        // Panic.
        panicf("hci got unknown frame type %d", data[0]);
    }
    if (len < expected) {
        panicf("hci recv buffer too small, got %u but wanted at least %u", len, expected);
    }
    if (!hci_recv_until(data, &recvd, expected))
        return 0;
#ifdef BT_DEBUG_PCAP
    hci_log_packet(data, recvd, 0x00000000);
#endif
    return recvd;
}

bool hci_check_send_available_uart(void) {
    return true;
}

const esp_bluedroid_hci_driver_callbacks_t *hci_callbacks;
struct semaphore hci_callbacks_sem;

esp_err_t hci_register_host_callback_uart(const esp_bluedroid_hci_driver_callbacks_t *callback) {
    if (hci_callbacks != NULL) {
        return ESP_FAIL;
    }
    hci_callbacks = callback;
    semaphore_release(&hci_callbacks_sem);
    return ESP_OK;
}

#define HCI_RECV_BUF_SIZE 1026

uint32_t hci_stack[HCI_STACK_SIZE / sizeof(uint32_t)];

void bluetooth_hci_task(void)
{
    uint8_t buf[HCI_RECV_BUF_SIZE];

    semaphore_wait(&hci_callbacks_sem, TIMEOUT_BLOCK);

    while (true) {
        size_t recvd = hci_recv_uart(buf, HCI_RECV_BUF_SIZE);
        if (hci_stop ||
            esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
            hci_callbacks = NULL;
            thread_exit();
            return;
        }
        hci_callbacks->notify_host_recv(buf, recvd);
    }
}

#define HCI_INIT_ATTEMPTS 3
#define HCI_INIT_TIMEOUT  HZ

/* Send one init command and wait (bounded) for its HCI event. The chip
 * occasionally ignores the first command after power-on, so nothing here
 * may block forever. */
static bool hci_init_cmd(const uint8_t *cmd, size_t len)
{
    uint8_t evt[3 + 255];

    hci_send_uart((uint8_t *)cmd, len);
    if (uart_rx_tmo(PORT_UART0, evt, 3, HCI_INIT_TIMEOUT) != 3 ||
        evt[0] != 0x04)
        return false;
    if (uart_rx_tmo(PORT_UART0, evt + 3, evt[2], HCI_INIT_TIMEOUT) != evt[2])
        return false;
#ifdef BT_DEBUG_PCAP
    hci_log_packet(evt, 3 + evt[2], 0x00000000);
#endif
    return true;
}

/* Power-cycle the chip, load its patch RAM and switch to 3 Mbaud. */
static bool hci_init_chip(void)
{
    /* Start from reset at 115200 baud, even if the chip was left running
     * (at 3 Mbaud) before a warm reboot. */
    gpio_set_level(GPIO_BT_REG_ON_HW4, 0);
    /* Stay off long enough for a full reset. Ingenic's X1000 BT power driver
     * waits 1 s after power-off; after only 100 ms the chip was seen not to
     * come back up when Bluetooth was turned off and on again. */
    sleep(HZ);
    uart_init(PORT_UART0, 115200);
    gpio_set_level(GPIO_BT_REG_ON_HW4, 1);
    /* Give the chip time to boot before the first HCI command, then wait
     * (bounded) for it to raise RTS, our CTS. The original firmware allows
     * 1 s after power-on. */
    sleep(HZ / 5);
    long deadline = current_tick + HZ - HZ / 5;
    while (!(REG_UART_UMSR(PORT_UART0) & BM_UART_UMSR_CTS) &&
           TIME_BEFORE(current_tick, deadline))
        sleep(1);

    // HCI_Reset
    if (!hci_init_cmd((uint8_t[]){0x01, 0x03, 0x0c, 0x00}, 4))
        return false;

    // Download_Minidriver
    if (!hci_init_cmd((uint8_t[]){0x01, 0x2e, 0xfc, 0x00}, 4))
        return false;

    sleep(HZ / 20);

    for (uint8_t **i = hcd_init_commands; *i != NULL; i++) {
        if (!hci_init_cmd(*i, ((size_t) (*i)[3]) + 4))
            return false;
    }

    // HCI_Reset
    if (!hci_init_cmd((uint8_t[]){0x01, 0x03, 0x0c, 0x00}, 4))
        return false;

    // Update_UART_Baud_Rate
    // Encoded_Baud_Rate: 0x0000 (unused)
    // Explicit_Baud_Rate: 0x002dc6c0 (DEC 3000000)
    if (!hci_init_cmd((uint8_t[]){0x01, 0x18, 0xfc, 0x06, 0x00, 0x00,
                                  0xc0, 0xc6, 0x2d, 0x00}, 10))
        return false;

    uart_set_baud(PORT_UART0, 3000000);
    return true;
}

/* Only the HW4 revision of the Eros Q native hardware has the BCM4343A1. */
bool bt_hci_available(void)
{
    return device_data.hw_rev == 4;
}

bool bt_hci_enable(void)
{
    if (device_data.hw_rev != 4) {
        panicf("hci not implemented for hw rev %d", device_data.hw_rev);
        return false;
    }

#ifdef BT_DEBUG_PCAP
    hci_log_fd = open("/hci_log.pcap", O_WRONLY | O_CREAT | O_TRUNC, 0666);
    write(hci_log_fd, (uint8_t[]){0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc9, 0x00, 0x00, 0x00}, 24);
    mutex_init(&hci_log_mutex);
#endif

    bool ok = false;
    for (int attempt = 0; attempt < HCI_INIT_ATTEMPTS && !ok; attempt++) {
        ok = hci_init_chip();
    }
    if (!ok) {
        uart_deinit(PORT_UART0);
        gpio_set_level(GPIO_BT_REG_ON_HW4, 0);
        return false;
    }

    hci_callbacks = NULL;
    semaphore_init(&hci_callbacks_sem, 1, 0);

    esp_bluedroid_hci_driver_operations_t operations = {
        .send = hci_send_uart,
        .check_send_available = hci_check_send_available_uart,
        .register_host_callback = hci_register_host_callback_uart,
    };
    esp_bluedroid_attach_hci_driver(&operations);
    hci_stop = false;
    hci_thread_id = create_thread(bluetooth_hci_task, hci_stack, HCI_STACK_SIZE, 0, "bt_hci" IF_PRIO(, PRIORITY_BLUETOOTH));
    return true;
}

void bt_hci_disable(void)
{
    /* Stop the receive thread and wait for it to exit. Left blocked in the
     * middle of a packet, it would take the replies meant for the next
     * bt_hci_enable(). */
    hci_stop = true;
    semaphore_release(&hci_callbacks_sem); /* if it never got started */
    uart_deinit(PORT_UART0); /* wakes it from uart_rx() */
    thread_wait(hci_thread_id);
    gpio_set_level(GPIO_BT_REG_ON_HW4, 0);
#ifdef BT_DEBUG_PCAP
    int old_fd = hci_log_fd;
    hci_log_fd = -1;
    if (old_fd != -1)
        close(old_fd);
#endif
}