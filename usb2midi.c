/*
 * USB to serial MIDI adapter
 *
 * Adapted from the TinyUSB midi_test example by Rene Stange
 */

/*
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "bsp/board.h"
#include "tusb.h"
#include "pico/util/queue.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

#define UART_ID uart0
#define BUFFER_SIZE 1024

queue_t uart_to_usb;

static bool led_usb_state = false;
static bool led_uart_state = false;

int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
void midi_task(void);
void led_task(void);
void on_uart_rx(void);

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  tusb_init();
  queue_init(&uart_to_usb, sizeof(uint8_t), BUFFER_SIZE);

  // Initialise UART 0
  uart_init(UART_ID, 31250);
  uart_set_fifo_enabled(UART_ID, true);
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
  // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
  gpio_set_function(16, GPIO_FUNC_UART);
  gpio_set_function(17, GPIO_FUNC_UART);
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);
  uart_set_irq_enables(UART_ID, true, false);

  while (1)
  {
    tud_task(); // tinyusb device task
    led_task();
    midi_task();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  led_usb_state = false;
  led_uart_state = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// UART Helper
//--------------------------------------------------------------------+

void on_uart_rx()
{
  while (uart_is_readable(UART_ID))
  {
    uint8_t data;
    uart_read_blocking(UART_ID, &data, 1);
    queue_add_blocking(&uart_to_usb, &data);
  }
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

void midi_task(void)
{
  if (!tud_midi_mounted())
  {
    return;
  }

  // Handle USB to UART direction
  uint8_t packet[4];
  if (tud_midi_available() && uart_is_writable(UART_ID))
  {
    tud_midi_packet_read(packet);

    static const size_t cin_to_length[] =
        {0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1};

    uint8_t cid = packet[0] & 0xF;
    {
      size_t length = cin_to_length[cid];
      if (length)
      {
        uart_write_blocking(UART_ID, packet + 1, length);
      }
    }

    led_usb_state = true;
  }
  else
  {
    led_usb_state = false;
  }

  // Handle UART to USB direction
  uint32_t n = queue_get_level(&uart_to_usb);
  uint8_t data[BUFFER_SIZE];
  if (n)
  {
    for (uint32_t i = 0; i < n; i++)
    {
      queue_remove_blocking(&uart_to_usb, &data[i]);
    }
    
    tud_midi_stream_write(0, data, n);
    led_uart_state = true;
  }
  else
  {
    led_uart_state = false;
  }
}

//--------------------------------------------------------------------+
// LED Task
//--------------------------------------------------------------------+

void led_task(void)
{
  static uint32_t last_active_ms = 0;

  if (led_usb_state || led_uart_state)
  {
    board_led_write(true);

    last_active_ms = board_millis();
  }
  else if (board_millis() - last_active_ms > 10)
  {
    board_led_write(false);
  }
}