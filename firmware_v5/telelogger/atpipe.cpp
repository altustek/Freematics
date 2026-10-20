/******************************************************************************
* USB serial to OBD co-processor / cellular modem pipe for typing AT commands
* Idea from the ankostis fork (atcmdspipe.cpp)
******************************************************************************/

#include <FreematicsPlus.h>
#include "driver/uart.h"
#include "atpipe.h"

// free until sys.begin() installs the library's own UART drivers
#define AT_PIPE_UART UART_NUM_1

#define KEY_CTRL_D 0x04
#define KEY_CTRL_E 0x05
#define KEY_CTRL_N 0x0E
#define KEY_CTRL_P 0x10

struct PipeTarget {
  const char* name;
  int rxPin;
  int txPin;
  int baudrate;
};

static const PipeTarget targets[] = {
  // UART link only (Model B); a Model A co-processor sits on SPI
  {"OBD co-processor", PIN_LINK_UART_RX, PIN_LINK_UART_TX, LINK_UART_BAUDRATE},
  {"cellular modem", PIN_BEE_UART_RXD, PIN_BEE_UART_TXD, (int)BEE_BAUDRATE},
};

static void openTarget(const PipeTarget& t)
{
  uart_config_t uart_config = {
    .baud_rate = t.baudrate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
  };
  uart_param_config(AT_PIPE_UART, &uart_config);
  uart_set_pin(AT_PIPE_UART, t.txPin, t.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(AT_PIPE_UART, 1024, 0, 0, NULL, 0);
  Serial.printf("\n[ATPIPE] %s (Ctrl+N next, Ctrl+P modem power, Ctrl+E echo, Ctrl+D end)\n", t.name);
}

static void toggleModemPower()
{
#ifdef PIN_BEE_PWR
  // same pulse as FreematicsESP32::xbTogglePower()
  pinMode(PIN_BEE_PWR, OUTPUT);
  digitalWrite(PIN_BEE_PWR, HIGH);
  delay(200);
  digitalWrite(PIN_BEE_PWR, LOW);
  Serial.println("\n[ATPIPE] Modem power toggled");
#endif
}

void atPipe(unsigned int timeoutSec)
{
  Serial.printf("[ATPIPE] Press a key within %us for an AT command pipe\n", timeoutSec);
  for (uint32_t t = millis(); !Serial.available(); delay(10)) {
    if (millis() - t >= timeoutSec * 1000UL) return;
  }
  // drop the key that opened the pipe
  while (Serial.available()) Serial.read();

  const byte count = sizeof(targets) / sizeof(targets[0]);
  byte cur = 0;
  bool echo = true;
  bool used = false;
  openTarget(targets[cur]);
  for (;;) {
    uint8_t buf[64];
    int n = uart_read_bytes(AT_PIPE_UART, buf, sizeof(buf), 0);
    if (n > 0) Serial.write(buf, n);
    int c = Serial.read();
    if (c < 0) {
      delay(1);
      continue;
    }
    if (c == KEY_CTRL_D) break;
    switch (c) {
    case KEY_CTRL_N:
      uart_driver_delete(AT_PIPE_UART);
      cur = (cur + 1) % count;
      openTarget(targets[cur]);
      break;
    case KEY_CTRL_P:
      toggleModemPower();
      used = true;
      break;
    case KEY_CTRL_E:
      echo = !echo;
      Serial.printf("\n[ATPIPE] Echo %s\n", echo ? "on" : "off");
      break;
    default: {
      uint8_t b = c;
      uart_write_bytes(AT_PIPE_UART, (const char*)&b, 1);
      if (echo) {
        if (c == '\r') Serial.print("\r\n");
        else Serial.write(b);
      }
      used = true;
    }
    }
  }
  uart_driver_delete(AT_PIPE_UART);
  if (used) {
    // modules were touched: start over from a clean state
    Serial.println("\n[ATPIPE] Restarting");
    delay(100);
    ESP.restart();
  }
  Serial.println("\n[ATPIPE] Closed");
}
