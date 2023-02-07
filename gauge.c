#include <math.h>
#include <stdio.h>

#include "can.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define PIN_CAN_SCK 2
#define PIN_CAN_MOSI 3
#define PIN_CAN_MISO 4
#define PIN_CAN_CS 9
#define PIN_CAN_INT 11

#define SPI_PORT spi0

#define PWM_SLICE 0
#define PWM_CHANNEL PWM_CHAN_A

void CAN_reset() {
  gpio_put(PIN_CAN_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
  gpio_put(PIN_CAN_CS, 1);
  busy_wait_us(100);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(PIN_CAN_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(PIN_CAN_CS, 1);
  return (data);
}

void CAN_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(PIN_CAN_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(PIN_CAN_CS, 1);
}

void CAN_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(PIN_CAN_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(PIN_CAN_CS, 1);
}

void CAN_configure() {
  // Configure speed to 500kbps based on 16MHz Crystal
  // Magic constants from
  // https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0xf0);
  CAN_reg_write(REG_CNF3, 0x86);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 0);  // Enable filters, no rollover
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks RXM0 and RXM1 to exact match (0x7FF)
  for (int n = 0; n < 2; n++) {
    uint32_t mask = 0x7FF;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set up filter RXF0 to match appropriate address
  uint32_t addr = 0x355;
  CAN_reg_write(REG_RXFnSIDH(0), addr >> 3);
  CAN_reg_write(REG_RXFnSIDL(0), addr << 5);
  CAN_reg_write(REG_RXFnEID8(0), 0);
  CAN_reg_write(REG_RXFnEID0(0), 0);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

void CAN_receive() {
  uint8_t intf = CAN_reg_read(REG_CANINTF);

  if (intf & FLAG_RXnIF(0)) {
    uint8_t soc = CAN_reg_read(REG_RXBnD0(0) + 0);
    soc = 100 - soc;
    pwm_set_chan_level(PWM_SLICE, PWM_CHANNEL, 9500 + soc * 100);
    CAN_reg_modify(REG_CANINTF, FLAG_RXnIF(0), 0x00);
  }
}

int main() {
  spi_init(SPI_PORT, 500 * 1000);
  gpio_set_function(PIN_CAN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CAN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CAN_MISO, GPIO_FUNC_SPI);
  gpio_init(PIN_CAN_CS);
  gpio_set_dir(PIN_CAN_CS, GPIO_OUT);
  gpio_init(PIN_CAN_INT);
  gpio_set_dir(PIN_CAN_INT, GPIO_IN);
  gpio_pull_up(PIN_CAN_INT);
  gpio_put(PIN_CAN_CS, 1);

  gpio_set_function(0, GPIO_FUNC_PWM);
  pwm_set_wrap(PWM_SLICE, 20000);
  pwm_set_chan_level(PWM_SLICE, PWM_CHANNEL, 19000);
  pwm_set_enabled(PWM_SLICE, true);

  sleep_ms(50);

  CAN_reset();
  CAN_configure();

  while (1) {
    CAN_receive();
  }

  return 0;
}