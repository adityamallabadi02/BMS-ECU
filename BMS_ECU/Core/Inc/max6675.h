/**
  ******************************************************************************
  * @file    max6675.h
  * @brief   MAX6675 SPI thermocouple driver — public interface
  ******************************************************************************
  */
#ifndef MAX6675_H
#define MAX6675_H

#include "stm32f4xx_hal.h"

/* Bit 2 of the 16-bit SPI word signals open-thermocouple fault */
#define MAX6675_FAULT_BIT   0x0004U

/**
  * @brief  Read one MAX6675 thermocouple.
  * @param  cs_port   GPIO port for chip-select (e.g. GPIOE)
  * @param  cs_pin    GPIO pin  for chip-select (e.g. GPIO_PIN_0)
  * @param  fault_out Set to 1 on open-circuit fault or SPI error, else 0
  * @retval Temperature in °C (ignore when fault_out == 1)
  */
float MAX6675_Read(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t *fault_out);

#endif /* MAX6675_H */
