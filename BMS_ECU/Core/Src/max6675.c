/**
  ******************************************************************************
  * @file    max6675.c
  * @brief   MAX6675 SPI thermocouple driver
  *
  * Wiring (per original project):
  *   TC1 CS  → PE0
  *   TC2 CS  → PE1
  *   SCK/SO  → SPI2 (hspi2, configured in main.c)
  *
  * Protocol:
  *   • Assert CS LOW, read 16 bits MSB-first, de-assert CS HIGH.
  *   • Bit 2 (D2) is the open-thermocouple fault flag.
  *   • Bits [14:3] are the 12-bit temperature (°C × 4 LSBs, i.e. 0.25 °C res).
  ******************************************************************************
  */

#include "max6675.h"
#include "main.h"      /* hspi2, GPIO defines */

extern SPI_HandleTypeDef hspi2;

/**
  * @brief  Read one MAX6675 thermocouple sensor.
  * @param  cs_port  GPIO port of the chip-select pin (e.g. GPIOE)
  * @param  cs_pin   GPIO pin  of the chip-select     (e.g. GPIO_PIN_0)
  * @param  fault_out  Pointer to fault flag output:
  *                      0 = OK, 1 = open thermocouple (or SPI error)
  * @retval Temperature in °C (valid only when *fault_out == 0)
  */
float MAX6675_Read(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t *fault_out)
{
    uint8_t  rx[2]    = {0, 0};
    uint16_t raw      = 0;
    float    temp_c   = 0.0f;

    *fault_out = 0;

    /* Assert CS (active LOW) */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

    /* Small setup delay — MAX6675 needs ≥100 ns CS-to-CLK; HAL overhead is
       sufficient at 42 MHz APB2, but add a tiny delay for safety on long wires */
    /* __NOP(); __NOP(); */   /* uncomment if you see glitches */

    /* Receive 2 bytes (16 bits) */
    if (HAL_SPI_Receive(&hspi2, rx, 2, 10) != HAL_OK)
    {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
        *fault_out = 1;
        return 0.0f;
    }

    /* De-assert CS */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* Reconstruct 16-bit word (MSB first) */
    raw = ((uint16_t)rx[0] << 8) | rx[1];

    /* Bit 2 (D2) = open-thermocouple fault */
    if (raw & MAX6675_FAULT_BIT)
    {
        *fault_out = 1;
        return 0.0f;
    }

    /* Bits [14:3] = 12-bit temperature value (0.25 °C per LSB) */
    temp_c = (float)((raw >> 3) & 0x0FFF) * 0.25f;

    return temp_c;
}
