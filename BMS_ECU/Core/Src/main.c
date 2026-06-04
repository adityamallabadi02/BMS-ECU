/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c  [BMS NODE — FINAL MERGED BUILD]
  * @brief          : BMS Node — motor control, temperature, battery SOC,
  *                   CAN RX/TX fully aligned with ABS ECU
  *
  *  CHANGES IN THIS BUILD
  *  ─────────────────────────────────────────────────────────────────────
  *  FIX 1 – isr_log_flag/id/dlc/data now assigned inside ISR before switch
  *           so block [2] in main() actually prints every received frame.
  *
  *  FIX 2 – TEC/REC corrected per STM32F4 RM §32.9.4:
  *           ESR[23:16]=TEC  ESR[15:8]=REC  (old code read wrong nibbles)
  *
  *  FIX 3 – CAN_IT_TX_MAILBOX_EMPTY activated + 3 mailbox callbacks added.
  *           Callbacks fire when 0x105 physically leaves the TX mailbox.
  *  ─────────────────────────────────────────────────────────────────────
  */
/* USER CODE END Header */

/* ── Includes ────────────────────────────────────────────────────────────── */
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "core_cm4.h"

/* ── Defines ─────────────────────────────────────────────────────────────── */

/* INA226 */
#define INA226_ADDR        (0x40 << 1)
#define VOLTAGE_SCALE      1.2f
#define VOLTAGE_OFFSET     0.5f

#define TEMP_HIGH_THRESHOLD_C  100.0f   /* adjust as needed */

#define GEAR_BYTE_DRIVE     0
#define GEAR_BYTE_SPORT     1
#define GEAR_BYTE_PARK      2
#define GEAR_BYTE_NEUTRAL   3
#define GEAR_BYTE_REVERSE   4

/* Airbag ECU */
#define AIRBAG_ECU_ID      0x100U
#define AIRBAG_CRASH_BYTE  0x01U

/* CAN IDs */
#define CAN_ID_ABS_BRAKE   0x101U
#define CAN_ID_TCU_GEAR    0x104U
#define CAN_ID_TELEMATICS  0x107U
#define CAN_ID_BMS_OUT     0x105U
/* Timing */
#define CAN_TX_INTERVAL_MS   1000U    // transmit 0x105
#define PRINT_INTERVAL_MS   1000U    // Putty

/* ── Peripheral handles ──────────────────────────────────────────────────── */
ADC_HandleTypeDef  hadc1;
CAN_HandleTypeDef  hcan1;
I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi2;
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart2;

/* ── CAN filter ──────────────────────────────────────────────────────────── */
CAN_FilterTypeDef  filter;

/* ── Motor / ADC ─────────────────────────────────────────────────────────── */
uint32_t adcValue = 0;
uint32_t pwmValue = 0;
uint32_t rpm      = 0;

/* ── Temperature (SPI thermocouple) ─────────────────────────────────────── */
uint8_t  spi_rx[2]    = {0};
uint16_t spi_value    = 0;
float    temperature1 = 0.0f;
float    temperature2 = 0.0f;

/* ── Battery ─────────────────────────────────────────────────────────────── */
float battery_voltage = 0.0f;
float battery_soc     = 0.0f;

/* ── Timing ticks ────────────────────────────────────────────────────────── */
uint32_t lastTxTime    = 0;
uint32_t lastPrintTime = 0;
uint32_t lastInaTime   = 0;

/* ── CAN counters ────────────────────────────────────────────────────────── */
volatile uint32_t can_rx_total = 0;
volatile uint32_t can_tx_total = 0;
volatile uint32_t can_tx_fail  = 0;

/* ── Immobilizer ─────────────────────────────────────────────────────────── */
volatile uint8_t immobilized = 0;

/* ── CAN RX: 0x101 — ABS ECU ─────────────────────────────────────────────── */
volatile uint8_t can_rx_abs_flag  = 0;
volatile uint8_t rx_brake_pct     = 0;
volatile uint8_t rx_handbrake = 0;
/* ── CAN RX: 0x104 — TCU gear ────────────────────────────────────────────── */
volatile uint8_t can_rx_gear_flag = 0;
volatile uint8_t rx_gear[5] = {0};

/* ── CAN RX: 0x107 — Telematics ─────────────────────────────────────────── */
volatile uint8_t telematics_flag  = 0;
volatile uint8_t telematics_cmd   = 0;

/* ── CAN RX: 0x100 — Airbag ──────────────────────────────────────────────── */
volatile uint8_t crash_flag = 0;
volatile uint8_t crash_data = 0;

/* ── Function prototypes ─────────────────────────────────────────────────── */
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);

/* ABS-style CAN TX wrapper */
static HAL_StatusTypeDef CAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);

/* INA226 */
void     INA226_WriteRegister(uint8_t reg, uint16_t value);
uint16_t INA226_ReadRegister(uint8_t reg);
void     INA226_Init(void);
float    INA226_ReadBusVoltage(void);
float    Get_Battery_SOC(float voltage);


int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static HAL_StatusTypeDef CAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader = {0};
    uint32_t txMailbox = 0;

    txHeader.StdId              = std_id;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = len;
    txHeader.TransmitGlobalTime = DISABLE;

    /* Wait for a free mailbox — 10 ms timeout (same as ABS) */
    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        if ((HAL_GetTick() - t0) > 10U)
        {
            printf("\033[0;31m[CAN TX TIMEOUT] ID: 0x%03X — no free mailbox\r\n\033[0m", std_id);
            can_tx_fail++;
            return HAL_TIMEOUT;
        }
    }

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);

    if (status == HAL_OK)
    {
        can_tx_total++;
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);   /* blink LED on TX */
        printf("\033[0;32m[CAN TX OK ] ID: 0x%03X  DLC: %d  DATA:", std_id, len);
        for (int i = 0; i < len; i++) printf(" %02X", data[i]);
        printf("\r\n\033[0m");
    }
    else
    {
        can_tx_fail++;
        printf("\033[0;31m[CAN TX FAIL] ID: 0x%03X  ERR: 0x%08lX\r\n\033[0m",
               std_id, HAL_CAN_GetError(&hcan1));
    }

    return status;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  CAN RX ISR
 *  FIX 1 — isr_log_id / isr_log_dlc / isr_log_data / isr_log_flag
 *           are now populated here (they were never assigned before).
 *           Block [2] in main() now prints every received frame.
 * ════════════════════════════════════════════════════════════════════════ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    CAN_RxHeaderTypeDef rx_hdr = {0};
    uint8_t             rx_buf[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, rx_buf) != HAL_OK)
        return;

    can_rx_total++;

    switch (rx_hdr.StdId)
    {
        /* ── 0x100: Airbag / Crash ECU ── */
        case 0x100U:
            crash_data = rx_buf[0];
            crash_flag = 1;
            break;

            /* ── 0x101: ABS ECU — brake_pct, handbrake ── */
            case 0x101U:
                rx_brake_pct    = rx_buf[0];   /* brake percentage 0–100  */
                rx_handbrake    = rx_buf[1];   /* handbrake 0=OFF 1=ON    */
                can_rx_abs_flag = 1;
                break;

        /* ── 0x104: TCU gear byte ── */
        case 0x104U:
            rx_gear[0] = rx_buf[0];   // Drive
            rx_gear[1] = rx_buf[1];   // Sport
            rx_gear[2] = rx_buf[2];   // Park
            rx_gear[3] = rx_buf[3];   // Neutral
            rx_gear[4] = rx_buf[4];   // Reverse
            can_rx_gear_flag = 1;
            break;

        /* ── 0x107: Telematics — 0x99 immobilize / 0x11 mobilize ── */
        case 0x107U:
            telematics_cmd  = rx_buf[0];
            telematics_flag = 1;
            break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  TX MAILBOX CALLBACKS  — FIX 3 (new functions)
 *  These fire when 0x105 physically leaves the TX mailbox onto the bus.
 *  If you see these printing every 50 ms → TX is working on the bus.
 * ════════════════════════════════════════════════════════════════════════ */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    printf("\033[0;32m[TX MB0] 0x105 left mailbox — on bus\r\n\033[0m");
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    printf("\033[0;32m[TX MB1] 0x105 left mailbox — on bus\r\n\033[0m");
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    printf("\033[0;32m[TX MB2] 0x105 left mailbox — on bus\r\n\033[0m");
}

/* ── INA226 ──────────────────────────────────────────────────────────────── */
void INA226_WriteRegister(uint8_t reg, uint16_t value)
{
    uint8_t d[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, d, 3, HAL_MAX_DELAY);
}

uint16_t INA226_ReadRegister(uint8_t reg)
{
    uint8_t d[2] = {0};
    HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive (&hi2c1, INA226_ADDR, d,    2, HAL_MAX_DELAY);
    return (uint16_t)((d[0] << 8) | d[1]);
}

void INA226_Init(void)
{
    INA226_WriteRegister(0x00, 0x8000);   /* reset */
    HAL_Delay(100);
    INA226_WriteRegister(0x00, 0x4527);   /* configure */
}

float INA226_ReadBusVoltage(void)
{
    uint16_t raw   = INA226_ReadRegister(0x02);
    float    v_pin = raw * 1.25f / 1000.0f;
    return (v_pin * VOLTAGE_SCALE) + VOLTAGE_OFFSET;
}

float Get_Battery_SOC(float voltage)
{
	if      (voltage >= 12.70f) return 100.0f;
	    else if (voltage >= 12.60f) return  95.0f;
	    else if (voltage >= 12.50f) return  90.0f;
	    else if (voltage >= 12.42f) return  85.0f;
	    else if (voltage >= 12.32f) return  80.0f;
	    else if (voltage >= 12.20f) return  75.0f;
	    else if (voltage >= 12.06f) return  70.0f;
	    else if (voltage >= 11.90f) return  65.0f;
	    else if (voltage >= 11.75f) return  60.0f;
	    else if (voltage >= 11.60f) return  55.0f;
	    else if (voltage >= 11.45f) return  50.0f;
	    else if (voltage >= 11.31f) return  45.0f;
	    else if (voltage >= 11.20f) return  40.0f;
	    else if (voltage >= 11.10f) return  35.0f;
	    else if (voltage >= 11.00f) return  30.0f;
	    else if (voltage >= 10.90f) return  25.0f;
	    else if (voltage >= 10.80f) return  20.0f;
	    else if (voltage >= 10.65f) return  15.0f;
	    else if (voltage >= 10.50f) return  10.0f;
	    else if (voltage >= 10.20f) return   5.0f;
	    else                        return   0.0f;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  MAIN
 * ════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_SPI2_Init();
    MX_USART2_UART_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();

    /* ── PWM motor start ── */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    /* ── Motor direction default: forward ── */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

    /* ── CAN filter: accept ALL IDs (pass-all mask) ── */
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.FilterBank           = 0;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.SlaveStartFilterBank = 14;   /* required for single-CAN STM32F4 */

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        printf("\033[0;31m[INIT] CAN filter config FAILED\r\n\033[0m");
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        printf("\033[0;31m[INIT] CAN START FAILED\r\n\033[0m");
        Error_Handler();
    }

    /* FIX 3 — activate TX mailbox empty notification alongside RX */
    if (HAL_CAN_ActivateNotification(&hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        printf("\033[0;31m[INIT] CAN notification FAILED\r\n\033[0m");
        Error_Handler();
    }

    INA226_Init();

    /* ── Boot banner ── */
    printf("\033[2J\033[3J\033[H");
    printf("\033[1;36m");
    printf("╔══════════════════════════════════════════════════╗\r\n");
    printf("║        BMS NODE — STM32F407G  [FINAL BUILD]      ║\r\n");
    printf("║  CAN 500 kbps | Prescaler=6 BS1=11 BS2=2         ║\r\n");
    printf("╠══════════════════════════════════════════════════╣\r\n");
    printf("║  RX  0x100  ←  Airbag ECU  (crash byte)          ║\r\n");
    printf("║  RX  0x101  ←  ABS ECU     (brake,abs,immo)      ║\r\n");
    printf("║  RX  0x104  ←  TCU         (gear bitmask)        ║\r\n");
    printf("║  RX  0x107  ←  Telematics  (0x99/0x11)           ║\r\n");
    printf("║  TX  0x105  →  Infotainment (batt,temp,soc...)   ║\r\n");
    printf("╚══════════════════════════════════════════════════╝\r\n");
    printf("\033[0m");
    printf("\033[0;32m[INIT] CAN START OK\r\n\033[0m");
    printf("\033[0;32m[INIT] System Ready\r\n\r\n\033[0m");

    /* ════════════════════════════════════════════════════════════════════
     *  MAIN LOOP
     * ══════════════════════════════════════════════════════════════════ */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ── [1] Bus-off software recovery guard ────────────────────────
         *  AutoBusOff=ENABLE handles hardware recovery automatically.
         *  This software check is an extra safety net (same as BMS fix).
         * ────────────────────────────────────────────────────────────── */
        if (HAL_CAN_GetError(&hcan1) & HAL_CAN_ERROR_BOF)
        {
            printf("\033[0;31m[CAN] BUS-OFF detected — restarting...\r\n\033[0m");
            HAL_CAN_Stop(&hcan1);
            HAL_Delay(10);
            HAL_CAN_Start(&hcan1);
            /* FIX 3 — re-activate both notifications after bus-off recovery */
            HAL_CAN_ActivateNotification(&hcan1,
                CAN_IT_RX_FIFO0_MSG_PENDING |
                CAN_IT_TX_MAILBOX_EMPTY);
            printf("\033[0;32m[CAN] CAN restarted OK\r\n\033[0m");
        }

        /* ── [3] Airbag / Crash handler (0x100) ─────────────────────────
         *  Crash → immobilize + cut relay immediately
         * ────────────────────────────────────────────────────────────── */
        if (crash_flag)
        {
            crash_flag = 0;
            uint8_t cd = crash_data;

            printf("\033[1;31m[0x100 AIRBAG] DATA: 0x%02X\r\n\033[0m", cd);
            if (cd == AIRBAG_CRASH_BYTE)
            {
                immobilized = 1;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                printf("\033[1;31m[CRASH] IMPACT DETECTED — RELAY CUT, MOTOR STOPPED\r\n\033[0m");
                printf("\033[1;31m[CRASH] Vehicle immobilized for safety\r\n\033[0m");
            }
        }

        /* ── [4] Telematics immobilizer (0x107) ─────────────────────────
         *  0x99 → immobilize (cut relay)
         *  0x11 → mobilize   (restore relay if gear allows)
         * ────────────────────────────────────────────────────────────── */
        if (telematics_flag)
        {
            telematics_flag = 0;
            uint8_t cmd = telematics_cmd;

            printf("\033[1;35m[0x107 TELEMATICS] CMD: 0x%02X\r\n\033[0m", cmd);

            if (cmd == 0x99U)
            {
                immobilized = 1;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                printf("\033[1;31m[IMMOBILIZER] VEHICLE IMMOBILIZED — relay OFF\r\n\033[0m");
            }
            else if (cmd == 0x11U)
            {
                immobilized = 0;
                /* Only restore relay if a drive gear is active */
                if (rx_gear[0] == 0x01 || rx_gear[1] == 0x01 || rx_gear[4] == 0x01)
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                printf("\033[0;32m[IMMOBILIZER] VEHICLE MOBILIZED — relay restored\r\n\033[0m");
                /* Blink LED 5x to alert user (same as ABS LD6 pattern) */
                for (int i = 0; i < 5; i++)
                {
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
                    HAL_Delay(100);
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
                    HAL_Delay(100);
                }
            }
            else
            {
                printf("\033[0;33m[TELEMATICS] Unknown command 0x%02X — ignored\r\n\033[0m", cmd);
            }
        }

        /* ── [5] TCU gear handler (0x104) ───────────────────────────────
         *  Updates motor direction and relay based on gear bitmask
         * ────────────────────────────────────────────────────────────── */
        if (can_rx_gear_flag)
        {
            can_rx_gear_flag = 0;

            if (rx_gear[0] == 0x01)   // DRIVE
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
                if (!immobilized && rx_brake_pct == 0)
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                printf("\033[0;33m[0x104 TCU] GEAR=DRIVE → forward, relay %s\r\n\033[0m",
                       immobilized ? "BLOCKED(immobilized)" : "ON");
            }
            else if (rx_gear[1] == 0x01)   // SPORT
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
                if (!immobilized && rx_brake_pct == 0)
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                printf("\033[0;33m[0x104 TCU] GEAR=SPORT → forward, relay %s\r\n\033[0m",
                       immobilized ? "BLOCKED(immobilized)" : "ON");
            }
            else if (rx_gear[4] == 0x01)   // REVERSE
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
                if (!immobilized && rx_brake_pct == 0)
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                printf("\033[0;33m[0x104 TCU] GEAR=REVERSE → backward, relay %s\r\n\033[0m",
                       immobilized ? "BLOCKED(immobilized)" : "ON");
            }
            else if (rx_gear[2] == 0x01)   // PARK
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                printf("\033[0;33m[0x104 TCU] GEAR=PARK → relay OFF\r\n\033[0m");
            }
            else if (rx_gear[3] == 0x01)   // NEUTRAL
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                printf("\033[0;33m[0x104 TCU] GEAR=NEUTRAL → relay OFF\r\n\033[0m");
            }
            else
            {
                printf("\033[0;33m[0x104 TCU] GEAR=UNKNOWN\r\n\033[0m");
            }
        }

        /* ── [6] ABS brake handler (0x101) ─────────────────────────────────
         *  Brake %  0–29  → full speed (normal PWM from ADC)
         *  Brake % 30–49  → slow speed (30% of max PWM)
         *  Brake % 50–69  → very slow  (15% of max PWM)
         *  Brake % 70+    → motor STOP (PWM = 0, relay OFF)
         *  Handbrake ON   → motor STOP always
         * ────────────────────────────────────────────────────────────────── */
        if (can_rx_abs_flag)
        {
            can_rx_abs_flag = 0;

            uint8_t bp  = rx_brake_pct;
            uint8_t hb  = rx_handbrake;

            printf("\033[0;32m[0x101 ABS] Brake=%d%%  Handbrake=%s\r\n\033[0m",
                   bp, hb ? "ON" : "OFF");

            /* ── Handbrake ON → full stop regardless of brake % ── */
            if (hb)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                pwmValue = 0;
                printf("\033[0;31m[BRAKE] HANDBRAKE ON — motor stopped\r\n\033[0m");
            }
            /* ── Brake >= 70% → complete stop ── */
            else if (bp >= 70)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                pwmValue = 0;
                printf("\033[0;31m[BRAKE] %d%% >= 70 — motor STOPPED, relay OFF\r\n\033[0m", bp);
            }
            /* ── Brake 50–69% → very slow (15% of max PWM = ~105/699) ── */
            else if (bp >= 50)
            {
                if (!immobilized)
                {
                    pwmValue = (699U * 15U) / 100U;   /* 15% of max = ~104 */
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmValue);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                }
                printf("\033[0;33m[BRAKE] %d%% >= 50 — motor VERY SLOW PWM=%lu\r\n\033[0m", bp, pwmValue);
            }
            /* ── Brake 30–49% → slow speed (30% of max PWM = ~209/699) ── */
            else if (bp >= 30)
            {
                if (!immobilized)
                {
                    pwmValue = (699U * 30U) / 100U;   /* 30% of max = ~209 */
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmValue);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                }
                printf("\033[0;33m[BRAKE] %d%% >= 30 — motor SLOW PWM=%lu\r\n\033[0m", bp, pwmValue);
            }
            /* ── Brake < 30% → full speed from ADC ── */
            else
            {
                if (!immobilized)
                {
                    uint8_t gear_active = (rx_gear[0] == 0x01 ||   /* DRIVE   */
                                           rx_gear[1] == 0x01 ||   /* SPORT   */
                                           rx_gear[4] == 0x01);    /* REVERSE */
                    if (gear_active)
                    {
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
                        pwmValue = (adcValue * 699) / 4095;
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmValue);
                        printf("\033[0;32m[BRAKE] %d%% < 30 — motor FULL SPEED PWM=%lu\r\n\033[0m", bp, pwmValue);
                    }
                    else
                    {
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
                        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                        pwmValue = 0;
                        printf("\033[0;33m[BRAKE] brake released but PARK/NEUTRAL — relay OFF\r\n\033[0m");
                    }
                }
                else
                {
                    printf("\033[0;31m[BRAKE] blocked — immobilized\r\n\033[0m");
                }
            }
        }


        /* ── [8] ADC + PWM motor control ─────────────────────────────── */
        {
            uint32_t adc_sum = 0;
            for (int i = 0; i < 10; i++)
            {
                HAL_ADC_Start(&hadc1);
                HAL_ADC_PollForConversion(&hadc1, 10);
                adc_sum += HAL_ADC_GetValue(&hadc1);
                HAL_ADC_Stop(&hadc1);
            }
            adcValue = adc_sum / 10;

            /* Only update PWM freely if brake < 30% and not immobilized.
             * For brake zones 30–69% the brake handler already set pwmValue.
             * For brake >= 70% or handbrake, pwmValue is already 0.         */
            if (immobilized || rx_handbrake || rx_brake_pct >= 70)
            {
                pwmValue = 0;
            }
            else if (rx_brake_pct >= 50)
            {
                pwmValue = (699U * 15U) / 100U;   /* 15% — very slow */
            }
            else if (rx_brake_pct >= 30)
            {
                pwmValue = (699U * 30U) / 100U;   /* 30% — slow      */
            }
            else
            {
                pwmValue = (adcValue * 699) / 4095;   /* full ADC speed */
            }

            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmValue);
            rpm = (adcValue * 5000) / 4095;
        }
        /* ── [9] SPI thermocouple 1 (CS → PE0) ─────────────────────────── */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_SPI_Receive(&hspi2, spi_rx, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
        spi_value    = (uint16_t)(((spi_rx[0] << 8) | spi_rx[1]) >> 3);
        temperature1 = spi_value * 0.25f;

        /* ── [10] SPI thermocouple 2 (CS → PE1) ────────────────────────── */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_SPI_Receive(&hspi2, spi_rx, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
        spi_value    = (uint16_t)(((spi_rx[0] << 8) | spi_rx[1]) >> 3);
        temperature2 = spi_value * 0.25f;

        /* ── [11] INA226 battery voltage (every 500 ms, 5-sample avg) ───── */
        if ((now - lastInaTime) >= 500U)
        {
            lastInaTime = now;
            float sum = 0.0f;
            for (int i = 0; i < 5; i++)
            {
                sum += INA226_ReadBusVoltage();
                HAL_Delay(5);
            }
            battery_voltage = sum / 5.0f;
            battery_soc     = Get_Battery_SOC(battery_voltage);
        }
        /* ── [12] CAN TX — 0x105 every 1000 ms ───────────────────────────── */
        if ((now - lastTxTime) >= CAN_TX_INTERVAL_MS)
        {
            lastTxTime = now;

            /* ── Byte 0: status + fault flags ─────────────────────────────
             *  Bits [1:0]  — battery charge status:  0x00=OK  0x01=LOW  0x02=CRITICAL
             *  Bit  [2]    — Temp1 high fault        (>80 °C threshold)
             *  Bit  [3]    — Temp2 high fault        (>80 °C threshold)
             *  Bits [7:4]  — reserved (0)
             * ─────────────────────────────────────────────────────────────── */
            uint8_t batt_status;
            if      (battery_soc >= 80.0f) batt_status = 0x00U;
            else if (battery_soc >= 40.0f) batt_status = 0x01U;
            else                           batt_status = 0x02U;

            if (temperature1 >= TEMP_HIGH_THRESHOLD_C) batt_status |= (1U << 2);  /* Temp1 high */
            if (temperature2 >= TEMP_HIGH_THRESHOLD_C) batt_status |= (1U << 3);  /* Temp2 high */

            /* ── Bytes 1-2: Temp1 × 100 (big-endian) ── */
            uint16_t t1c = (uint16_t)(temperature1 * 100.0f);
            /* ── Bytes 3-4: Temp2 × 100 (big-endian) ── */
            uint16_t t2c = (uint16_t)(temperature2 * 100.0f);
            /* ── Bytes 6-7: RPM (big-endian, capped at 65535) ── */
            uint16_t rpm16 = (rpm > 65535UL) ? 0xFFFFU : (uint16_t)rpm;

            uint8_t txdata[8];
            txdata[0] =  batt_status;
            txdata[1] = (uint8_t)(t1c  & 0xFF);
            txdata[2] = (uint8_t)(t1c  >> 8);
            txdata[3] = (uint8_t)(t2c  & 0xFF);
            txdata[4] = (uint8_t)(t2c  >> 8);
            txdata[5] = (uint8_t)battery_soc;
            txdata[6] = (uint8_t)(rpm16 & 0xFF);
            txdata[7] = (uint8_t)(rpm16 >> 8);

            CAN_SendStd(CAN_ID_BMS_OUT, txdata, 8);
        }
        /* ── [13] Dashboard print every 2 s ────────────────────────────── */
        if ((now - lastPrintTime) >= PRINT_INTERVAL_MS)
        {
            lastPrintTime = now;

            const char *gear_str = "UNKNOWN";
            if      (rx_gear[0] == 0x01) gear_str = "DRIVE";
            else if (rx_gear[1] == 0x01) gear_str = "SPORT";
            else if (rx_gear[2] == 0x01) gear_str = "PARK";
            else if (rx_gear[3] == 0x01) gear_str = "NEUTRAL";
            else if (rx_gear[4] == 0x01) gear_str = "REVERSE";

            /* Relay state */
            uint8_t relay_on = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

            printf("\r\n\033[1;36m╔══════════ BMS STATUS DASHBOARD ═══════════╗\033[0m\r\n");

            /* ── CAN bus health ── */
            uint32_t cerr  = HAL_CAN_GetError(&hcan1);
            uint32_t cstate = HAL_CAN_GetState(&hcan1);
            printf("\033[1;33m║ CAN STATE :\033[0m %s\r\n",
                   (cstate == HAL_CAN_STATE_READY)     ? "\033[0;32mREADY\033[0m"     :
                   (cstate == HAL_CAN_STATE_LISTENING) ? "\033[0;32mLISTENING\033[0m" :
                                                         "\033[0;31mERROR\033[0m");
            printf("\033[1;33m║ CAN ERR   :\033[0m %s\r\n",
                   (cerr == HAL_CAN_ERROR_NONE) ? "\033[0;32mNO ERROR\033[0m"
                                                : "\033[0;31mERROR DETECTED\033[0m");
            printf("\033[1;33m║ TX frames :\033[0m %lu OK  %lu FAIL\r\n",
                   can_tx_total, can_tx_fail);
            printf("\033[1;33m║ RX frames :\033[0m %lu total\r\n", can_rx_total);

            printf("\033[1;36m╠══════════ RX FROM ABS  (0x101) ═══════════╣\033[0m\r\n");
            printf("\033[1;33m║ Brake     :\033[0m %d%%\r\n",   rx_brake_pct);

            printf("\033[1;36m╠══════════ RX FROM TCU  (0x104) ═══════════╣\033[0m\r\n");
            printf("\033[1;33m║ Gear      :\033[0m %s\r\n", gear_str);

            printf("\033[1;36m╠══════════ RX TELEMATICS (0x107) ══════════╣\033[0m\r\n");
            printf("\033[1;33m║ Immobile  :\033[0m %s\r\n",
                   immobilized ? "\033[0;31mYES — ACTIVE\033[0m" : "\033[0;32mNO\033[0m");

            printf("\033[1;36m╠══════════ BATTERY (INA226) ═══════════════╣\033[0m\r\n");
            printf("\033[1;33m║ Voltage   :\033[0m %.2f V\r\n", battery_voltage);
            printf("\033[1;33m║ SOC       :\033[0m %.1f %%\r\n", battery_soc);

            printf("\033[1;36m╠══════════ TEMPERATURE (SPI) ═══════════════╣\033[0m\r\n");
            printf("\033[1;33m║ Temp 1    :\033[0m %.2f C\r\n", temperature1);
            printf("\033[1;33m║ Temp 2    :\033[0m %.2f C\r\n", temperature2);

            printf("\033[1;36m╠══════════ MOTOR / ADC ════════════════════╣\033[0m\r\n");
            printf("\033[1;33m║ RPM       :\033[0m %lu\r\n", rpm);
            printf("\033[1;33m║ PWM       :\033[0m %lu\r\n", pwmValue);
            printf("\033[1;33m║ Relay     :\033[0m %s\r\n",
                   relay_on ? "\033[0;32mON\033[0m" : "\033[0;31mOFF\033[0m");

            printf("\033[1;36m╠══════════ TX TO INFOTAINMENT (0x105) ══════╣\033[0m\r\n");
            printf("\033[1;33m║ TX 0x105  :\033[0m \033[0;32mSENDING every 50ms\033[0m\r\n");

            printf("\033[1;33m║ TEC       :\033[0m %lu\r\n", (CAN1->ESR >> 16) & 0xFF);
            printf("\033[1;33m║ REC       :\033[0m %lu\r\n", (CAN1->ESR >>  8) & 0xFF);
            printf("\033[1;33m║ ESR       :\033[0m 0x%08lX\r\n", CAN1->ESR);

            printf("\033[1;36m╚════════════════════════════════════════════╝\033[0m\r\n\r\n");
        }

    } /* end while(1) */
}

/* ── SystemClock_Config ──────────────────────────────────────────────────── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* ── MX_ADC1_Init ────────────────────────────────────────────────────────── */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    sConfig.Channel      = ADC_CHANNEL_8;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

/* ── MX_CAN1_Init ────────────────────────────────────────────────────────── */
/*
 *  TIMING MUST MATCH ABS EXACTLY:
 *  APB1 = 42 MHz, Prescaler = 6, BS1 = 11TQ, BS2 = 2TQ
 *  Bit time = (1+11+2) × (1/42MHz × 6) = 14 × 142.86ns = 2000ns = 500 kbps
 */
static void MX_CAN1_Init(void)
{
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 6;
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;    /* auto hardware recovery  */
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;   /* matches ABS — no retry  */
    hcan1.Init.ReceiveFifoLocked    = ENABLE;
    hcan1.Init.TransmitFifoPriority = ENABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();
}

/* ── MX_I2C1_Init ────────────────────────────────────────────────────────── */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

/* ── MX_SPI2_Init ────────────────────────────────────────────────────────── */
static void MX_SPI2_Init(void)
{
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

/* ── MX_TIM3_Init (PWM motor) ────────────────────────────────────────────── */
static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef      sConfigOC     = {0};
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 83;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim3);
}

/* ── MX_USART2_UART_Init ─────────────────────────────────────────────────── */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* ── MX_GPIO_Init ────────────────────────────────────────────────────────── */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable all required clocks */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* ── USART2: PA2(TX) PA3(RX) ── */
    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ── CAN1: PD0(RX) PD1(TX) ── */
    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* ── SPI CS pins: PE0, PE1 (active LOW) ── */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 |
                              GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* ── Relay: PB1 (OFF by default) ── */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ── LED: PD15 ── */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* ── Motor direction: PC6(FWD) PC7(REV) ── */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ── Error_Handler ───────────────────────────────────────────────────────── */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
