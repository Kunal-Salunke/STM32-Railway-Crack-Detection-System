/* USER CODE BEGIN Header */
/**
 * ============================================================
 *  MOTOR + ULTRASONIC TEST — STM32 Nucleo F446RE
 * ============================================================
 *  Motor (L298N):
 *    ENA=PA8  ENB=PA9
 *    IN1=PA10 IN2=PB3  IN3=PB5  IN4=PB4
 *
 *  SPEED CONTROL (Software PWM via GPIO toggle):
 *    Motor A (ENA/PA8) — SLOWED DOWN (~50% duty cycle)
 *    Motor B (ENB/PA9) — FULL SPEED
 *    If rover still curves same way, swap which EN pin is toggled.
 *    See comments in motor_forward() below.
 *
 *  Ultrasonic HC-SR04:
 *    TRIG = PA0 (A0)   — GPIO Output
 *    ECHO = PA1 (A1)   — GPIO Input
 *    Timer: TIM2 used for microsecond counting (direct registers)
 * ============================================================
 */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>

/* ================================================================
 *  PIN DEFINITIONS
 * ================================================================ */

/* Motor direction pins (L298N) — DO NOT CHANGE */
#define IN1_PORT   GPIOA
#define IN1_PIN    GPIO_PIN_10

#define IN2_PORT   GPIOB
#define IN2_PIN    GPIO_PIN_3

#define IN3_PORT   GPIOB
#define IN3_PIN    GPIO_PIN_5

#define IN4_PORT   GPIOB
#define IN4_PIN    GPIO_PIN_4

/* Motor enable pins — DO NOT CHANGE */
#define ENA_PORT   GPIOA
#define ENA_PIN    GPIO_PIN_8

#define ENB_PORT   GPIOA
#define ENB_PIN    GPIO_PIN_9

/* Ultrasonic HC-SR04 pins */
#define TRIG_PORT  GPIOA
#define TRIG_PIN   GPIO_PIN_0   /* A0 */

#define ECHO_PORT  GPIOA
#define ECHO_PIN   GPIO_PIN_1   /* A1 */

/* IR Obstacle Sensor — digital output, active LOW */
#define IR_PORT    GPIOA
#define IR_PIN     GPIO_PIN_4   /* A2 (PA4) */

/* Timeout: ~30ms = 30000 us (max range ~500cm) */
#define ECHO_TIMEOUT_US  30000UL

/* ================================================================
 *  HANDLES
 * ================================================================ */
UART_HandleTypeDef huart2;   /* TeraTerm via ST-Link    */
UART_HandleTypeDef huart1;   /* HC-05 Bluetooth (PB6)   */

/* ================================================================
 *  PROTOTYPES
 * ================================================================ */
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_BT_Init(void);   /* HC-05 Bluetooth UART */
/* Motor forward declarations — defined later but called in run_and_measure */
static void motor_enable(void);
static void motor_forward(void);
static void motor_stop(void);

/* ================================================================
 *  TIM2 MICROSECOND FUNCTIONS
 *  TIM2 clock = 84MHz, PSC=83 -> 1MHz -> 1 tick = 1us
 * ================================================================ */
static void delay_us(uint32_t us)
{
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

/* ================================================================
 *  ULTRASONIC — Read raw echo pulse in microseconds
 *  Returns 0 on timeout or invalid signal
 * ================================================================ */
static uint32_t ultrasonic_read_raw_us(void)
{
    /* Ensure TRIG starts LOW — stabilise for 5us */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    delay_us(5);

    /* Send 10µs trigger pulse */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    /* Wait for ECHO to go HIGH — HC-SR04 takes up to 1ms to respond */
    TIM2->CNT = 0;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
    {
        if (TIM2->CNT > 38000UL) return 0UL;  /* no response timeout */
    }

    /* Measure how long ECHO stays HIGH */
    TIM2->CNT = 0;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        if (TIM2->CNT > 38000UL) return 0UL;  /* echo too long timeout */
    }

    return TIM2->CNT;
}

/* ================================================================
 *  ULTRASONIC — Read distance in cm with filtering
 *  Returns 999 if no valid reading
 * ================================================================ */
/* ================================================================
 *  UART HELPER — sends to TeraTerm AND Bluetooth simultaneously
 * ================================================================ */
static void uart_print(const char *msg)
{
    uint16_t len = (uint16_t)strlen(msg);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, 200);  /* TeraTerm  */
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 200);  /* Bluetooth */
}

/* ================================================================
 *  ULTRASONIC — Reliable reading: average of 3 valid samples
 *
 *  HC-SR04 blind zone is ~0-2cm (pulse < 116us).
 *  Objects in blind zone are reported as "< 2 cm".
 *  No object / out of range returns 999.
 * ================================================================ */
static uint32_t ultrasonic_reliable_cm(void)
{
    uint32_t total = 0;
    uint8_t  count = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        uint32_t p = ultrasonic_read_raw_us();

        /* Accept pulses 58us–23200us (valid sensor output range) */
        if (p > 0 && p <= 23200UL)
        {
            total += p;
            count++;
        }
        HAL_Delay(15);   /* 15ms gap between sub-readings */
    }

    if (count == 0) return 999UL;   /* no valid echo — nothing in range */

    uint32_t avg_us = total / count;

    /* Blind zone: avg < 116us means object is closer than ~2cm */
    if (avg_us < 116UL) return 0UL; /* 0 = "< 2cm" sentinel */

    return avg_us / 58UL;
}

/* ================================================================
 *  Run motors for duration_ms, printing distance every ~100ms
 * ================================================================ */
static void run_and_measure(uint32_t duration_ms)
{
    uint32_t start = HAL_GetTick();
    char buf[48];

    while ((HAL_GetTick() - start) < duration_ms)
    {
        uint32_t dist = ultrasonic_reliable_cm();

        if (dist == 0UL)
        {
            uart_print("  Distance: < 2 cm\r\n");   /* blind zone */
            uart_print("  *** OBSTACLE DETECTED ***\r\n");
        }
        else if (dist != 999UL)
        {
            snprintf(buf, sizeof(buf), "  Distance: %lu cm\r\n", dist);
            uart_print(buf);

            /* Warn if object is closer than 20 cm */
            if (dist < 20UL)
                uart_print("  *** OBSTACLE DETECTED ***\r\n");
        }
        /* dist == 999: no object, print nothing */

        /* IR Sensor — active LOW: reads 0 when object detected */
        /* 1 = object within threshold (~20cm) → motors RUN    */
        /* 0 = nothing detected / beyond 20cm  → motors STOP   */
        uint8_t ir = (HAL_GPIO_ReadPin(IR_PORT, IR_PIN) == GPIO_PIN_RESET) ? 1 : 0;
        snprintf(buf, sizeof(buf), "  IR: %u\r\n", ir);
        uart_print(buf);

        /* Motor control based on IR value */
        if (ir == 1)
        {
            motor_forward();
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }
        else
        {
            motor_stop();
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            uart_print("  *** WARNING!! CRACK DETECTED ***\r\n");
        }

        HAL_Delay(20);   /* total ~100ms per reading (3x15ms + 20ms + sensor time) */
    }
}



/* ================================================================
 *  MOTOR CONTROL — Motor B full speed, Motor A software-PWM @ 50%
 *
 *  Software PWM: ENA is toggled every PWM_HALF_PERIOD_MS milliseconds.
 *  This gives Motor A ~50% duty cycle, effectively halving its speed.
 *
 *  HOW TO SWAP if the wrong motor is being slowed:
 *    In motor_forward() below, swap ENA_PIN <-> ENB_PIN in the toggle.
 *    i.e. change  HAL_GPIO_TogglePin(ENA_PORT, ENA_PIN)  to
 *                 HAL_GPIO_TogglePin(ENB_PORT, ENB_PIN)
 *    and change   HAL_GPIO_WritePin(ENB_PORT, ENB_PIN, GPIO_PIN_SET)  to
 *                 HAL_GPIO_WritePin(ENA_PORT, ENA_PIN, GPIO_PIN_SET)
 * ================================================================ */

#define PWM_HALF_PERIOD_MS   5U   /* toggle every 5ms = 10ms period = 100Hz */

static uint32_t _pwm_last_toggle = 0;

static void motor_enable(void)
{
    /* Called once at startup / from motor_stop recovery — both full on */
    HAL_GPIO_WritePin(ENA_PORT, ENA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENB_PORT, ENB_PIN, GPIO_PIN_SET);
}

static void motor_forward(void)
{
    /* Direction pins — set both motors forward */
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);

    /* Motor B — FULL SPEED (ENB stays HIGH) */
    HAL_GPIO_WritePin(ENB_PORT, ENB_PIN, GPIO_PIN_SET);

    /* Motor A — SLOWED via software PWM toggle on ENA */
    /* Toggle ENA every PWM_HALF_PERIOD_MS milliseconds (~50% duty) */
    uint32_t now = HAL_GetTick();
    if ((now - _pwm_last_toggle) >= PWM_HALF_PERIOD_MS)
    {
        HAL_GPIO_TogglePin(ENA_PORT, ENA_PIN);
        _pwm_last_toggle = now;
    }
}

static void motor_backward(void)
{
    motor_enable();
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
}

static void motor_stop(void)
{
    HAL_GPIO_WritePin(ENA_PORT, ENA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENB_PORT, ENB_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();          /* microsecond timer for ultrasonic */
    MX_USART2_UART_Init();   /* TeraTerm via ST-Link, 115200     */
    MX_USART1_BT_Init();     /* HC-05 Bluetooth, 9600 baud       */

    motor_stop();

    uart_print("\r\n");
    uart_print("==============================================\r\n");
    uart_print("  MOTOR + ULTRASONIC TEST                   \r\n");
    uart_print("  Motors : IN1=PA10 IN2=PB3 IN3=PB5 IN4=PB4\r\n");
    uart_print("           ENA=PA8  ENB=PA9 (GPIO HIGH)     \r\n");
    uart_print("  Sensor : TRIG=PA0(A0)  ECHO=PA1(A1)      \r\n");
    uart_print("  UART   : 115200 baud (TeraTerm)           \r\n");
    uart_print("==============================================\r\n");
    uart_print("  Starting in 3s...\r\n\r\n");

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);


    while (1)
    {
        /* Motor controlled by IR inside run_and_measure */
        run_and_measure(1000);
    }
}

/* ================================================================
 *  System Clock — 84 MHz from HSI via PLL
 * ================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    o.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    o.HSIState            = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState        = RCC_PLL_ON;
    o.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    o.PLL.PLLM            = 16;
    o.PLL.PLLN            = 336;
    o.PLL.PLLP            = RCC_PLLP_DIV4;
    o.PLL.PLLQ            = 2;
    o.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/* ================================================================
 *  TIM2 — microsecond counter (direct register, no HAL callbacks)
 *  84 MHz / (PSC+1=84) = 1 MHz  ->  1 tick = 1 us
 *  ARR = 0xFFFFFFFF (TIM2 is 32-bit)
 * ================================================================ */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1  = 0;             /* disable timer before config     */
    TIM2->PSC  = 83;            /* 84MHz / 84 = 1MHz -> 1us/tick   */
    TIM2->ARR  = 0xFFFFFFFF;    /* max period (TIM2 is 32-bit)     */
    TIM2->EGR  = TIM_EGR_UG;   /* FORCE update: loads PSC into    */
                                /*   shadow register immediately   */
                                /*   WITHOUT this PSC is ignored!  */
    TIM2->SR  &= ~TIM_SR_UIF;  /* clear update interrupt flag     */
    TIM2->CNT  = 0;
    TIM2->CR1 |= TIM_CR1_CEN;  /* start counter                   */
}

/* ================================================================
 *  USART2 — 115200 8N1 (ST-Link virtual COM / TeraTerm)
 * ================================================================ */
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

/* ================================================================
 *  USART1 — HC-05 Bluetooth, 9600 baud
 *  TX = PB6 (Arduino D10)  →  HC-05 RXD
 *  RX = PB7 (CN10 morpho)  ←  HC-05 TXD  (optional, receive only)
 *
 *  Clock and GPIO enabled HERE before HAL_UART_Init so the
 *  HAL_UART_MspInit callback in hal_msp.c (which only handles
 *  USART2) does not need to be modified.
 * ================================================================ */
static void MX_USART1_BT_Init(void)
{
    GPIO_InitTypeDef g = {0};

    /* Enable clocks */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();   /* already on, safe to call again */

    /* PB6 = USART1_TX (AF7) — connects to HC-05 RXD */
    g.Pin       = GPIO_PIN_6;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB7 = USART1_RX (AF7) — connects to HC-05 TXD (optional) */
    g.Pin       = GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &g);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;   /* HC-05 default data baud */
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* ================================================================
 *  GPIO Init — all motor + ultrasonic + LED pins
 * ================================================================ */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Pre-set all outputs LOW */
    HAL_GPIO_WritePin(ENA_PORT,      ENA_PIN,              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENB_PORT,      ENB_PIN,              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_PORT,      IN1_PIN,              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT,      IN2_PIN,              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT,      IN3_PIN,              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT,      IN4_PIN,              GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TRIG_PORT,     TRIG_PIN,             GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,              GPIO_PIN_RESET);

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    /* LED LD2 (PA5) */
    g.Pin = LD2_Pin;
    HAL_GPIO_Init(LD2_GPIO_Port, &g);

    /* ENA (PA8) + ENB (PA9) + IN1 (PA10) + TRIG (PA0) — all GPIOA outputs */
    g.Pin = ENA_PIN | ENB_PIN | IN1_PIN | TRIG_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    /* IN2 (PB3) + IN3 (PB5) + IN4 (PB4) — all GPIOB outputs */
    g.Pin = IN2_PIN | IN3_PIN | IN4_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    /* ECHO (PA1) — input with pull-down (fast for accurate timing) */
    g.Pin   = ECHO_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ECHO_PORT, &g);

    /* IR sensor OUT (PA4 / A2) — input with pull-up (active LOW module) */
    g.Pin   = IR_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IR_PORT, &g);
}

/* ================================================================
 *  Error Handler — LED ON, halt
 * ================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif