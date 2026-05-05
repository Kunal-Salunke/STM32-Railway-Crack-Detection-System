/* USER CODE BEGIN Header */
/**
 * ============================================================
 *  RailwayTrackCrackDetection System — STM32 Nucleo F446RE
 *  Pure direct-register (NO HAL, NO CMSIS headers).
 *  Compiles with ONLY  -I../Inc  (the default project setting).
 * ============================================================
 *
 *  L298N Motor Driver:
 *    ENA = PA8   ENB = PA9
 *    IN1 = PA10  IN2 = PB3   IN3 = PB5   IN4 = PB4
 *
 *  HC-SR04 Ultrasonic:
 *    TRIG = PA0   ECHO = PA1
 *    Timer: TIM2  (1 tick = 1 µs)
 *
 *  IR Obstacle Sensor (active LOW):
 *    OUT  = PA4
 *
 *  ESP32 Interface (USART3) — PB10=TX, PC11=RX:
 *    TX = PB10  (AF7)  → ESP32 GPIO16 (RX2)
 *    RX = PC11  (AF7)  → ESP32 GPIO17 (TX2)
 *    GND → GND  (shared ground, mandatory)
 *
 *  TeraTerm debug (USART2 via ST-Link):
 *    TX = PA2  RX = PA3   @ 115200 baud
 *
 *  Onboard LED LD2 = PA5
 * ============================================================
 */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* ================================================================
 *  CLOCK SPEED CONSTANTS  (after SystemClock_Config)
 *  HSI → PLL → SYSCLK = 84 MHz
 *  APB1 = 42 MHz (÷2)   APB2 = 84 MHz (÷1)
 * ================================================================ */
#define SYSCLK_HZ   84000000UL

/* ================================================================
 *  PIN BIT MASKS
 * ================================================================ */
#define ENA_BIT   (1UL <<  8)   /* PA8  - Motor A enable          */
#define ENB_BIT   (1UL <<  9)   /* PA9  - Motor B enable          */
#define IN1_BIT   (1UL << 10)   /* PA10 - Motor A dir 1           */
#define IN2_BIT   (1UL <<  3)   /* PB3  - Motor A dir 2           */
#define IN3_BIT   (1UL <<  5)   /* PB5  - Motor B dir 1           */
#define IN4_BIT   (1UL <<  4)   /* PB4  - Motor B dir 2           */
#define TRIG_BIT  (1UL <<  0)   /* PA0  - Ultrasonic trigger      */
#define ECHO_BIT  (1UL <<  1)   /* PA1  - Ultrasonic echo         */
#define IR_BIT    (1UL <<  4)   /* PA4  - IR sensor output        */
#define LED_BIT   (1UL <<  5)   /* PA5  - Onboard LED LD2         */

/* Soft-PWM half-period for Motor-A speed reduction */
#define PWM_HALF_PERIOD_MS   5U

/* ================================================================
 *  SYSTICK MILLISECOND COUNTER
 * ================================================================ */
static volatile uint32_t g_tick_ms = 0;

/* Called every 1 ms by the SysTick interrupt */
void SysTick_Handler(void)
{
    g_tick_ms++;
}

static uint32_t get_tick(void)
{
    return g_tick_ms;
}

static void delay_ms(uint32_t ms)
{
    uint32_t t = get_tick();
    while ((get_tick() - t) < ms) { /* busy-wait */ }
}

/* ================================================================
 *  TIM2 MICROSECOND DELAY
 * ================================================================ */
static void delay_us(uint32_t us)
{
    TIM2->CNT = 0;
    while (TIM2->CNT < us) { /* busy-wait */ }
}

/* ================================================================
 *  UART HELPERS  (polling, no interrupts)
 * ================================================================ */
static void usart_putchar(USART_TypeDef *U, char c)
{
    while (!(U->SR & USART_SR_TXE)) { /* wait for TX register empty */ }
    U->DR = (uint32_t)(uint8_t)c;
}

static void usart_puts(USART_TypeDef *U, const char *s)
{
    while (*s)
        usart_putchar(U, *s++);
}

/* Print to both TeraTerm (USART2) and ESP32 (USART3) */
static void uart_print(const char *msg)
{
    usart_puts(USART2, msg);
    usart_puts(USART3, msg);
}

/* ================================================================
 *  MOTOR CONTROL
 * ================================================================ */
static uint32_t s_pwm_toggle_ms = 0;

static void motor_stop(void)
{
    GPIOA->BSRR = ((ENA_BIT | ENB_BIT | IN1_BIT) << 16); /* LOW */
    GPIOB->BSRR = ((IN2_BIT | IN3_BIT | IN4_BIT) << 16); /* LOW */
}

static void motor_enable(void)
{
    GPIOA->BSRR = ENA_BIT | ENB_BIT;
}

static void motor_forward(void)
{
    /* Direction: IN1=H IN2=L   IN3=H IN4=L */
    GPIOA->BSRR = IN1_BIT;                /* IN1 HIGH */
    GPIOB->BSRR = (IN2_BIT << 16);        /* IN2 LOW  */
    GPIOB->BSRR = IN3_BIT;                /* IN3 HIGH */
    GPIOB->BSRR = (IN4_BIT << 16);        /* IN4 LOW  */

    /* Motor B — full speed */
    GPIOA->BSRR = ENB_BIT;

    /* Motor A — ~50% duty via software PWM on ENA */
    uint32_t now = get_tick();
    if ((now - s_pwm_toggle_ms) >= PWM_HALF_PERIOD_MS)
    {
        GPIOA->ODR    ^= ENA_BIT;
        s_pwm_toggle_ms = now;
    }
}

static void motor_backward(void)
{
    motor_enable();
    GPIOA->BSRR = (IN1_BIT << 16);        /* IN1 LOW  */
    GPIOB->BSRR = IN2_BIT;                /* IN2 HIGH */
    GPIOB->BSRR = (IN3_BIT << 16);        /* IN3 LOW  */
    GPIOB->BSRR = IN4_BIT;                /* IN4 HIGH */
}

/* ================================================================
 *  ULTRASONIC  HC-SR04
 *  Returns pulse width in µs; 0 = timeout / no response
 * ================================================================ */
static uint32_t ultrasonic_raw_us(void)
{
    /* Stabilise TRIG LOW for 5 µs */
    GPIOA->BSRR = (TRIG_BIT << 16);
    delay_us(5);

    /* 10 µs trigger pulse */
    GPIOA->BSRR = TRIG_BIT;
    delay_us(10);
    GPIOA->BSRR = (TRIG_BIT << 16);

    /* Wait for ECHO HIGH (timeout 38 ms) */
    TIM2->CNT = 0;
    while (!(GPIOA->IDR & ECHO_BIT))
    {
        if (TIM2->CNT > 38000UL) return 0UL;
    }

    /* Measure echo pulse width */
    TIM2->CNT = 0;
    while (GPIOA->IDR & ECHO_BIT)
    {
        if (TIM2->CNT > 38000UL) return 0UL;
    }

    return TIM2->CNT;
}

/* Returns distance in cm; 999 = out of range; 0 = < 2 cm blind zone */
static uint32_t ultrasonic_cm(void)
{
    uint32_t total = 0;
    uint8_t  count = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        uint32_t p = ultrasonic_raw_us();
        if (p > 0 && p <= 23200UL)
        {
            total += p;
            count++;
        }
        delay_ms(15);
    }

    if (count == 0)      return 999UL;
    uint32_t avg = total / count;
    if (avg  < 116UL)    return 0UL;   /* blind zone */
    return avg / 58UL;
}

/* ================================================================
 *  MAIN SENSING LOOP
 * ================================================================ */
static void run_and_measure(uint32_t duration_ms)
{
    uint32_t start = get_tick();
    char buf[48];

    while ((get_tick() - start) < duration_ms)
    {
        /* Ultrasonic */
        uint32_t dist = ultrasonic_cm();
        if (dist == 0UL)
        {
            uart_print("  Distance: < 2 cm\r\n");
            uart_print("  *** OBSTACLE DETECTED ***\r\n");
        }
        else if (dist != 999UL)
        {
            snprintf(buf, sizeof(buf), "  Distance: %lu cm\r\n", dist);
            uart_print(buf);
            if (dist < 20UL)
                uart_print("  *** OBSTACLE DETECTED ***\r\n");
        }

        /* IR sensor — active LOW: bit=0 → object detected */
        uint8_t ir = (GPIOA->IDR & IR_BIT) ? 0U : 1U;
        snprintf(buf, sizeof(buf), "  IR: %u\r\n", ir);
        uart_print(buf);

        if (ir == 1U)
        {
            motor_forward();
            GPIOA->BSRR = LED_BIT;               /* LED ON  */
        }
        else
        {
            motor_stop();
            GPIOA->BSRR = (LED_BIT << 16);       /* LED OFF */
            uart_print("  *** WARNING!! CRACK DETECTED ***\r\n");
        }

        delay_ms(20);
    }
}

/* ================================================================
 *  SYSTEM CLOCK CONFIG — HSI → PLL → 84 MHz
 * ================================================================ */
static void SystemClock_Config(void)
{
    /* 1. Enable PWR and set voltage scaling to Scale 3 */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR       = (PWR->CR & ~PWR_CR_VOS) | (2UL << PWR_CR_VOS_Pos);

    /* 2. Turn on HSI and wait */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { /* wait */ }

    /* 3. Configure PLL:
     *    Source = HSI (16 MHz)
     *    /M=16  *N=336  /P=4  /Q=2
     *    PLLCLK = 16/16 * 336 / 4 = 84 MHz              */
    RCC->PLLCFGR = (16UL  << RCC_PLLCFGR_PLLM_Pos)   /* M */
                 | (336UL << RCC_PLLCFGR_PLLN_Pos)    /* N */
                 | (1UL   << RCC_PLLCFGR_PLLP_Pos)    /* P: 01 → ÷4 */
                 | (2UL   << RCC_PLLCFGR_PLLQ_Pos);   /* Q (USB etc.) */
    /* PLLSRC bit22 = 0 → HSI (already cleared above) */

    /* 4. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { /* wait */ }

    /* 5. Flash latency for 84 MHz (2 WS) + prefetch + caches */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY)
               | FLASH_ACR_LATENCY_2WS
               | FLASH_ACR_PRFTEN
               | FLASH_ACR_ICEN
               | FLASH_ACR_DCEN;

    /* 6. AHB ÷1, APB1 ÷2 (42 MHz), APB2 ÷1 (84 MHz) */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    /* 7. Switch SYSCLK to PLL */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { /* wait */ }

    /* 8. SysTick every 1 ms */
    SysTick_Config(SYSCLK_HZ / 1000UL);
}

/* ================================================================
 *  TIM2 — 1 MHz microsecond counter
 *  APB1 timer clock = 84 MHz (×2 multiplier when APB1 prescaler ≠ 1)
 *  PSC = 83  → 84 MHz / 84 = 1 MHz → 1 tick = 1 µs
 * ================================================================ */
static void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1  = 0;
    TIM2->PSC  = 83;
    TIM2->ARR  = 0xFFFFFFFFUL;
    TIM2->EGR  = TIM_EGR_UG;        /* force update — loads PSC shadow */
    TIM2->SR  &= ~TIM_SR_UIF;
    TIM2->CNT  = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ================================================================
 *  USART2 — 115200 8N1 (ST-Link virtual COM / TeraTerm)
 *  TX = PA2  RX = PA3  (AF7)
 *  APB1 clock = 42 MHz
 *  BRR mantissa = floor(42 000 000 / (16 * 115200)) = 22
 *  BRR fraction = round(0.8125 * 16) = 13
 * ================================================================ */
static void USART2_Init(void)
{
    /* PA2 & PA3 → AF7 */
    GPIOA->MODER   &= ~((3UL << (2*2)) | (3UL << (3*2)));
    GPIOA->MODER   |=  ((2UL << (2*2)) | (2UL << (3*2)));  /* AF */
    GPIOA->OSPEEDR |=  ((3UL << (2*2)) | (3UL << (3*2)));  /* Very High */
    GPIOA->AFR[0]  &= ~((0xFUL << (2*4)) | (0xFUL << (3*4)));
    GPIOA->AFR[0]  |=  ((7UL   << (2*4)) | (7UL   << (3*4))); /* AF7 */

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->BRR = (22UL << 4) | 13UL;   /* 115200 @ 42 MHz */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* ================================================================
 *  USART3 — 9600 baud (ESP32)  PB10=TX (AF7)  PC11=RX (AF7)
 *  APB1 clock = 42 MHz
 *  BRR: mantissa = 273, fraction = 7  (9600 @ 42 MHz)
 *  TX: PB10 → GPIOB->AFR[1], index = 10-8 = 2
 *  RX: PC11 → GPIOC->AFR[1], index = 11-8 = 3
 *  (GPIOC clock already enabled in GPIO_Init)
 * ================================================================ */
static void USART3_Init(void)
{
    /* PB10 → TX, Alternate Function mode, AF7 */
    GPIOB->MODER   &= ~(3UL << (10*2));
    GPIOB->MODER   |=  (2UL << (10*2));          /* AF */
    GPIOB->OSPEEDR |=  (3UL << (10*2));          /* Very high speed */
    GPIOB->AFR[1]  &= ~(0xFUL << ((10-8)*4));
    GPIOB->AFR[1]  |=  (7UL   << ((10-8)*4));    /* AF7 */

    /* PC11 → RX, Alternate Function mode, AF7 */
    GPIOC->MODER   &= ~(3UL << (11*2));
    GPIOC->MODER   |=  (2UL << (11*2));          /* AF */
    GPIOC->PUPDR   &= ~(3UL << (11*2));
    GPIOC->PUPDR   |=  (1UL << (11*2));          /* Pull-up on RX */
    GPIOC->AFR[1]  &= ~(0xFUL << ((11-8)*4));
    GPIOC->AFR[1]  |=  (7UL   << ((11-8)*4));    /* AF7 */

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    USART3->BRR = (273UL << 4) | 7UL;            /* 9600 @ 42 MHz */
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* ================================================================
 *  GPIO INIT — motor / sensor / LED pins
 * ================================================================ */
static void GPIO_Init(void)
{
    /* Enable GPIO clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN
                  | RCC_AHB1ENR_GPIOHEN;
    (void)RCC->AHB1ENR;   /* read-back to flush write */

    /* Pre-set all outputs LOW */
    GPIOA->BSRR = ((ENA_BIT | ENB_BIT | IN1_BIT | TRIG_BIT | LED_BIT) << 16);
    GPIOB->BSRR = ((IN2_BIT | IN3_BIT | IN4_BIT) << 16);

    /* --- GPIOA output pins: 0 (TRIG), 5 (LED), 8 (ENA), 9 (ENB), 10 (IN1) --- */
    const uint32_t pa_out[] = {0, 5, 8, 9, 10};
    for (uint32_t i = 0; i < 5U; i++)
    {
        uint32_t p = pa_out[i];
        GPIOA->MODER   &= ~(3UL << (p * 2));
        GPIOA->MODER   |=  (1UL << (p * 2));   /* output */
        GPIOA->OTYPER  &= ~(1UL << p);          /* push-pull */
        GPIOA->OSPEEDR |=  (3UL << (p * 2));   /* very high speed */
        GPIOA->PUPDR   &= ~(3UL << (p * 2));   /* no pull */
    }

    /* PA1 (ECHO) — input, pull-down */
    GPIOA->MODER  &= ~(3UL << (1 * 2));   /* input */
    GPIOA->PUPDR  &= ~(3UL << (1 * 2));
    GPIOA->PUPDR  |=  (2UL << (1 * 2));   /* pull-down */

    /* PA4 (IR) — input, pull-up (sensor is active LOW) */
    GPIOA->MODER  &= ~(3UL << (4 * 2));   /* input */
    GPIOA->PUPDR  &= ~(3UL << (4 * 2));
    GPIOA->PUPDR  |=  (1UL << (4 * 2));   /* pull-up */

    /* --- GPIOB output pins: 3 (IN2), 4 (IN4), 5 (IN3) --- */
    const uint32_t pb_out[] = {3, 4, 5};
    for (uint32_t i = 0; i < 3U; i++)
    {
        uint32_t p = pb_out[i];
        GPIOB->MODER   &= ~(3UL << (p * 2));
        GPIOB->MODER   |=  (1UL << (p * 2));
        GPIOB->OTYPER  &= ~(1UL << p);
        GPIOB->OSPEEDR |=  (3UL << (p * 2));
        GPIOB->PUPDR   &= ~(3UL << (p * 2));
    }
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    SystemClock_Config();   /* 84 MHz SYSCLK, SysTick 1 ms */
    GPIO_Init();
    TIM2_Init();            /* 1 µs microsecond counter     */
    USART2_Init();          /* TeraTerm  115200 baud        */
    USART3_Init();          /* ESP32      9600 baud         */

    motor_stop();

    uart_print("\r\n");
    uart_print("==============================================\r\n");
    uart_print("  DEFENCE SYSTEM — STM32F446RE Nucleo        \r\n");
    uart_print("  ENA=PA8  ENB=PA9              	             \r\n");
    uart_print("  IN1=PA10 IN2=PB3 IN3=PB5 IN4=PB4          \r\n");
    uart_print("  TRIG=PA0  ECHO=PA1  IR=PA4                 \r\n");
    uart_print("  ESP32 Dashboard → USART3 PB10(TX)/PB11(RX)  \r\n");
    uart_print("==============================================\r\n");
    uart_print("  Starting in 3 s...\r\n\r\n");

    GPIOA->BSRR = LED_BIT;          /* LED ON  during count-down */
    delay_ms(3000);
    GPIOA->BSRR = (LED_BIT << 16);  /* LED OFF */

    while (1)
    {
        run_and_measure(1000);
    }
}

/* ================================================================
 *  ERROR HANDLER — LED solid ON, hang
 * ================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    GPIOA->BSRR = LED_BIT;
    while (1) { /* hang */ }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
