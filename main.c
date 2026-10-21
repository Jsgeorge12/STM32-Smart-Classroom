/*
 * SMART CLASSROOM AUTOMATION - APPENDIX SOURCE
 * Target: STM32F401RCT6, HSI 16 MHz; direct register access, no HAL.
 *
 * GPIO/header allocation (not physical package lead numbers):
 * PA1 outside IR; PA0 inside IR; PB0 light 1; PA4 light 2.
 * PA5 LDR analog output; PA8 DHT11 data; PC13 occupancy indicator.
 * PB6 I2C1 SCL and PB7 I2C1 SDA: OLED and DS3231 share this bus.
 * PA9 USART1 TX; PA10 USART1 RX; PA13/PA14 remain reserved for SWD.
 * PB8 TIM4_CH3 motor enable; PB14/PB15 motor direction outputs.
 * PB12 light ON button; PB13 light OFF button; each switches to GND.
 *
 * Execution model:
 * Main performs sensor transactions, RTC polling, UART parsing and display work.
 * SysTick runs every 1 ms: occupancy, button debounce and output decisions.
 * TIM3 interrupts every 100 us: ten slots form 1 kHz software LED PWM.
 * TIM2 provides 1 us timestamps for DHT11 EXTI8 edge decoding.
 * TIM4 produces 1 kHz motor PWM in hardware without a timer interrupt.
 * Lower interrupt priority numbers have higher urgency:
 * DHT EXTI 0x40, SysTick 0x80, buttons 0xA0, TIM3 0xC0, USART1 0xE0.
 *
 * Automatic lighting: valid RTC within 09:00 <= time < 17:00, occupied,
 * and valid ambient-light data. Manual button overrides bypass these gates.
 * Motor starts OFF; FAN AUTO enables temperature-based automatic operation.
 * PWM duty is a command, not a measurement of motor RPM.
 * Occupancy assumes one person passes through the sensors at a time.
 *
 * This edition changes comments and formatting only. Keep the source and
 * its numbered listing together: editing this file changes line references.
 */

/* LDR DIMMING VERSION: LED 1 moved to PB0. PA5 analog; PB0/PA4 software PWM.
 * TIM3 reserved for PWM; TIM2 remains reserved for DHT11.
 * RTC scheduling and USART1 serial monitoring/configuration included.
 * L298N motor: PB8 ENA (TIM4 CH3), PB14 IN1, PB15 IN2.
 * Motor boots OFF. PB12/PB13 EXTI buttons force lights ON/OFF; LIGHT AUTO restores automation.
 */
/* Complete replacement Src/main.c -- STM32F401RCT6, HSI 16 MHz.
 * IR: PA1 outside, PA0 inside; LEDs: PB0 (1+), PA4 (6+), PC13 occupied.
 * SSD1306 128x64: PB6 SCL, PB7 SDA, 3.3 V, common GND.
 * DHT11 module: OUT PA8, + 3.3 V, - GND; onboard data pull-up required.
 * Temperature/humidity use integer tenths, no floating point.
 * First sensor request after 2 s; subsequent requests every 6 s.
 * Do not single-step during DHT reception; Resume and allow a fresh cycle.
 * Reserve TIM2, EXTI8, IRQ23; keep one SysTick_Handler and one EXTI9_5_IRQHandler.
 * Existing occupancy algorithm/thresholds retained from the supplied code.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* ================= REGISTER DEFINITIONS ================= */
/* Volatile MMIO accesses must reach the hardware on every read/write. */
#define REG32(a) (*(volatile uint32_t *)(a))
#define GPIOA_OTYPER REG32(0x40020004UL)  /* output type: push-pull or open-drain */
#define GPIOA_OSPEEDR REG32(0x40020008UL) /* output slew-rate fields */
#define GPIOA_BSRR REG32(0x40020018UL)    /* atomic pin set (low half) and reset (high half) */
#define RCC_CR REG32(0x40023800UL)
#define RCC_CFGR REG32(0x40023808UL)
#define RCC_APB1RSTR REG32(0x40023820UL)
#define RCC_AHB1ENR REG32(0x40023830UL)
#define RCC_APB1ENR REG32(0x40023840UL)
#define GPIOA_MODER REG32(0x40020000UL)   /* two-bit pin mode fields */
#define GPIOA_PUPDR REG32(0x4002000CUL)   /* internal pull-up/pull-down fields */
#define GPIOA_IDR REG32(0x40020010UL)     /* sampled input pin levels */
#define GPIOB_MODER REG32(0x40020400UL)   /* two-bit pin mode fields */
#define GPIOB_OTYPER REG32(0x40020404UL)  /* output type: push-pull or open-drain */
#define GPIOB_OSPEEDR REG32(0x40020408UL) /* output slew-rate fields */
#define GPIOB_PUPDR REG32(0x4002040CUL)   /* internal pull-up/pull-down fields */
#define GPIOB_BSRR REG32(0x40020418UL)    /* atomic pin set (low half) and reset (high half) */
#define GPIOB_AFRL REG32(0x40020420UL)    /* alternate functions for pins 0..7 */
#define GPIOC_MODER REG32(0x40020800UL)   /* two-bit pin mode fields */
#define GPIOC_OTYPER REG32(0x40020804UL)  /* output type: push-pull or open-drain */
#define GPIOC_OSPEEDR REG32(0x40020808UL) /* output slew-rate fields */
#define GPIOC_PUPDR REG32(0x4002080CUL)   /* internal pull-up/pull-down fields */
#define GPIOC_BSRR REG32(0x40020818UL)    /* atomic pin set (low half) and reset (high half) */
#define I2C1_CR1 REG32(0x40005400UL)      /* control register 1 */
#define I2C1_CR2 REG32(0x40005404UL)      /* control register 2 */
#define I2C1_OAR1 REG32(0x40005408UL)
#define I2C1_DR REG32(0x40005410UL)  /* data register */
#define I2C1_SR1 REG32(0x40005414UL) /* status register 1 */
#define I2C1_SR2 REG32(0x40005418UL) /* status register 2 */
#define I2C1_CCR REG32(0x4000541CUL)
#define I2C1_TRISE REG32(0x40005420UL)
#define SYST_CSR REG32(0xE000E010UL)
#define SYST_RVR REG32(0xE000E014UL)
#define SYST_CVR REG32(0xE000E018UL)
/* ================= CONFIGURATION ================= */
#define SENSOR_ACTIVE_LOW 1U
#define LED_ACTIVE_LOW 1U
#define FILTER_MS 15U
#define CLEAR_MS 50U
#define PASSAGE_TIMEOUT_MS 3000U
#define I2C_TIMEOUT_MS 20U
#define S1 (1U << 1) /* PA1: outside */
#define S2 (1U << 0) /* PA0: inside */
#define BOTH (S1 | S2)
/* ================= LIVE EXPRESSIONS ================= */
/* Volatile globals expose live state to ISRs and the debugger.
 * Static state below is private to this translation unit. */
volatile uint32_t occupancy = 0;
volatile uint32_t entries = 0;
volatile uint32_t exits = 0;
volatile uint32_t rejected_sequences = 0;
volatile uint32_t exits_at_zero = 0;
volatile uint32_t milliseconds = 0;
volatile uint32_t sensor_raw = 0;
volatile uint32_t sensor_filtered = 0;
volatile uint32_t oled_ready = 0;
volatile uint32_t oled_address = 0; /* 7-bit address */
volatile uint32_t oled_error = 0;
volatile uint32_t oled_updates = 0;
/*
 * oled_error:
 * 0 = no error
 * 1 = bus busy timeout
 * 2 = NACK: device did not acknowledge
 * 3 = I2C event timeout
 * 4 = bus/arbitration/overrun error
 */
typedef enum
{
    WAIT_CLEAR = 0,
    READY,
    ENTRY_FIRST,
    EXIT_FIRST,
    ENTRY_COMPLETE,
    EXIT_COMPLETE
} CounterState;
volatile CounterState counter_state = WAIT_CLEAR;
static uint32_t candidate = 0;
static uint32_t filter_ticks = 0;
static uint32_t clear_ticks = 0;
static uint32_t sequence_start = 0;
/* Eight pages of 128 bytes; each byte represents eight vertical pixels. */
static uint8_t oled_buffer[1024];
/* ================= CLOCK AND GPIO ================= */
/* Enable HSI, select it as SYSCLK and use undivided peripheral buses.
 * All timer divisors and serial timing below assume this 16 MHz clock.
 */
static void clock_init(void)
{
    /* Enable internal 16 MHz oscillator. */
    RCC_CR |= (1U << 0);
    while ((RCC_CR & (1U << 1)) == 0U)
    {
    }
    /* Select HSI as system clock. */
    RCC_CFGR &= ~3U;
    while ((RCC_CFGR & (3U << 2)) != 0U)
    {
    }
    /* AHB, APB1 and APB2 prescalers = /1. */
    RCC_CFGR &= ~((15U << 4) | (7U << 10) | (7U << 13));
}
/* Enable GPIO ports and configure IR inputs plus the three LED outputs.
 * Write initial output levels before changing pin modes to avoid startup flashes.
 */
static void gpio_init(void)
{
    RCC_AHB1ENR |= (1U << 0) | (1U << 1) | (1U << 2);
    (void)RCC_AHB1ENR;
    /* PA0 and PA1: inputs with pull-ups. */
    GPIOA_MODER &= ~0xFU;
    GPIOA_PUPDR = (GPIOA_PUPDR & ~0xFU) | 0x5U;
    /* PC13 LED: initially off. */
    GPIOC_BSRR = LED_ACTIVE_LOW ? (1U << 13) : (1U << 29);
    GPIOC_OTYPER &= ~(1U << 13);
    GPIOC_OSPEEDR &= ~(3U << 26);
    GPIOC_PUPDR &= ~(3U << 26);
    GPIOC_MODER = (GPIOC_MODER & ~(3U << 26)) | (1U << 26);
    /* PB0 = light 1, PA4 = light 2. */
    /* Push-pull, low speed, no pull resistors; initially LOW. */
    GPIOA_BSRR = (1U << 20);
    GPIOA_OTYPER &= ~(1U << 4);
    GPIOA_OSPEEDR &= ~(3U << 8);
    GPIOA_PUPDR &= ~(3U << 8);
    GPIOA_MODER = (GPIOA_MODER & ~(3U << 8)) | (1U << 8);

    GPIOB_BSRR = (1U << 16);
    GPIOB_OTYPER &= ~(1U << 0);
    GPIOB_OSPEEDR &= ~(3U << 0);
    GPIOB_PUPDR &= ~(3U << 0);
    GPIOB_MODER = (GPIOB_MODER & ~(3U << 0)) | (1U << 0);
}

/* LDR calibration: bright 400-500, room 800, covered 3800.
 * Hysteresis: OFF->DIM >=650; DIM->OFF <=550;
 * DIM->FULL >=2100; FULL->DIM <=1900. Large jumps handled directly.
 * Software PWM on PB0/PA4, timed by TIM3: 10 slots at 10 kHz
 * gives nominal 1 kHz PWM. Interrupt latency introduces small edge jitter.
 * TIM3 IRQ priority is lower than DHT EXTI and occupancy SysTick.
 * Before ADC validity / after ADC timeout or 500 ms stale data: lights OFF.
 * A disconnected analog wire cannot reliably be identified by ADC alone.
 */
volatile uint32_t ldr_raw = 0;
volatile uint32_t ldr_samples = 0;
volatile uint32_t ldr_error = 0;
volatile uint32_t ldr_filtered = 0;
volatile uint32_t ldr_valid = 0;
volatile uint32_t light_level = 0; /* 0 sunlight/OFF, 1 room/DIM, 2 dark/FULL */
volatile uint32_t led1_duty = 0;   /* percent: 0, 30, 100 */
volatile uint32_t led2_duty = 0;
static volatile uint32_t ldr_last_good = 0;
static volatile uint32_t pwm_command = 0; /* packed slot counts for both LEDs */
#define TIM3_CR1 REG32(0x40000400UL)      /* control register 1 */
#define TIM3_DIER REG32(0x4000040CUL)     /* timer interrupt/DMA enable */
#define TIM3_SR REG32(0x40000410UL)       /* status flags */
#define TIM3_EGR REG32(0x40000414UL)      /* timer event generation */
#define TIM3_PSC REG32(0x40000428UL)      /* timer prescaler */
#define TIM3_ARR REG32(0x4000042CUL)      /* timer auto-reload period */

/* Configure TIM3 update events at 10 kHz and enable IRQ29.
 * The interrupt implements LED PWM by writing GPIO; no LED timer AF is used.
 */
static void pwm_init(void)
{
    RCC_APB1ENR |= (1U << 1);
    (void)RCC_APB1ENR;
    RCC_APB1RSTR |= (1U << 1);
    RCC_APB1RSTR &= ~(1U << 1);
    TIM3_PSC = 15U;
    TIM3_ARR = 99U; /* 16 MHz /16 /100 = 10 kHz */
    TIM3_EGR = 1U;
    TIM3_SR = 0U;
    (*(volatile uint8_t *)0xE000E41DUL) = 0xC0U; /* IRQ29 priority */
    REG32(0xE000E280UL) = (1U << 29);
    REG32(0xE000E100UL) = (1U << 29);
    TIM3_DIER = 1U;
    TIM3_CR1 = 1U;
}
/* Advance one of ten PWM slots and drive PB0/PA4 using atomic BSRR writes.
 * Latch the packed duties at phase zero so a cycle uses one consistent command.
 */
void TIM3_IRQHandler(void)
{
    static uint32_t phase = 0;
    static uint32_t command = 0;
    if ((TIM3_SR & 1U) == 0U)
        return;
    TIM3_SR = ~1U;
    if (phase == 0U)
        command = pwm_command; /* latch once per PWM cycle */
    GPIOB_BSRR = (phase < (command & 15U)) ? 1U : (1U << 16);
    GPIOA_BSRR = (phase < ((command >> 4) & 15U)) ? (1U << 4) : (1U << 20);
    phase++;
    if (phase == 10U)
        phase = 0U;
}
/* Select SUN, ROOM or DARK from the filtered ADC value.
 * Different rising/falling thresholds prevent repeated switching near a boundary.
 */
static void light_classify(uint32_t x, uint32_t first)
{
    if (first)
        light_level = (x < 600U) ? 0U : ((x > 2000U) ? 2U : 1U);
    else if (light_level == 0U)
    {
        if (x >= 2100U)
            light_level = 2;
        else if (x >= 650U)
            light_level = 1;
    }
    else if (light_level == 1U)
    {
        if (x <= 550U)
            light_level = 0;
        else if (x >= 2100U)
            light_level = 2;
    }
    else
    {
        if (x <= 550U)
            light_level = 0;
        else if (x <= 1900U)
            light_level = 1;
    }
}
/* Smooth a new ADC sample with 75% previous value and 25% new value.
 * The first valid sample initializes the filter directly; record its freshness.
 */
static void light_sample(uint32_t raw)
{
    uint32_t first = !ldr_valid;
    ldr_filtered = first ? raw : ((3U * ldr_filtered + raw + 2U) / 4U);
    light_classify(ldr_filtered, first);
    ldr_last_good = milliseconds;
    ldr_valid = 1U;
}
/* Daily classroom window: 09:00 inclusive to 17:00 exclusive.
 * Published by main; read by SysTick. 0=time error, 1=closed, 2=open.
 * PC13 continues to indicate occupancy even outside classroom hours.
 */
volatile uint32_t class_status = 0;
static volatile uint32_t class_last_good = 0;
/* Map a valid 24-hour clock hour to CLOSED (1) or OPEN (2).
 * An out-of-range hour returns the error state (0).
 */
static uint32_t class_state_for_hour(uint32_t hour)
{
    if (hour > 23U)
        return 0U;
    return (hour >= 9U && hour < 17U) ? 2U : 1U;
}
/* Return the published classroom state only while RTC data is fresh.
 * A three-second timeout disables automatic outputs through the error state.
 */
static uint32_t class_get_state(void)
{
    if ((uint32_t)(milliseconds - class_last_good) >= 3000U)
        return 0U;
    return class_status;
}
volatile uint32_t light_override = 0; /* 0 AUTO, 1 forced ON, 2 forced OFF */
/* Combine occupancy, schedule, LDR freshness and manual override.
 * Pack each LED duty as 0..10 slots; PC13 independently indicates occupancy.
 */
static void led_update(void)
{
    uint32_t people = occupancy;
    uint32_t slots = 0;
    if (class_get_state() == 2U && ldr_valid && !ldr_error &&
        (uint32_t)(milliseconds - ldr_last_good) < 500U)
        slots = (light_level == 2U) ? 10U : ((light_level == 1U) ? 3U : 0U);
    uint32_t one = people > 0U ? slots : 0U;
    uint32_t two = people > 5U ? slots : 0U;
    if (light_override == 1U)
    {
        one = 10U;
        two = 10U;
    }
    else if (light_override == 2U)
    {
        one = 0U;
        two = 0U;
    }
    led1_duty = one * 10U;
    led2_duty = two * 10U;
    pwm_command = one | (two << 4);
    /* Only TIM3 writes external LED pins after initialization. */
    uint32_t on = (people > 0U);
    if (LED_ACTIVE_LOW)
        GPIOC_BSRR = on ? (1U << 29) : (1U << 13);
    else
        GPIOC_BSRR = on ? (1U << 13) : (1U << 29);
}
/* ================= OCCUPANCY COUNTER ================= */
/* Sample both IR inputs each millisecond and validate their combined pattern.
 * Recognize ordered crossings, reject ambiguous starts/timeouts, and count only
 * after both sensors have cleared. Exits cannot make occupancy negative.
 */
static void counter_update(void)
{
    uint32_t raw = GPIOA_IDR & BOTH;
    if (SENSOR_ACTIVE_LOW)
    {
        raw ^= BOTH;
    }
    sensor_raw = raw;
    /* Filter changes to the two-sensor pattern. */
    if (raw != candidate)
    {
        candidate = raw;
        filter_ticks = 1U;
    }
    else if (filter_ticks < FILTER_MS)
    {
        filter_ticks++;
    }
    if (filter_ticks >= FILTER_MS)
    {
        sensor_filtered = candidate;
    }
    uint32_t detected = sensor_filtered;
    if ((raw == 0U) && (detected == 0U))
    {
        if (clear_ticks < CLEAR_MS)
        {
            clear_ticks++;
        }
    }
    else
    {
        clear_ticks = 0U;
    }
    if ((counter_state == ENTRY_FIRST) || (counter_state == EXIT_FIRST) ||
        (counter_state == ENTRY_COMPLETE) || (counter_state == EXIT_COMPLETE))
    {
        if ((uint32_t)(milliseconds - sequence_start) >= PASSAGE_TIMEOUT_MS)
        {
            rejected_sequences++;
            counter_state = WAIT_CLEAR;
            return;
        }
    }
    switch (counter_state)
    {
    case WAIT_CLEAR:
        if (clear_ticks >= CLEAR_MS)
        {
            counter_state = READY;
        }
        break;
    case READY:
        if (detected == S1)
        {
            sequence_start = milliseconds;
            counter_state = ENTRY_FIRST;
        }
        else if (detected == S2)
        {
            sequence_start = milliseconds;
            counter_state = EXIT_FIRST;
        }
        else if (detected == BOTH)
        {
            rejected_sequences++;
            counter_state = WAIT_CLEAR;
        }
        break;
    case ENTRY_FIRST:
        if ((detected & S2) != 0U)
        {
            counter_state = ENTRY_COMPLETE;
        }
        break;
    case EXIT_FIRST:
        if ((detected & S1) != 0U)
        {
            counter_state = EXIT_COMPLETE;
        }
        break;
    case ENTRY_COMPLETE:
        if (clear_ticks >= CLEAR_MS)
        {
            if (occupancy < UINT32_MAX)
            {
                occupancy++;
            }
            entries++;
            counter_state = WAIT_CLEAR;
        }
        break;
    case EXIT_COMPLETE:
        if (clear_ticks >= CLEAR_MS)
        {
            exits++;
            if (occupancy > 0U)
            {
                occupancy--;
            }
            else
            {
                exits_at_zero++;
            }
            counter_state = WAIT_CLEAR;
        }
        break;
    default:
        counter_state = WAIT_CLEAR;
        break;
    }
}
static void motor_service(void);
static void buttons_service(void);
/* Run the fixed 1 ms control tasks without blocking peripheral transactions.
 * Occupancy continues to update during closed classroom hours.
 */
void SysTick_Handler(void)
{
    milliseconds++;
    counter_update();
    buttons_service();
    led_update();
    motor_service();
}
/* ================= DHT11 ON PA8 =================
 * TIM2 counts microseconds; EXTI8 timestamps sensor edges.
 * No HAL, generated peripheral setup, or interrupt masking.
 * Reserve TIM2 and EXTI8 for this driver.
 * Classic DHT11 format, including decimal bytes and sign bit.
 * dht_error: 0 OK/waiting, 1 idle line low, 2 response timeout,
 *            3 malformed pulse, 4 checksum, 5 invalid data range.
 * Last successful readings remain in debugger variables after a failure,
 * but dht_valid becomes 0 and the screen explicitly shows ERROR.
 */
#define REG8(a) (*(volatile uint8_t *)(a))
#define RCC_APB2ENR REG32(0x40023844UL)
#define SYSCFG_EXTICR3 REG32(0x40013810UL)
#define EXTI_IMR REG32(0x40013C00UL)
#define EXTI_RTSR REG32(0x40013C08UL)
#define EXTI_FTSR REG32(0x40013C0CUL)
#define EXTI_PR REG32(0x40013C14UL)
#define NVIC_ISER0 REG32(0xE000E100UL)
#define NVIC_ICPR0 REG32(0xE000E280UL)
#define NVIC_IPR23 REG8(0xE000E417UL)
#define SYST_PRIORITY REG8(0xE000ED23UL)
#define TIM2_CR1 REG32(0x40000000UL) /* control register 1 */
#define TIM2_EGR REG32(0x40000014UL) /* timer event generation */
#define TIM2_CNT REG32(0x40000024UL) /* current timer count */
#define TIM2_PSC REG32(0x40000028UL) /* timer prescaler */
#define TIM2_ARR REG32(0x4000002CUL) /* timer auto-reload period */
#define DHT_PIN (1U << 8)
#define DHT_INTERVAL_MS 6000U
volatile int32_t temperature_x10 = 0; /* 283 means 28.3 C */
volatile uint32_t humidity_x10 = 0;   /* 650 means 65.0 %RH */
volatile uint32_t dht_valid = 0;
volatile uint32_t dht_error = 0;
volatile uint32_t dht_reads = 0;
volatile uint32_t dht_failures = 0;
volatile uint32_t dht_last_good_ms = 0;
volatile uint8_t dht_raw[5] = {0};
volatile uint32_t dht_phase = 0; /* 0 idle, 1 start LOW, 2 receiving */
static volatile uint32_t dht_rx_state = 0;
static volatile uint32_t dht_bits = 0;
static volatile uint32_t dht_done = 0;
static volatile uint32_t dht_edge_time = 0;
static uint32_t dht_started_ms = 0;
static uint32_t dht_released_us = 0;
static uint32_t dht_last_attempt_ms = 0;
static uint32_t display_revision = 0;
/* Configure a free-running 1 MHz TIM2 and open-drain PA8.
 * Route EXTI8 to PA8 and detect both edges for pulse-width decoding.
 */
static void dht_init(void)
{
    RCC_APB1ENR |= 1U;         /* TIM2, APB1 = 16 MHz */
    RCC_APB2ENR |= (1U << 14); /* SYSCFG */
    (void)RCC_APB1ENR;
    (void)RCC_APB2ENR;
    TIM2_CR1 = 0;
    TIM2_PSC = 15U;
    TIM2_ARR = UINT32_MAX;
    TIM2_EGR = 1U;
    TIM2_CR1 = 1U;
    /* Released open-drain PA8; the module supplies a pull-up. */
    GPIOA_BSRR = DHT_PIN;
    GPIOA_OTYPER |= DHT_PIN;
    GPIOA_OSPEEDR &= ~(3U << 16);
    GPIOA_PUPDR = (GPIOA_PUPDR & ~(3U << 16)) | (1U << 16);
    GPIOA_MODER = (GPIOA_MODER & ~(3U << 16)) | (1U << 16);
    EXTI_IMR &= ~DHT_PIN;
    SYSCFG_EXTICR3 &= ~15U; /* EXTI8 maps to PA8 */
    EXTI_RTSR |= DHT_PIN;
    EXTI_FTSR |= DHT_PIN;
    EXTI_PR = DHT_PIN;
    NVIC_ICPR0 = (1U << 23);
    NVIC_IPR23 = 0x40U; /* DHT edges can preempt SysTick */
    SYST_PRIORITY = 0x80U;
    NVIC_ISER0 = (1U << 23);
}
/* Short edge decoder, called only by EXTI9_5_IRQHandler. */
/* Decode the response handshake and 40 data bits from measured edge intervals.
 * On a malformed pulse, stop reception and publish an error to the main loop.
 */
static void dht_edge(uint32_t high, uint32_t now)
{
    uint32_t width = now - dht_edge_time;
    switch (dht_rx_state)
    {
    case 0: /* Ignore our release rising edge; await response LOW. */
        if (!high)
        {
            dht_edge_time = now;
            dht_rx_state = 1;
        }
        break;
    case 1: /* Response LOW is nominally 80 us. */
        if (!high || width < 40U || width > 120U)
            goto bad;
        dht_edge_time = now;
        dht_rx_state = 2;
        break;
    case 2: /* Response HIGH is nominally 80 us. */
        if (high || width < 40U || width > 120U)
            goto bad;
        dht_edge_time = now;
        dht_rx_state = 3;
        break;
    case 3: /* Each data bit begins with about 50 us LOW. */
        if (!high || width < 25U || width > 90U)
            goto bad;
        dht_edge_time = now;
        dht_rx_state = 4;
        break;
    case 4: /* HIGH: approximately 27 us for 0, 70 us for 1. */
        if (high || width < 10U || width > 100U)
            goto bad;
        dht_raw[dht_bits / 8U] =
            (uint8_t)((dht_raw[dht_bits / 8U] << 1) | (width > 50U ? 1U : 0U));
        dht_bits++;
        dht_edge_time = now;
        if (dht_bits == 40U)
        {
            EXTI_IMR &= ~DHT_PIN;
            dht_done = 1U;
        }
        else
            dht_rx_state = 3;
        break;
    default:
        goto bad;
    }
    return;
bad:
    EXTI_IMR &= ~DHT_PIN;
    dht_error = 3U;
    dht_done = 1U;
}
/* Timestamp a DHT edge, acknowledge EXTI8, and pass it to the decoder.
 * Only an active reception may update the current DHT message.
 */
void EXTI9_5_IRQHandler(void)
{
    if ((EXTI_PR & DHT_PIN) != 0U)
    {
        uint32_t now = TIM2_CNT;
        uint32_t high = (GPIOA_IDR & DHT_PIN) != 0U;
        EXTI_PR = DHT_PIN; /* Write one to clear */
        if (dht_phase == 2U && !dht_done)
            dht_edge(high, now);
    }
}
/* Check the checksum and field ranges before publishing temperature/humidity.
 * Values use tenths to avoid floating-point formatting.
 */
static int dht_decode(void)
{
    uint8_t sum = (uint8_t)(dht_raw[0] + dht_raw[1] + dht_raw[2] + dht_raw[3]);
    if (sum != dht_raw[4])
    {
        dht_error = 4U;
        return 0;
    }
    uint32_t fraction = dht_raw[3] & 0x7FU;
    uint32_t h = (uint32_t)dht_raw[0] * 10U + dht_raw[1];
    int32_t t = (int32_t)dht_raw[2] * 10 + (int32_t)fraction;
    if (dht_raw[1] > 9U || fraction > 9U || h > 1000U || t > 800)
    {
        dht_error = 5U;
        return 0;
    }
    if ((dht_raw[3] & 0x80U) != 0U)
        t = -t;
    temperature_x10 = t;
    humidity_x10 = h;
    return 1;
}
/* Advance the nonblocking DHT request state machine from main.
 * Generate a 20 ms LOW start pulse, release the line and allow up to 10 ms
 * for reception; publish success or invalidate the reading after failure.
 */
static void dht_service(void)
{
    uint32_t now = milliseconds;
    if (dht_phase == 0U)
    {
        /* Wait two seconds after boot and six seconds between attempts. */
        if (!dht_reads && !dht_failures && dht_last_attempt_ms == 0U)
        {
            if (now < 2000U)
                return;
        }
        else if ((uint32_t)(now - dht_last_attempt_ms) < DHT_INTERVAL_MS)
            return;
        dht_last_attempt_ms = now;
        if ((GPIOA_IDR & DHT_PIN) == 0U)
        {
            dht_error = 1U;
            dht_valid = 0U;
            dht_failures++;
            display_revision++;
            return;
        }
        EXTI_IMR &= ~DHT_PIN;
        GPIOA_BSRR = (DHT_PIN << 16); /* Start signal LOW */
        dht_started_ms = now;
        dht_phase = 1U;
        return;
    }
    if (dht_phase == 1U)
    {
        if ((uint32_t)(now - dht_started_ms) < 20U)
            return;
        for (uint32_t i = 0; i < 5U; i++)
            dht_raw[i] = 0;
        dht_bits = 0;
        dht_rx_state = 0;
        dht_done = 0;
        dht_error = 0;
        EXTI_PR = DHT_PIN;
        NVIC_ICPR0 = (1U << 23);
        dht_released_us = TIM2_CNT;
        dht_phase = 2U;
        EXTI_IMR |= DHT_PIN;
        GPIOA_BSRR = DHT_PIN; /* Release; never actively drive HIGH */
        return;
    }
    if (!dht_done && (uint32_t)(TIM2_CNT - dht_released_us) < 10000U)
        return;
    EXTI_IMR &= ~DHT_PIN;
    if (!dht_done)
        dht_error = 2U;
    GPIOA_BSRR = DHT_PIN;
    if (!dht_error && dht_bits == 40U && dht_decode())
    {
        dht_valid = 1U;
        dht_reads++;
        dht_last_good_ms = milliseconds;
    }
    else
    {
        dht_valid = 0U;
        dht_failures++;
    }
    dht_phase = 0U;
    display_revision++;
}
/* ================= I2C1 DRIVER ================= */
/* Reset I2C1 and restore 100 kHz standard-mode timing at a 16 MHz APB1 clock.
 */
static void i2c1_reset(void)
{
    RCC_APB1RSTR |= (1U << 21);
    RCC_APB1RSTR &= ~(1U << 21);
    I2C1_CR1 = 0U;
    I2C1_CR2 = 16U; /* APB1 = 16 MHz */
    I2C1_OAR1 = (1U << 14);
    I2C1_CCR = 80U; /* Approximately 100 kHz */
    I2C1_TRISE = 17U;
    I2C1_CR1 = 1U; /* Peripheral enable */
}
/* Assign PB6/PB7 to AF4 open-drain I2C1 and initialize bus timing.
 * The OLED and RTC use different addresses on the same two wires.
 */
static void i2c1_init(void)
{
    RCC_AHB1ENR |= (1U << 1);
    RCC_APB1ENR |= (1U << 21);
    (void)RCC_AHB1ENR;
    (void)RCC_APB1ENR;
    /* PB6/PB7 alternate-function mode. */
    GPIOB_MODER = (GPIOB_MODER & ~((3U << 12) | (3U << 14))) | (2U << 12) | (2U << 14);
    /* Open drain. */
    GPIOB_OTYPER |= (1U << 6) | (1U << 7);
    GPIOB_OSPEEDR = (GPIOB_OSPEEDR & ~((3U << 12) | (3U << 14))) | (1U << 12) | (1U << 14);
    /* Weak pull-ups supplement module/external pull-ups. */
    GPIOB_PUPDR = (GPIOB_PUPDR & ~((3U << 12) | (3U << 14))) | (1U << 12) | (1U << 14);
    /* AF4: I2C1 SCL/SDA. */
    GPIOB_AFRL = (GPIOB_AFRL & ~((15U << 24) | (15U << 28))) | (4U << 24) | (4U << 28);
    i2c1_reset();
}
/* All I2C functions run in main, with SysTick enabled. */
/* Wait for an I2C event with error detection and a 20 ms time limit.
 * Return 1 on the requested event, or 0 with a diagnostic error code.
 */
static int i2c1_wait(uint32_t flag)
{
    uint32_t start = milliseconds;
    while (1)
    {
        uint32_t sr = I2C1_SR1;
        if ((sr & (1U << 10)) != 0U)
        {
            oled_error = 2U;
            return 0;
        }
        if ((sr & ((1U << 8) | (1U << 9) | (1U << 11))) != 0U)
        {
            oled_error = 4U;
            return 0;
        }
        if ((sr & flag) != 0U)
        {
            return 1;
        }
        if ((uint32_t)(milliseconds - start) >= I2C_TIMEOUT_MS)
        {
            oled_error = 3U;
            return 0;
        }
    }
}
/* Wait for the shared I2C bus to become idle, with a bounded timeout.
 */
static int i2c1_idle(void)
{
    uint32_t start = milliseconds;
    while ((I2C1_SR2 & (1U << 1)) != 0U)
    {
        if ((uint32_t)(milliseconds - start) >= I2C_TIMEOUT_MS)
        {
            oled_error = 1U;
            return 0;
        }
    }
    return 1;
}
/* End a failed transaction if still bus master, then reset the peripheral.
 */
static void i2c1_abort(void)
{
    /* STOP only if we still own the bus. */
    if ((I2C1_SR2 & 1U) != 0U)
    {
        I2C1_CR1 |= (1U << 9);
        uint32_t start = milliseconds;
        while ((I2C1_CR1 & (1U << 9)) != 0U)
        {
            if ((uint32_t)(milliseconds - start) >= I2C_TIMEOUT_MS)
            {
                break;
            }
        }
    }
    i2c1_reset();
}
/* Send an SSD1306 control byte followed by a command/data block.
 * Return 1 on success; abort and recover I2C after a failed event.
 */
static int oled_write(uint8_t control, const uint8_t *data, uint32_t length)
{
    if (!i2c1_idle())
    {
        goto fail;
    }
    I2C1_SR1 &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));
    I2C1_CR1 |= (1U << 8);   /* START */
    if (!i2c1_wait(1U << 0)) /* SB */
    {
        goto fail;
    }
    I2C1_DR = oled_address << 1; /* Address + write */
    if (!i2c1_wait(1U << 1))     /* ADDR */
    {
        goto fail;
    }
    /* Clear ADDR. */
    (void)I2C1_SR1;
    (void)I2C1_SR2;
    if (!i2c1_wait(1U << 7)) /* TXE */
    {
        goto fail;
    }
    I2C1_DR = control;
    for (uint32_t i = 0; i < length; i++)
    {
        if (!i2c1_wait(1U << 7))
        {
            goto fail;
        }
        I2C1_DR = data[i];
    }
    if (!i2c1_wait(1U << 2)) /* BTF */
    {
        goto fail;
    }
    I2C1_CR1 |= (1U << 9); /* STOP */
    if (!i2c1_idle())
    {
        goto fail;
    }
    return 1;
fail:
    i2c1_abort();
    return 0;
}
/* ================= DS3231 RTC: shared I2C1, PB6/PB7 =================
 * VCC=3V3; GND=GND; SCL=PB6; SDA=PB7; SQW and 32K unused.
 * No battery is required while powered. Daily lighting schedule: 09:00-17:00.
 * An invalid/stopped clock is initialized to this file's compile time.
 * Compile on a computer with the correct local date/time, then flash promptly.
 * After total power loss without a battery, an OLD binary restores OLD build time:
 * rebuild this file (Project > Clean), then flash again to refresh it.
 * Set RTC_FORCE_BUILD_TIME to 1 for ONE flash to replace a valid but wrong clock;
 * return it to 0 and rebuild/flash afterwards. Normally keep this at 0.
 * Debugger: suspend, edit request, then Resume (do not single-step I2C).
 * rtc_set_request: 0=idle; 1=stored build timestamp;
 * 2=08:59:50 opening demo; 3=16:59:50 closing demo;
 * 4=restore saved real date/time plus elapsed time; no manual fields needed.
 * The first demo saves real RTC time. Further demos retain that timeline.
 * Elapsed time comes from the DS3231, including time paused in the debugger.
 * Keep the board powered and do not reset/reflash during a demo: anchors are RAM.
 * Start a demo only when the displayed real date/time is correct.
 * Request returns to 0 after a successful clock write.
 * Demo commands change the real RTC clock; restore actual time afterwards.
 * A day/time change is seen within approximately one RTC polling interval (1 s).
 * rtc_error: 0 OK, 1 bus busy, 2 NACK, 3 timeout, 4 bus error,
 *            5 invalid calendar/build timestamp, 6 oscillator-stop flag.
 */
#define RTC_FORCE_BUILD_TIME 0U
#define RTC_ADDRESS 0x68U
volatile uint32_t rtc_valid = 0, rtc_error = 0, rtc_reads = 0;
volatile uint32_t rtc_hour = 0, rtc_minute = 0, rtc_second = 0;
volatile uint32_t rtc_day = 0, rtc_month = 0, rtc_year = 0;
volatile uint32_t rtc_weekday = 0, rtc_status = 0, rtc_set_count = 0;
volatile uint32_t rtc_set_request = RTC_FORCE_BUILD_TIME;
volatile uint32_t rtc_demo_active = 0;
static uint32_t rtc_real_anchor = 0, rtc_demo_anchor = 0;
static uint32_t rtc_last_poll = 0, rtc_last_good = 0;

typedef struct
{
    uint32_t year, month, day, weekday, hour, minute, second;
} RtcTime;
static RtcTime rtc_serial_time; /* Validated TIME command, request 5. */

/* Validate date/time fields for the supported years 2000 through 2099.
 * February uses the leap-year rule appropriate to this restricted range.
 */
static int rtc_calendar_ok(const RtcTime *t)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (t->year < 2000U || t->year > 2099U || t->month < 1U || t->month > 12U ||
        t->weekday < 1U || t->weekday > 7U || t->hour > 23U || t->minute > 59U ||
        t->second > 59U)
        return 0;
    uint32_t limit = days[t->month - 1U];
    if (t->month == 2U && (t->year % 4U) == 0U)
        limit++;
    return t->day >= 1U && t->day <= limit;
}
/* Convert an unsigned decimal field into the RTC packed-BCD representation.
 */
static uint8_t rtc_bcd(uint32_t n)
{
    return (uint8_t)(((n / 10U) << 4) | (n % 10U));
}
/* Convert packed BCD to decimal; return 255 for an invalid BCD digit.
 */
static uint32_t rtc_unbcd(uint8_t n)
{
    if ((n & 15U) > 9U || (n >> 4) > 9U)
        return 255U;
    return (n >> 4) * 10U + (n & 15U);
}
/* Decode a coherent RTC register snapshot, including 12/24-hour conversion.
 * Reject reserved-bit and calendar errors before accepting the time.
 */
static int rtc_decode(const uint8_t *b, RtcTime *t)
{
    if ((b[0] & 0x80U) || (b[1] & 0x80U) || (b[2] & 0x80U) || (b[3] & 0xF8U) ||
        (b[4] & 0xC0U) || (b[5] & 0xE0U))
        return 0;
    t->second = rtc_unbcd(b[0]);
    t->minute = rtc_unbcd(b[1]);
    if (b[2] & 0x40U)
    {
        uint32_t h = rtc_unbcd(b[2] & 0x1FU);
        if (h < 1U || h > 12U)
            return 0;
        t->hour = (h % 12U) + ((b[2] & 0x20U) ? 12U : 0U);
    }
    else
        t->hour = rtc_unbcd(b[2]);
    t->weekday = b[3];
    t->day = rtc_unbcd(b[4]);
    t->month = rtc_unbcd(b[5]);
    uint32_t y = rtc_unbcd(b[6]);
    if (y > 99U)
        return 0;
    t->year = 2000U + y;
    return rtc_calendar_ok(t);
}
/* Convert compiler date/time strings into a validated local calendar value.
 * This is a build timestamp, not a live connection to the laptop clock.
 */
static int rtc_build_time(RtcTime *t)
{
    const char *d = __DATE__, *h = __TIME__;
    const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    t->month = 0;
    for (uint32_t i = 0; i < 12U; i++)
        if (d[0] == months[3U * i] && d[1] == months[3U * i + 1U] &&
            d[2] == months[3U * i + 2U])
            t->month = i + 1U;
    if (!t->month)
        return 0;
    t->year = (uint32_t)(d[7] - '0') * 1000U + (uint32_t)(d[8] - '0') * 100U +
              (uint32_t)(d[9] - '0') * 10U + (uint32_t)(d[10] - '0');
    t->day = (d[4] == ' ' ? 0U : (uint32_t)(d[4] - '0') * 10U) + (uint32_t)(d[5] - '0');
    t->hour = (uint32_t)(h[0] - '0') * 10U + (uint32_t)(h[1] - '0');
    t->minute = (uint32_t)(h[3] - '0') * 10U + (uint32_t)(h[4] - '0');
    t->second = (uint32_t)(h[6] - '0') * 10U + (uint32_t)(h[7] - '0');
    t->weekday = 1U;
    if (!rtc_calendar_ok(t))
        return 0;
    /* Sunday=1 ... Saturday=7; Gregorian weekday calculation. */
    static const uint8_t offsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    uint32_t y = t->year - (t->month < 3U ? 1U : 0U);
    t->weekday = (y + y / 4U - y / 100U + y / 400U + offsets[t->month - 1U] + t->day) % 7U + 1U;
    return 1;
}
/* These helpers use existing bounded I2C waits, in main only. rtc_service
 * saves/restores oled_error so a missing RTC does not overwrite OLED diagnostics. */
/* Address the DS3231 for writing and select its starting register.
 * Leave the transaction open for data or a repeated START.
 */
static int rtc_address_write(uint8_t reg)
{
    if (!i2c1_idle())
        return 0;
    I2C1_SR1 &= ~((1U << 8) | (1U << 9) | (1U << 10) | (1U << 11));
    I2C1_CR1 &= ~((1U << 10) | (1U << 11)); /* ACK=0, POS=0 */
    I2C1_CR1 |= (1U << 8);
    if (!i2c1_wait(1U << 0))
        return 0;
    I2C1_DR = RTC_ADDRESS << 1;
    if (!i2c1_wait(1U << 1))
        return 0;
    (void)I2C1_SR1;
    (void)I2C1_SR2;
    if (!i2c1_wait(1U << 7))
        return 0;
    I2C1_DR = reg;
    return i2c1_wait(1U << 2);
}
/* Write consecutive RTC registers and terminate with STOP.
 * Preserve the bus error code if recovery is needed.
 */
static int rtc_write_regs(uint8_t reg, const uint8_t *b, uint32_t n)
{
    if (!rtc_address_write(reg))
        goto fail;
    for (uint32_t i = 0; i < n; i++)
    {
        if (!i2c1_wait(1U << 7))
            goto fail;
        I2C1_DR = b[i];
    }
    if (!i2c1_wait(1U << 2))
        goto fail;
    I2C1_CR1 |= (1U << 9);
    if (!i2c1_idle())
        goto fail;
    return 1;
fail:
    rtc_error = oled_error;
    i2c1_abort();
    return 0;
}
/* Read calendar, control and status registers in one 16-byte snapshot.
 * Use BTF pacing and the final-three-byte sequence to end reception cleanly.
 */
static int rtc_read_snapshot(uint8_t b[16])
{
    if (!rtc_address_write(0U))
        goto fail;
    I2C1_CR1 |= (1U << 10); /* ACK all except final byte; POS remains zero */
    I2C1_CR1 |= (1U << 8);  /* repeated START latches coherent DS3231 calendar */
    if (!i2c1_wait(1U << 0))
        goto fail;
    I2C1_DR = (RTC_ADDRESS << 1) | 1U;
    if (!i2c1_wait(1U << 1))
        goto fail;
    (void)I2C1_SR1;
    (void)I2C1_SR2;
    uint32_t i = 0;
    /* BTF pacing permits interrupt preemption without losing final-byte timing.
     * RM0368 N>2 receive sequence: ACK low, read N-2, BTF, STOP, read N-1/N. */
    while (i < 13U)
    {
        if (!i2c1_wait(1U << 2))
            goto fail;
        b[i++] = (uint8_t)I2C1_DR;
    }
    if (!i2c1_wait(1U << 2))
        goto fail;
    I2C1_CR1 &= ~(1U << 10);
    b[i++] = (uint8_t)I2C1_DR;
    if (!i2c1_wait(1U << 2))
        goto fail;
    I2C1_CR1 |= (1U << 9);
    b[i++] = (uint8_t)I2C1_DR;
    b[i] = (uint8_t)I2C1_DR;
    if (!i2c1_idle())
        goto fail;
    return 1;
fail:
    rtc_error = oled_error;
    i2c1_abort();
    return 0;
}
/* Seconds since 2000-01-01; the supported calendar is 2000..2099. */
/* Return a month length, including February in a leap year.
 * Callers provide already validated month/year values.
 */
static uint32_t rtc_month_days(uint32_t year, uint32_t month)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return days[month - 1U] + ((month == 2U && year % 4U == 0U) ? 1U : 0U);
}
/* Convert a valid calendar to seconds since 2000-01-01 for demo timing.
 */
static uint32_t rtc_to_seconds(const RtcTime *t)
{
    uint32_t days = 0;
    for (uint32_t y = 2000U; y < t->year; y++)
        days += 365U + (y % 4U == 0U);
    for (uint32_t m = 1U; m < t->month; m++)
        days += rtc_month_days(t->year, m);
    days += t->day - 1U;
    return days * 86400U + t->hour * 3600U + t->minute * 60U + t->second;
}
/* Convert elapsed seconds back into a calendar within 2000..2099.
 * Use a 64-bit input so an out-of-range restore calculation is rejected.
 */
static int rtc_from_seconds(uint64_t seconds, RtcTime *t)
{
    if (seconds >= 3155760000ULL)
        return 0; /* 2100-01-01 is outside range */
    uint32_t days = (uint32_t)(seconds / 86400U);
    uint32_t tod = (uint32_t)(seconds % 86400U);
    t->weekday = (days + 6U) % 7U + 1U; /* 2000-01-01 was Saturday */
    t->year = 2000U;
    while (days >= 365U + (t->year % 4U == 0U))
    {
        days -= 365U + (t->year % 4U == 0U);
        t->year++;
    }
    t->month = 1U;
    while (days >= rtc_month_days(t->year, t->month))
    {
        days -= rtc_month_days(t->year, t->month);
        t->month++;
    }
    t->day = days + 1U;
    t->hour = tod / 3600U;
    t->minute = (tod % 3600U) / 60U;
    t->second = tod % 60U;
    return rtc_calendar_ok(t);
}
/* Pending anchors are committed only when the clock write succeeds. */
static uint32_t rtc_pending_real = 0;
/* Resolve build-time, opening/closing demo, restore or serial TIME requests.
 * During demos, retain the saved real timeline plus elapsed RTC seconds.
 */
static int rtc_requested_time(const uint8_t old[16], uint32_t request, RtcTime *t)
{
    if (request == 5U)
    {
        *t = rtc_serial_time;
        return rtc_calendar_ok(t);
    }
    if (request > 5U)
        return 0;
    if (request < 2U)
        return rtc_build_time(t);
    if ((old[15] & 0x80U) || !rtc_decode(old, t))
        return 0;
    uint32_t current = rtc_to_seconds(t);
    uint64_t real = current;
    if (rtc_demo_active)
    {
        if (current < rtc_demo_anchor)
            return 0;
        real = (uint64_t)rtc_real_anchor + (current - rtc_demo_anchor);
    }
    RtcTime actual;
    if (!rtc_from_seconds(real, &actual))
        return 0;
    rtc_pending_real = (uint32_t)real;
    if (request == 4U)
    {
        *t = actual; /* With no active demo this simply preserves current time. */
    }
    else
    {
        t->hour = (request == 2U) ? 8U : 16U;
        t->minute = 59U;
        t->second = 50U;
    }
    return rtc_calendar_ok(t);
}
/* Commit demo anchors only after a successful calendar write.
 * Clear the request and increment the clock-setting diagnostic counter.
 */
static void rtc_commit_request(uint32_t request, const RtcTime *t)
{
    if (request == 2U || request == 3U)
    {
        rtc_real_anchor = rtc_pending_real;
        rtc_demo_anchor = rtc_to_seconds(t);
        rtc_demo_active = 1U;
    }
    else
        rtc_demo_active = 0U;
    rtc_set_request = 0U;
    rtc_set_count++;
}
/* Apply the requested calendar; the historical function name is retained.
 * Enable backup oscillation and clear the oscillator-stop flag after setting time.
 */
static int rtc_set_build_time(const uint8_t old[16])
{
    RtcTime t;
    uint32_t request = rtc_set_request;
    if (!rtc_requested_time(old, request, &t))
    {
        rtc_error = 5U;
        return 0;
    }
    if (request == 4U && !rtc_demo_active)
    {
        rtc_set_request = 0U;
        return 1;
    }
    uint8_t control = (uint8_t)(old[14] & 0x7FU); /* EOSC=0: keep running on backup */
    if (!rtc_write_regs(0x0EU, &control, 1U))
        return 0;
    uint8_t b[7] = {rtc_bcd(t.second),      rtc_bcd(t.minute), rtc_bcd(t.hour),
                    (uint8_t)t.weekday,     rtc_bcd(t.day),    rtc_bcd(t.month),
                    rtc_bcd(t.year - 2000U)};
    if (!rtc_write_regs(0U, b, 7U))
    {
        rtc_demo_active = 0U;
        return 0;
    }
    rtc_commit_request(request, &t);
    /* Clear OSF only after valid time written. Writing 1 preserves alarm flags. */
    uint8_t status = (uint8_t)((old[15] & 0x08U) | 0x03U);
    if (!rtc_write_regs(0x0FU, &status, 1U))
        return 0;
    return 1;
}
/* Poll the DS3231 once per second and publish schedule eligibility.
 * Initialize a stopped/invalid clock, process requests and isolate RTC errors
 * from OLED diagnostics on their shared I2C bus.
 */
static void rtc_service(void)
{
    uint32_t now = milliseconds;
    if (now < 1000U || (uint32_t)(now - rtc_last_poll) < 1000U)
        return;
    rtc_last_poll = now;
    uint32_t saved_oled_error = oled_error;
    uint8_t b[16];
    RtcTime t;
    if (!rtc_read_snapshot(b))
        goto fail;
    rtc_status = b[15];
    if (rtc_set_request || (b[15] & 0x80U) || !rtc_decode(b, &t))
    {
        if (!rtc_set_build_time(b))
            goto fail;
        if (!rtc_read_snapshot(b))
            goto fail;
        rtc_status = b[15];
    }
    if (b[15] & 0x80U)
    {
        rtc_error = 6U;
        goto fail;
    }
    if (!rtc_decode(b, &t))
    {
        rtc_error = 5U;
        goto fail;
    }
    rtc_hour = t.hour;
    rtc_minute = t.minute;
    rtc_second = t.second;
    rtc_day = t.day;
    rtc_month = t.month;
    rtc_year = t.year;
    rtc_weekday = t.weekday;
    rtc_last_good = milliseconds;
    rtc_reads++;
    rtc_valid = 1U;
    rtc_error = 0U;
    class_last_good = rtc_last_good;
    class_status = class_state_for_hour(t.hour);
    oled_error = saved_oled_error;
    return;
fail:
    class_status = 0U;
    rtc_valid = 0U;
    oled_error = saved_oled_error;
}

/* ================= OLED FONT AND DRAWING ================= */
/* Five columns per character; bit 0 is the top pixel. */
static const uint8_t digit_font[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x62, 0x51, 0x49, 0x49, 0x46}, {0x22, 0x41, 0x49, 0x49, 0x36},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3E, 0x49, 0x49, 0x49, 0x32}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x26, 0x49, 0x49, 0x49, 0x3E}};
static const char font_letters[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ:-.%";
static const uint8_t letter_font[][5] = {{0x7E, 9, 9, 9, 0x7E},
                                         {0x7F, 0x49, 0x49, 0x49, 0x36},
                                         {0x3E, 0x41, 0x41, 0x41, 0x22},
                                         {0x7F, 0x41, 0x41, 0x22, 0x1C},
                                         {0x7F, 0x49, 0x49, 0x49, 0x41},
                                         {0x7F, 9, 9, 9, 1},
                                         {0x3E, 0x41, 0x49, 0x49, 0x7A},
                                         {0x7F, 8, 8, 8, 0x7F},
                                         {0, 0x41, 0x7F, 0x41, 0},
                                         {0x20, 0x40, 0x41, 0x3F, 1},
                                         {0x7F, 8, 0x14, 0x22, 0x41},
                                         {0x7F, 0x40, 0x40, 0x40, 0x40},
                                         {0x7F, 2, 0x0C, 2, 0x7F},
                                         {0x7F, 2, 4, 8, 0x7F},
                                         {0x3E, 0x41, 0x41, 0x41, 0x3E},
                                         {0x7F, 9, 9, 9, 6},
                                         {0x3E, 0x41, 0x51, 0x21, 0x5E},
                                         {0x7F, 9, 0x19, 0x29, 0x46},
                                         {0x46, 0x49, 0x49, 0x49, 0x31},
                                         {1, 1, 0x7F, 1, 1},
                                         {0x3F, 0x40, 0x40, 0x40, 0x3F},
                                         {0x1F, 0x20, 0x40, 0x20, 0x1F},
                                         {0x3F, 0x40, 0x38, 0x40, 0x3F},
                                         {0x63, 0x14, 8, 0x14, 0x63},
                                         {7, 8, 0x70, 8, 7},
                                         {0x61, 0x51, 0x49, 0x45, 0x43},
                                         {0, 0x36, 0x36, 0, 0},
                                         {8, 8, 8, 8, 8},
                                         {0, 0x60, 0x60, 0, 0},
                                         {0x63, 0x13, 8, 0x64, 0x63}};
/* Find five bitmap columns for a supported character; unknown characters are blank.
 */
static const uint8_t *oled_glyph(char c)
{
    if (c >= '0' && c <= '9')
        return digit_font[(uint32_t)(c - '0')];
    for (uint32_t i = 0; font_letters[i]; i++)
        if (font_letters[i] == c)
            return letter_font[i];
    return (const uint8_t *)0;
}
/* Set one pixel in the 128 by 64 framebuffer, clipping out-of-range positions.
 */
static void oled_pixel(uint32_t x, uint32_t y)
{
    if ((x < 128U) && (y < 64U))
    {
        oled_buffer[(y / 8U) * 128U + x] |= (uint8_t)(1U << (y % 8U));
    }
}
/* Draw characters from the bitmap font with integer pixel scaling.
 * Each character advances by five columns plus one spacing column.
 */
static void oled_text(uint32_t x, uint32_t y, const char *text, uint32_t scale)
{
    while (*text != '\0')
    {
        const uint8_t *glyph = oled_glyph(*text++);
        if (glyph != (const uint8_t *)0)
        {
            for (uint32_t col = 0; col < 5U; col++)
            {
                for (uint32_t row = 0; row < 7U; row++)
                {
                    if ((glyph[col] & (1U << row)) != 0U)
                    {
                        for (uint32_t dx = 0; dx < scale; dx++)
                        {
                            for (uint32_t dy = 0; dy < scale; dy++)
                            {
                                oled_pixel(x + col * scale + dx, y + row * scale + dy);
                            }
                        }
                    }
                }
            }
        }
        x += 6U * scale;
    }
}
/* Format an unsigned integer without floating point; the caller supplies space.
 */
static void decimal_string(uint32_t value, char *out)
{
    char reverse[10];
    uint32_t n = 0;
    do
    {
        reverse[n++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value);
    for (uint32_t i = 0; i < n; i++)
        out[i] = reverse[n - 1U - i];
    out[n] = '\0';
}
/* Format the bounded sensor value in tenths with one decimal digit.
 */
static void fixed_string(int32_t value, char *out)
{
    uint32_t n = 0;
    if (value < 0)
    {
        out[n++] = '-';
        value = -value;
    }
    decimal_string((uint32_t)value / 10U, &out[n]);
    while (out[n])
        n++;
    out[n++] = '.';
    out[n++] = (char)('0' + (uint32_t)value % 10U);
    out[n] = '\0';
}
/* Rebuild the six display rows: people, temperature, humidity, date, time, class.
 * Display WAIT or ERROR when a required reading is unavailable.
 */
static void oled_render_count(uint32_t value)
{
    char number[16];
    uint32_t state = class_get_state();
    for (uint32_t i = 0; i < sizeof(oled_buffer); i++)
        oled_buffer[i] = 0;
    oled_text(0, 0, "PEOPLE:", 1);
    decimal_string(value, number);
    oled_text(48, 0, number, 1);
    oled_text(0, 10, "TEMP:", 1);
    oled_text(0, 20, "HUM:", 1);
    if (dht_valid)
    {
        fixed_string(temperature_x10, number);
        oled_text(36, 10, number, 1);
        oled_text(90, 10, "C", 1);
        fixed_string((int32_t)humidity_x10, number);
        oled_text(36, 20, number, 1);
        oled_text(90, 20, "%RH", 1);
    }
    else
    {
        oled_text(36, 10, dht_error ? "ERROR" : "WAIT", 1);
        oled_text(36, 20, dht_error ? "ERROR" : "WAIT", 1);
    }
    oled_text(0, 30, "DATE:", 1);
    oled_text(0, 40, "TIME:", 1);
    if (rtc_valid && state != 0U)
    {
        char date[11];
        date[0] = (char)('0' + rtc_day / 10U);
        date[1] = (char)('0' + rtc_day % 10U);
        date[2] = '-';
        date[3] = (char)('0' + rtc_month / 10U);
        date[4] = (char)('0' + rtc_month % 10U);
        date[5] = '-';
        date[6] = (char)('0' + rtc_year / 1000U);
        date[7] = (char)('0' + (rtc_year / 100U) % 10U);
        date[8] = (char)('0' + (rtc_year / 10U) % 10U);
        date[9] = (char)('0' + rtc_year % 10U);
        date[10] = '\0';
        oled_text(36, 30, date, 1);
        char clock[9];
        clock[0] = (char)('0' + rtc_hour / 10U);
        clock[1] = (char)('0' + rtc_hour % 10U);
        clock[2] = ':';
        clock[3] = (char)('0' + rtc_minute / 10U);
        clock[4] = (char)('0' + rtc_minute % 10U);
        clock[5] = ':';
        clock[6] = (char)('0' + rtc_second / 10U);
        clock[7] = (char)('0' + rtc_second % 10U);
        clock[8] = '\0';
        oled_text(36, 40, clock, 1);
    }
    else
    {
        oled_text(36, 30, "--", 1);
        if (rtc_error)
        {
            oled_text(36, 40, "ERR:", 1);
            decimal_string(rtc_error, number);
            oled_text(60, 40, number, 1);
        }
        else
            oled_text(36, 40, "WAIT", 1);
    }
    oled_text(0, 50, "CLASS:", 1);
    oled_text(42, 50, state == 2U ? "OPEN" : (state == 1U ? "CLOSED" : "TIME ERROR"), 1);
}
/* Transfer all eight framebuffer pages to the OLED.
 * Stop immediately on a failed command or data transfer.
 */
static int oled_flush(void)
{
    for (uint32_t page = 0; page < 8U; page++)
    {
        uint8_t position[3] = {(uint8_t)(0xB0U + page), 0x00U, 0x10U};
        if (!oled_write(0x00U, position, sizeof(position)))
        {
            return 0;
        }
        if (!oled_write(0x40U, &oled_buffer[page * 128U], 128U))
        {
            return 0;
        }
    }
    return 1;
}
/* Probe addresses 0x3C and 0x3D, configure the display, and draw the first frame.
 */
static int oled_init(void)
{
    static const uint8_t commands[] = {
        0xAE,       /* Display OFF */
        0xD5, 0x80, /* Display clock */
        0xA8, 0x3F, /* 64 rows */
        0xD3, 0x00, /* Display offset */
        0x40,       /* Start line 0 */
        0x8D, 0x14, /* Charge pump ON */
        0x20, 0x02, /* Page addressing */
        0xA1,       /* Segment remap */
        0xC8,       /* COM scan direction */
        0xDA, 0x12, /* COM configuration */
        0x81, 0x7F, /* Contrast */
        0xD9, 0xF1, /* Precharge */
        0xDB, 0x40, /* VCOMH */
        0xA4,       /* Display RAM */
        0xA6,       /* Normal display */
        0x2E        /* Scrolling OFF */
    };
    const uint8_t off = 0xAEU;
    const uint8_t on = 0xAFU;
    uint32_t found = 0U;
    for (uint32_t address = 0x3CU; address <= 0x3DU; address++)
    {
        oled_address = address;
        if (oled_write(0x00U, &off, 1U))
        {
            found = 1U;
            break;
        }
    }
    if (found == 0U)
    {
        oled_address = 0U;
        return 0;
    }
    if (!oled_write(0x00U, commands, sizeof(commands)))
    {
        return 0;
    }
    oled_render_count(occupancy);
    if (!oled_flush())
    {
        return 0;
    }
    if (!oled_write(0x00U, &on, 1U))
    {
        return 0;
    }
    oled_error = 0U;
    return 1;
}
/* ================= ADC ACQUISITION ================= */
/* ---------- LDR: PA5 / ADC1 channel 5 ---------- */
#define ADC1_SR REG32(0x40012000UL)  /* status flags */
#define ADC1_CR1 REG32(0x40012004UL) /* control register 1 */
#define ADC1_CR2 REG32(0x40012008UL) /* control register 2 */
#define ADC1_SMPR2 REG32(0x40012010UL)
#define ADC1_SQR1 REG32(0x4001202CUL)
#define ADC1_SQR2 REG32(0x40012030UL)
#define ADC1_SQR3 REG32(0x40012034UL)
#define ADC1_DR REG32(0x4001204CUL) /* data register */
#define ADC_CCR REG32(0x40012304UL)
static uint32_t ldr_last_start = 0;
static uint32_t ldr_pending = 0;
/* Configure PA5 as analog input and ADC1 for a single 12-bit channel-5 conversion.
 * Use an 8 MHz ADC clock and the longest sample time for the sensor output.
 */
static void ldr_init(void)
{
    RCC_AHB1ENR |= (1U << 0); /* GPIOA clock */
    RCC_APB2ENR |= (1U << 8); /* ADC1 clock */
    (void)RCC_APB2ENR;
    /* PA5 analog mode, no pull-up/down. */
    GPIOA_MODER |= (3U << 10);
    GPIOA_PUPDR &= ~(3U << 10);
    ADC1_CR2 = 0U;
    ADC1_CR1 = 0U; /* 12-bit resolution */
    /* APB2 = 16 MHz; ADC clock = APB2 / 2 = 8 MHz. */
    ADC_CCR &= ~(3U << 16);
    /* Channel 5: long sampling time for the LDR divider. */
    ADC1_SMPR2 = (ADC1_SMPR2 & ~(7U << 15)) | (7U << 15);
    ADC1_SQR1 = 0U; /* One conversion */
    ADC1_SQR2 = 0U;
    ADC1_SQR3 = 5U; /* Channel 5 */
    ADC1_SR = 0U;
    ADC1_CR2 = 1U; /* ADC ON */
    ldr_last_start = milliseconds;
    ldr_pending = 0U;
}
/* Start an ADC conversion every 50 ms and poll completion without busy waiting.
 * A 10 ms conversion timeout invalidates the sample and reinitializes ADC1.
 */
static void ldr_service(void)
{
    uint32_t now = milliseconds;
    if (ldr_pending)
    {
        if ((ADC1_SR & (1U << 1)) != 0U) /* EOC */
        {
            ldr_raw = ADC1_DR & 0xFFFU;
            ldr_samples++;
            light_sample(ldr_raw);
            ldr_error = 0U;
            ldr_pending = 0U;
        }
        else if ((uint32_t)(now - ldr_last_start) >= 10U)
        {
            ldr_error = 1U;
            ldr_valid = 0U;
            ldr_init(); /* Recover; retry after settling */
        }
        return;
    }
    if ((uint32_t)(now - ldr_last_start) >= 50U)
    {
        ldr_last_start = now;
        ADC1_SR = 0U;
        ADC1_CR2 |= (1U << 30); /* Software start */
        ldr_pending = 1U;
    }
}

/* PB12 ON / PB13 OFF, each button to GND; internal pull-ups.
 * EXTI IRQ40 captures a falling edge and masks that line during debounce.
 * SysTick validates 30 ms stable LOW, then waits for 30 ms stable HIGH
 * before rearming. No delays in either handler. Held-at-boot buttons must
 * be released first. If both are pressed, OFF wins. Overrides latch until
 * LIGHT AUTO or reset. OLED layout and motor mode are unchanged.
 */
#define BUTTON_MASK ((1U << 12) | (1U << 13))
#define GPIOB_IDR REG32(0x40020410UL)              /* sampled input pin levels */
static volatile uint32_t button_state[2] = {0, 0}; /* 0 armed, 1 validate, 2 release */
static volatile uint32_t button_low[2] = {0, 0}, button_high[2] = {0, 0};
volatile uint32_t light_on_presses = 0, light_off_presses = 0;
/* Configure PB12/PB13 pull-up inputs and map falling edges to EXTI12/13.
 * Require a clean release before arming either button after reset.
 */
static void buttons_init(void)
{
    RCC_AHB1ENR |= 1U << 1;
    RCC_APB2ENR |= 1U << 14;
    (void)RCC_APB2ENR;
    EXTI_IMR &= ~BUTTON_MASK;
    GPIOB_MODER &= ~((3U << 24) | (3U << 26));
    GPIOB_PUPDR = (GPIOB_PUPDR & ~((3U << 24) | (3U << 26))) | (1U << 24) | (1U << 26);
    /* EXTICR4: EXTI12 and EXTI13 -> port B (value 1). */
    REG32(0x40013814UL) = (REG32(0x40013814UL) & ~0xFFU) | 0x11U;
    EXTI_RTSR &= ~BUTTON_MASK;
    EXTI_FTSR |= BUTTON_MASK;
    EXTI_PR = BUTTON_MASK;
    for (uint32_t i = 0; i < 2; i++)
    {
        button_state[i] = 2; /* Require clean release at boot. */
        button_low[i] = 0;
        button_high[i] = 0;
    }
    REG8(0xE000E428UL) = 0xA0U; /* IRQ40, below SysTick/DHT, above PWM */
    REG32(0xE000E284UL) = 1U << 8;
    REG32(0xE000E104UL) = 1U << 8;
}
/* Capture button edges, mask the affected lines, and begin debounce validation.
 * Keep the handler short; SysTick measures stable press and release intervals.
 */
void EXTI15_10_IRQHandler(void)
{
    uint32_t pending = EXTI_PR & EXTI_IMR & BUTTON_MASK;
    EXTI_IMR &= ~pending;
    EXTI_PR = pending;
    for (uint32_t i = 0; i < 2; i++)
    {
        if (pending & (1U << (12U + i)))
        {
            button_low[i] = 0;
            button_high[i] = 0;
            button_state[i] = 1; /* Publish last; SysTick can preempt us. */
        }
    }
}
/* Validate 30 ms LOW before accepting a press and 30 ms HIGH before rearming.
 * PB12 forces both lights ON; PB13 forces both OFF and takes precedence.
 * The override remains latched until LIGHT AUTO or reset.
 */
static void buttons_service(void)
{
    uint32_t pins = GPIOB_IDR, events = 0;
    for (uint32_t i = 0; i < 2; i++)
    {
        uint32_t bit = 1U << (12U + i), state = button_state[i];
        if (!state)
            continue;
        if (!(pins & bit))
        {
            button_high[i] = 0;
            if (state == 1U && ++button_low[i] >= 30U)
            {
                events |= 1U << i;
                button_state[i] = 2;
            }
        }
        else
        {
            button_low[i] = 0;
            if (++button_high[i] >= 30U)
            {
                button_high[i] = 0;
                button_state[i] = 0;
                EXTI_PR = bit;
                EXTI_IMR |= bit;
            }
        }
    }
    if (events & 1U)
    {
        light_on_presses++;
        if (pins & (1U << 13))
            light_override = 1; /* Do not override held OFF. */
    }
    if (events & 2U)
    {
        light_off_presses++;
        light_override = 2;
    }
}

/* ================= L298N MOTOR / FAN DEMONSTRATOR =================
 * PB8 AF2 TIM4_CH3 -> ENA (REMOVE ENA jumper); PB14 -> IN1; PB15 -> IN2.
 * Remove regulator jumper when providing external 5 V logic power.
 * OUT1/OUT2 -> motor. Common GND. Use a motor-rated external supply.
 * Add 10k ENA-to-GND resistor to hold the bridge disabled during MCU reset.
 * TIM4 hardware PWM at 1 kHz; TIM2/TIM3 and all existing pins are preserved.
 * Mode: 0 forced OFF (boot default), 1 AUTO, 2 manual duty.
 * AUTO: occupied + class OPEN + valid DHT; otherwise OFF.
 * Initial temperature bands: <28 C 40%, 28..<32 C 70%, >=32 C 100%.
 * Downward transitions use 0.5 C hysteresis. These duties need motor calibration.
 * No automatic startup boost: verify supply voltage and reliable starting first.
 * Manual commands bypass occupancy, time and temperature. PWM is not RPM feedback.
 * Debugger halt can leave hardware PWM running: FAN OFF before suspending.
 */
#define MOTOR_TIM_CR1 REG32(0x40000800UL) /* control register 1 */
#define MOTOR_TIM_EGR REG32(0x40000814UL) /* timer event generation */
#define MOTOR_TIM_CCMR2 REG32(0x4000081CUL)
#define MOTOR_TIM_CCER REG32(0x40000820UL)
#define MOTOR_TIM_PSC REG32(0x40000828UL) /* timer prescaler */
#define MOTOR_TIM_ARR REG32(0x4000082CUL) /* timer auto-reload period */
#define MOTOR_TIM_CCR3 REG32(0x4000083CUL)
#define MOTOR_MEDIUM_X10 280
#define MOTOR_HIGH_X10 320
#define MOTOR_HYST_X10 5
volatile uint32_t motor_mode = 0, motor_manual_duty = 0, motor_duty = 0;
static uint32_t motor_band = 0, motor_band_valid = 0;
/* Configure fixed motor direction and TIM4 channel 3 PWM on PB8 AF2.
 * Start with zero duty; the external driver supplies motor current.
 */
static void motor_init(void)
{
    RCC_AHB1ENR |= 1U << 1;
    (void)RCC_AHB1ENR;
    /* ENA stays LOW until timer is configured. Direction fixed: IN1=1, IN2=0. */
    GPIOB_BSRR = (1U << 24) | (1U << 30) | (1U << 31);
    GPIOB_OTYPER &= ~((1U << 8) | (1U << 14) | (1U << 15));
    GPIOB_OSPEEDR &= ~((3U << 16) | (3U << 28) | (3U << 30));
    GPIOB_PUPDR = (GPIOB_PUPDR & ~((3U << 16) | (3U << 28) | (3U << 30))) | (2U << 16);
    GPIOB_MODER = (GPIOB_MODER & ~((3U << 16) | (3U << 28) | (3U << 30))) | (1U << 16) |
                  (1U << 28) | (1U << 30);
    RCC_APB1ENR |= 1U << 2;
    (void)RCC_APB1ENR;
    RCC_APB1RSTR |= 1U << 2;
    RCC_APB1RSTR &= ~(1U << 2);
    MOTOR_TIM_CR1 = 0;
    MOTOR_TIM_PSC = 15;
    MOTOR_TIM_ARR = 999; /* 16 MHz / 16 / 1000 = 1 kHz */
    MOTOR_TIM_CCR3 = 0;
    MOTOR_TIM_CCMR2 = (6U << 4) | (1U << 3); /* CH3 PWM1, preload */
    MOTOR_TIM_CCER = 1U << 8;                /* CH3 enabled, active high */
    MOTOR_TIM_EGR = 1;
    REG32(0x40020424UL) = (REG32(0x40020424UL) & ~15U) | 2U; /* PB8 AF2 */
    GPIOB_MODER = (GPIOB_MODER & ~(3U << 16)) | (2U << 16);
    GPIOB_BSRR = (1U << 14) | (1U << 31);
    MOTOR_TIM_CR1 = (1U << 7) | 1U;
}
/* Choose OFF, manual duty or temperature-based AUTO duty each millisecond.
 * AUTO requires occupancy, open hours and a valid DHT sample younger than 15 s.
 * Use temperature hysteresis and write the hardware PWM compare register.
 */
static void motor_service(void)
{
    uint32_t duty = 0;
    if (motor_mode == 2U)
    {
        duty = motor_manual_duty;
        motor_band_valid = 0;
    }
    else if (motor_mode == 1U && occupancy > 0U && class_get_state() == 2U && dht_valid &&
             !dht_error && (uint32_t)(milliseconds - dht_last_good_ms) < 15000U)
    {
        int32_t t = temperature_x10;
        if (!motor_band_valid)
        {
            motor_band = t >= MOTOR_HIGH_X10 ? 2U : (t >= MOTOR_MEDIUM_X10 ? 1U : 0U);
            motor_band_valid = 1;
        }
        else if (motor_band == 0U)
        {
            if (t >= MOTOR_HIGH_X10)
                motor_band = 2;
            else if (t >= MOTOR_MEDIUM_X10)
                motor_band = 1;
        }
        else if (motor_band == 1U)
        {
            if (t >= MOTOR_HIGH_X10)
                motor_band = 2;
            else if (t <= MOTOR_MEDIUM_X10 - MOTOR_HYST_X10)
                motor_band = 0;
        }
        else
        {
            if (t <= MOTOR_MEDIUM_X10 - MOTOR_HYST_X10)
                motor_band = 0;
            else if (t <= MOTOR_HIGH_X10 - MOTOR_HYST_X10)
                motor_band = 1;
        }
        duty = motor_band == 2U ? 100U : (motor_band == 1U ? 70U : 40U);
    }
    else
        motor_band_valid = 0;
    if (duty > 100U)
        duty = 100U;
    MOTOR_TIM_CCR3 = duty * 10U; /* 1000 produces continuous HIGH; zero LOW */
    motor_duty = duty;
}

/* ================= USART1: PA9 TX / PA10 RX, AF7 =================
 * Adapter RXD->PA9, TXD->PA10, GND->GND. Adapter power pins unused.
 * 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control.
 * Interrupt-driven queues; no blocking UART waits or printf retarget needed.
 * Commands end in CR or LF. Configure terminal local echo/line editing ON.
 * USART priority is below DHT, SysTick and PWM.
 */
#define UART_SR REG32(0x40011000UL) /* status flags */
#define UART_DR REG32(0x40011004UL) /* data register */
#define UART_BRR REG32(0x40011008UL)
#define UART_CR1 REG32(0x4001100CUL) /* control register 1 */
#define UART_CR2 REG32(0x40011010UL) /* control register 2 */
#define UART_CR3 REG32(0x40011014UL)
#define UART_TX_MASK 2047U
#define UART_RX_MASK 255U
static volatile uint8_t uart_tx[2048], uart_rx[256];
static volatile uint32_t uart_th, uart_tt, uart_rh, uart_rt;
volatile uint32_t uart_rx_errors = 0, uart_tx_dropped = 0;
static uint32_t uart_stream = 1, uart_pending = 0, uart_pending_poll = 0;
/* Queue a complete message if the transmit ring has room; otherwise count a drop.
 * Publish the head index after copying, then enable transmit-empty interrupts.
 */
static void uart_send(const char *s)
{
    uint32_t n = (uint32_t)strlen(s), h = uart_th;
    if (n > ((uart_tt - h - 1U) & UART_TX_MASK))
    {
        uart_tx_dropped++;
        return;
    }
    for (uint32_t i = 0; i < n; i++)
    {
        uart_tx[h] = (uint8_t)s[i];
        h = (h + 1U) & UART_TX_MASK;
    }
    uart_th = h; /* Publish only after all bytes are present. */
    UART_CR1 |= 1U << 7;
}
/* Move received bytes into the RX ring and queued bytes into the transmitter.
 * Count receive errors/overflow; command parsing remains in the main loop.
 */
void USART1_IRQHandler(void)
{
    uint32_t sr = UART_SR;
    if (sr & 0x2FU)
    { /* RXNE or PE/FE/NE/ORE: SR then DR clears errors. */
        uint8_t c = (uint8_t)UART_DR;
        uint32_t next = (uart_rh + 1U) & UART_RX_MASK;
        if ((sr & 15U) || next == uart_rt)
            uart_rx_errors++;
        else if (sr & (1U << 5))
        {
            uart_rx[uart_rh] = c;
            uart_rh = next;
        }
    }
    if ((sr & (1U << 7)) && (UART_CR1 & (1U << 7)))
    {
        if (uart_tt != uart_th)
        {
            UART_DR = uart_tx[uart_tt];
            uart_tt = (uart_tt + 1U) & UART_TX_MASK;
        }
        else
            UART_CR1 &= ~(1U << 7);
    }
}
/* Configure PA9/PA10 AF7 for USART1 at approximately 115200 baud, 8N1.
 * Enable RX interrupts and announce terminal readiness.
 */
static void uart_init(void)
{
    RCC_APB2ENR |= 1U << 4;
    (void)RCC_APB2ENR;
    REG32(0x40023824UL) |= 1U << 4;
    REG32(0x40023824UL) &= ~(1U << 4);
    GPIOA_OTYPER &= ~((1U << 9) | (1U << 10));
    GPIOA_OSPEEDR = (GPIOA_OSPEEDR & ~((3U << 18) | (3U << 20))) | (1U << 18) | (1U << 20);
    GPIOA_PUPDR = (GPIOA_PUPDR & ~((3U << 18) | (3U << 20))) | (1U << 20);
    REG32(0x40020024UL) = (REG32(0x40020024UL) & ~0xFF0U) | 0x770U;
    GPIOA_MODER = (GPIOA_MODER & ~((3U << 18) | (3U << 20))) | (2U << 18) | (2U << 20);
    UART_CR1 = 0;
    UART_CR2 = 0;
    UART_CR3 = 0;
    UART_BRR = 139U;            /* round(16000000 / 115200), oversampling by 16 */
    REG8(0xE000E425UL) = 0xE0U; /* USART1 IRQ37 */
    REG32(0xE000E284UL) = 1U << 5;
    REG32(0xE000E104UL) = 1U << 5;
    UART_CR1 = (1U << 13) | (1U << 5) | (1U << 3) | (1U << 2);
    uart_send("\r\nCLASSROOM UART READY - type HELP then Enter\r\n");
}
/* Format the current readings and control modes for the serial terminal.
 * Motor duty is explicitly reported as commanded PWM rather than measured RPM.
 */
static void uart_status(void)
{
    char line[320], temp[24], hum[24], clock[80];
    if (dht_valid && !dht_error)
    {
        int32_t t = temperature_x10;
        uint32_t a = (uint32_t)(t < 0 ? -t : t);
        snprintf(temp, sizeof temp, "%s%lu.%luC", t < 0 ? "-" : "", (unsigned long)(a / 10U),
                 (unsigned long)(a % 10U));
        uint32_t h = humidity_x10;
        snprintf(hum, sizeof hum, "%lu.%lu%%", (unsigned long)(h / 10U),
                 (unsigned long)(h % 10U));
    }
    else
    {
        strcpy(temp, "ERROR");
        strcpy(hum, "ERROR");
    }
    if (rtc_valid && !rtc_error && class_get_state() != 0U)
        snprintf(clock, sizeof clock, "%04lu-%02lu-%02lu %02lu:%02lu:%02lu",
                 (unsigned long)rtc_year, (unsigned long)rtc_month, (unsigned long)rtc_day,
                 (unsigned long)rtc_hour, (unsigned long)rtc_minute, (unsigned long)rtc_second);
    else
        strcpy(clock, "TIME ERROR");
    uint32_t state = class_get_state();
    snprintf(
        line, sizeof line,
        "%s | PEOPLE=%lu | TEMP=%s HUM=%s | ADC=%lu LDR=%s | L1=%lu%% L2=%lu%% | CLASS=%s%s\r\n",
        clock, (unsigned long)occupancy, temp, hum, (unsigned long)ldr_filtered,
        (!ldr_valid || ldr_error || (uint32_t)(milliseconds - ldr_last_good) >= 500U)
            ? "ERROR"
            : (light_level == 2U ? "DARK" : (light_level == 1U ? "ROOM" : "SUN")),
        (unsigned long)led1_duty, (unsigned long)led2_duty,
        state == 2U ? "OPEN" : (state == 1U ? "CLOSED" : "TIME ERROR"),
        rtc_demo_active ? " DEMO" : "");
    uart_send(line);
    snprintf(line, sizeof line, "FAN MODE=%s DUTY=%lu%% (commanded, not measured RPM)\r\n",
             motor_mode == 1U ? "AUTO" : (motor_mode == 2U ? "MANUAL" : "OFF"),
             (unsigned long)motor_duty);
    uart_send(line);
    snprintf(line, sizeof line, "LIGHT MODE=%s\r\n",
             light_override == 1U ? "FORCED ON"
                                  : (light_override == 2U ? "FORCED OFF" : "AUTO"));
    uart_send(line);
}
/* Accept only the exact YYYY-MM-DD HH:MM:SS syntax and a valid calendar.
 * Compute the weekday after validating all numeric fields.
 */
static int uart_parse_time(const char *s, RtcTime *t)
{
    if (strlen(s) != 19U)
        return 0;
    for (uint32_t i = 0; i < 19U; i++)
    {
        char separator =
            (i == 4U || i == 7U) ? '-' : (i == 10U ? ' ' : ((i == 13U || i == 16U) ? ':' : 0));
        if (separator)
        {
            if (s[i] != separator)
                return 0;
        }
        else if (s[i] < '0' || s[i] > '9')
            return 0;
    }
    t->year = (uint32_t)(s[0] - '0') * 1000U + (uint32_t)(s[1] - '0') * 100U +
              (uint32_t)(s[2] - '0') * 10U + (uint32_t)(s[3] - '0');
    t->month = (uint32_t)(s[5] - '0') * 10U + (uint32_t)(s[6] - '0');
    t->day = (uint32_t)(s[8] - '0') * 10U + (uint32_t)(s[9] - '0');
    t->hour = (uint32_t)(s[11] - '0') * 10U + (uint32_t)(s[12] - '0');
    t->minute = (uint32_t)(s[14] - '0') * 10U + (uint32_t)(s[15] - '0');
    t->second = (uint32_t)(s[17] - '0') * 10U + (uint32_t)(s[18] - '0');
    t->weekday = 1U;
    if (!rtc_calendar_ok(t))
        return 0;
    t->weekday = (rtc_to_seconds(t) / 86400U + 6U) % 7U + 1U;
    return 1;
}
/* Dispatch one complete uppercase terminal command.
 * Manual motor commands bypass automatic gates; LIGHT AUTO releases the button
 * override. Queue RTC changes for rtc_service rather than writing in the ISR.
 */
static void uart_command(const char *s)
{
    uint32_t request = 0;
    if (!strcmp(s, "LIGHT AUTO"))
    {
        light_override = 0;
        uart_send("OK LIGHT AUTO\r\n");
        return;
    }
    if (!strcmp(s, "FAN OFF"))
    {
        motor_mode = 0;
        uart_send("OK FAN OFF\r\n");
        return;
    }
    if (!strcmp(s, "FAN ON"))
    {
        motor_manual_duty = 100;
        motor_mode = 2;
        uart_send("OK FAN MANUAL 100% - automatic rules bypassed\r\n");
        return;
    }
    if (!strcmp(s, "FAN AUTO") || !strcmp(s, "AUTO"))
    {
        motor_mode = 1;
        uart_send("OK FAN AUTO\r\n");
        return;
    }
    if (!strncmp(s, "FAN SPEED ", 10U))
    {
        const char *p = s + 10;
        uint32_t value = 0, digits = 0;
        for (; *p; p++)
        {
            if (*p < '0' || *p > '9' || ++digits > 3U)
                goto bad_speed;
            value = value * 10U + (uint32_t)(*p - '0');
        }
        if (!digits || value > 100U)
            goto bad_speed;
        motor_manual_duty = value;
        motor_mode = 2;
        uart_send("OK FAN MANUAL duty set - automatic rules bypassed\r\n");
        return;
    bad_speed:
        uart_send("ERROR: use FAN SPEED 0 through FAN SPEED 100\r\n");
        return;
    }
    if (!strcmp(s, "HELP"))
    {
        uart_send("HELP | STATUS | STREAM ON | STREAM OFF\r\n"
                  "FAN OFF | FAN ON | FAN SPEED 0..100 | FAN AUTO (or AUTO)\r\n"
                  "LIGHT AUTO - restore automatic lights after PB12/PB13 override\r\n"
                  "TIME YYYY-MM-DD HH:MM:SS (local time, years 2000-2099)\r\n"
                  "DEMO OPEN | DEMO CLOSE | RESTORE\r\n"
                  "Demos change the RTC. RESTORE before resetting/powering off.\r\n");
        return;
    }
    if (!strcmp(s, "STATUS"))
    {
        uart_status();
        return;
    }
    if (!strcmp(s, "STREAM ON"))
    {
        uart_stream = 1;
        uart_send("OK STREAM ON\r\n");
        return;
    }
    if (!strcmp(s, "STREAM OFF"))
    {
        uart_stream = 0;
        uart_send("OK STREAM OFF\r\n");
        return;
    }
    if (!strcmp(s, "DEMO OPEN"))
        request = 2;
    else if (!strcmp(s, "DEMO CLOSE"))
        request = 3;
    else if (!strcmp(s, "RESTORE"))
        request = 4;
    else if (!strncmp(s, "TIME ", 5U))
    {
        RtcTime t;
        if (!uart_parse_time(s + 5, &t))
        {
            uart_send("ERROR: use TIME YYYY-MM-DD HH:MM:SS with a valid date/time\r\n");
            return;
        }
        if (uart_pending || rtc_set_request)
        {
            uart_send("BUSY: RTC request pending\r\n");
            return;
        }
        rtc_serial_time = t;
        request = 5;
    }
    else
    {
        uart_send("Unknown command. Type HELP\r\n");
        return;
    }
    if (uart_pending || rtc_set_request)
    {
        uart_send("BUSY: RTC request pending\r\n");
        return;
    }
    if (request == 4U && !rtc_demo_active)
    {
        uart_send("No active demo to restore\r\n");
        return;
    }
    rtc_set_request = request;
    uart_pending = 1;
    uart_pending_poll = rtc_last_poll;
    uart_send("RTC request queued\r\n");
}
/* Consume queued characters, handle line endings/backspace and reject damaged input.
 * Acknowledge completed RTC requests and stream status every five seconds.
 */
static void uart_service(void)
{
    static char line[80];
    static uint32_t used = 0, discard = 0, seen_errors = 0, last_stream = 0;
    if (uart_pending && rtc_last_poll != uart_pending_poll)
    {
        uart_pending = 0;
        if (!rtc_set_request && rtc_valid && !rtc_error)
        {
            uart_send("OK RTC updated\r\n");
            uart_status();
        }
        else
        {
            rtc_set_request = 0; /* Do not silently retry a failed serial command. */
            uart_send("ERROR: RTC update/readback failed; check displayed time and retry\r\n");
        }
    }
    /* On damaged input, discard through a newline before accepting commands. */
    if (seen_errors != uart_rx_errors)
    {
        seen_errors = uart_rx_errors;
        uart_rt = uart_rh;
        used = 0;
        discard = 1;
        uart_send("RX error/overflow: press Enter, then retry\r\n");
    }
    uint32_t budget = 256;
    while (uart_rt != uart_rh && budget--)
    {
        char c = (char)uart_rx[uart_rt];
        uart_rt = (uart_rt + 1U) & UART_RX_MASK;
        if (c == '\r' || c == '\n')
        {
            if (discard)
                uart_send("Input discarded; retry command\r\n");
            else if (used)
            {
                line[used] = 0;
                uart_command(line);
            }
            used = 0;
            discard = 0;
        }
        else if (!discard)
        {
            if (c == '\b' || c == 127)
            {
                if (used)
                    used--;
            }
            else if (c >= 32 && c <= 126)
            {
                if (used >= sizeof(line) - 1U)
                {
                    discard = 1;
                    used = 0;
                }
                else
                    line[used++] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
            }
            else
            {
                discard = 1;
                used = 0;
            }
        }
    }
    if (uart_stream && (uint32_t)(milliseconds - last_stream) >= 5000U)
    {
        last_stream = milliseconds;
        uart_status();
    }
}

/* Initialize outputs and peripherals, then run cooperative foreground services.
 * Defer RTC, UART parsing and OLED transfers during DHT transactions.
 * Interrupts continue occupancy, button and PWM work throughout the main loop.
 */
int main(void)
{
    SYST_CSR = 0;
    clock_init();
    gpio_init();
    dht_init();
    pwm_init();
    motor_init();
    buttons_init();
    SYST_RVR = 16000U - 1U;
    SYST_CVR = 0;
    SYST_CSR = 7U;
    uart_init();
    i2c1_init();
    ldr_init();
    uint32_t start = milliseconds;
    while ((uint32_t)(milliseconds - start) < 100U)
    {
    }
    oled_ready = (uint32_t)oled_init();
    uint32_t last_attempt = milliseconds, last_check = milliseconds;

    while (1)
    {
        dht_service();
        ldr_service();
        /* Finish the sensor start pulse promptly; EXTI captures reception.
           SysTick keeps counting people throughout this interval. */
        if (dht_phase != 0U)
            continue;
        rtc_service();
        uart_service();
        uint32_t now = milliseconds;
        if (!oled_ready)
        {
            if ((uint32_t)(now - last_attempt) >= 2000U)
            {
                i2c1_reset();
                oled_ready = (uint32_t)oled_init();
                last_attempt = milliseconds;
            }
            continue;
        }
        if ((uint32_t)(now - last_check) >= 300U)
        {
            last_check = now;
            uint32_t count = occupancy;
            /* Refresh occupancy, temperature, humidity, date, time and class status. */
            if (1)
            {
                oled_render_count(count);
                if (oled_flush())
                {
                    oled_error = 0;
                    oled_updates++;
                }
                else
                {
                    oled_ready = 0;
                    last_attempt = milliseconds;
                }
            }
        }
    }
}
