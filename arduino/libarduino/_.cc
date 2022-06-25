 /*
 Arduinutil - Arduino-like library written in C
 Supported microcontrollers:
 See Arduinutil.h
 Copyright 2016 Djones A. Boni
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
 http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define INTERRUPTS_DISABLE() __asm __volatile("cli" ::: "memory")
#define INTERRUPTS_ENABLE()  __asm __volatile("sei" ::: "memory")

#define CRITICAL_VAL()   uint8_t __istate_val
#define CRITICAL_ENTER() __asm __volatile( \
        "in %0, __SREG__ \n\t"             \
        "cli             \n\t"             \
        :"=r" (__istate_val) ::"memory")
#define CRITICAL_EXIT()  __asm __volatile( \
        "out __SREG__, %0 \n\t"            \
        ::"r" (__istate_val) :"memory")

#define WAIT_INT()  __asm __volatile("sleep" ::: "memory")
#define WAIT_BUSY() __asm __volatile("nop" ::: "memory")

/** Disable digital inputs of analog pins for lower power consumption. */
void disableDigitalInputsOfAnalogPins(uint32_t didr_bits)
{
    CRITICAL_VAL();

    CRITICAL_ENTER();
    {
        uint8_t prr_save = PRR;
        PRR &= ~(1U << PRADC);

        /* PRR0[PRADC] must be 0. */
        DIDR0 |= didr_bits;
        DIDR1 |= didr_bits >> 8;

        PRR = prr_save;
    }
    CRITICAL_EXIT();
}

#ifdef __cplusplus
extern "C" {
#else
/* Avoid C90 complaining about inline in some AVR headers. */
#define inline __inline
#endif

/*******************************************************************************
 Timer.c
 ******************************************************************************/

#define TIMER_COUNT_TO_MS(x) ((x) * TIMER_PRESCALER / (F_CPU / 1000UL) )
#define TIMER_MS_TO_COUNT(x) ((x) * (F_CPU / 1000UL) / TIMER_PRESCALER )

#define TIMER_COUNT_TO_US(x) ((x) * (TIMER_PRESCALER * 125UL) / (F_CPU / 8000UL) )
#define TIMER_US_TO_COUNT(x) ((x) * (F_CPU / 8000UL) / (TIMER_PRESCALER * 125UL) )

/*******************************************************************************
 Digital.c
 ******************************************************************************/

#define ANALOGIO 14U
#define MAXIO    (ANALOGIO + 6U)


enum DigitalInterruptModes {
    /* LOW = 0x00U, */
    CHANGE = 0x01U,
    FALLING = 0x02U,
    RISING = 0x03U
};

/*******************************************************************************
 Analog.c
 ******************************************************************************/

/* Analog/Digital pins */
#define A0       (ANALOGIO + 0U)
#define A1       (ANALOGIO + 1U)
#define A2       (ANALOGIO + 2U)
#define A3       (ANALOGIO + 3U)
#define A4       (ANALOGIO + 4U)
#define A5       (ANALOGIO + 5U)
/* Analog only pins */
#define A6       (ANALOGIO + 6U)
#define A7       (ANALOGIO + 7U)
/* Analog only internal */
#define ATEMP    (ANALOGIO + 8U)
#define A1V1     (ANALOGIO + 14U)

enum AnalogReferences {
    EXTERNAL = 0x00U, /* External voltage on AREF. */
    INTERNALVCC = 0x01U, /* Internal voltage VCC. */
    INTERNAL1V1 = 0x03U, /* Internal voltage reference 1.1V. */
    /* Arduino IDE compatibility. */
    DEFAULT = INTERNALVCC,
    INTERNAL = INTERNAL1V1
};

/*******************************************************************************
 Pwm.c
 ******************************************************************************/

enum PwmModes {
    PWM_DISABLE  = 0U,
    PWM_NOINVERT = 2U,
    PWM_INVERT   = 3U
};

/*******************************************************************************
 Others
 ******************************************************************************/


/** Disable all peripherals clocks for lower power consumption.
 Note: Changes on peripheral registers will not have effect after disabling its
 clock. As an example, you should disable ADC (ADCSRA[ADEN]=0) before disabling
 its clock (PRR[PRADC]=1), otherwise it will still be enabled and consuming
 power. */
void disablePeripheralsClocks(void)
{
    PRR = 0xFFU;
}


/*******************************************************************************
 Serial.c
 ******************************************************************************/

#define SERIAL_CUSTOM_UBRR_DIV16     0x40000000UL
#define SERIAL_CUSTOM_UBRR_DIV8      0x80000000UL
#define SERIAL_CUSTOM_UBRR_MASK      0xC0000000UL

#define SERIAL_CONF(A,B,C) ((A)|((B)<<8UL)|((C)<<16UL))
#define SERIAL_5N1 SERIAL_CONF(0x00UL, 0x98UL, 0x00UL)
#define SERIAL_6N1 SERIAL_CONF(0x00UL, 0x98UL, 0x02UL)
#define SERIAL_7N1 SERIAL_CONF(0x00UL, 0x98UL, 0x04UL)
#define SERIAL_8N1 SERIAL_CONF(0x00UL, 0x98UL, 0x06UL)
#define SERIAL_5N2 SERIAL_CONF(0x00UL, 0x98UL, 0x08UL)
#define SERIAL_6N2 SERIAL_CONF(0x00UL, 0x98UL, 0x0AUL)
#define SERIAL_7N2 SERIAL_CONF(0x00UL, 0x98UL, 0x0CUL)
#define SERIAL_8N2 SERIAL_CONF(0x00UL, 0x98UL, 0x0EUL)
#define SERIAL_5E1 SERIAL_CONF(0x00UL, 0x98UL, 0x20UL)
#define SERIAL_6E1 SERIAL_CONF(0x00UL, 0x98UL, 0x22UL)
#define SERIAL_7E1 SERIAL_CONF(0x00UL, 0x98UL, 0x24UL)
#define SERIAL_8E1 SERIAL_CONF(0x00UL, 0x98UL, 0x26UL)
#define SERIAL_5E2 SERIAL_CONF(0x00UL, 0x98UL, 0x28UL)
#define SERIAL_6E2 SERIAL_CONF(0x00UL, 0x98UL, 0x2AUL)
#define SERIAL_7E2 SERIAL_CONF(0x00UL, 0x98UL, 0x2CUL)
#define SERIAL_8E2 SERIAL_CONF(0x00UL, 0x98UL, 0x2EUL)
#define SERIAL_5O1 SERIAL_CONF(0x00UL, 0x98UL, 0x30UL)
#define SERIAL_6O1 SERIAL_CONF(0x00UL, 0x98UL, 0x32UL)
#define SERIAL_7O1 SERIAL_CONF(0x00UL, 0x98UL, 0x34UL)
#define SERIAL_8O1 SERIAL_CONF(0x00UL, 0x98UL, 0x36UL)
#define SERIAL_5O2 SERIAL_CONF(0x00UL, 0x98UL, 0x38UL)
#define SERIAL_6O2 SERIAL_CONF(0x00UL, 0x98UL, 0x3AUL)
#define SERIAL_7O2 SERIAL_CONF(0x00UL, 0x98UL, 0x3CUL)
#define SERIAL_8O2 SERIAL_CONF(0x00UL, 0x98UL, 0x3EUL)

#ifdef __cplusplus
} /* extern "C" */
#endif




#ifdef __cplusplus
extern "C" {
#endif

#define QUEUE_ENABLE                 1
#define MAILBOX_ENABLE               1
#define SEMAPHORE_ENABLE             1
#define MUTEX_ENABLE                 1

#define SERIAL_ENABLE                0
#define SERIAL_RBUFSZ                64U
#define SERIAL_TBUFSZ                64U
#define SERIAL_PRINT_BUFSZ           32U

#define I2C_ENABLE                   0
#define I2C_PRESCALER                64U

#define WATCHDOG_ENABLE              0
#define WATCHDOG_AUTOINIT            0
#define WATCHDOG_AUTOINIT_TIMEOUT    0U

#define TIMER_ENABLE                 0
#define TIMER_PRESCALER              1024U

#define ANALOG_ENABLE                0

#define PWM_ENABLE                   0
#define TIMER1_PRESCALER             1024U
#define TIMER2_PRESCALER             1024U
#define TIMER1_OVERFLOW_INTERRUPT    0
#define TIMER2_OVERFLOW_INTERRUPT    0

#define DIGITAL_EXTERNAL_INT_ENABLE  0
#define DIGITAL_ATTACH_INT_ENABLE    0

typedef uint8_t Size_t;

#define ASSERT(expr)                                   \
  do {                                                 \
    if((expr) == 0)                                    \
    {                                                  \
      INTERRUPTS_DISABLE();                            \
      /*assert_function(__FILE__, __LINE__, #expr);*/  \
      for(;;) {}                                       \
    }                                                  \
  } while(0)

#ifdef __cplusplus
} /* extern "C" */
#endif




#define SET_BITS(reg, bits, cast)   do{reg=(cast)((reg)|(bits));}while(0U)
#define CLEAR_BITS(reg, bits, cast) do{reg=(cast)((reg)&(~(bits)));}while(0U)

/** Microcontroller initialization. */
void init(void)
{
    /* Configure all pins of the microcontroller as input with pull-up
     (DDR=0,PORT=FF), even those not used by Arduino. Avoid power consumption of
     floating input pins.
     Analog pins have the pull-ups disabled, but the digital inputs for these
     pins are also disabled.
     You can change pin direction and pull-up with pinMode() or make changes
     directly here. */
    DDRB = 0U;
    PORTB = 0xFFU;
    DDRC = 0U;
    PORTC = 0xC0U;
    DDRD = 0U;
    PORTD = 0xFFU;

    disableDigitalInputsOfAnalogPins(0xFFFFFFFF);
    disablePeripheralsClocks();

    /* Configure external interrupts registers. */
    EIMSK = 0U;
    EIFR = 0x03U;
    PCICR = 0U;
    PCIFR = 0x07U;
    PCMSK0 = 0U;
    PCMSK1 = 0U;
    PCMSK2 = 0U;
    PCICR = 0x07U;

    INTERRUPTS_ENABLE();

    set_sleep_mode(0); /* IDLE */
    sleep_enable();
}


/** Enable peripherals clocks. */
void enablePeripheralsClocks(void)
{
    PRR = 0U;
}


/** Enable digital inputs of analog pins.
 *
 * 0x00FF bits correspond to A7-A0
 * 0x0300 bits correspond to analog comparator
 */
void enableDigitalInputsOfAnalogPins(uint32_t didr_bits)
{
    CRITICAL_VAL();

    CRITICAL_ENTER();
    {
        uint8_t prr_save = PRR;
        PRR &= ~(1U << PRADC);

        /* PRR0[PRADC] must be 0. */
        DIDR0 &= ~didr_bits;
        DIDR1 &= ~(didr_bits >> 8);

        PRR = prr_save;
    }
    CRITICAL_EXIT();
}

 /*
 Arduinutil - Arduino-like library written in C
 Supported microcontrollers:
 See Arduinutil.h
 Copyright 2016 Djones A. Boni
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
 http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */



#define SET_BITS(reg, bits, cast)   do{reg=(cast)((reg)|(bits));}while(0U)
#define CLEAR_BITS(reg, bits, cast) do{reg=(cast)((reg)&(~(bits)));}while(0U)

PROGMEM static volatile uint8_t * const portIOLIST[] = {
        /* 0 - 13 */
        &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTD, &PORTB,
        &PORTB, &PORTB, &PORTB, &PORTB, &PORTB,
        /* A0 - A5 */
        &PORTC, &PORTC, &PORTC, &PORTC, &PORTC, &PORTC
};

PROGMEM static volatile uint8_t * const pinIOLIST[] = {
        /* 0 - 13 */
        &PIND, &PIND, &PIND, &PIND, &PIND, &PIND, &PIND, &PIND, &PINB, &PINB,
        &PINB, &PINB, &PINB, &PINB,
        /* A0 - A5 */
        &PINC, &PINC, &PINC, &PINC, &PINC, &PINC
};

PROGMEM static volatile uint8_t * const ddrIOLIST[] = {
        /* 0 - 13 */
        &DDRD, &DDRD, &DDRD, &DDRD, &DDRD, &DDRD, &DDRD, &DDRD, &DDRB, &DDRB,
        &DDRB, &DDRB, &DDRB, &DDRB,
        /* A0 - A5 */
        &DDRC, &DDRC, &DDRC, &DDRC, &DDRC, &DDRC
};

PROGMEM static const uint8_t bitIOLIST[] = {
        /* 0 - 13 */
        0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 0U, 1U, 2U, 3U, 4U, 5U,
        /* A0 - A5 */
        0U, 1U, 2U, 3U, 4U, 5U
};

/** Change pin configuration. The modes are INPUT, OUTPUT and INPUT_PULLUP.
 Note: Use pin ANALOGIO+X for analog pin X. */
void pinMode(uint8_t io, uint8_t mode)
{
    volatile uint8_t *port = (volatile uint8_t *)pgm_read_word(&portIOLIST[io]);
    volatile uint8_t *ddr = (volatile uint8_t *)pgm_read_word(&ddrIOLIST[io]);
    uint8_t bit = pgm_read_byte(&bitIOLIST[io]);
    CRITICAL_VAL();

    ASSERT(io < MAXIO);

    CRITICAL_ENTER();

    switch(mode)
    {
    case 0:
        /* Input. */
        CLEAR_BITS(*ddr, (1U << bit), uint8_t);
        /* Disable pull-up. */
        CLEAR_BITS(*port, (1U << bit), uint8_t);
        break;
    case 1:
        /* Output. */
        SET_BITS(*ddr, (1U << bit), uint8_t);
        break;
    case 2:
        /* Input. */
        CLEAR_BITS(*ddr, (1U << bit), uint8_t);
        /* Enable pull-up. */
        SET_BITS(*port, (1U << bit), uint8_t);
        break;
    default:
        ASSERT(0); /* Invalid pin mode. */
        break;
    }

    CRITICAL_EXIT();
}

/** Change pin output value or input pull-up. The values are LOW and HIGH.
 Note: Use pin ANALOGIO+X for analog pin X. */
void digitalWrite(uint8_t io, uint8_t value)
{
    volatile uint8_t *port = (volatile uint8_t *)pgm_read_word(&portIOLIST[io]);
    uint8_t bit = pgm_read_byte(&bitIOLIST[io]);
    CRITICAL_VAL();

    ASSERT(io < MAXIO);

    CRITICAL_ENTER();

    switch(value)
    {
    case 0:
        CLEAR_BITS(*port, (1U << bit), uint8_t);
        break;
    default:
        SET_BITS(*port, (1U << bit), uint8_t);
        break;
    }

    CRITICAL_EXIT();
}

/** Read pin input value. The values returned are LOW and HIGH.
 Note: Use pin ANALOGIO+X for analog pin X. */
uint8_t digitalRead(uint8_t io)
{
    volatile uint8_t *pin = (volatile uint8_t *)pgm_read_word(&pinIOLIST[io]);
    uint8_t bit = pgm_read_byte(&bitIOLIST[io]);

    ASSERT(io < MAXIO);

    return (*pin & (1U << bit)) != 0U;
}

#if (DIGITAL_EXTERNAL_INT_ENABLE != 0)

static volatile uint8_t * convIoToInterruptRegister(uint8_t io)
{
    volatile uint8_t *reg;
    void *port = (void *)pgm_read_word(&portIOLIST[io]);

    ASSERT(io < MAXIO);

    if(port == &PORTB)
    {
        reg = &PCMSK0;
    }
    else if(port == &PORTC)
    {
        reg = &PCMSK1;
    }
    else if(port == &PORTD)
    {
        reg = &PCMSK2;
    }
    else
    {
        ASSERT(0); /* Invalid port. */
    }

    return reg;
}

/** Enable external interrupt.
 The processor interrupts whenever any of the enabled pins state changes.
 The 3 available interrupts vectors are shared within each port. Therefore
 application must check by software which pin changed and to what state.
 Pins 0-7 (PORTD) use the interrupt vector PCINT2_vect.
 Pins 8-13 (PORTB) use the interrupt vector PCINT0_vect.
 Pins A0-A5 (PORTC) use the interrupt vector PCINT1_vect.
 *  // SETUP
 *  digitalWrite(13, digitalRead(8));
 *  enableExternalInterrupt(8);
 *
 *  // PORTB ISR
 *  ISR(PCINT0_vect) {
 *      static uint8_t port_last;
 *
 *      uint8_t port_now = PINB;
 *      uint8_t port_changed = port_last ^ port_now;
 *      port_last = port_now;
 *
 *      if(port_changed & (1 << PINB0)) {
 *          if(port_now & (1 << PINB0)) {
 *              // Rising edge
 *          }
 *          else {
 *              // Falling edge
 *          }
 *      }
 *
 *      // Check other pins
 *  }
 */
void enableExternalInterrupt(uint8_t io)
{
    volatile uint8_t *reg = convIoToInterruptRegister(io);
    uint8_t bit = pgm_read_byte(&bitIOLIST[io]);
    CRITICAL_VAL();

    CRITICAL_ENTER();
    {
        *reg |= (1U << bit);
    }
    CRITICAL_EXIT();
}

/** Disable external interrupt. */
void disableExternalInterrupt(uint8_t io)
{
    volatile uint8_t *reg = convIoToInterruptRegister(io);
    uint8_t bit = pgm_read_byte(&bitIOLIST[io]);
    CRITICAL_VAL();

    CRITICAL_ENTER();
    {
        *reg &= ~(1U << bit);
    }
    CRITICAL_EXIT();
}

#endif /* DIGITAL_EXTERNAL_INT_ENABLE */

#if (DIGITAL_ATTACH_INT_ENABLE != 0)

static void (*extIntVector[2U])(void) = {0};

/** Convert a digital pin to a interrupt number.
 In this implementation it does nothing other than avoid breaking people code.
 */
uint8_t digitalPinToInterrupt(uint8_t pin)
{
    return pin;
}

/** Enable external interrupt.
 Only pins 2 and 3 are able to use this function.
 isr is a pointer to the function to be called when the interrupt fires.
 mode is the interrupt mode. See enum DigitalInterruptModes. */
void attachInterrupt(uint8_t pin, void (*isr)(void), uint8_t mode)
{
    CRITICAL_VAL();

    mode &= 0x03U;

    CRITICAL_ENTER();

    switch(pin) {
    case 2U:
        EIMSK &= ~(1U << INT0);
        EIFR = (1U << INTF0);
        EICRA = (EICRA & ~(0x03U << ISC00)) | (mode << ISC00);
        extIntVector[0U] = isr;
        EIMSK |= (1U << INT0);
        break;
    case 3U:
        EIMSK &= ~(1U << INT1);
        EIFR = (1U << INTF1);
        EICRA = (EICRA & ~(0x03U << ISC10)) | (mode << ISC10);
        extIntVector[1U] = isr;
        EIMSK |= (1U << INT1);
        break;
    default:
        ASSERT(0); /* Invalid interrupt pin. */
    }

    CRITICAL_EXIT();
}

/** Disable external interrupt. */
void detachInterrupt(uint8_t pin)
{
    CRITICAL_VAL();

    CRITICAL_ENTER();

    switch(pin) {
    case 2U:
        EIMSK &= ~(1U << INT0);
        break;
    case 3U:
        EIMSK &= ~(1U << INT1);
        break;
    default:
        ASSERT(0); /* Invalid interrupt pin. */
    }

    CRITICAL_EXIT();
}

ISR(INT0_vect)
{
    extIntVector[0U]();
}

ISR(INT1_vect)
{
    extIntVector[1U]();
}

#endif /* DIGITAL_ATTACH_INT_ENABLE*/