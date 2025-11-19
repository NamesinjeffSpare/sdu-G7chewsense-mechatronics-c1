#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BAUD 115200

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdint.h>

// Each sample sent over UART takes approximately this many bytes (CSV format)
#define SAMPLE_BYTES 30

// ADC runs in free-running mode at this sample rate
#define ADC_SAMPLE_RATE 9600

// UART can transmit roughly BAUD / 10 bytes per second (1 start + 8 data + 1 stop bit)
#define UART_BYTES_PER_SEC (BAUD / 10)

// How many samples per second can fit through the UART
#define MAX_SAMPLES_PER_SEC (UART_BYTES_PER_SEC / SAMPLE_BYTES)

// We only send 1 in N samples to avoid flooding UART
#define DATA_SKIP_COUNT (ADC_SAMPLE_RATE / MAX_SAMPLES_PER_SEC)

// ADC channel used for EMG signal input (A0) add more l8r 
#define EMG_ADC_CHANNEL 0

// Size of the ring buffer holding outgoing samples
#define BUF_SIZE 128

volatile uint32_t millis_counter = 0;

typedef struct {
    uint16_t adc;       // Raw ADC value (10-bit)
    uint32_t t_ms;      // Timestamp in milliseconds
    uint32_t index;     // Sequential sample index
} sample_t;

// Ring buffer and pointers
volatile sample_t sample_buf[BUF_SIZE];
volatile uint8_t buf_head = 0;   // Points to the next write location
volatile uint8_t buf_tail = 0;   // Points to the next read location

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= (1 << U2X0);  // Enable double speed if recommended
#else
    UCSR0A &= ~(1 << U2X0);
#endif

    // 8 data bits, no parity, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // Enables TX)
    UCSR0B = (1 << TXEN0);
}

// Send a single byte over UART (blocking)
void uart_send(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0))) {
        ; // Wait until transmit buffer is empty
    }
    UDR0 = data;
}

// Send a null-terminated string over UART
void uart_puts(const char *s) {
    while (*s) {
        uart_send((uint8_t)*s++);
    }
}

// Print a 16-bit unsigned integer
void uart_print_uint16(uint16_t value) {
    char buf[6];
    uint8_t i = 0;

    if (value == 0) {
        uart_send('0');
        return;
    }

    while (value > 0 && i < sizeof(buf) - 1) {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (i > 0) {
        uart_send((uint8_t)buf[--i]);
    }
}

// Print a 32-bit unsigned integer
void uart_print_uint32(uint32_t value) {
    char buf[11];
    uint8_t i = 0;

    if (value == 0) {
        uart_send('0');
        return;
    }

    while (value > 0 && i < sizeof(buf) - 1) {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (i > 0) {
        uart_send((uint8_t)buf[--i]);
    }
}

// Timer2: 1ms Timebase
void timer2_init(void) {
    TCCR2A = (1 << WGM21);     
    TCCR2B = (1 << CS22);      
    OCR2A  = 249;              
    TIMSK2 = (1 << OCIE2A);    
}

ISR(TIMER2_COMPA_vect) {
    millis_counter++;
}

// ADC Setup and ISR
void adc_init(void) {
    ADMUX = (1 << REFS0) | (EMG_ADC_CHANNEL & 0x0F);  // AVcc ref, select channel
    ADCSRA = (1 << ADEN)  |
             (1 << ADATE) |   // Auto trigger
             (1 << ADIE)  |   // Enable interrupt
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Prescaler = 128

    ADCSRB = 0x00;              // Free-running mode
    ADCSRA |= (1 << ADSC);      // Start conversions
}

// ISR: Captures ADC samples, but only stores every Nth one (data skip)
ISR(ADC_vect) {
    static uint16_t skip_counter = 0;
    static uint32_t sample_index = 0;

    uint16_t adc_value = ADC;
    skip_counter++;

    if (skip_counter >= DATA_SKIP_COUNT) {
        skip_counter = 0;
        sample_index++;

        uint8_t next_head = (buf_head + 1) % BUF_SIZE;

        // Only write if buffer not full
        if (next_head != buf_tail) {
            sample_buf[buf_head].adc   = adc_value;
            sample_buf[buf_head].t_ms  = millis_counter;
            sample_buf[buf_head].index = sample_index;
            buf_head = next_head;
        }
    }
}

int main(void) {
    uart_init();
    timer2_init();
    adc_init();
    sei(); 
    // Send CSV header for external tools to parse
    uart_puts("# EMG data\n");
    uart_puts("# sample,adc,millis_ms\n");
    while (1) {
        if (buf_head != buf_tail) {
            cli();  // Temporarily disable interrupts for atomic read
            sample_t s = sample_buf[buf_tail];
            buf_tail = (buf_tail + 1) % BUF_SIZE;
            sei();

            // Send formatted data line
            uart_print_uint32(s.index);
            uart_send(',');
            uart_print_uint16(s.adc);
            uart_send(',');
            uart_print_uint32(s.t_ms);
            uart_send('\n');
        }
    }

    return 0;
}
