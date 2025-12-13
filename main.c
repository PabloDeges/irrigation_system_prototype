#define F_CPU 16000000UL // 16mhz clock speed of arduino board

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// simple soil moisture monitor on atmega328p microcontroller 
// LED ON -> soil moisture levels okay
// LED OFF -> soil moisture too dry -> activate pump later

#define MOISTURE_THOLD 800
#define TEMPERATURE_THOLD 27
#define HUMID_THOLD 35
#define TIMEDEC_TEMP 1800
#define TIMEADD_HUMID 600

#define DHT_PIN 2    // PD2 / Arduino digital pin 2
#define DHT_PORT PORTD
#define DHT_PINR PIND
#define DHT_DDR DDRD

volatile uint8_t wdt_flag = 0;

ISR(WDT_vect) {
    wdt_flag = 1;  // Set flag to wake up
}

void sleep_wdt() { // sleep using watchdog timer
    cli(); // Disable interrupts
    wdt_reset();

    // Enable WDT interrupt, set 8 s -> max possible sleep on atm
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // 8 s

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei(); // Enable interrupts

    sleep_cpu(); // Sleep until WDT interrupt
    sleep_disable();

    wdt_flag = 0; // Clear wakeup flag
}

void sleep_seconds(uint8_t seconds) {
    uint16_t cycles = seconds / 8; // 8 s per WDT cycle
    for(uint16_t i = 0; i < cycles; i++) {
        sleep_wdt();
    }
}


void adc_init(void)
{
    //reference avcc - ADC0 selected
    ADMUX = 0x40;        // 01000000

    // adc enable -  prescaler = divide by 128 for appropriate clockspeed for analog in
    ADCSRA = 0x87;       // 10000111
}

uint16_t adc_read(uint8_t channel)
{
    // select channel / pin 0 on board
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // bitmask kram to preserve first 4 und last 4

    // start conversion
    ADCSRA |= 0x40; // or with  01000000 to start conv

    // end conversion -> when read done, HW sets bit to 0
    while (ADCSRA & 0x40);

    return ADC; // value btween 0-1023, lowest possible value with sensor 300 -> real range 300-1023
	// 1032 equals complete dry, 300 equals comepletely submerged

}

//dht11 sensor functs

void DHT_set_output() { DHT_DDR |= (1 << DHT_PIN); }
void DHT_set_input()  { DHT_DDR &= ~(1 << DHT_PIN); }

void DHT_write_high() { DHT_PORT |= (1 << DHT_PIN); }
void DHT_write_low()  { DHT_PORT &= ~(1 << DHT_PIN); }
uint8_t DHT_read()    { return (DHT_PINR & (1 << DHT_PIN)) != 0; }

uint8_t DHT_data[5];  // humidity_high, humidity_low, temp_high, temp_low, checksum

uint8_t DHT_read_sensor()
{
    uint8_t i, j;
    
    // start signal
    DHT_set_output();
    DHT_write_low();
    _delay_ms(20);      // ≥18 ms
    DHT_write_high();
    _delay_us(40);      // 20–40 us
    DHT_set_input();
    
    // wait for sensor response (~80us LOW, then 80us HIGH)
    uint16_t timeout = 1000;
    while(DHT_read() && timeout--) _delay_us(1);  // wait for LOW
    timeout = 1000;
    while(!DHT_read() && timeout--) _delay_us(1); // wait for HIGH
    timeout = 1000;
    while(DHT_read() && timeout--) _delay_us(1);  // wait for LOW
    
    // read the 40 bits
    for(j = 0; j < 5; j++)
    {
        DHT_data[j] = 0;
        for(i = 0; i < 8; i++)
        {
            // wait for HIGH
            timeout = 1000;
            while(!DHT_read() && timeout--) _delay_us(1);
            
            // measure pulse length
            _delay_us(35);
            if(DHT_read()) DHT_data[j] |= (1 << (7-i));
            
            // wait for LOW
            timeout = 1000;
            while(DHT_read() && timeout--) _delay_us(1);
        }
    }
    
    // 4. Verify checksum
    if(DHT_data[0] + DHT_data[1] + DHT_data[2] + DHT_data[3] != DHT_data[4])
        return 0;  // error
    return 1;      // success
}




int main(void)
{
    uint16_t adc_value;

    uint8_t humidity;
    uint8_t temperature;

    long time_before_next_read_s = 3600; // *1000 for ms, dont forget 

    // set PB5 as output (onboard Led) -> basic monitoring of soil status, replace or add pump later
    DDRB |= (1 << 5); // shift by 5 to reference pb5 because 5th bit in DDRB

    adc_init();

    

    while (1)
    {
        adc_value = adc_read(0);   // read A0

        if (!DHT_read_sensor()) {
            humidity = DHT_data[0];
            temperature = DHT_data[2];
        }
        int temperature_time_dec;
        int humidity_time_add;

        if(temperature >= TEMPERATURE_THOLD) { // higher rate of evaporation -> higher probability of soil being dry
            temperature_time_dec = TIMEDEC_TEMP;
        } else {
            temperature_time_dec = 0;
        }

        if(humidity >= HUMID_THOLD) { // higher moisture in the air that the plant can use, -> lower prob of dry, less impact than hot air tho
            humidity_time_add = TIMEADD_HUMID;
        } else {
            humidity_time_add = 0;
        }



        if (adc_value > MOISTURE_THOLD)
        {
            // LED on
            PORTB |= (1 << 5);
        }
        else
        {
            // LED off
            PORTB &= ~(1 << 5); // tilde operator -> invert / NOT, &= -> only affect bit 5
        }


        long time_to_sleep = time_before_next_read_s + TIMEADD_HUMID - TIMEDEC_TEMP; // in ms
        
        
        sleep_seconds(time_to_sleep);
        
    }
}
