#define F_CPU 16000000UL // 16mhz crystal clock speed of arduino board

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// simple soil moisture monitor on atmega328p microcontroller 

#define TIME_BTWN_WATER_BURST 10 // wait 10 seconds to give the water time to dissolve into the soil, prevents overwatering
#define RAW_TTN_CHECK_SEC 43200UL // 12hours - how long to sleep before next sensor poll, without weights for temp & humidity
#define TIME_WEIGHT_TEMP_SEC 21600UL // +/- 6h 
#define TIME_WEIGHT_HUMID_SEC 7200UL // +/- 2h
#define TIME_WEIGHT_OVERWATERED 18000UL 

#define TEMP_LOW 16 // if lower, lower rate of evaporation
#define TEMP_HIGH 25 // if higher, higher rate of evaporation

#define HUMID_LOW 35 // if lower, highter rate of evaporation
#define HUMID_HIGH 60 // if higher, low rate of evaporation

// NOT CALIBRATED YET
#define SOIL_DRY_RED 800 // dry soil, water!
#define SOIL_OKAY_YELLOW 650 // desired level, between recently watered and too dry
#define SOIL_WET_GREEN 450 // recently watered, very wet soil
#define SOIL_SOAKED_BLUE 380 // way too much water

// SETUP DHT11 Temperature and Humidity Sensor
#define DHT_PIN 2    // PD2
#define DHT_PORT PORTD
#define DHT_PINR PIND
#define DHT_DDR DDRD

// SETUP LEDs
#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_RED 128
#define LED_YELLOW 64
#define LED_GREEN 32
#define LED_BLUE 16


volatile uint8_t wdt_flag = 0; // watchdog timer flag for deep sleep

ISR(WDT_vect) { // interrupt service routine, what to do on interrupt by WDT
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

void sleep_seconds(unsigned long seconds)
{
    unsigned long cycles = seconds / 8UL; // 8 s per WDT cycle

    for (unsigned long i = 0; i < cycles; i++)
    {
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
    
    uint8_t humidity;
    uint8_t temperature;

    uint16_t adc_value;
    // set PB5 as output (onboard Led) -> basic monitoring of soil status, replace or add pump later
    DDRB |= (1 << 5); // shift by 5 to reference pb5 because 5th bit in DDRB
    adc_init();

    DDRD |= 0b11110000; // enable digital pins 4-7 as output (LEDs)


    while (1)
    {
        adc_value = adc_read(0);   // VAL BETWEEN 300(SUBMERGED) AND 1023(DRY) - read A0 - soil humidity sensor 

        if (!DHT_read_sensor()) {
            humidity = DHT_data[0];
            temperature = DHT_data[2];
        }
        // MODIFY DEEP SLEEP TIME OF NEXT CYCLE USING AMBIENT TEMP & HUMIDITY 
        long time_before_next_measurement = RAW_TTN_CHECK_SEC;

        if(humidity < HUMID_LOW) { // TODO: später mit function und pointerübergabe ersetzen
            time_before_next_measurement -= TIME_WEIGHT_HUMID_SEC;
        }
        else if (humidity > HUMID_HIGH){ 
            time_before_next_measurement += TIME_WEIGHT_HUMID_SEC;
        }

        if(temperature < TEMP_LOW) { // TODO: später mit function und pointerübergabe ersetzen
            time_before_next_measurement += TIME_WEIGHT_TEMP_SEC;
        }
        else if (temperature > TEMP_HIGH){ 
            time_before_next_measurement -= TIME_WEIGHT_TEMP_SEC;
        }

        // WATERING DECISION

        PORTD &= 240; // 0b11110000 - set all LEDs to LOW

        
        if(adc_value > SOIL_DRY_RED) {
            PORTD &= 240; // 0b11110000 - set all LEDs to LOW
            PORTD |= LED_RED;
            while (adc_value > SOIL_WET_GREEN) {
                // NO PUMP AVAILABLE YET, BUT SEND HIGH SIGNAL FOR 1-2 SEC HERE
                // ============== ACTIVATE WATER PUMP THROUGH HIGH SIGNAL ON DIGI PIN CONTROLLING TRANSISTOR PROBABLY ==============
                sleep_seconds(TIME_BTWN_WATER_BURST);
                adc_value = adc_read(0); 
            }
        }
        else if(adc_value > SOIL_OKAY_YELLOW) {
            PORTD &= 240; // 0b11110000 - set all LEDs to LOW
            PORTD |= LED_YELLOW;
        }
        else if(adc_value > SOIL_WET_GREEN) {
            PORTD &= 240; // 0b11110000 - set all LEDs to LOW
            PORTD |= LED_GREEN;
        }else if(adc_value < SOIL_SOAKED_BLUE) {
            PORTD &= 240; // 0b11110000 - set all LEDs to LOW
            PORTD |= LED_BLUE;
            sleep_seconds(TIME_WEIGHT_OVERWATERED); // even longer sleep before next check, because soil has more water than normal, no need to check earlier
        }
             
        sleep_seconds(time_before_next_measurement); // go into sleep to save power, before next measuring
        
    }
}
