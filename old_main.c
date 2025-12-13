#define F_CPU 16000000UL

#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>


void adc_init(void)
{
    ADMUX = (1 << REFS0);  // AVcc reference
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}




int main() {

	adc_init();

	DDRB |= _BV(DDB5); // enable port 13 to be written to -> onboard LED

	int analog_val = 0;

	//turn led on 13 off:
	int led_off = _BV(PORTB5);
	//turn led on
	int led_on = (PORTB ^= _BV(PORTB5)); // flip bits / xor: 0 -> 1 & 1 -> 0 ------ LED ON OFF 
	while (1) {

		analog_val = adc_read(0x00); // btween 0 - 1023 -> 10bits - 1023 complete dry, around 300 fully submerged

		if (analog_val > 500) { // halb submerged
			PORTB = led_on;
		}
		else {
			PORTB = led_off;
		}

		_delay_ms(500); // 0,5hz abtastrate

	}

}


