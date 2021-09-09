#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
  DDRB |= (1<<DDB5);            // pinMode(13, OUTPUT);
  while(1)
  {
    PORTB |= (1 << PORTB5);     // digitalWrite(13, HIGH);
    _delay_ms(1000);
    PORTB &=  ~(1<<PORTB5);     // digitalWrite(13, LOW);
    _delay_ms(1000);
  }
}
