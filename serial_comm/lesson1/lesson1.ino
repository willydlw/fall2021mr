/* Serial Transmit Lesson 1: Transmits a count in the range [0,255] with
 *  the println command.
 *  
 *  print transmits numbers using an ASCII character for each digit
 * 
 * Example: 
 *  The value 1 is transmitted as '1'
 *  The value 12 is transmitted as '1' '2'
 *  The value 203 is transmitted as '2' '0' '3'
 *          
 * 
 * println transmits numbers as characters and appends the newline character
*/

#include <Arduino.h>

int main()
{
  byte count = 7;

  init();
  
  // default config is 8 data bits, no parity, one stop bit  8N1
  Serial.begin(9600);

  while(1)
  {
    Serial.println(count);
    ++count;
    delay(500);
  }

  return 0;
}
