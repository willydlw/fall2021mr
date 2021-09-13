/* Serial Transmit Example 2b: send data type unsigned int with write command.
 * 
 * Serial.write(buf, len) writes binary data to the serial port. Sent as a series of bytes
 *  
 * Example: The value 1   is transmitted as 00000000 00000001  0x00 0x01
 *          The value 9   is transmitted as 00000000 00001001  0x00 0x09
 *          The value 32  is transmitted as 00000000 00100000  0x00 0x20
 *          The value 256 is tranmsitted as 00000001 00000000  0x01 0x00
 * 
*/

#include <Arduino.h>

int main()
{
  unsigned int count = 65533;             // 0 - 65535
  byte buf[2];
  
  init();
  Serial.begin(9600);

  while(1)
  {
    // transmit higher order byte first, lower order byte second
    buf[1] = count & 0xff;            // low order byte
    buf[0] = (count >> 8) & 0xff;     // higher order byte
    Serial.write(buf, sizeof(buf));
    ++count;
    delay(500);
  }

  return 0;
}
