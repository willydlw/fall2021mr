/* Serial Receive Lesson 3: read a string and transmit it back
 * 
 * Serial.readStringUntil(terminator) reads characters from the serial buffer into a String.
 * 
*/

#include <Arduino.h>

int main()
{
  String msg;
  
  init();
  Serial.begin(38400);

  while(1)
  {
    if(Serial.available() > 0)
    {
      msg = Serial.readStringUntil('\n');
  
      // echo back the message received
      Serial.println(msg);
    }
    else
    {
      delay(50);
    }
  }
  return 0;
}
