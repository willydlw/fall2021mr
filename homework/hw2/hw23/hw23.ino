
/* Homework  2,  Problem 3
 *    Use Arduino Serial.write functions to transmit binary values of x and y. 
 *    
 *    x,y are type unsigned int
 *    
 *    Target Hardware: Arduino Uno, ATMega 328p
 *    Software:  Arduino 1.8.12
 *    
 *    
 *    Illustrates using milliseconds timer and time intervals to perform actions
 *    that require different time delays.
 */

// global constants
const unsigned long BLINK_INTERVAL = 500;           // unit: milliseconds
const unsigned long POSITION_UPDATE_INTERVAL = 300; // unit: milliseconds


int main()
{
  // local variables
  unsigned long blinkTime;                         // unit: milliseconds
  unsigned long xyTime;                            // unit: milliseconds
  int ledPinState;

  unsigned int x, y;

  init();

  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);         
  ledPinState = 0;
  digitalWrite(LED_BUILTIN, ledPinState);            
  blinkTime = millis();
  xyTime = millis();

  while(1)
  {
    if( (millis() - blinkTime) >= BLINK_INTERVAL)
    {
      ledPinState = ledPinState ^ 0x01;     // toggle state
      digitalWrite(LED_BUILTIN, ledPinState);
      blinkTime = millis();
    }
  
    if( (millis() - xyTime) >= POSITION_UPDATE_INTERVAL)
    {
      // initially tested with values x  = 0xABCD, y  = 0x1234
      // to confirm byte order of  transmission  and  reception
      x = 0xABCD; //(unsigned int)random(0, 513);
      y = 0x1234; //(unsigned int)random(0, 513);

      /* Directly access variable memory address
       *  cast to byte pointer and write 2 bytes
       *  sizeof(unsigned int) produced a value of 2 bytes
       */ 
      if(Serial.availableForWrite() > 3){
          Serial.write((byte*)&x,2);
          Serial.write((byte*)&y,2);
      }
      
      xyTime = millis();
    }
  }

  return 0; 
}
