
/* Lesson 5
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
      x = (unsigned int)random(0, 513);
      y = (unsigned int)random(0, 513);
      
      Serial.print(x);
      Serial.print(":");
      Serial.println(y);
      
      xyTime = millis();
    }
  }

  return 0; 
}
