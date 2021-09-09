/*  Compare dynamic memory and program memory usage
 *   to example 1. This example uses F macro to store
 *   strings in program memory.
 *    
*/
const unsigned long BLINK_INTERVAL = 500;
const unsigned long PRINT_INTERVAL = 550;
unsigned long blinkTime, printTime;
byte ledState;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(38400);
  ledState = 1;
  digitalWrite(LED_BUILTIN, ledState);
  blinkTime = millis();
  printTime = millis();
  Serial.println(F("setup function complete"));
}

void loop() {
  if( (millis() - blinkTime) > BLINK_INTERVAL)
  {
    ledState = ledState ^ (byte)0x01;
    digitalWrite(LED_BUILTIN, ledState);
    blinkTime = millis();
  }
  else if( (millis() - printTime) > PRINT_INTERVAL)
  {
    Serial.print(F("Time to write a message, printTime: "));
    Serial.println(printTime);
    printTime = millis();
  }
  else
  {
    unsigned long count = millis();
    Serial.println(F("kill time in loop"));
    while( (millis() - count) < 250); 
  }
  

}
