/*  Compare dynamic memory and program memory usage
 *   to example 2. This example stores strings in ram.
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
  Serial.println("setup function complete");
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
    Serial.print("Time to write a message, printTime: ");
    Serial.println(printTime);
    printTime = millis();
  }
  else
  {
    unsigned long count = millis();
    Serial.println("kill time in loop");
    while( (millis() - count) < 250); 
  }
  

}
