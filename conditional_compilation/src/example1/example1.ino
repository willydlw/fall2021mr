/* Use preprocessor directive to create an alias for the
 *  Serial.print, Serial.println functions.
 *  
 *  Everywhere we type serialDebug(x), the preprocessor 
 *  replaces that code with Serial.print(x)
 *  
 *  Everywhere we type serialDebugln(x), the preprocessor 
 *  replaces that code with Serial.println(x)
 *  
 *  To remove the serial debug statements from the compiled
 *  program, change #define debug(x) Serial.print(x)
 *  to #define debug(x) 
 *  
 *  This says to replace debug(x) with nothing
 */
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)


const unsigned long BLINK_INTERVAL = 500;
unsigned long blinkTime;
unsigned long wasteTime;
byte ledState;

void setup()
{
  Serial.begin(38400);
  delay(250);
  debugln("Serial Debug Ready");
    
  pinMode(LED_BUILTIN, OUTPUT);
  ledState = 0;
  digitalWrite(LED_BUILTIN, ledState);
  blinkTime = millis();
}

void loop()
{
    if(millis() - blinkTime >= BLINK_INTERVAL){
        ledState = ledState ^ 0x01;     // toggle state 
        digitalWrite(LED_BUILTIN, ledState);
        blinkTime = millis();
        debug("ledState: ");
        debugln(ledState);
    }
    else{
      wasteTime = millis();
      while((millis() - wasteTime) < 200);
    }
}
