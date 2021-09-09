/* Use preprocessor directive to create a synmbol named DEBUG
 *  
 *  Surround statements with 
 *  
 *  #ifdef DEBUG
 *    Serial.println("message");
 *  #endif
 *  
 *  When the symbol #define DEBUG is commented out or removed from
 *  the program all the statement surrounded by #ifdef DEBUG  #endif
 *  will not be compiled into the program.
 *  
 */
 
#define DEBUG

const unsigned long BLINK_INTERVAL = 500;
unsigned long blinkTime;
unsigned long wasteTime;
byte ledState;

void setup()
{
  #ifdef DEBUG
  Serial.begin(38400);
  delay(250);
  Serial.println("Serial Debug Ready");
  #endif
    
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
        
        #ifdef DEBUG
        Serial.print("ledState: ");
        Serial.println(ledState);
        #endif 
    }
    else{
      wasteTime = millis();
      while((millis() - wasteTime) < 200);
    }
}
