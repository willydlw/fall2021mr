/* Use preprocessor directive to create a synmbol named DEBUG
 *  
 *  Surround statements with 
 *  
 *  #if DEBUG
 *    Serial.println("message");
 *  #endif
 *  
 *  When the program is compiled with # define DEBUG 1 
 *  the conditional statements #if DEBUG are true and the source code
 *  is compiled into the program.
 *  
 *  When the program is compiled with #define DEBUG 0
 *  the conditional statements #if DEBUG are false and the sourc code
 *  is not compiled into the program
 *  
 */
 
#define DEBUG 1

const unsigned long BLINK_INTERVAL = 500;
unsigned long blinkTime;
unsigned long wasteTime;
byte ledState;

void setup()
{
  #if DEBUG
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
        
        #if DEBUG
        Serial.print("ledState: ");
        Serial.println(ledState);
        #endif 
    }
    else{
      wasteTime = millis();
      while((millis() - wasteTime) < 200);
    }
}
