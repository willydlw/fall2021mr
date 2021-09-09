# Arduino Programming

## Where is the main function?

The Arduino IDE presents a sketch as

```
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
```

Experienced C/C++ programmers know each program requires a main function as the starting point. Arduino hides this function from the user, adding it during the build process. The main.cpp file, shown below, is found in the /hardware/arduino/cores/arduino folder of your Arduino installation. 


## main() function

```

#include <Arduino.h>

int main(void)
{
    init();

#if defined(USBCON)
    USBDevice.attach();
#endif
   
    setup();
   
    for (;;) {
        loop();
        if (serialEventRun) serialEventRun();
    }
       
    return 0;
}
```

<br><br>

### init() function

The init() function sets the register configuration of some components used by the Arduino standard library. It sets the registers of all available timers, the ADC control and status registers, and disconnects pins 0 and 1 from the USART so they can be used as normal digital I/O. (They can be reconnected with Serial.begin()). It is defined in hardware/arduino/avr/cores/arduino/wiring.c

<br><br>

### #if defined(USBCON)

Some Arduino boards, such as the Leonardo, have an onboard USB device. This is needed only for those boards.

### setup(), loop(), serialEvent

We can now see why the setup function is called only once before the loop function is called repeatedly in the infinite for loop structure. It is also easier to understand the serial event documentation that explains the serialEvent() function is only called at the end of each loop function execution, if there is receieved data available.
<br><br>

### Rewrite the blink sketch using main()

Create the Arduino sketch shown below. Program execution starts with the first line of the main function. Notice that the loop() function has been replaced with an infinite while loop.

```
#include <Arduino.h>

int main(void)
{
  init();
  pinMode(LED_BUILTIN, OUTPUT);
  while(true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  return 0;
}
```

If you encounter an error uploading your sketch in Ubuntu, see [common_linux_arduino_errors](common_linux_arduino_errors.md)
