# Arduino 

Arduino is an open-source hardware and software company that manufactures single-board microcontrollers and kits. Their mission is to provide a low-cost and easy way to use sensors and actuators that interact with their environment.

"Arduino board designs use a variety of microprocessors and controllers. The boards are equipped with sets of digital and analog input/output (I/O) pins that may be interfaced to various expansion boards ('shields') or breadboards (for prototyping) and other circuits. The boards feature serial communications interfaces, including Universal Serial Bus (USB) on some models, which are also used for loading programs from personal computers. The microcontrollers can be programmed using the C and C++ programming languages, using a standard API which is also known as the "Arduino language". In addition to using traditional compiler toolchains, the Arduino project provides an integrated development environment (IDE) and a command line tool (arduino-cli) developed in Go." [1](https://en.wikipedia.org/wiki/Arduino#:~:text=Most%20Arduino%20boards%20consist%20of,SAM3X8E%20was%20introduced%20in%202012.)

<br><br>

## Arduino Nano

The programming examples in this module will refer to the Arduino Nano, but are applicable to other Arduino boards as well. The Arduino nano is often used in robotics applications, due to its small form factor. 

See [arduino_nano](arduino_nano.md) for technical specifications.

<br><br>

## Arduino Programming 

The Arduino software libraries and the IDE (integrated development environment) make programming the microcontroller relatively easy. There is a performance price associated with using some of the Arduino software libraries. To illustrate, let's look at the Arduino blink sketch versus the AVR blink sketch. [arduino_vs_atmel_blink](arduino_vs_atmel_blink.md)

Now that you've seen that cryptic blink sketch, you likely have more appreciation for the Arduino software. We will use many Arduino libraries in this course to speed development. However, we will write our sketches in a slightly different way, using the traditional C++ main function, and eliminating the use of global variables.

<br><br>

## Replacing setup() and loop() with main()

Your Arduino homework in this course will use the C++ main function versus the loop() function. This method is preferred to eliminate the use of global variables. Read [programming_with_main](programming_with_main.md)

<br><br>

## Optimizing Arduino Code Memory Space

We often use the serial print functions to provide debugging message information. The additional code and string messages require program memory storage. As your programs grow larger, using too much ram memory will cause your program to slow down and possibly fail while running. Read [optimizing_arduino_code_memory](optimizing_arduino_code_memory.md) to learn how to optimize your code.
