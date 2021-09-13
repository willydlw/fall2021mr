## Lesson 2 - Binary Transmission & Reception

These examples illustrate how the Arduino Serial.write function transmits data as binary values.
<br>

### Arduino Serial.write()

> The Serial.write function writes binary data to the serial port. The data is sent as a byte or a series of bytes.
>
> #### Syntax
>
> Serial.write(val)             
> Serial.write(str)
> Serial.write(buf, len)
>
> ##### Parameters
> Serial: serial port object <br>
> val: a value to send as a single byte <br>
> str: a string to send as a series of bytes <br>
> buf: and array to send as a series of bytes <br>
> len: the number of bytes to be sent from the array <br>

<br>

### Lesson 2a - Arduino transmits a single binary byte value

#### Arduino Source Code - lesson2a.ino

Description
- The program serially transmits a single value in the range [0, 255]. 
- Serial.write function transmits binary data

Examples
- The value 1   is transmitted as 00000001
- The value 9   is transmitted as 00001001
- The value 32  is transmitted as 00100000
- The value 255 is tranmsitted as 11111111

Instructions
- Upload the code to the Arduino.
- If you open the Serial monitor program, you may see no output or unexpected characters. The Serial monitor attempts to display the ASCII character representation of the binary value. Many of these values are non-printable characters.
- The Serial Monitor program must be closed before running the python program.
<br>

#### python source code - lesson2a.py

Description
- The program opens the serial connection to the Arduino. The path "/dev/ttyACM0" is hard-coded in the program. If your path differs, change it before running the program.
- The program loops for a count of 10
  - If serial data has been received
    - read a single byte
    - print the byte in its byte array form (note this is hexadecimal, not binary)
    - print the byte in its unsigned integer form

Instructions
- Run the program by typing `python3 lesson2a.py` in a terminal command line.
- Study the program and its output. 
<br>

**pySerial read function**
> read(size=1)
> 
> Parameters: size - number of bytes to read
>
> returns bytes read from the port
> return type: bytes
>
> If a timeout is set it may return less characters as requested. With no timeout it will block until the requested number of bytes is read.
  
<br>

#### Example Output
<br>

![Lesson 2a output](./images/lesson2a_output.png "python lesson 2a output")
<br>
<br>

### Lesson 2b - Arduino transmits an array of binary byte values

#### Arduino source code - lesson2b.ino

Description
- The count value data type is unsigned int, which uses 2 bytes of memory. It stores numbers in the range [0, 65535]. 
- The Serial.write(buf,len) function transmits the data as a series of bytes.
  - The unsigned int must be converted into individual bytes and stored in an array. The higher order byte is stored in the first array element, index position 0. The lower order byte is stored in array index 1.
  <br>

  ```
  byte buf[2];
  // transmit higher order byte first, lower order byte second
  buf[1] = count & 0xff;            // low order byte
  buf[0] = (count >> 8) & 0xff;     // higher order byte
  Serial.write(buf, sizeof(buf));
  ```

<br>

Example
- The value 1   is transmitted as 00000000 00000001  0x00 0x01
- The value 9   is transmitted as 00000000 00001001  0x00 0x09
- The value 32  is transmitted as 00000000 00100000  0x00 0x20
- The value 256 is tranmsitted as 00000001 00000000  0x01 0x00

<br>
Instructions
- Upload the code to the Arduino.

<br>
<br>

#### python source code - lesson2b.py

Description
- The program opens the serial connection to the Arduino. The path "/dev/ttyACM0" is hard-coded in the program. If your path differs, change it before running the program.
- The program loops for a count of 5
  - If serial data has been received
    - read two bytes 
    - print the bytes in their byte array form (note this is hexadecimal, not binary)
    - convert the byte array to an unsigned integer
    - print the unsigned integer value

Instructions
- Run the program by typing `python3 lesson2b.py` in a terminal command line.
- Study the program and its output. 
<br>

#### Lesson 2b Example Output
<br>

![Lesson 2b output](./images/lesson2b_output.png "python lesson 2b output")

<br>
