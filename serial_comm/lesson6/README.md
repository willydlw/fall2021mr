# Lesson 6 - Arduino Serial.readBytes()

Description

python program [lesson6.py](lesson6.py) converts an integer value to a byte array, uses the to_bytes() method. The value 512 is converted to a bytes array of 2 bytes, big endian. The bytes array is then serially transmitted from python.

```
value = 512
value_b = value.to_bytes(2, 'big')
ser.write(value_b)
```

Arduino program

The Arduino program uses the readBytes function to read 2 bytes, storing each byte in the array data. The readBytes function returns the number of bytes read and stores it in numRead. When numRead is 2, the built-in led blinks to provide the user confirmation that the correct number of bytes were read.

```
byte data[2];
byte numRead;
numRead = Serial.readBytes(data,2);
if(numRead == 2)
{
    blink(500);
    int val = data[0] << 8 | data[1];
    if(val == 512)
        Serial.println(val);
    else
        Serial.println("harrumph");
}
```

The bytes are then converted to a two-byte integer: `int val = data[0] << 8 | data[1];`

The value 512 is echoed back to python or the message harrumph is echoed back when the value is not 512. Change this test condition to a number other than 512 to see the harrumph message.

## Run the programs

Upload the Arduino program.
Run the python program.
Use CTRL+C to stop the python program.

