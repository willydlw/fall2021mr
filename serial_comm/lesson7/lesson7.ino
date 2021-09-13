/* Purpose: experiment with Serial.readBytesUntil
 *  
 * Use the Serial monitor program to send different 
 * messages. 
 * 
 * 
 */
char messageBuffer[5];
size_t bytesReceived;
bool messageReceived;

void setup()
{
  Serial.begin(9600);
  messageReceived = false;
}

void loop()
{
  // will read until the \n is encountered,
  // or until 4 bytes are read
  // or until it times out. (default timeout is 1 second)
  bytesReceived = Serial.readBytesUntil('\n', messageBuffer, 4);
  if(bytesReceived > 0)
  {
    Serial.print("message received: ");
    Serial.print(messageBuffer);
  }

  Serial.print("bytesReceived: ");
  Serial.println(bytesReceived);
  
}
