// Time serial println, write functions

#include <Arduino.h>


uint32_t timePrintln(unsigned int count, unsigned long& transTime)
{
  unsigned long startTime, stopTime;
  size_t bytesSent;             // sizeof(size_t) is 2 bytes
  uint32_t totalBytesSent = 0;
  
  unsigned int i;
  startTime = millis();
  for(i = 0; i < count; ++i)
  {
    bytesSent = Serial.println(i);
    totalBytesSent = totalBytesSent + (uint32_t)bytesSent;
  }

  // ensure all bytes have been transmitted
  if(Serial.availableForWrite() < 64)
  {
    // Waits for the transmission of outgoing serial data to complete. 
    Serial.flush();
  }
  
  stopTime = millis();
  transTime =  stopTime - startTime;
  return totalBytesSent;
}


uint32_t timeWrite(unsigned int count, unsigned long& transTime)
{
  unsigned long startTime, stopTime;
  size_t bytesSent;             
  uint32_t totalBytesSent = 0;

  // used to compare to int type Serial.availableForWrite() returns
  const int sizeofUInt = (int)sizeof(unsigned int);
  
  unsigned int i;
  startTime = millis();
  for(i = 0; i < count; ++i)
  {
    while(Serial.availableForWrite() < sizeofUInt)
    {
      // wait
      ;   // no op takes one cpu cycle
    }
    
    bytesSent = Serial.write((byte*)&i, sizeofUInt);
    totalBytesSent = totalBytesSent + (uint32_t)bytesSent;
  }

  // ensure all bytes have been transmitted
  if(Serial.availableForWrite() < 64)
  {
    // Waits for the transmission of outgoing serial data to complete. 
    Serial.flush();
  }
  
  stopTime = millis();
  transTime =  stopTime - startTime;
  return totalBytesSent;
}


int main(void)
{
  unsigned long elapsedPrintTime = 0;
  unsigned long elapsedWriteTime = 0;
  
  uint32_t totalPrintBytes = 0;
  uint32_t totalWriteBytes = 0;
  
  unsigned int countLimit = 65535;
 
  init();
  Serial.begin(57600);
  
  // clear out any old data in the transmit buffer
  Serial.flush();

  totalPrintBytes = timePrintln(countLimit, elapsedPrintTime);
  totalWriteBytes = timeWrite(countLimit, elapsedWriteTime);

  while(1){
    Serial.print("\ntotal Print Bytes: ");
    Serial.println(totalPrintBytes);
    Serial.print("elapsed print time [ms]: ");
    Serial.println(elapsedPrintTime);
    Serial.print("total write Bytes: ");
    Serial.println(totalWriteBytes);
    Serial.print("elapsed write time [ms]: ");
    Serial.println(elapsedWriteTime);
    Serial.flush();
    delay(5000);
    
  }
}
