/* Purpose: Transmit accel, gyro measurements
 *  
 *  Configuration - ZumoIMU::enableDefault()
 *    Accelerometer: 
 *      Sets data aquistion rate to 50 Hz
 *      enables all axes x,y,z
 *      
 *    Gyroscope:
 *      Sets data aquistion rate to 189.4 HZ
 *      enables all axes x,y,z
 *           
 *    Serial - 57600 baud
 *      
 */
#include <Wire.h>
#include <ZumoIMU.h>
#include <Arduino.h>

// Global Constants
const unsigned long SENSOR_SAMPLE_TIME_INTERVAL = 100;   // milliseconds


// call this mysetup to avoid confusion with Arduino setup function
void mysetup(ZumoIMU &imu)
{
  // Transmit error messages, data
  Serial.begin(9600);

  // Initialize the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initialize IMU
  if (!imu.init())
  {
    // Failed to detect the sensors and communicate via I2C
    while(1)
    {
      uint32_t msg = 0xDEADBEEF;
      Serial.write((byte*)&msg, 4);
      delay(500);
    }
  }

  /*  Accelerometer: 
   *   Sets data aquistion rate to 50 Hz
   *   enables all axes x,y,z
   *   
   *  Magnetometer:
   *   Sets data aquisition rate to 6.25 Hz
   *   enables all axes x,y,z
   *   
   *  Gyroscope:
   *    Sets data aquistion rate to 189.4 HZ
   *    enables all axes x,y,z
   */
  imu.enableDefault();
 
}

void testSerialSignedWrite(void)
{
      if(Serial.availableForWrite() > 15)
      {
        int16_t ax, ay, az, gx, gy, gz;
        int16_t buffer[6];
        
        ax = -32768, ay = 32767, az = 16384;
        gx = -256, gy = 257, gz = -1024;
        buffer[0] = ax;
        buffer[1] = ay;
        buffer[2] = az;
        buffer[3] = gx;
        buffer[4] = gy;
        buffer[5] = gz;
        
        Serial.write((byte*)buffer, 12);
     

      } 
}

void testSerialUnsignedWrite(void)
{
  if(Serial.availableForWrite() > 15)
      {
        unsigned int ax, ay, az, gx, gy, gz;
        unsigned int buffer[6];
        
        ax = 0x0123, ay = 0x4567, az = 0x89AB;
        gx = 0xCDEF, gy = 0xDEAD, gz = 0xBEEF;
        buffer[0] = ax;
        buffer[1] = ay;
        buffer[2] = az;
        buffer[3] = gx;
        buffer[4] = gy;
        buffer[5] = gz;
        
        Serial.write((byte*)buffer, 12);
     
      } 
}


int main(void)
{
  ZumoIMU imu;
  
  unsigned long sensorTime;
  

  int16_t buffer[6];

 
  

  /* Arduino init function configures
   *  timers
   *  analog to digital
   *  The bootloader connects pins 0 and 1 to the USART; 
   *  this function disconnects them so they can be used as normal 
   *  digital i/o; they will be reconnected in Serial.begin()
   */
  init();
  
  mysetup(imu);

  sensorTime = millis();
 

  while(1)
  {
    
    if((millis() - sensorTime) >= SENSOR_SAMPLE_TIME_INTERVAL)
    {
      //testSerialUnsignedWrite();
     
      // assuming data is ready to be read, throwing away magnetometer data
      imu.read();
      
      sensorTime = millis();

      if(Serial.availableForWrite() > 11)
      {
        buffer[0] = imu.a.x;
        buffer[1] = imu.a.y;
        buffer[2] = imu.a.z;
        buffer[3] = imu.g.x;
        buffer[4] = imu.g.y;
        buffer[5] = imu.g.z;
        Serial.write((byte*)buffer, 12);
      }
    } 
  }

  return 0;
}

 
