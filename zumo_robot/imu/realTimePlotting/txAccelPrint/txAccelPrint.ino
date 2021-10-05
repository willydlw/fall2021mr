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
 *    Serial - 9600 baud
 *      Transmits using print, println
 *      Formatted to work with Arduino Serial plotter
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
      Serial.println(F("imu failed to initialize"));
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


/* Format will work with Arduino Serial Plotter */
void sendData(int16_t x, int16_t y, int16_t z)
{
  Serial.print(x);
  Serial.print(","); 
  Serial.print(y);
  Serial.print(","); 
  Serial.println(z);
}


int main(void)
{
  ZumoIMU imu;
  unsigned long sensorTime;
  
  /* Arduino init function configures
   *  timers
   *  analog to digital
   *  The bootloader connects pins 0 and 1 to the USART; 
   *  this function disconnects them so they can be used as normal 
   *  digital i/o; they will be reconnected in Serial.begin()
   */
  init();
  
  /* Start I2C, Serial, configure imu */
  mysetup(imu);

  sensorTime = millis();
 
  while(1)
  {
    if((millis() - sensorTime) >= SENSOR_SAMPLE_TIME_INTERVAL)
    {
      // assuming data is ready to be read
      imu.readAcc();
      sendData(imu.a.x, imu.a.y, imu.a.z);
      sensorTime = millis();
    } 
  }

  return 0;
}

 
