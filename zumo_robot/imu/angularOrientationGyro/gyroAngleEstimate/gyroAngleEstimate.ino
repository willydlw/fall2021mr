/* Purpose: Transmit accel measurements
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

//#define PRINT_POSITION
#define PLOT_POSITION             // turn on for serial plotter


// Global Constants
const unsigned long SENSOR_SAMPLE_TIME_INTERVAL = 50;   // milliseconds


// call this mysetup to avoid confusion with Arduino setup function
void mysetup(ZumoIMU &imu)
{
  // Transmit error messages, data
  Serial.begin(57600);

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

void printAngleData(ZumoIMU::vector<double> data, int decimalPlaces)
{
  Serial.print("x: \t");   Serial.print(data.x, decimalPlaces);
  Serial.print("\ty: \t"); Serial.print(data.y, decimalPlaces);
  Serial.print("\tz: \t"); Serial.println(data.z, decimalPlaces);
}


int main(void)
{
  ZumoIMU imu;
  ZumoIMU::vector<double> deltaPosition = {0.0,0.0,0.0};
  ZumoIMU::vector<double> gyroPosition = {0.0,0.0,0.0};
  unsigned long sensorTime, elapsedTimems;
  unsigned int sampleCount = 0;
  
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
    if((elapsedTimems = (millis() - sensorTime)) >= SENSOR_SAMPLE_TIME_INTERVAL)
    {
      // assuming data is ready to be read
      imu.readGyro();
      sensorTime = millis();
      ++sampleCount;
      
      #ifdef DEBUG
        Serial.println("before offset");
        sendData(imu.g.x, imu.g.y, imu.g.z);
      #endif

      /* This is where bias offset values should 
       *  be applied. Ex: imu.g.x += offset.x
       */
      
      // calculate the change in position 
      // scale the data to deg/s 
      //deltaPosition.x = (double)imu.g.x / 134.0 * elapsedTimems / 1000.0;
      //deltaPosition.y = (double)imu.g.y / 134.0 * elapsedTimems / 1000.0;
      //deltaPosition.z = (double)imu.g.z / 134.0 * elapsedTimems / 1000.0;

      // 8.75 mdps/digit
      deltaPosition.x = (double)imu.g.x * (8.75/1000.0) * elapsedTimems / 1000.0;
      deltaPosition.y = (double)imu.g.y * (8.75/1000.0) * elapsedTimems / 1000.0;
      deltaPosition.z = (double)imu.g.z * (8.75/1000.0) * elapsedTimems / 1000.0;

      // update angular position
      gyroPosition.x += deltaPosition.x;
      gyroPosition.y += deltaPosition.y;
      gyroPosition.z += deltaPosition.z;


      #ifdef PRINT_POSITION
        if( sampleCount % 100 == 0 )
        {
          Serial.println("gyro position, deg");
          printAngleData(&gyroPosition,4);
          sampleCount = 0;
        }
      #endif
      #ifdef PLOT_POSITION
        Serial.print(gyroPosition.x);
        Serial.print(",");
        Serial.print(gyroPosition.y);
        Serial.print(",");
        Serial.println(gyroPosition.z);
      #endif
    } 
  }

  return 0;
}

 
