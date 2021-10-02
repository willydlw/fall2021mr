#include <Wire.h>
#include <ZumoShield.h>


void setup(ZumoIMU& imu)
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
      Serial.println(F("Failed to initialize IMU sensors."));
      delay(100);
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

void transmitAccelData(const ZumoIMU::vector<int16_t>& raw, const ZumoIMU::vector<float> scaled)
{
  char report[90];
  snprintf_P(report, sizeof(report),
    PSTR("A: %6d %6d %6d    A: %6.3f %6.3f %6.3f g"),
    raw.x, raw.y, raw.z,
    scaled.x, scaled.y, scaled.z);
  Serial.println(report);
}


int main(void)
{
  ZumoIMU imu;
  ZumoIMU::vector<int16_t> accelRawData = {0,0,0};
  ZumoIMU::vector<float> accelGData = {0.0f,0.0f,0.0f};
  setup(imu);

  while(1){
    imu.readAcc();
    // copy data in case imu updates before transmit operations
    // are complete
    accelRawData.x = imu.a.x;
    accelRawData.y = imu.a.y;
    accelRawData.z = imu.a.z;
    convertAccelDataToG(accelRawData, accelGData);
    transmitAccelData(accelRawData, accelGData);
    delay(5000);
  }
  
}

 

void convertAccelDataToG(ZumoIMU::vector<int16_t> &rawData, 
                         ZumoIMU::vector<float> &scaleData)
{
  /* Assumes full-scale set to +/- 2G
   *  Data measurment resolution is 2 bytes, 16 bits
  *   Data type int16_t range [-32768, +32767] 
  *  
  *   0g to 2g maps to 0 to 32767
  *   (2g - 0g)/2 = 1g   (32767-0)/2 = 16383.5
  *   
  *   Thus, 16384 is approximately 1g
  *
  */
}
