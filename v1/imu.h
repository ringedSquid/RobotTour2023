#ifndef imu_h
#define imu_h

#include <REG.h>
#include <wit_c_sdk.h>
#include <Arduino.h>

#define ACC_UPDATE    0x01
#define GYRO_UPDATE   0x02
#define ANGLE_UPDATE  0x04
#define MAG_UPDATE    0x08
#define READ_UPDATE   0x80 

class IMU {
  private:
    static void AutoScanSensor();
    static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t ucMs);

    uint16_t delayInterval;
    uint32_t baudRate;
    
  public:
    //Can be used to store vel, pos, rot, etc
    struct matrix3d{
      double x;
      double y;
      double z;
    }

    IMU(uint16_t iDelayInterval, uint32_t iBaudRate);

    void init();

    matrix3d getAcc();
    matrix3d getHead();
    matrix3d getAngVel();
    
   
    
}




#endif
