#include <REG.h>
#include <wit_c_sdk.h>
#include <Arduino.h>

#include "imu.h"

IMU::IMU(uint16_t iDelayInterval, uint32_t iBaudRate) {
  delayInterval = iDelayInterval;
  baudRate = iBaudRate;
}

void IMU::init() {
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
}

static void AutoScanSensor();

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  Serial2.write(p_data, uiSize);
  Serial2.flush();
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  
}
static void Delayms(uint16_t ucMs);
