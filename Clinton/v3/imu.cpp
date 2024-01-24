#include "REG.h"
#include "IMUsdk.h"
#include "imu.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

IMU::IMU(uint32_t iintervalus) {
  intervalus = iintervalus;
}

void IMU::init(Vector3d iPose, Vector3d iOrient) {
  orient = iOrient;
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  //set baud rate
  WitSetUartBaud(WIT_BAUD_115200);
  WitSetBandwidth(BANDWIDTH_256HZ);
  WitSetContent(RSW_ANGLE);
  //
  s_cDataUpdate = 0;
  oldus = micros();
}

void IMU::update() {
  if (micros() - oldus > intervalus) {
    while (Serial2.available()) {
      WitSerialDataIn(Serial2.read());
    }
    if (s_cDataUpdate) {
      orient(0) = sReg[Roll+0] / 32768.0f * 180.0f;
      orient(1) = sReg[Roll+1] / 32768.0f * 180.0f;
      orient(2) = sReg[Roll+2] / 32768.0f * 180.0f;
    }
  }
}

Vector3d IMU::getOrient() {
  Vector3d modifiedOrient = Vector3d(
                                     orient(0)-orientOffset(0),
                                     orient(1)-orientOffset(1),
                                     orient(2)-orientOffset(2)
                                     );
  return modifiedOrient;
}

Vector3d IMU::getOrientRaw() {
  return orient;
}
