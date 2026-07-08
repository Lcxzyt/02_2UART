#include "StreamOutput.h"
#include "Bluetooth.h"
#include "CmdDispatch.h"
#include "IMUTest.h"
#include "Motor.h"
#include "Serial.h"
#include <stdbool.h>

void StreamOutput_PrintVofa(void)
{
    if (g_StreamTarget == STREAM_TARGET_BLUETOOTH) {
        Bluetooth_Printf("SPD,%d,%d,%d,%d\r\n",
                         (int)Motor_GetActual_L(),
                         (int)Motor_GetActual_R(),
                         (int)Motor_GetPwm_L(),
                         (int)Motor_GetPwm_R());
    } else {
        Serial_Printf("SPD,%d,%d,%d,%d\r\n",
                      (int)Motor_GetActual_L(),
                      (int)Motor_GetActual_R(),
                      (int)Motor_GetPwm_L(),
                      (int)Motor_GetPwm_R());
    }
}

void StreamOutput_PrintImu(void)
{
    IMUTest_Data imu;
    bool ok;

    ok = IMUTest_Read(&imu);
    if (g_StreamTarget == STREAM_TARGET_BLUETOOTH) {
        Bluetooth_Printf("IMU,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                         ok ? 1 : 0,
                         (int)imu.AccelX, (int)imu.AccelY, (int)imu.AccelZ,
                         (int)imu.GyroX,  (int)imu.GyroY,  (int)imu.GyroZ,
                         (int)imu.MagX,   (int)imu.MagY,   (int)imu.MagZ,
                         (int)imu.RollDeg, (int)imu.PitchDeg, (int)imu.YawDeg);
    } else {
        Serial_Printf("IMU,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                      ok ? 1 : 0,
                      (int)imu.AccelX, (int)imu.AccelY, (int)imu.AccelZ,
                      (int)imu.GyroX,  (int)imu.GyroY,  (int)imu.GyroZ,
                      (int)imu.MagX,   (int)imu.MagY,   (int)imu.MagZ,
                      (int)imu.RollDeg, (int)imu.PitchDeg, (int)imu.YawDeg);
    }
}
