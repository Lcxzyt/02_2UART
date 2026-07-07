#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

/* ============================================================================
 * IMU 模块 — MPU6050 + QMC5883P 姿态解算与卡尔曼滤波
 *
 * 硬件层: MPU6050（6 轴 IMU）、QMC5883P（3 轴磁力计）
 * 传输层: MyI2C（软件 I2C, PB10=SCL, PB11=SDA）
 *
 * 职责:
 *   1. 封装传感器初始化、原始数据读取
 *   2. 原始值 → 物理量转换（°/s, m/s², μT）
 *   3. 加速度计 → roll/pitch
 *   4. 磁力计 → yaw（含倾斜补偿）
 *   5. 一维卡尔曼滤波 / 互补滤波（roll、pitch 轴独立）
 * ============================================================================ */

/* ========================== 数据结构 ========================== */

/** 原始传感器数据（ADC 计数） */
typedef struct {
    int16_t AccelX, AccelY, AccelZ;   // 加速度计原始值
    int16_t GyroX,  GyroY,  GyroZ;    // 陀螺仪原始值
    int16_t MagX,   MagY,   MagZ;     // 磁力计原始值
} IMU_RawData;

/** 标度转换后的物理量 */
typedef struct {
    float AccelX, AccelY, AccelZ;      // 加速度    m/s²
    float GyroX,  GyroY,  GyroZ;       // 角速度    °/s
    float MagX,   MagY,   MagZ;        // 磁场强度  μT
    float Temp;                        // 芯片温度  °C (MPU6050)
} IMU_ScaledData;

/** 欧拉角 */
typedef struct {
    float Roll;     // 横滚角  °  (绕 X 轴)
    float Pitch;    // 俯仰角  °  (绕 Y 轴)
    float Yaw;      // 偏航角  °  (绕 Z 轴, 0=北)
} IMU_Attitude;

/* ========================== 一维卡尔曼滤波器 ========================== */

/**
 * 状态向量: [angle, gyro_bias]^T
 * 预测: angle += (gyro_rate - bias) * dt
 * 校正: accelerometer angle → 修正 angle 与 bias
 */
typedef struct {
    float Q_angle;      // 过程噪声协方差（角度, 默认 0.001）
    float Q_bias;       // 过程噪声协方差（偏置, 默认 0.003）
    float R_measure;    // 测量噪声协方差（加速度计, 默认 0.03）

    float angle;        // 滤波器输出的角度估计值 (°)
    float bias;         // 陀螺仪偏置估计值 (°/s)
    float rate;         // 去除偏置后的角速度 (°/s)

    float P[2][2];      // 误差协方差矩阵
} IMU_Kalman1D;

/* ========================== 互补滤波器 ========================== */

typedef struct {
    float alpha;        // 互补系数 (0~1), 越大越信任陀螺仪, 默认 0.98
    float angle;        // 当前角度 (°)
    float last_time;    // 上一次更新时间戳（本实现使用 dt 参数，保留备用）
} IMU_Complementary;

/* ========================== 磁力计硬铁校准 ========================== */

/**
 * 硬铁校准：采集旋转过程中三轴 min/max，求中点作为偏移量
 *
 * 用法:
 *   1. IMU_MagCalibReset(&calib);
 *   2. 缓慢旋转传感器 360°（绕 Z 轴），每步调用 IMU_MagCalibUpdate(&calib, &raw);
 *   3. IMU_MagCalibApply(&calib);  → 计算 OffsetX/Y/Z
 *   4. 之后 IMU_ReadScaled() 自动减去偏移
 */
typedef struct {
    int16_t MinX, MaxX;      // X 轴采集范围
    int16_t MinY, MaxY;      // Y 轴采集范围
    int16_t MinZ, MaxZ;      // Z 轴采集范围
    float   OffsetX;         // 计算出的硬铁偏移 (μT)
    float   OffsetY;
    float   OffsetZ;
    uint8_t valid;           // 校准数据是否有效
} IMU_MagCalib;

/* ══════════════════════════════════════════
   函数声明
   ══════════════════════════════════════════ */

/* —————— 初始化与状态 —————— */

/**
 * IMU 初始化：MPU6050 唤醒 + 配置 + bypass, QMC5883P 配置
 * 前置条件：MyI2C_Init() 已调用
 * 返回值:  1 = 成功, 0 = 失败
 */
uint8_t IMU_Init(void);

/**
 * 检查磁力计数据是否就绪（轮询 QMC5883P SR 寄存器）
 * 返回值: 1 = 有新数据, 0 = 等待中
 */
uint8_t IMU_IsMagReady(void);

/**
 * 获取传感器型号信息（调试用）
 * buf: 输出缓冲区, len: 缓冲区长度
 */
void IMU_GetInfo(char *buf, uint16_t len);

/* —————— 数据采集 —————— */

/** 读取所有传感器原始 ADC 值 */
void IMU_ReadRaw(IMU_RawData *raw);

/** 读取并转换为物理量 */
void IMU_ReadScaled(IMU_ScaledData *sc);

/* —————— 角度解算（无滤波） —————— */

/**
 * 由加速度计计算横滚角和俯仰角
 * roll  = atan2(ay, az)
 * pitch = atan2(-ax, sqrt(ay² + az²))
 *
 * 注意：有运动加速度时角度不准，需要卡尔曼/互补滤波融合陀螺仪
 */
void IMU_AccelToAngles(const IMU_ScaledData *sc, float *roll, float *pitch);

/**
 * 由磁力计计算偏航角（带倾斜补偿）
 * roll, pitch: 当前横滚/俯仰角 (°)，用于将磁力计投影到水平面
 * 返回值: yaw (°)，0° = 磁北, 90° = 东
 */
float IMU_ComputeHeading(const IMU_ScaledData *sc, float roll, float pitch);

/**
 * 一次性计算全部欧拉角（roll/pitch 直接来自加速度计, yaw 来自磁力计倾斜补偿）
 */
void IMU_GetAttitudeRaw(IMU_ScaledData *sc, IMU_Attitude *att);

/* —————— 卡尔曼滤波 —————— */

/**
 * 初始化一维卡尔曼滤波器
 * dt:        采样周期 (s), 例如 0.01
 * Q_angle:   角度过程噪声, 典型值 0.001
 * Q_bias:    偏置过程噪声, 典型值 0.003
 * R_measure: 测量噪声,     典型值 0.03
 */
void IMU_KalmanInit(IMU_Kalman1D *kf, float Q_angle, float Q_bias, float R_measure);

/**
 * 卡尔曼滤波更新一步
 * measured_angle: 加速度计推算角度 (°)
 * gyro_rate:      陀螺仪角速度 (°/s)
 * dt:             本步时间间隔 (s)
 * 返回值: 滤波后的角度 (°)
 */
float IMU_KalmanUpdate(IMU_Kalman1D *kf, float measured_angle, float gyro_rate, float dt);

/* —————— 互补滤波 —————— */

/**
 * 初始化互补滤波器
 * alpha: 陀螺仪权重, 典型值 0.98 (高速运动) 或 0.95 (低速)
 */
void IMU_CompInit(IMU_Complementary *cf, float alpha);

/**
 * 互补滤波更新一步
 * accel_angle: 加速度计推算角度 (°)
 * gyro_rate:   陀螺仪角速度 (°/s)
 * dt:          本步时间间隔 (s)
 * 返回值: 滤波后的角度 (°)
 */
float IMU_CompUpdate(IMU_Complementary *cf, float accel_angle, float gyro_rate, float dt);

/* —————— 高层融合接口 —————— */

/**
 * 使用卡尔曼滤波获取 roll/pitch + 磁力计 yaw
 * kf_roll, kf_pitch: 卡尔曼滤波器实例指针（需提前初始化）
 * sc:                传感器物理量
 * att:               输出的融合姿态角
 */
void IMU_GetAttitudeKF(IMU_Kalman1D *kf_roll, IMU_Kalman1D *kf_pitch,
                       IMU_ScaledData *sc, IMU_Attitude *att, float dt);

/* —————— 磁力计硬铁校准 —————— */

/** 重置校准采集状态（开始新一轮 min/max 采集） */
void IMU_MagCalibReset(IMU_MagCalib *calib);

/** 每步喂入原始磁力计数据，更新 min/max */
void IMU_MagCalibUpdate(IMU_MagCalib *calib, const IMU_RawData *raw);

/** 根据采集的 min/max 计算偏移量并标为有效 */
void IMU_MagCalibApply(IMU_MagCalib *calib);

/** 手动设置硬铁偏移量 (μT)，跳过采集流程 */
void IMU_SetMagOffsets(float ox, float oy, float oz);

/** 获取当前硬铁偏移量 */
void IMU_GetMagOffsets(float *ox, float *oy, float *oz);

/* —————— 便捷接口（供上层直接获取 Yaw） —————— */

/**
 * IMU_GetYaw — 一键读取当前偏航角
 * 内部自动读取传感器、加速度计算 roll/pitch、磁力计倾斜补偿
 * 返回值: yaw (°)，0=磁北，90=东，逆时针 0~360
 * 耗时约 2ms（软件 I2C 读取两个传感器）
 */
float IMU_GetYaw(void);

#endif /* __IMU_H */
