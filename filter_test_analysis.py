#!/usr/bin/env python3
"""滤波效果分析"""

# 分析静止状态下的数据（最后稳定的40个样本）
stable_data = """
1,2122,840,16330,-45,-30,-10,-251,975,-2397,3,-7,132
1,2127,842,16336,-45,-30,-11,-251,975,-2397,3,-7,132
1,2132,831,16335,-45,-31,-10,-250,976,-2397,3,-7,131
1,2129,836,16335,-45,-30,-10,-249,976,-2399,3,-7,131
1,2127,832,16332,-44,-30,-10,-251,975,-2398,3,-7,131
1,2127,825,16329,-45,-30,-10,-251,974,-2398,3,-7,132
1,2128,827,16336,-45,-30,-10,-251,974,-2399,3,-7,132
1,2134,833,16335,-44,-30,-10,-251,974,-2398,3,-7,132
1,2137,834,16333,-45,-30,-10,-250,974,-2397,3,-7,132
1,2136,835,16334,-48,-30,-10,-249,974,-2397,3,-7,132
"""

# 提取数据
import statistics

ax_vals, ay_vals, az_vals = [], [], []
gx_vals, gy_vals, gz_vals = [], [], []
mx_vals, my_vals, mz_vals = [], [], []
roll_vals, pitch_vals, yaw_vals = [], [], []

for line in stable_data.strip().split('\n'):
    if not line.strip():
        continue
    parts = line.split(',')
    if len(parts) >= 13:
        ax_vals.append(int(parts[1]))
        ay_vals.append(int(parts[2]))
        az_vals.append(int(parts[3]))
        gx_vals.append(int(parts[4]))
        gy_vals.append(int(parts[5]))
        gz_vals.append(int(parts[6]))
        mx_vals.append(int(parts[7]))
        my_vals.append(int(parts[8]))
        mz_vals.append(int(parts[9]))
        roll_vals.append(int(parts[10]))
        pitch_vals.append(int(parts[11]))
        yaw_vals.append(int(parts[12]))

def analyze(name, vals):
    min_v = min(vals)
    max_v = max(vals)
    avg_v = statistics.mean(vals)
    std_v = statistics.stdev(vals) if len(vals) > 1 else 0
    jitter = (max_v - min_v) / 2.0
    return {
        'name': name,
        'min': min_v,
        'max': max_v,
        'avg': avg_v,
        'std': std_v,
        'range': max_v - min_v,
        'jitter': jitter
    }

print("=" * 80)
print("滤波效果分析 (静止状态，最后10个稳定样本)")
print("=" * 80)
print()

# 加速度计
ax = analyze('AX', ax_vals)
ay = analyze('AY', ay_vals)
az = analyze('AZ', az_vals)
print("加速度计:")
print(f"  AX: {ax['min']:5d} ~ {ax['max']:5d}  范围={ax['range']:3d}  抖动=±{ax['jitter']:.1f}")
print(f"  AY: {ay['min']:5d} ~ {ay['max']:5d}  范围={ay['range']:3d}  抖动=±{ay['jitter']:.1f}")
print(f"  AZ: {az['min']:5d} ~ {az['max']:5d}  范围={az['range']:3d}  抖动=±{az['jitter']:.1f}")
print()

# 陀螺仪
gx = analyze('GX', gx_vals)
gy = analyze('GY', gy_vals)
gz = analyze('GZ', gz_vals)
print("陀螺仪:")
print(f"  GX: {gx['min']:3d} ~ {gx['max']:3d}  范围={gx['range']:2d}  抖动=±{gx['jitter']:.1f}")
print(f"  GY: {gy['min']:3d} ~ {gy['max']:3d}  范围={gy['range']:2d}  抖动=±{gy['jitter']:.1f}")
print(f"  GZ: {gz['min']:3d} ~ {gz['max']:3d}  范围={gz['range']:2d}  抖动=±{gz['jitter']:.1f}")
print()

# 磁力计
mx = analyze('MX', mx_vals)
my = analyze('MY', my_vals)
mz = analyze('MZ', mz_vals)
print("磁力计:")
print(f"  MX: {mx['min']:4d} ~ {mx['max']:4d}  范围={mx['range']:2d}  抖动=±{mx['jitter']:.1f}")
print(f"  MY: {my['min']:4d} ~ {my['max']:4d}  范围={my['range']:2d}  抖动=±{my['jitter']:.1f}")
print(f"  MZ: {mz['min']:5d} ~ {mz['max']:5d}  范围={mz['range']:2d}  抖动=±{mz['jitter']:.1f}")
print()

# 姿态角
roll = analyze('Roll', roll_vals)
pitch = analyze('Pitch', pitch_vals)
yaw = analyze('Yaw', yaw_vals)
print("姿态角:")
print(f"  Roll:  {roll['min']:3d}° ~ {roll['max']:3d}°  范围={roll['range']:2d}°  抖动=±{roll['jitter']:.1f}°")
print(f"  Pitch: {pitch['min']:3d}° ~ {pitch['max']:3d}°  范围={pitch['range']:2d}°  抖动=±{pitch['jitter']:.1f}°")
print(f"  Yaw:   {yaw['min']:3d}° ~ {yaw['max']:3d}°  范围={yaw['range']:2d}°  抖动=±{yaw['jitter']:.1f}°")
print()

print("=" * 80)
print("对比结果")
print("=" * 80)
print()
print("滤波前 (用户之前提供的数据):")
print("  MX抖动: ±8")
print("  MY抖动: ±8")
print("  Yaw抖动: ±2°")
print()
print("滤波后 (当前数据):")
print(f"  MX抖动: ±{mx['jitter']:.1f}  改善={(8 - mx['jitter'])/8*100:.0f}%")
print(f"  MY抖动: ±{my['jitter']:.1f}  改善={(8 - my['jitter'])/8*100:.0f}%")
print(f"  Yaw抖动: ±{yaw['jitter']:.1f}°  改善={(2 - yaw['jitter'])/2*100:.0f}%")
print()

if yaw['jitter'] <= 0.5:
    print("✅ 优秀! Yaw抖动 ≤ ±0.5°，滤波效果非常好")
elif yaw['jitter'] <= 1.0:
    print("✅ 良好! Yaw抖动 ≤ ±1°，滤波效果明显")
elif yaw['jitter'] <= 1.5:
    print("⚠ 一般。Yaw抖动 ≤ ±1.5°，可考虑调整滤波系数")
else:
    print("⚠ 需要优化。Yaw抖动仍然较大")

print()
print("=" * 80)
print("建议")
print("=" * 80)

if mx['jitter'] > 2.0 or my['jitter'] > 2.0:
    print("磁力计抖动仍然较大，建议:")
    print("  - 降低 FILTER_ALPHA_MAG 从 0.2 到 0.15 (更强滤波)")
    print("  - 检查环境是否有强磁场干扰")
elif yaw['jitter'] > 1.0:
    print("Yaw角抖动仍然较大，建议:")
    print("  - 降低 FILTER_ALPHA_ANGLE 从 0.3 到 0.2 (更强滤波)")
    print("  - 降低 FILTER_ALPHA_MAG 从 0.2 到 0.15")
else:
    print("✅ 滤波效果已经很好!")
    print("  - 当前设置适合大多数应用")
    print("  - 如需更快响应，可适当提高滤波系数")
    print("  - 如需更稳定，可适当降低滤波系数")

print("=" * 80)
