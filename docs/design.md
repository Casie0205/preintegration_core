# preintegration_core 设计

## 目录

- [Data contract](#data-contract)
- [Configuration](#configuration)
- [Integration sequence](#integration-sequence)
- [Coning and sculling](#coning-and-sculling)
- [State composition](#state-composition)
- [Intentional omissions](#intentional-omissions)
- [Validation plan](#validation-plan)

本文定义纯 IMU 相对积分的状态、输入和数据契约。

## Data contract

`ImuIncrement` 表示一个区间末端的机体系增量：

```text
dt_s
delta_angle_rad
delta_velocity_mps
```

零偏、尺度因子和杆臂必须由调用方预先处理。库不保存原始 IMU 序列，也不重新积分历史零偏。

如果外部求解器从第二帧开始积分，可用 `seedPreviousIncrement()` 写入第一帧历史锚点。
该操作不累计时间、样本数或相对增量。

`PreintegratedDelta` 表示 `b0` 与 `bk` 之间的相对测量：

```text
delta_q_b0_bk, maps bk coordinates into b0
delta_v_mps, expressed in b0
delta_p_m,   expressed in b0
duration_s
```

## Configuration

`RuntimeConfig` 与 `inertial_core` 使用相同字段名和默认值：

```text
enable_earth_rotation
enable_transport_rate
enable_coriolis
enable_ellipsoid_radii
gravity_model
earth_rotation_rate_radps
constant_gravity_mps2
semi_major_axis_m
eccentricity_squared
fallback_sphere_radius_m
enable_coning_sculling
equal_interval_tolerance_s
```

相对积分递推使用 `enable_coning_sculling` 和 `equal_interval_tolerance_s`。
`composeWithEnvironment()` 使用地球旋转、运输率和哥氏开关处理外部提供的地球向量。

## Integration sequence

每帧执行：

1. 校验 `dt` 和有限值；
2. 计算圆锥、划桨交叉项；
3. 用更新前的 `delta_q` 将速度增量旋转到 `b0`；
4. 更新相对位置和速度；
5. 用旋转矢量更新相对姿态；
6. 缓存当前增量。

## Coning and sculling

当前实现保留相邻两帧的二阶修正：

```text
dv = dv_k + 0.5 * theta_k x dv_k
     + 1/12 * (theta_(k-1) x dv_k + dv_(k-1) x theta_k)

dtheta = theta_k + 1/12 * theta_(k-1) x theta_k
```

`0.5 * theta x dv` 每帧都生效。两个 `1/12` 项使用等采样间隔推导，因此默认要求相邻
`dt` 差值不超过 `equal_interval_tolerance_s`。设置 `enable_coning_sculling=false`
可退化为只使用当前帧增量，便于隔离积分差异。

## State composition

`compose()` 使用以下标准关系：

```text
R_j = R_i * DeltaR
v_j = v_i + g * dt + R_i * DeltaV
p_j = p_i + v_i * dt + 0.5 * g * dt^2 + R_i * DeltaP
```

该函数只用于无图状态下查看纯积分结果，不计算地球项。

`composeWithEnvironment()` 在地球开关开启时，额外应用导航系旋转、运输率和哥氏项：

```text
q_nn = Exp(-(omega_ie + omega_en) * dt)
q_bn_j = q_nn * q_bn_i * DeltaR
a_nav = gravity - (2 * omega_ie + omega_en) x velocity
v_j = v_i + a_nav * dt + R_i * DeltaV
p_j = p_i + v_i * dt + 0.5 * a_nav * dt^2 + R_i * DeltaP
```

地球向量由调用方或对比驱动程序提供，库本身不重新实现 WGS84 机械编排。
`makeEarthEnvironment()` 提供短区间局部近似，便于使用与 `inertial_core` 相同的配置字段；
长距离、高纬度或高动态场景仍应使用 `inertial_core` 的完整地球模型。

## Intentional omissions

以下内容被明确排除，防止库重新演化为求解器：

- GTSAM/Ceres 类型；
- GNSS 残差；
- 零偏随机游走；
- 预积分协方差和雅可比；
- 固定滞后窗口和边缘化；
- 初始对准。

地球模型与机械编排由 `inertial_core` 负责，两个库通过输入/输出数据而不是框架状态耦合。

## Validation plan

- 常值加速度解析解；
- 常值角速度解析解；
- 复合旋转/平移下的 `compose()` 坐标一致性；
- 圆锥/划桨开关一致性和首帧 sculling 项；
- 独立细步参考积分，对多轴旋转和三维比力逐项对照；
- 非法 `dt`、非有限输入、非等间隔和非法组合状态拒绝；
- GTSAM `PreintegratedCombinedMeasurements` 的增量与 `predict()` 外部回归；
- OB_GINS `PreintegrationNormal` 的同离散公式外部回归。

参考积分测试位于 `tests/test_preintegration_reference.cpp`，每个采样区间细分 200 次，
独立计算 `delta_q`、`delta_v` 和 `delta_p`。

GTSAM 回归位于 `tests/test_preintegration_gtsam.cpp`，把同一分段常值增量细分 100 次后
同时送入两套实现。OB_GINS 回归位于 `tests/test_preintegration_ob_gins.cpp`，使用
`seedPreviousIncrement()` 对齐首帧，再比较相同的二阶离散递推。
