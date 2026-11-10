# preintegration_core 设计

## 目录

- [Data contract](#data-contract)
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

`PreintegratedDelta` 表示 `b0` 与 `bk` 之间的相对测量：

```text
delta_q_b0_bk, maps bk coordinates into b0
delta_v_mps, expressed in b0
delta_p_m,   expressed in b0
duration_s
```

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

该函数只用于无图状态下查看纯积分结果。它不计算地球自转、运输率或哥氏项。

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
- 圆锥/划桨开关一致性；
- 与 GTSAM/OB 预积分在无地球项的短区间回归；
- 不同采样率下相对增量的收敛性。
