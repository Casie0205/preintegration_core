# preintegration_core

## 目录

- [Purpose](#purpose)
- [Boundary](#boundary)
- [Build](#build)
- [API](#api)
- [Conventions](#conventions)
- [Validation](#validation)
- [References](#references)
- [License](#license)

最小纯 IMU 预积分库，只产生相对 `DeltaR/DeltaV/DeltaP`，不包含任何 GNSS 因子、
平滑器、滑窗或优化器。

## Purpose

库接收已经完成时间对齐和零偏补偿的 IMU 增量，输出两个关键帧之间的：

```text
delta_q_b0_bk  relative body attitude, maps bk coordinates into b0
delta_v_mps    relative velocity, expressed in b0
delta_p_m      relative position, expressed in b0
duration_s     integrated duration
```

这不是因子图节点、残差、协方差或优化问题。

## Boundary

`preintegration_core` 明确不负责：

- GNSS 观测和因子；
- 零偏状态估计；
- IMU 噪声协方差；
- 图结构、边缘化、固定滞后平滑；
- 地球自转、运输率和哥氏项；
- 时间插值。

调用方必须先完成时间对齐和零偏补偿。若需要地球机械编排，应使用独立的
`inertial_core`，不要将图优化接口混入本库。

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
ctest --test-dir build --output-on-failure
```

依赖只有 C++17 和 Eigen3。

## API

```cpp
#include "preintegration_core/preintegrator.hpp"

preintegration_core::Preintegrator integrator;
integrator.add(bias_free_increment);

const auto& delta = integrator.delta();
```

可选的 `compose()` 只处理常值重力，不包含地球自转或哥氏项：

```cpp
const auto next = preintegration_core::compose(initial, delta, gravity_ned);
```

## Conventions

- IMU 增量基于 FRD 机体系；
- `delta_q_b0_bk` 把当前体坐标系 `bk` 向量旋转到初始体坐标系 `b0`；
- `delta_v_mps` 和 `delta_p_m` 位于 `b0`；
- `compose()` 中的 `NavState.q_bn` 为 body-to-NED；
- `0.5 * theta x dv` 每帧都计算；
- 两个 `1/12` 圆锥/划桨交叉项要求相邻 `dt` 相等，默认容差为 `1e-9 s`。

完整接口约定见 [docs/design.md](docs/design.md)。

## Validation

当前测试覆盖：

- 常值比力到速度和位置；
- 常值角速度到姿态增量；
- 时间累计和样本计数；
- 无图状态组合中的重力、姿态顺序和北/东坐标旋转；
- 首帧 sculling 项和关闭圆锥/划桨后的退化行为；
- 独立 200 倍细步参考积分，对照多轴旋转和三维比力；
- `clear()` 状态复位、四元数归一化和非法输入拒绝。

测试分为 `preintegration_core_tests` 与 `preintegration_core_reference_tests`
两个 CTest 目标。参考测试位于
[`tests/test_preintegration_reference.cpp`](tests/test_preintegration_reference.cpp)，
使用 200 倍细步独立积分，不调用库内预积分递推。

## References

- `D:/Code/OB_GINS/src/preintegration`
- `D:/Code/GNSS_INS_FGO/src/model/ImuModel.cpp`
- `D:/Code/GraphRTK-INS/src/LibGREAT/gfactor/gintegration_base.h`

## License

本库基于公开 GNSS/INS 预积分方法整理和重写，发布为 GPL-3.0。
