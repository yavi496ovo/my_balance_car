# 硬件接线手册和 PID 结构说明

本文档按当前工程源码整理，适用于 `car_balance_32` 的 STM32F407 平衡车工程。

## 1. 接线总表

### 1.1 电机驱动

当前代码按双路 H 桥驱动设计，例如 TB6612FNG、DRV8833 同类模块。不同模块的引脚名可能不同，按功能对应即可。

| 功能 | STM32 引脚 | 外设/驱动端 | 代码位置 | 说明 |
| --- | --- | --- | --- | --- |
| 左电机 PWM | PA6 / TIM3_CH1 | 左电机 PWM / PWMA | `BSP/motor.h` | PWM 范围由代码映射为 `-1000 ~ 1000` |
| 右电机 PWM | PA7 / TIM3_CH2 | 右电机 PWM / PWMB | `BSP/motor.h` | TIM3 ARR=8399，周期计数为 8400 |
| 左电机方向 1 | PB12 | AIN1 / 左 IN1 | `BSP/motor.h` | 正 PWM 时置高 |
| 左电机方向 2 | PB13 | AIN2 / 左 IN2 | `BSP/motor.h` | 正 PWM 时置低 |
| 右电机方向 1 | PB14 | BIN1 / 右 IN1 | `BSP/motor.h` | 正 PWM 时置高 |
| 右电机方向 2 | PB15 | BIN2 / 右 IN2 | `BSP/motor.h` | 正 PWM 时置低 |
| 驱动使能 | PE0 | STBY / EN | `BSP/motor.h` | `motor_enable(1)` 后置高 |
| 逻辑电源 | 3.3V 或模块要求电压 | VCC | 硬件 | 与驱动模块逻辑电平匹配 |
| 电机电源 | 电池正极 | VM / VIN | 硬件 | 按电机额定电压选择 |
| 公共地 | GND | GND | 硬件 | MCU、驱动、电池负极必须共地 |

电机输出逻辑：

| 代码 PWM | IN1 | IN2 | 含义 |
| --- | --- | --- | --- |
| `> 0` | 高 | 低 | 正转 |
| `< 0` | 低 | 高 | 反转 |
| `0` | 低 | 低 | 停止滑行 |
| `motor_brake()` | 高 | 高 | 刹车 |

如果某个轮子方向相反，优先修改 `APP/pid_app.c` 中的 `Left_Direction` 或 `Right_Direction` 为 `-1.0f`，不要先改接线。

### 1.2 编码器

| 功能 | STM32 引脚 | 外设端 | 代码位置 | 说明 |
| --- | --- | --- | --- | --- |
| 左编码器 A 相 | PA0 / TIM2_CH1 | 左编码器 A | `BSP/encoder.c` | GPIO 上拉，TIM2 编码器模式 |
| 左编码器 B 相 | PA1 / TIM2_CH2 | 左编码器 B | `BSP/encoder.c` | GPIO 上拉 |
| 右编码器 A 相 | PD12 / TIM4_CH1 | 右编码器 A | `BSP/encoder.c` | GPIO 上拉，TIM4 编码器模式 |
| 右编码器 B 相 | PD13 / TIM4_CH2 | 右编码器 B | `BSP/encoder.c` | GPIO 上拉 |
| 编码器电源 | 按编码器规格 | VCC | 硬件 | 优先使用 3.3V 输出信号 |
| 编码器地 | GND | GND | 硬件 | 与 MCU 共地 |

当前编码器软件极性：

| 轮子 | 定时器 | 软件符号 |
| --- | --- | --- |
| 左轮 | TIM2 | `ENCODER_LEFT_SIGN = -1.0f` |
| 右轮 | TIM4 | `ENCODER_RIGHT_SIGN = 1.0f` |

如果手动让车轮向前转时，速度反馈方向不符合预期，修改 `BSP/encoder.h` 中对应的 `ENCODER_*_SIGN`。

### 1.3 JY61P 姿态模块和串口

| 功能 | STM32 引脚 | JY61P / 串口端 | 代码位置 | 说明 |
| --- | --- | --- | --- | --- |
| USART1_TX | PA9 | JY61P RXD | `Core/Src/usart.c` | 上电会发送 JY61P 自动配置命令 |
| USART1_RX | PA10 | JY61P TXD | `Core/Src/usart.c` | 接收姿态数据 |
| 电源 | 3.3V 或模块要求电压 | VCC | 硬件 | 确认模块支持的供电范围 |
| 地 | GND | GND | 硬件 | 与 MCU 共地 |

串口参数为 `115200, 8N1`。代码会先尝试按 9600 发送配置，再切到 115200 发送配置，用于把 JY61P 恢复到 115200、100Hz、ACC/GYRO/ANGLE 输出。

当前控制只使用 JY61P 的角度和陀螺仪数据，默认控制轴为 Y 轴：

| 项目 | 当前设置 |
| --- | --- |
| 控制角度轴 | `JY61P_CONTROL_ANGLE_AXIS = JY61P_AXIS_Y` |
| 控制角速度轴 | `JY61P_CONTROL_GYRO_AXIS = JY61P_CONTROL_ANGLE_AXIS` |
| 角度低通 | `JY61P_ANGLE_LPF_ALPHA = 0.6f` |
| 角速度低通 | `JY61P_GYRO_LPF_ALPHA = 0.6f` |
| 数据超时 | `JY61P_DATA_TIMEOUT_MS = 100U` |

安装 JY61P 时，车身前后俯仰应主要反映到 Y 轴角度。如果方向相反，修改 `JY61P_CONTROL_ANGLE_SIGN` 或 `JY61P_CONTROL_GYRO_SIGN`。

### 1.4 VOFA 调试串口注意事项

工程当前也通过 USART1 发送 VOFA JustFloat 数据，周期 20 ms。因此 USART1 同时承担：

- 接收 JY61P 数据：PA10。
- 给 JY61P 发送初始化命令：PA9。
- 发送 VOFA 调试数据：PA9。

如果需要用 USB-TTL 看 VOFA，可以把 USB-TTL 的 RX 接到 PA9，GND 共地。不要把 USB-TTL 的 TX 与 JY61P TX 同时接到 PA10，否则两个发送端会争用同一个 MCU RX。

更稳妥的调试方式是后续增加第二路 UART，把 JY61P 和 VOFA 分开。

## 2. 上电前检查

1. 电机电源只接电机驱动的 VM/VIN，不要直接接 MCU 供电脚。
2. MCU、驱动、编码器、JY61P、USB-TTL 必须共地。
3. 所有进入 STM32 的信号优先使用 3.3V TTL 电平。
4. 第一次调 PID 时把车轮架空，确认电机方向和编码器方向后再落地。
5. JY61P 固定牢靠，车直立静止时角度应接近 `Med_Angle`。
6. 正常平衡时 `Speed_Polarity_Test` 必须为 `0U`。

## 3. 当前 PID 结构

当前平衡控制的核心在 `APP/pid_app.c`，调度周期为 5 ms。项目里虽然有通用 `components/PID/pid.c`，但当前平衡车控制链没有实例化 `PID_T`，而是在 `pid_app.c` 中直接写了速度环、直立环和转向环公式。

### 3.1 数据流

```text
JY61P angle/gyro  --->  直立环 PD  ----+
                                      +--> 左右电机 PWM
编码器 left/right --->  速度环 PI  ----+

Target_Turn ------->  转向环 PD  ------> 左右轮差速
```

实际执行顺序：

1. `jy61p_app_task()` 从 USART1 环形缓冲区解析 JY61P 数据，更新 `angle_deg` 和 `gyro_dps`。
2. `encoder_app_task()` 每 10 ms 读取 TIM2/TIM4 编码器计数，更新左右轮速度。
3. `pid_app_task()` 每 5 ms 计算速度环、直立环、转向环，并调用 `motor_set_output()`。
4. `motor.c` 把 `-1000 ~ 1000` 的逻辑 PWM 转成 TIM3 比较值和方向 GPIO。

### 3.2 速度外环

函数：`Velocity(Target_Speed, encoder_L, encoder_R)`

公式：

```text
Err = encoder_L + encoder_R - Target_Speed
Err_LowOut = (1 - Velocity_Filter) * Err + Velocity_Filter * last_Err_LowOut
speed_integral += Err_LowOut
speed_integral = limit(speed_integral, +/- Velocity_Integral_Limit)
Velocity_Ki = Velocity_Kp / 200
velocity_out = Velocity_Kp * Err_LowOut + Velocity_Ki * speed_integral
```

`velocity_out` 不是直接给电机的 PWM，而是作为直立环的目标角度修正量：

```text
target_angle_deg = Med_Angle + velocity_out
```

当前参数：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `Target_Speed` | `0.0f` | 目标速度，0 表示原地平衡 |
| `Velocity_Kp` | `0.00f` | 当前速度环实际关闭 |
| `Velocity_Ki` | `Velocity_Kp / 200` | 由代码自动计算 |
| `Velocity_Filter` | `0.5f` | 速度误差低通滤波系数 |
| `Velocity_Integral_Limit` | `20000.0f` | 积分限幅 |

因此当前版本主要是直立环在工作，速度外环还没有参与闭环控制。

### 3.3 直立内环

函数：`Vertical(target_angle_deg, Angle, gyro_Y)`

公式：

```text
vertical_out = Vertical_Kp * (Angle - target_angle_deg) + Vertical_Kd * gyro_Y
```

当前参数：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `Med_Angle` | `-1.62f` | 机械中值，车直立静止时的角度 |
| `Vertical_Kp` | `45.0f` | 角度比例项 |
| `Vertical_Kd` | `1.0f` | 角速度阻尼项 |

当 JY61P 未就绪时，代码使用 `angle = Med_Angle`、`gyro = 0`，避免上电时因无效姿态数据导致电机乱动。

### 3.4 转向环

函数：`Turn(gyro_Z, Target_Turn)`

公式：

```text
turn_out = Turn_Kp * Target_Turn + Turn_Kd * gyro_Z
turn_out = limit(turn_out, +/- Turn_Limit)
```

当前调用为：

```c
pid_app.turn_out = Turn(0.0f, Target_Turn);
```

也就是说当前没有把真实 Z 轴角速度传入转向环。并且参数也是关闭状态：

| 参数 | 当前值 |
| --- | --- |
| `Target_Turn` | `0.0f` |
| `Turn_Kp` | `0.0f` |
| `Turn_Kd` | `0.0f` |
| `Turn_Limit` | `500.0f` |

### 3.5 电机输出混合

公式：

```text
PWM_out = Output_Direction * vertical_out
left_raw = PWM_out - turn_out
right_raw = PWM_out + turn_out
left_raw/right_raw = limit(raw, +/- Output_Limit)
left_pwm = left_raw * Left_Direction
right_pwm = right_raw * Right_Direction
```

当前参数：

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `Output_Limit` | `900.0f` | 电机输出限幅 |
| `Output_Direction` | `1.0f` | 整体输出方向 |
| `Left_Direction` | `1.0f` | 左电机方向修正 |
| `Right_Direction` | `1.0f` | 右电机方向修正 |

最终 `Float_To_PWM()` 会再限制到 `MOTOR_PWM_MAX = 1000`，然后写入电机驱动。

### 3.6 调试数据

VOFA 每 20 ms 发送 12 个 float 通道：

| 通道 | 含义 |
| --- | --- |
| 0 | 当前角度 `angle_deg` |
| 1 | 目标角度 `target_angle_deg` |
| 2 | 速度环输出 `speed_angle_deg` |
| 3 | 角速度 `gyro_dps` |
| 4 | 直立环输出 `balance_pwm` |
| 5 | 转向环输出 `turn_pwm` |
| 6 | 左电机 PWM |
| 7 | 右电机 PWM |
| 8 | 目标速度 |
| 9 | 速度反馈 |
| 10 | 运行状态 |
| 11 | 故障状态 |

## 4. 调参顺序建议

1. 架空车轮，确认电机正 PWM 方向一致。
2. 手动转动车轮，确认左右编码器速度的正负方向一致。
3. 保持 `Velocity_Kp = 0`、`Turn_Kp = 0`，先调直立环 `Med_Angle`、`Vertical_Kp`、`Vertical_Kd`。
4. 直立稳定后再逐步增大 `Velocity_Kp`，让速度外环参与回正。
5. 最后再接入转向控制，并给转向环传入真实 Z 轴角速度。
