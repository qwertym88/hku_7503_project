> **硬件属性已纠正（2026-11-19）**：当前机构实际为180°位置舵机，1500µs会转到90°，不是停转。本目录保留为旧连续旋转假设下的历史调试工具，**不要用于当前机构归零/停止**。请烧录正常底盘并按[位置升降说明](../chassis/LIFTER.md)配置0°/80°。以下为旧工具说明。

# MG996R连续旋转舵机停转调试

用户已确认：**S1（GPIO8），MG996R 360°连续旋转舵机**。程序保留目录名 `servo_zero`，但这里的zero是零转速/停转，不是轴角0°。默认50Hz、1500µs，上电等待3秒后在S1持续发送；四路轮电机制动，不启动ROS或角度扫描。

**这种连续旋转舵机无法靠PWM命令回到指定角度。** 同样的1500µs会使轴在当前位置停转，不会找到机械零点，也不提供位置闭环保持力。具体停转脉宽可能因个体偏差需要微调。程序没有编码器/限位反馈，不能自动归位或保证抬升机构保持载荷。

## 安装应该如何进行

1. 先脱开舵盘/连杆与负载，断电按板卡丝印接线到S1。信号接S1、地接GND、电源按具体舵机规格；外供电时共地，避免电源回灌，USB能通信不证明供电电流足够。
2. 输出1500µs，观察是否静止；若有慢转，先在脱载条件下用1495或1505µs比较，向更接近静止的一侧按5µs步长微调，再以1µs细调。不同个体的方向对应关系以观察为准，不假设正反方向。
3. 真正的安装零位须以机构的机械标记/限位为准。支撑好机构，断电后把机构和舵盘对齐安装基准；不要强拧齿轮轴或用持续堵转找零。
4. 每次停转只表示速度接近0，不能把当前轴角当作“已回零”，也不能依赖这个脉冲承托抬升负载。需要可重复自动升降时，连续旋转方案须增加限位/编码器反馈；若机构按固定角度动作，需采用定位式舵机或其他位置闭环执行器。

定位式舵机与连续旋转舵机的1500µs含义不同。Yahboom官方示例的“角度0=中位”用于其角度模型，不能照搬成这颗连续舵机的机械0°。

## 编译与配置

仓库根目录：

```bash
bash firmware/servo_zero/idf.sh build
```

默认已是S1/1500µs。微调停转脉宽：

```bash
bash firmware/servo_zero/idf.sh menuconfig
```

进入 `Continuous servo neutral calibration`：

| 配置 | 默认 | 说明 |
|---|---|---|
| `SERVO_ZERO_PORT` | 1 | 用户实际使用S1/GPIO8；2=S2/GPIO21，仅保留复用配置 |
| `SERVO_ZERO_PULSE_US` | 1500 | 当前用于停转微调，限制1400～1600；范围内仍可能转动，不是整段都停转 |
| `SERVO_ZERO_START_DELAY_MS` | 3000 | PWM开始前等待，之后持续发送固定脉宽 |

已有sdkconfig时编辑defaults不会覆盖现有配置，应通过menuconfig保存后重建。本工具不依赖ROS，也不修改micro-ROS共享缓存。

## 烧录与查看日志

独立镜像会替换正常底盘应用，上电3秒后输出所选脉宽；先脱开抬升连杆/负载。停止占用串口的agent，仅在容器正在运行时执行stop：

```bash
docker stop hku-chassis-agent
bash firmware/servo_zero/idf.sh -p /dev/ttyUSB0 flash
```

查看日志：

```bash
bash firmware/servo_zero/idf.sh -p /dev/ttyUSB0 monitor
```

Ctrl+]退出monitor；不要让agent和monitor同时占串口。日志只报告PWM已开始及配置值，不报告舵机实际停转/角度到位。此时不运行 `/chassis`、`/odom`、`/imu/data` 或 `/lifter`。

改停转脉宽后执行build/flash，每次从脱载、支撑稳固的状态开始观察。当前只完成编译及主机检查，尚未对实物舵机校准停转脉宽。

## 恢复正常底盘

先可靠支撑抬升机构、关闭舵机电源并退出monitor，再运行：

```bash
bash firmware/chassis/idf.sh -p /dev/ttyUSB0 flash
```

之后按[正常底盘说明](../chassis/README.md)恢复唯一agent。正常底盘固件尚无升降舵机控制，本程序不能代替完整 `/lifter`。

## 来源与验证范围

GPIO8/21、50Hz/1MHz MCPWM定义来自本机Yahboom `06. Drive PWM servo.pdf` 和 `Samples.zip/esp32_samples/pwm_servo/components/servo`。本程序单通道固定脉宽，不采用官方示例的双路默认位置和往复扫角度。四轮制动复用已有 `pwm_motor`/Espressif BDC组件；未复制官方servo组件。

连续旋转PWM控制转速/方向与中立值的一般原理可参考[Parallax官方连续旋转控制参考](https://learn.parallax.com/reference/propeller-blocklyprop-block-reference/servo/)。该资料用于说明控制类型，不能替代这颗MG996R的供电/死区/停转值规格。

主机检查运行实际app控制流程，验证轮制动先于单通道PWM、固定脉宽、S1/S2选择及非法端口/超出微调范围的配置拒绝。未验证真实脉宽、舵机停转、轴角或载荷能力。
