# 测试记录表

> 2026-11-11起采用v2.1真实雷达建图/实时定位+固定路线基线，验收按[保守方案验收](保守方案验收.md)。下列已有M3记录仅证明原感知能力，不计入新路线完成；新方案未测，不提前添加通过记录。

> 规则见《单模块测试说明》末尾：**没有证据（截图/日志/bag）的测试等于没测。**
> 每次测试追加一行；`证据` 列写仓库内路径或终端输出摘要。

| 日期 | 模块 | 测试项 | 结果 | 证据 | 测试人 |
|---|---|---|---|---|---|
| 2026-11-10 | M3 | 离线解码回归：官方 6 码 + 合成难例 72 张（旋转/倾斜/模糊/明暗/低分辨率/噪声） | 72/72 通过（含 18 张极端样例） | `tests/qr_dataset/`；`python3 tools/qr_eval.py tests/qr_dataset/` 输出 100% PASS | M3 |
| 2026-11-10 | M3 | 多帧投票去抖单元测试（单帧误读不上报 / 不刷屏 / 消失重报 / 间歇识别 / 多码择优） | 8/8 通过 | `colcon test --packages-select qr_perception`：Ran 8 tests, OK | M3 |
| 2026-11-10 | M3 | ROS 薄封装：图片目录回放 6 类码 → `/qr`（本机，无相机） | 6 类码各发布**恰好一次**（回放循环 3+ 圈无刷屏）；`ros2 topic echo /qr` 收到 `std_msgs/String` | `ros2 launch qr_perception qr_perception.launch.py source:=file file_path:=/tmp/qr_replay` 节点日志 | M3 |
| 2026-11-10 | M3 | `/camera/image_raw` 内部图像流（bgr8，frame_id=camera） | `ros2 topic echo /camera/image_raw --once` 正常出图 | 终端输出 header.frame_id=camera | M3 |
| 2026-11-10 | M3 | 打印样张可扫性（`tests/qr_print_sheet.png` 整张一次解出 6 码） | 6/6 | `qr_make_dataset.py --sheet` + 解码验证脚本输出 | M3 |
| 2026-11-11 | M3 | **Pi 实拍**：rpicam-vid 取流 640x480@30，400 帧连续流稳定性 | 400 帧零断流，整循环 **16.9 fps**（≥15 达标，M3-1） | Pi 终端输出 `最后实测帧率: 16.9 fps`；关键修复：rpicam-vid 默认 `-t 5000` 只跑 5s，必须 `-t 0` | M3 |
| 2026-11-11 | M3 | **Pi 实拍**：6 类 QR 实机识别（手机/打印样张举码，多轮） | 6/6 全部识别，每码多轮确认；投票去抖正常（同码不刷屏、重现重报） | `tests/qr_shots/*.png`（6 张提交件，标注后仍可复验解码）；会话日志 62 次截图/确认 | M3 |
| 2026-11-11 | M3 | **跨机全栈**：Pi 跑节点（回放源）→ 本机 `ros2 topic echo /qr`（ROS_DOMAIN_ID=42，局域网 DDS） | 6 类码各恰好 1 条消息，跨机去抖正确 | 本机终端输出（data: START … data: END） | M3 |
| 2026-11-11 | M3 | Pi 离线回归（Pi 的 cv2 4.6 环境） | 72/72 = 100% PASS | `tools/setup_pi_qr.sh` 第 4 步输出 | M3 |
| 2026-11-12 | M2 | 首次底盘固件交叉编译（无开发板） | ESP-IDF5.1.2/esp32s3编译通过；不计为烧录、运动或看门狗实测通过 | `bash firmware/chassis/idf.sh build`：`Project build complete`；`chassis.bin` 321856B，SHA256 `09a9748d4a544ef8a82389f55e9b2478d156b0f64f41b4fb90d28b8061280388`；依赖提交见`firmware/chassis/micro_ros_sources.lock.json` | Codex辅助 |

| 2026-11-12 | M2 | ESP32-S3首次烧录与静止串口ROS通信 | 写入校验通过；100条odom平均19.998Hz、最大间隔51.1ms；未发送运动指令，运动及看门狗物理测试未做 | `tests/firmware/2026-11-12_flash.md`、同目录JSON；原Flash已完整本地备份 | Codex辅助 |

| 2026-11-12 | M2 | 架空前进指令：0.03m/s、20Hz、约2s | 未通过：odom持续发布但编码器速度全程为0；物理观察待用户补充；已发零速，后退/转向暂缓，不能据此计算停车延时 | `tests/firmware/2026-11-12_motion_forward.json` | Codex执行／用户外部观察 |

| 2026-11-12 | M2 | 用户要求继续后的架空前进复测及诊断 | 未通过：用户观察四轮无动作；板端确认收到指令/目标与PID更新，编码器几乎无变化；已停车，后退/转向未执行，待供电接线检查 | `tests/firmware/2026-11-12_motion_investigation.md`及其JSON记录 | Codex执行／用户外部观察 |

| 2026-11-18 | M2 | domain42只读复核及官方单轮PWM测试准备 | 当前odom约19.992Hz、停车时counts全0；新增独立测试固件交叉编译、控制逻辑主机检查及正常固件构建回归通过；未烧录测试版、未发送运动指令，不计运动验收通过 | `tests/firmware/2026-11-18_readonly_baseline.json`、`2026-11-18_motor_test_preparation.md`；`firmware/motor_test/README.md` | Codex辅助 |

| 2026-11-18 | M2 | 用户确认架空供电后的M1单轮开环PWM测试及手转检查 | 运动未通过：测试版烧录校验通过，M1输入40、20Hz共10条，0.504467秒后发零；板端确认PWM软件输出，用户确认完全无动作，counts全0；用户随后报告可正反手转，17秒监听170条仍未采到计数变化；末尾输出全零，待检查线束／接口供电 | `tests/firmware/2026-11-18_m1_pwm_test.md`、`2026-11-18_m1_pwm_pulse.json`、`2026-11-18_manual_encoder_observation.json`、`2026-11-18_motor_test_flash.log` | Codex执行／用户外部观察 |

| 2026-11-18 | M2（电机M3） | 用户重新检查线路后的M3单向开环复测 | 单向开环与反馈基本检查通过：M3输入40、20Hz共10条，0.505537秒后发零；M3计数0→574，其他三路保持0，无拒绝／故障；末尾输出全零、计数稳定，用户确认右前轮转动并已停下。反向、整车、PID闭环和停车时延未验收 | `tests/firmware/2026-11-18_m3_pwm_test.md`、`2026-11-18_m3_pwm_pulse.json` | Codex执行／用户外部观察 |

## 已知问题与注意事项（M3 实测教训）

- **Ubuntu 24.04 没有 `python3-picamera2`**（Pi OS 专属包）。取流用 `rpicam` 帧源（rpicam-vid 子进程，零依赖）；
  想走 picamera2 需 `sudo apt install python3-libcamera` + `pip install --user --break-system-packages picamera2==0.3.10`（版本必须配 noble 的 libcamera 0.2.0）。
- **rpicam-vid 默认 `-t 5000`（只跑 5 秒）**，长跑必须加 `-t 0`。曾误判为"管道背压卡死"。
- Pi5 + Ubuntu 的相机管线长跑（>10 分钟）偶发 `imx708_stop_streaming failed`，重启取流进程即恢复；
  节点已带断流自动重连（3 次）。比赛单轮 3 分钟，风险低。
- pyzbar 兜底在 Pi 上暂不可用（pip 装了但缺 `libzbar0` 系统库，需 sudo）；OpenCV 主路 100% 覆盖测试集，不阻塞。
- `deep_every_n` 实测（Pi5，640x480 空画面）：2→13fps，4→15fps，已设默认 4。识别到码时总是走轻量路径（19ms），确认延迟 ~2 帧。

| 2026-11-18 | M2 | 200×170mm正常底盘固件重编译、烧录、静态通信 | 编译及三个写入段哈希校验通过；8秒160条odom、16条诊断；无速度发布者，未做运动验收 | [烧录及通信记录](firmware/2026-11-18_geometry_flash.md) | Codex辅助 |

| 2026-11-18 | M2 | 板载ICM42670P及/imu/data接入的软件检查 | ESP-IDF编译、SI/新鲜度主机测试、实际发布回调桩测试通过；未烧录、未做硬件IMU验收，50Hz为目标 | [IMU实现与验证](firmware/2026-11-18_imu_integration.md) | Codex辅助 |

| 2026-11-18 | M2 | 含IMU固件烧录、平放静态读取与零偏测量 | 三段写入校验通过；1500样本IMU约49.992Hz、odom约19.989Hz，WHOAMI=0x67且无I2C错误；零偏未补偿，完整安装外参/三轴旋转待验证 | [IMU板端验证](firmware/2026-11-18_imu_flash_verification.md) | Codex采集／用户确认姿态与手动方向 |

| 2026-11-18 | M2 | 交付进度重评与提交前软件复核 | 正常固件重建通过，应用 SHA256 与已烧录版本一致；SI/新鲜度原生测试、实际 IMU 回调测试及构建脚本语法检查通过。补齐分项缺口、IMU 使用限制与后续验收；未重新烧录、未发运动命令，不新增硬件验收结论 | [M2进度与交付清单](../docs/给队友看/M2底盘进度与交付清单.md)、[提交前复核](firmware/2026-11-18_m2_progress_review.md) | Codex辅助 |

| 2026-11-19 | M2 | 更换电机后的反馈保护、IMU校准通路、烧录与只读检查 | 用户已换新电机，反馈待复测；新增无反馈/反向故障锁存、离线零偏校准及有界台架工具。真实电机控制注入测试、IMU回调补偿测试和校准11项通过；三段烧录校验通过，IMU约49.999Hz/odom约19.996Hz，rx=0/输出0。未发送运动指令，不计新M4或四轮验收通过 | [本轮开发与板测](firmware/2026-11-19_feedback_guard.md)及同目录JSON/日志 | Codex开发/只读采集；用户报告换电机 |

| 2026-11-19 | M2 | 舵机安装归零独立工程 | ESP-IDF5.1.2编译通过；实际app主机桩检查S1/S2单通道固定脉宽、轮电机制动先于舵机输出通过；默认S1/1500µs/50Hz为官方中位，型号/接口/零位待操作者确认。未烧录、未驱动实机舵机 | [操作说明](../firmware/servo_zero/README.md)、[构建记录](firmware/2026-11-19_servo_zero_build.json)、`check_servo_zero.py` | Codex辅助 |

| 2026-11-19 | M2 | 用户确认MG996R 360°连续旋转、S1后修正舵机程序 | 明确PWM控制转速/方向，不能自动定位机械0°。独立工具改为1500µs停转中立值调试，配置限制1400～1600µs；重新编译及实际app主机检查通过。未烧录/实物停转校准；上行定位中位说明属于型号确认前的假设 | [修正后说明](../firmware/servo_zero/README.md)、[最新构建记录](firmware/2026-11-19_servo_zero_build.json) | 用户确认型号；Codex修正 |

| 2026-11-19 | M2 | 正常固件接入S1连续舵机升降 | `/lifter`与arm/stop/jog、独立截止、轮互锁/故障锁存软件完成；纯C状态、实际驱动/ROS回调、诊断容量及电机互锁主机测试通过，默认禁用/启用夹具两种配置均编译通过。未烧录/执行升降；90°时间与20载荷循环待实测，位置不标传感器确认 | [升降说明](../firmware/chassis/LIFTER.md)、[构建记录](firmware/2026-11-19_lifter_build.json)、[实现与验证](firmware/2026-11-19_lifter_integration.md) | 用户确认机构条件；Codex实现/软件验证 |


2026-11-19：根据用户纠正，将M2升降从连续旋转定时模式改为180°位置控制，默认0°/80°可设、免arm、持续保持目标脉宽；主机状态机/实际驱动/ROS回调与电机、IMU回归通过。新版构建与未上板限制见[验证记录](firmware/2026-11-19_lifter_position.md)。


| 2026-11-19 | M2 | IMU静态陀螺零偏校准完成 | 原始/补偿后各65秒，原始2881/补偿后2880有效样本；头文件指纹匹配，板端补偿已启用，Z均值1.32108→0.03825°/s（降低97.1%）。本次IMU47.985Hz/odom20.001Hz，窗口内健康、无运动命令。轴方向、重启/温度重复性与整车验收待做 | [完成记录](firmware/2026-11-19_imu_calibration.md)、[归档摘要](firmware/2026-11-19_imu_calibration_summary.json) | 用户执行校准/烧录；Codex复核保存数据 |


2026-11-19：按用户新轮安装约定（向前raw counts减小）为M1～M4增加-1反馈方向转换，统一作用于PID/保护/odom，保留原始诊断计数；实际motor/car_motion注入与升降/IMU回归、完整构建通过，未烧录/实机测试。见[方向修订记录](firmware/2026-11-19_encoder_direction.md)。


2026-11-19：用户查明换轮布局后轮ID接错，纠正后四轮前进counts均增加。参考最近提交4943022撤销四轮取反，保留后续保护/互锁、IMU校准和位置舵机，前后/左右转向与PID/保护回归通过；本次未烧录。此前负向假设归档作废，见[恢复记录](firmware/2026-11-19_encoder_baseline_restore.md)。


2026-11-19：按用户要求移除300ms无反馈与反向反馈停车锁存，撤销未提交的1秒启动宽限。保留480ms命令超时、升降互锁及IMU校准。实际motor/实际PID基线对比、升降/ROS、7项台架工具及IMU回归、完整构建通过；未烧录。见[移除记录](firmware/2026-11-19_feedback_guard_removed.md)。


2026-11-19：用户确认当前已能控制小车移动；此前另行确认IMU手动三轴旋转符号与升降基本功能正常。记录为基础功能确认，未扩展成物理停车、里程计精度或载荷验收；下一步优先停车、每圈计数和地面标定。见[基本移动恢复记录](firmware/2026-11-19_motion_recovered.md)。


| 2026-11-19 | M2 | 简单地面运动语句初步验证 | 用户确认定时前进/右转后前进的命令组初步有效；名义目标前进0.3m、右侧位移0.2m采用转车后前进，无横移、无距离/转角实测结果，不计精度或停车验收 | [运动记录](firmware/2026-11-19_motion_recovered.md)、[命令](../docs/给队友看/运行说明.md) | 用户操作/观察；Codex归档 |
