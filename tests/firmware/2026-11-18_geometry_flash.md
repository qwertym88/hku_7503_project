# 正常底盘固件重新编译、烧录和静态通信检查

2026-11-18，Asia/Shanghai。用户授权重新编译及烧录。

- 几何参数：左右200mm、前后170mm，ROBOT_APB=0.185m；标称轮径65mm。
- `bash firmware/chassis/idf.sh build`成功，分区大小检查通过，应用335360字节。
- 应用SHA256：`d9b326b6d7ecad41e6be83f586004d2f1193f2eb6c60ea822e1d3928a7b7bb1c`。
- 停止唯一的hku-chassis-agent，释放/dev/ttyUSB0后，执行`bash firmware/chassis/idf.sh -p /dev/ttyUSB0 flash`。
- 目标ESP32-S3 revision v0.2，MAC 9c:13:9e:1f:d2:c4，与此前记录一致。bootloader@0x0、partition table@0x8000、应用@0x10000写入后均显示`Hash of data verified`；随后RTS复位成功。
- 恢复唯一的后台Docker agent，domain42、921600波特率，日志确认session established。板端自行建立会话，无需用户再按RESET。
- 只读观察8秒，收到160条odom和16条诊断；odom按接收时间估算20.131Hz（短窗口含启动/传输抖动，目标20Hz）。节点为/chassis，/cmd_vel订阅者1、发布者0，/odom唯一发布者为/chassis。
- 最后诊断rx=0、rejected=0、enabled=0、target/speed/pid_output全0，counts全0。没有发送任何运动指令，未验证接线后的四轮方向、编码器与PID闭环。
- ADC估算电池电压约4.84V，未用万用表复核，不能当作真实电池或电机电源电压结论。

当前板上为正常chassis应用，已经替换motor_test；hku-chassis-agent继续在后台运行。当前支持/cmd_vel与/odom；IMU及举升仍未实现。

完整只读观测数据：[2026-11-18_geometry_flash_communication.json](2026-11-18_geometry_flash_communication.json)。
