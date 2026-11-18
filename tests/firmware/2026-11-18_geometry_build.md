# 轮距修正后的正常底盘固件构建

2026-11-18，Asia/Shanghai。用户重新测量：前后170mm、左右200mm；尺寸以轮组中心间距为基准。

- `firmware/chassis/components/car_motion/car_motion.h`：`ROBOT_LENGTH` 从0.165m改为0.170m，`ROBOT_WIDTH` 保持0.200m。
- `ROBOT_APB=(ROBOT_WIDTH+ROBOT_LENGTH)/2` 更新为0.185m；轮径仍为标称65mm。
- 执行 `bash firmware/chassis/idf.sh build` 成功，重新编译car_motion及main，链接并生成应用镜像，分区大小检查通过。
- 应用镜像：`firmware/chassis/build/chassis.bin`，335360字节，SHA256：`d9b326b6d7ecad41e6be83f586004d2f1193f2eb6c60ea822e1d3928a7b7bb1c`。
- 构建仍有已有的rclc定时器弃用及诊断rcl_publish返回值警告，无编译错误。

本轮仅修改参数、文档并编译，没有烧录、复位或发送运动指令。编译通过不代表四轮闭环及里程计已通过实车验收。最近实测记录中的板上应用仍为独立motor_test；该测试应用不使用底盘几何参数，本轮没有修改或编译它。
