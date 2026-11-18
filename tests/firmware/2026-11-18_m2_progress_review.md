# M2 进度重评与提交前复核

2026-11-18，Asia/Shanghai。用户要求重新评估底盘进度、补充文档，并将已有暂存及未暂存改动一起提交。本轮只做仓库检查、文档整理和软件验证，未操作开发板或发送电机指令。

## 评估与文档

按 M2-1～M2-6 核对实现与证据，结论为基础固件已烧录、单电机及 IMU 基础验证已有结果，整车交付尚未验收。M4 编码器待换；M1～M3 正常是用户现场确认，不伪造完整自动测试矩阵。

新增[M2 进度与交付清单](../../docs/给队友看/M2底盘进度与交付清单.md)，逐项列实现、证据、缺口、后续顺序和记录模板。同步根 README、HANDOFF、固件说明、团队任务/接口/运行说明及 bridge 交接；消除当前仍是测试版或 IMU 未烧录的过时说法。历史测试记录和失败原始数据保留。

重点明确：

- M4 无编码器反馈时正常 PID 尚无故障停车保护；更换前保持断开，不做四轮闭环精度验收。
- IMU 零偏只测量未补偿，无姿态估计，完整外参/旋转符号与时间同步仍待验证。
- `/lifter` 尚未实现；物理急停、停车时延、带载供电、手动取放及 M4/M5 联合验收尚未完成。
- 1040 counts/转、轮径和旋转系数是当前配置，尚缺地面标定；不把静态话题或软件状态当作物理精度/停车证明。
- 本机 Docker agent 已验证，Pi 部署/固定设备名/统一启动仍未完成。旧 EKF/launch 接线需按新契约改造。

## 本次软件复核

| 检查 | 结果 |
|---|---|
| `bash firmware/chassis/idf.sh build` | ESP-IDF5.1.2 / esp32s3 构建与分区检查通过 |
| 应用文件 | 356432B，SHA256 `035bb902db397cdcfb27ee69410e4820d3e117eb0574a841ad629905f6d0b65d`，与已烧录版本一致 |
| `gcc -std=c11 -Wall -Wextra -Werror -Ifirmware/chassis/components/icm42670p tests/firmware/imu_sample_test.c -lm -o /tmp/imu_sample_test`，运行该程序 | SI 单位/符号、无效/重复/过期/未来样本及恢复通过 |
| `python3 tests/firmware/check_imu_publication.py` | 实际 IMU 发布回调的载荷、时间戳、新鲜度门控与发布异常检查通过 |
| `bash -n firmware/chassis/idf.sh firmware/motor_test/idf.sh` | 语法通过 |
| 待提交 JSON/Python、Markdown 相对文件链接 | JSON 解析、Python AST 解析和链接目标文件存在性检查通过；不是所有历史锚点或外网链接的联网验证 |
| 官方 IMU 来源清单 | `official_sources.json` 中标记未修改的 18 个文件逐项 SHA256 一致；其中 `inv_imu` 16 文件保持原始字节及版权/许可说明；增加目录级 `.gitattributes -text`，避免仓库自动换行转换使提交后的校验值变化 |
| 提交内容 | 包含此前暂存的底盘/单轮工程、测试证据与未暂存的 IMU 实现、工具及文档；忽略的 `.ref/`、构建产物和原 Flash 备份不提交 |

导入组件保留原始源码格式，完整 `git diff --check` 会报告部分组件原有的行尾空白/文件末空行；不为排版更改官方源码。提交前对组件目录以外的改动执行 `git diff --cached --check -- . ':(exclude)firmware/chassis/components/**'`，通过。已检查常见凭据格式，无命中；原始测量 JSON 总量约 2.4MB 的待提交内容可直接入库。

本轮未新增物理测试结论；未验证 motor_test 全新重建、全新机器依赖恢复、温漂或整车联合运行。已有烧录证据继续以[IMU 实测记录](2026-11-18_imu_flash_verification.md)为准。
