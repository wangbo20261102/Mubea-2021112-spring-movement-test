# Mubea 弹簧运动试验：UR机械臂与MCEA电缸联动

2026-11-12研究归档。使用UR CB3机械臂与大寰MCEA电缸，按实测A–G点完成一轮弹簧运动流程。最终版本支持一次确认后连续运行，任务速度为机械臂30 mm/s、电缸1 mm/s，每段到位停顿1秒，最终机械臂回A、电缸回0 mm。

## 零基础学习

全部8个Python文件已添加逐行中文注释。建议先读[零基础代码阅读指南](docs/零基础代码阅读指南.md)，再按指南顺序阅读模拟、任务入口和通信代码。注释覆盖与逻辑一致性结果见[注释验证记录](reports/comment_validation.json)。

## 文件入口

| 文件 | 内容 |
|---|---|
| [ag_run.py](ag_run.py) | 可独立使用的连续运行主程序；手动启动，输入一次RUN |
| [docs/研究记录.md](docs/研究记录.md) | 需求变化、实测点、姿态、完整顺序、实现和结论 |
| [docs/运行说明.md](docs/运行说明.md) | 安装、启动、连续恢复与日志 |
| [docs/设备与通信.md](docs/设备与通信.md) | 设备、接口、读回状态与Modbus寄存器 |
| [docs/问题与修复.md](docs/问题与修复.md) | 本次报错、原因、处理及适用版本 |
| [docs/验证记录.md](docs/验证记录.md) | 成功实机日志摘要、用户反馈及当前离线验证 |
| [simulate_ag.py](simulate_ag.py) | 不连接设备的几何模拟，支持导出计划和HTML |
| [ag_trial.py](ag_trial.py) | 模块版连续主程序；默认只读，--run才进入实际运行 |
| [test_fixed_orientation.py](test_fixed_orientation.py) | 固定姿态、连续顺序、恢复、异常处理离线测试 |
| [run_offline_tests.py](run_offline_tests.py) | 同时验证模块版与单文件版，屏蔽实际通信 |
| [tools/build_single_file.py](tools/build_single_file.py) | 从模块源文件重新生成ag_run.py |
| [data/task_config.json](data/task_config.json) | 实测坐标及当前任务参数的机器可读归档 |
| [reports/hardware_success_summary.json](reports/hardware_success_summary.json) | 已完成一轮的去标识实机结果摘要 |

## 使用

本次环境为Windows、Python3.12；依赖固定在requirements.txt。

```powershell
python -m pip install -r requirements.txt
python ag_run.py --preview
python run_offline_tests.py
```

设备就绪后手动启动：

```powershell
python ag_run.py
```

输入一次 `RUN` 后连续完成一轮；若需要中断后恢复，按提示输入一次 `RECOVER`，随后连续恢复并完成新一轮。没有逐段GO确认，不无限循环。只读检查使用 `python ag_run.py --check`。

离线模拟：

```powershell
python simulate_ag.py --out-dir simulation_output
```

主程序默认机器人IP为192.168.21.5、电缸COM4，可用 `--ip`、`--port`指定。姿态固定为13:10:55的实际读取值，J6自由；详细条件见运行说明。Python文件导入不连接设备。

## 证据边界

本地日志记录了一轮由用户启动的逐步版实机成功运行：2026-11-12 13:31:08至13:40:17（北京时间），包含恢复准备、全部目标和最终A/0 mm。用户随后确认已复现所有流程。当前连续版仅移除逐段等待，模块版和单文件版各通过21项离线测试；归档与上传期间未连接设备，尚无当前连续版的完整实机成功记录。

原始串口帧、调试堆栈和本机环境日志保留于原工作目录；仓库提供可复现的程序、完整研究说明及精选结果摘要。设备序列号、本机用户名和认证信息不进入发布文件。本仓库未指定开源许可证。
