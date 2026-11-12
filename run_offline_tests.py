# 学习：离线测试入口：屏蔽真实网络和串口，分别检查模块版与单文件版。
# 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
"""Run both delivery variants offline; never connect to hardware."""
# 学习：导入contextlib，用于临时重定向终端输出。
import contextlib
# 学习：导入importlib.util，用于按文件路径加载待测试模块。
import importlib.util
# 学习：导入io，用于在内存中接收测试输出。
import io
# 学习：导入json，用于读写JSON数据文件。
import json
# 学习：导入py_compile，用于检查Python文件能否编译。
import py_compile
# 学习：导入socket，用于进行网络通信。
import socket
# 学习：导入sys，用于访问启动参数和退出状态。
import sys
# 学习：导入unittest，用于组织并运行离线测试。
import unittest
# 学习：从pathlib导入Path，用于以Path对象处理文件与目录。
from pathlib import Path
# 学习：从unittest.mock导入patch，用于用Mock和patch模拟设备及临时替换函数。
from unittest.mock import patch
# 学习：导入serial，用于通过pyserial访问串口。
import serial
# 学习：导入test_fixed_orientation，用于调用该模块提供的功能。
import test_fixed_orientation as tests

# 学习：把“Path(__file__).resolve().parent”保存到ROOT（当前文件所在目录）；单个=表示赋值。
ROOT = Path(__file__).resolve().parent


# 学习：定义no_hardware函数：若离线测试试图连接真实设备，立即使测试失败；def本身只是定义，调用时才执行函数体。
def no_hardware(*args, **kwargs):
    # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
    raise AssertionError('Hardware communication is forbidden during offline tests')


# 学习：定义main函数：离线测试入口：屏蔽真实网络和串口，分别检查模块版与单文件版。；def本身只是定义，调用时才执行函数体。
def main():
    # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到results（两个版本的离线测试结果列表）；单个=表示赋值。
    results = []
    # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
    with patch.object(socket, 'create_connection', side_effect=no_hardware), patch.object(serial, 'Serial', side_effect=no_hardware):
        # 学习：for逐项循环：从[('module', 'ag_trial.py'), ('single_file', 'ag_run.py')]取一个元素放入(name, filename)，再执行缩进部分。
        for name, filename in [('module', 'ag_trial.py'), ('single_file', 'ag_run.py')]:
            # 学习：把“调用importlib.util.spec_from_file_location，按本行参数取得结果”保存到spec（按路径加载Python模块的说明对象）；单个=表示赋值。
            spec = importlib.util.spec_from_file_location('offline_' + name, ROOT / filename)
            # 学习：把“调用importlib.util.module_from_spec，按本行参数取得结果”保存到app（被检查的程序模块）；单个=表示赋值。
            app = importlib.util.module_from_spec(spec)
            # 学习：调用spec.loader.exec_module，按本行参数取得结果。
            spec.loader.exec_module(app)
            # 学习：把“app（被检查的程序模块）”保存到tests.app；单个=表示赋值。
            tests.app = app
            # 学习：把“调用io.StringIO，按本行参数取得结果”保存到output（生成文件或测试内存输出）；单个=表示赋值。
            output = io.StringIO()
            # 学习：with限定资源或临时替换的有效范围：调用contextlib.redirect_stdout，按本行参数取得结果；离开时自动执行对应清理。
            with contextlib.redirect_stdout(output):
                # 学习：把“执行已组织好的离线测试用例”保存到result（当前函数累积的结果）；单个=表示赋值。
                result = unittest.TextTestRunner(stream=output, verbosity=2).run(
                    # 学习：续写上方赋值的数据/参数：unittest.defaultTestLoader.loadTestsFromModule(tests))；这些内容仍属于同一条赋值语句。
                    unittest.defaultTestLoader.loadTestsFromModule(tests))
            # 学习：调用py_compile.compile，按本行参数取得结果。
            py_compile.compile(str(ROOT / filename), doraise=True)
            # 学习：把一个新元素追加到列表末尾。
            results.append(dict(target=name, tests=result.testsRun, success=result.wasSuccessful(),
                                # 学习：续写上方调用的参数或文本：failures=len(result.failures), errors=len(result.errors)))。
                                failures=len(result.failures), errors=len(result.errors)))
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print(f'{name}: {result.testsRun} tests; success={result.wasSuccessful()}')
            # 学习：判断条件not result.wasSuccessful()；成立(True)时执行下方缩进代码。
            if not result.wasSuccessful():
                # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
                print(output.getvalue())
    # 学习：把“创建一份键值数据字典，或复制已有字典”保存到report（运行或测试报告字典）；单个=表示赋值。
    report = dict(mode='offline_only', hardware_connected=False, motion_commands_sent=False,
                  # 学习：续写上方赋值的数据/参数：results=results)；这些内容仍属于同一条赋值语句。
                  results=results)
    # 学习：把“计算ROOT / 'reports'”保存到folder（输出目录）；单个=表示赋值。
    folder = ROOT / 'reports'
    # 学习：创建输出目录，已存在时按参数决定是否允许。
    folder.mkdir(exist_ok=True)
    # 学习：以UTF-8写入文本文件。
    (folder / 'validation_continuous.json').write_text(
        # 学习：续写上方调用的参数或文本：json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')。
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    # 学习：return把条件all((x['success'] for x in results))成立取前一值，否则取后一值交回调用者，并立即结束当前函数。
    return 0 if all(x['success'] for x in results) else 1


# 学习：只有直接启动该文件时才执行入口；被其他文件import时跳过。
if __name__ == '__main__':
    # 学习：运行入口函数并以其返回值结束进程；0通常表示成功，1表示失败。
    raise SystemExit(main())
