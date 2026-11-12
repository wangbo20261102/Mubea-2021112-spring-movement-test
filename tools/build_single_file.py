# 学习：主程序生成工具：选取模块代码并保留中文注释，组合成独立ag_run.py；不连接设备。
# 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
"""Rebuild the standalone runner and preserve the Chinese learning comments."""
# 学习：导入ast，用于解析Python语法结构，不执行被解析代码。
import ast
# 学习：从pathlib导入Path，用于以Path对象处理文件与目录。
from pathlib import Path

# 学习：把“Path(__file__).resolve().parents[1]”保存到root（项目根目录）；单个=表示赋值。
root = Path(__file__).resolve().parents[1]
# 学习：把“建立不重复元素的集合”保存到SIBLING_MODULES（已经合并到单文件中的本地模块名集合）；单个=表示赋值。
SIBLING_MODULES = {'mcea_read', 'mcea_direction_test', 'simulate_ag'}


# 学习：定义source_with_comments函数：取出一个顶层定义及其学习注释，省略已经合并的同目录模块导入；def本身只是定义，调用时才执行函数体。
def source_with_comments(node, lines):
    # 学习：语法树行号从1开始、列表下标从0开始，所以减1定位代码首行。
    start = node.lineno - 1
    # 学习：while循环：只要start > 0 and (not lines[start - 1].strip() or lines[start - 1].lstrip().startswith('#'))成立就继续；完成、异常或超时会结束等待。
    while start > 0 and (not lines[start - 1].strip() or lines[start - 1].lstrip().startswith('#')):
        # 学习：继续向上扩展片段起点，把该定义前面的中文注释和空行一起保留。
        start -= 1
    # 学习：把“转换或创建set类型的数据”保存到omitted（合并时需要跳过的源代码行号集合）；单个=表示赋值。
    omitted = set()
    # 学习：for逐项循环：从ast.walk(node)取一个元素放入child，再执行缩进部分。
    for child in ast.walk(node):
        # 学习：判断条件isinstance(child, ast.ImportFrom) and child.module in SIBLING_MODULES；成立(True)时执行下方缩进代码。
        if isinstance(child, ast.ImportFrom) and child.module in SIBLING_MODULES:
            # 学习：把需要省略的一组源代码行号加入集合。
            omitted.update(range(child.lineno - 1, child.end_lineno))
            # 学习：把“计算child.lineno - 2”保存到previous（上一行的下标）；单个=表示赋值。
            previous = child.lineno - 2
            # 学习：判断条件previous >= start and lines[previous].lstrip().startswith('# 学习：')；成立(True)时执行下方缩进代码。
            if previous >= start and lines[previous].lstrip().startswith('# 学习：'):
                # 学习：把一个值加入集合，自动去重。
                omitted.add(previous)
    # 学习：return把计算''.join((line for index, line in enumerate(lines[start:node.end_lineno], start) if index not in omitted)).stri…交回调用者，并立即结束当前函数。
    return ''.join(line for index, line in enumerate(lines[start:node.end_lineno], start) if index not in omitted).strip('\n') + '\n'


# 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到parts（独立主程序的代码片段列表）；单个=表示赋值。
parts = []
# 学习：for逐项循环：从[('mcea_read.py', {'crc16', 'Reader'}), ('mcea_direction_test.py', {'Device', 'validate', 'bounds'}), ('simula…取一个元素放入(filename, names)，再执行缩进部分。
for filename, names in [
    # 学习：续写for要遍历的数据范围或元素集合，本行不会单独启动新循环。
    ('mcea_read.py', {'crc16', 'Reader'}),
    # 学习：续写for要遍历的数据范围或元素集合，本行不会单独启动新循环。
    ('mcea_direction_test.py', {'Device', 'validate', 'bounds'}),
    # 学习：续写for要遍历的数据范围或元素集合，本行不会单独启动新循环。
    ('simulate_ag.py', {'motion_profile', 'displacement', 'build_plan'}),
    # 学习：续写for要遍历的数据范围或元素集合，本行不会单独启动新循环。
    ('ag_trial.py', None),
# 学习：关闭上方开始的括号/列表/字典；它和前面的多行内容合成一条完整表达式。
]:
    # 学习：把“以UTF-8读入文件全文”保存到source（读入的Python源代码文本）；单个=表示赋值。
    source = (root / filename).read_text(encoding='utf-8')
    # 学习：把“把源代码解析为语法树，只分析结构不运行代码”保存到tree（源代码对应的语法树）；单个=表示赋值。
    tree = ast.parse(source)
    # 学习：把“按源代码行切分，按参数保留换行符”保存到lines（保留换行的源代码行列表）；单个=表示赋值。
    lines = source.splitlines(keepends=True)
    # 学习：for逐项循环：从tree.body取一个元素放入node，再执行缩进部分。
    for node in tree.body:
        # 学习：把“判断一个对象是否属于指定的数据或语法节点类型”保存到selected（是否选中当前定义合并进主程序）；单个=表示赋值。
        selected = isinstance(node, (ast.Import, ast.ImportFrom, ast.Assign, ast.AnnAssign))
        # 学习：把“selected or (isinstance(node, (ast.FunctionDef, ast.ClassDef)) and (names is None or node.name in names))”保存到selected（是否选中当前定义合并进主程序）；单个=表示赋值。
        selected = selected or isinstance(node, (ast.FunctionDef, ast.ClassDef)) and (names is None or node.name in names)
        # 学习：判断条件isinstance(node, ast.ImportFrom) and node.module in SIBLING_MODULES；成立(True)时执行下方缩进代码。
        if isinstance(node, ast.ImportFrom) and node.module in SIBLING_MODULES:
            # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
            continue
        # 学习：判断条件selected；成立(True)时执行下方缩进代码。
        if selected:
            # 学习：把一个新元素追加到列表末尾。
            parts.append(source_with_comments(node, lines))

# 学习：保存独立主程序顶部的说明文字；这里的\n表示将来写入文件时的换行。
header = '# 学习：独立连续运行主程序；各功能已在下方逐行说明，坐标m、姿态rad；启动后仍需一次RUN或RECOVER确认。\n"""A-G manually started, continuous single-cycle hardware runner.\nRobot 192.168.21.5; cylinder COM4,115200,8N1,id1.\nHold the orientation captured from hardware on 2026-11-12 13:10:55 UTC+8.\nNo rotation-to-zero stage. No joint lock. Robot30/cylinder1 mm/s.\nAfter interruption, RECOVER confirms continuous return-to-start preparation and one new cycle.\nDependencies: pyserial==3.5, ur_rtde==1.6.5.\nImporting this file never connects or moves hardware.\nUser launch: python ag_run.py (one RUN confirmation, then continuous execution).\nRead only: --check. Offline preview (starting at A): --preview.\nGenerated from reviewed local trial modules; hardware motion not validated.\n"""\n'
# 学习：保存主程序入口代码文本；它将在生成的ag_run.py末尾成为可执行代码，当前只作为字符串存放。
tail = "\n# 学习：只有直接启动本文件时才进入主流程；导入文件不会启动设备。\nif __name__ == '__main__':\n    # 学习：没有命令行参数时，为单文件程序选择实际连续运行入口；仍需手动输入RUN或RECOVER。\n    if len(sys.argv) == 1:\n        # 学习：添加--run参数；各段之间无需输入GO。\n        sys.argv.extend(['--run'])\n    # 学习：运行主函数并把成功/失败状态码交给操作系统。\n    raise SystemExit(main())\n"
# 学习：把“计算root / 'ag_run.py'”保存到output（生成文件或测试内存输出）；单个=表示赋值。
output = root / 'ag_run.py'
# 学习：把“计算header + '\n\n'.join(parts) + tail”保存到combined（最终合并的独立主程序文本）；单个=表示赋值。
combined = header + '\n\n'.join(parts) + tail
# 学习：把源代码解析为语法树，只分析结构不运行代码。
ast.parse(combined)
# 学习：以UTF-8写入文本文件。
output.write_text(combined, encoding='utf-8', newline='\n')
# 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
print('Updated', output)
