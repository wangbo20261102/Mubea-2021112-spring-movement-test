# 学习：电缸通信与历史方向测试模块；联动程序只复用Device、validate、bounds，历史独立测试仍保留原确认。
# 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
"""MCEA supervised 10 mm direction test. No automatic homing or brake release."""
# 学习：导入argparse，用于读取命令行参数。
import argparse
# 学习：导入json，用于读写JSON数据文件。
import json
# 学习：导入struct，用于把数值与通信字节报文相互转换。
import struct
# 学习：导入time，用于计时及短暂等待。
import time
# 学习：从datetime导入datetime，用于处理日期、时间和时区。
from datetime import datetime
# 学习：从pathlib导入Path，用于以Path对象处理文件与目录。
from pathlib import Path

# 学习：从mcea_read导入Reader, crc16，用于复用只读通信与CRC校验。
from mcea_read import Reader, crc16

# 学习：把“100”保存到SPEED_RAW（电缸速度100原始值，即1 mm/s）；单个=表示赋值。
SPEED_RAW = 100  # 0.01 mm/s units: 1 mm/s
# 学习：把“1”保存到ACCEL_PERCENT（电缸加减速度百分比）；单个=表示赋值。
ACCEL_PERCENT = 1
# 学习：把“1000”保存到DISTANCE_RAW（历史方向测试距离1000原始值，即10 mm）；单个=表示赋值。
DISTANCE_RAW = 1000  # 10 mm
# 学习：把“5”保存到TOLERANCE_RAW（电缸位置容差5原始值，即0.05 mm）；单个=表示赋值。
TOLERANCE_RAW = 5  # 0.05 mm


# 学习：定义Device类：继承Reader，增加受限写入、暂停和状态快照；self表示当前创建的那个对象。
class Device(Reader):
    # 学习：定义value函数：读取指定寄存器的数值；没有有效值就抛出异常；def本身只是定义，调用时才执行函数体。
    def value(self, address):
        # 学习：把“读取电缸一个寄存器，解析有效应答并保留原始收发记录”保存到result（当前函数累积的结果）；单个=表示赋值。
        result = self.read(address)
        # 学习：判断条件'value' not in result；成立(True)时执行下方缩进代码。
        if 'value' not in result:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError(f"Read 0x{address:04X}: {result.get('error')}")
        # 学习：return把result['value']（有效的寄存器数值）交回调用者，并立即结束当前函数。
        return result['value']

    # 学习：定义write_ack函数：发送允许的写报文，并等待地址、数量和CRC一致的应答；失败不自动重发；def本身只是定义，调用时才执行函数体。
    def write_ack(self, body):
        # 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
        """Accept only control-word writes or the six-word position command."""
        # 学习：把“body[1]”保存到function（Modbus功能码）；单个=表示赋值。
        function = body[1]
        # 学习：判断条件function == 6；成立(True)时执行下方缩进代码。
        if function == 6:
            # 学习：把“按指定格式解码通信字节（I为4字节，d为8字节浮点数）”保存到(address, value)；单个=表示赋值。
            address, value = struct.unpack('>HH', body[2:6])
            # 学习：判断条件address != 5637 or value != 40；成立(True)时执行下方缩进代码。
            if address != 0x1605 or value != 0x0028:
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise ValueError('Only the enabled-pause word may be written with function 06')
            # 学习：把“body（尚未附加CRC的Modbus报文）”保存到expected（本测试期望得到的值）；单个=表示赋值。
            expected = body
        # 学习：判断条件function == 16；成立(True)时执行下方缩进代码。
        elif function == 16:
            # 学习：把“按指定格式解码通信字节（I为4字节，d为8字节浮点数）”保存到(address, count)；单个=表示赋值。
            address, count = struct.unpack('>HH', body[2:6])
            # 学习：判断条件address != 5632 or count != 6 or body[6] != 12 or (len(body) != 19)；成立(True)时执行下方缩进代码。
            if address != 0x1600 or count != 6 or body[6] != 12 or len(body) != 19:
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise ValueError('Unexpected motion packet')
            # 学习：把“按指定格式解码通信字节（I为4字节，d为8字节浮点数）”保存到(target, push, speed, accel, force, control)；单个=表示赋值。
            target, push, speed, accel, force, control = struct.unpack('>6H', body[7:])
            # 学习：判断条件not (0 <= target <= 5000 and push == 0 and (speed == SPEED_RAW) and (accel == ACCEL_PERCENT) and (force == 0) and (control in (32, 33)))；成立(True)时执行下方缩进代码。
            if not (0 <= target <= 5000 and push == 0 and speed == SPEED_RAW
                    # 学习：续写当前多行表达式中的一部分：and accel == ACCEL_PERCENT and force == 0 and control in (0x20, 0x21)):。
                    and accel == ACCEL_PERCENT and force == 0 and control in (0x20, 0x21)):
                # 学习：抛出异常：目标或反馈超出允许范围；交由上层错误处理，不继续本段正常流程。
                raise ValueError('Motion parameters outside this test configuration')
            # 学习：把“body[:6]”保存到expected（本测试期望得到的值）；单个=表示赋值。
            expected = body[:6]
        # 学习：else处理前面条件不成立的分支。
        else:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise ValueError('Unsupported write function')
        # 学习：把“计算body + crc16(body)”保存到request（包含CRC的Modbus请求）；单个=表示赋值。
        request = body + crc16(body)
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到record（当前一次通信的收发记录）；单个=表示赋值。
        record = {'tx': request.hex(' '), 'operation': 'write'}
        # 学习：把一个新元素追加到列表末尾。
        self.log.append(record)
        # 学习：清除旧串口输入，避免把旧报文当作新应答。
        self.serial.reset_input_buffer()
        # 学习：向当前通信接口发送已构造的字节报文；请求类型由报文中的功能码决定。
        self.serial.write(request)
        # 学习：等待输出缓冲区的数据发送完成。
        self.serial.flush()
        # 学习：把“转换或创建bytearray类型的数据”保存到data（接收的数据缓冲区）；单个=表示赋值。
        data = bytearray()
        # 学习：把“计算time.monotonic() + 1.0”保存到deadline（允许本操作完成的最晚时刻）；单个=表示赋值。
        deadline = time.monotonic() + 1.0
        # 学习：while循环：只要time.monotonic() < deadline成立就继续；完成、异常或超时会结束等待。
        while time.monotonic() < deadline:
            # 学习：把另一组元素或字节追加到缓冲区末尾。
            data.extend(self.serial.read(self.serial.in_waiting or 1))
            # 学习：for逐项循环：从range(len(data) - 4)取一个元素放入offset，再执行缩进部分。
            for offset in range(len(data) - 4):
                # 学习：判断条件data[offset] != self.id；成立(True)时执行下方缩进代码。
                if data[offset] != self.id:
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：把“data[offset + 1]”保存到code；单个=表示赋值。
                code = data[offset + 1]
                # 学习：把“条件code == function | 128成立取前一值，否则取后一值”保存到size；单个=表示赋值。
                size = 5 if code == (function | 0x80) else 8
                # 学习：判断条件code not in (function, function | 128) or offset + size > len(data)；成立(True)时执行下方缩进代码。
                if code not in (function, function | 0x80) or offset + size > len(data):
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：把“转换或创建bytes类型的数据”保存到frame（截取的一帧完整通信数据）；单个=表示赋值。
                frame = bytes(data[offset:offset + size])
                # 学习：判断条件crc16(frame[:-2]) != frame[-2:]；成立(True)时执行下方缩进代码。
                if crc16(frame[:-2]) != frame[-2:]:
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：判断条件code == function | 128；成立(True)时执行下方缩进代码。
                if code == (function | 0x80):
                    # 学习：批量更新字典中的字段。
                    record.update(rx=frame.hex(' '), error=f'Modbus exception {frame[2]}')
                    # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                    raise RuntimeError(record['error'])
                # 学习：判断条件frame[:-2] != expected；成立(True)时执行下方缩进代码。
                if frame[:-2] != expected:
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：把“把字节转为便于查看的十六进制文本”保存到record['rx']（接收报文的十六进制文本）；单个=表示赋值。
                record['rx'] = frame.hex(' ')
                # 学习：暂停当前线程0.02秒。
                time.sleep(0.02)
                # 学习：return把控制权交回调用者，并立即结束当前函数。
                return
        # 学习：批量更新字典中的字段。
        record.update(rx=bytes(data).hex(' '), error='Write acknowledgement timeout')
        # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError('Write result uncertain. No automatic retry.')

    # 学习：定义pause函数：保持电机使能，写0x28请求电缸软件暂停；def本身只是定义，调用时才执行函数体。
    def pause(self):
        # Keep motor enabled; request pause without forcing brake release.
        # 学习：发送一帧写报文并校验对应应答，不自动重发。
        self.write_ack(struct.pack('>BBHH', self.id, 6, 0x1605, 0x0028))

    # 学习：定义snapshot函数：将站号、位置、回零、报警等寄存器读成一份状态字典；def本身只是定义，调用时才执行函数体。
    def snapshot(self):
        # 学习：return把逐项遍历并计算，生成所需数据：{name: self.value(address) for name, address in (('id', 770), ('position', 5644), ('homing', 51…交回调用者，并立即结束当前函数。
        return {name: self.value(address) for name, address in (
            # 学习：续写当前多行表达式中的一部分：('id', 0x0302), ('position', 0x160C), ('homing', 0x0200),。
            ('id', 0x0302), ('position', 0x160C), ('homing', 0x0200),
            # 学习：续写当前多行表达式中的一部分：('status', 0x1611), ('alarm', 0x160F), ('control', 0x1605),。
            ('status', 0x1611), ('alarm', 0x160F), ('control', 0x1605),
            # 学习：续写当前多行表达式中的一部分：('io', 0x0402), ('limit', 0x0306), ('offset', 0x0308))}。
            ('io', 0x0402), ('limit', 0x0306), ('offset', 0x0308))}


# 学习：定义validate函数：核对电缸站号、回零、报警、使能及控制模式；def本身只是定义，调用时才执行函数体。
def validate(state, device_id):
    # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到problems（启动条件问题列表）；单个=表示赋值。
    problems = []
    # 学习：取出0x1611状态字；接下来用&(1<<位号)分别提取状态位。
    word = state['status']
    # 学习：判断条件state['id'] != device_id；成立(True)时执行下方缩进代码。
    if state['id'] != device_id:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Device ID mismatch')
    # 学习：判断条件state['homing'] != 1 or not word & 1 << 6；成立(True)时执行下方缩进代码。
    if state['homing'] != 1 or not word & (1 << 6):
        # 学习：把一个新元素追加到列表末尾。
        problems.append('尚未回零。请用厂家软件完成回零后再测试。')
    # 学习：判断条件state['alarm'] or word & 1 << 9；成立(True)时执行下方缩进代码。
    if state['alarm'] or word & (1 << 9):
        # 学习：把一个新元素追加到列表末尾。
        problems.append(f"存在报警：0x{state['alarm']:04X}")
    # 学习：判断条件not word & 1 << 10；成立(True)时执行下方缩进代码。
    if not word & (1 << 10):
        # 学习：把一个新元素追加到列表末尾。
        problems.append('电机未使能。')
    # 学习：判断条件word & 1 << 8；成立(True)时执行下方缩进代码。
    if word & (1 << 8):
        # 学习：把一个新元素追加到列表末尾。
        problems.append('电机仍在运动。')
    # 学习：判断条件state['io'] != 0；成立(True)时执行下方缩进代码。
    if state['io'] != 0:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('I/O 控制已开启，不能同时进行本测试。')
    # A completed home may leave Bit1 set. The next full command clears it.
    # 学习：允许使能位与定位位；若已经回零，也容许上次完成后遗留的回零位，其余模式位仍禁止。
    allowed_control = 0x0023 if state['homing'] == 1 and word & (1 << 6) else 0x0021
    # 学习：判断条件state['control'] & ~allowed_control；成立(True)时执行下方缩进代码。
    if state['control'] & ~allowed_control:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('控制字包含暂停、回零、点动或其他模式位，请先检查厂家软件。')
    # 学习：return把problems（启动条件问题列表）交回调用者，并立即结束当前函数。
    return problems


# 学习：定义bounds函数：按名义行程、配置行程和原点偏置计算方向测试保守范围，返回原始位置单位；def本身只是定义，调用时才执行函数体。
def bounds(state):
    # Conservative allowance for nominal stroke, configured stroke and origin offset.
    # 学习：保守余量=1 mm加原点偏置绝对值；全部使用0.01 mm原始整数单位。
    margin = 100 + abs(state['offset'])
    # 学习：返回方向测试下界及上界：在名义50 mm和配置行程中取较小值，再减去保守余量。
    return margin, min(5000, state['limit']) - margin


# 学习：定义plan函数：生成历史电缸方向测试的往返目标，并核对其处于保守范围；def本身只是定义，调用时才执行函数体。
def plan(start, low, high):
    # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到targets（依次执行的目标列表）；单个=表示赋值。
    targets = [start + DISTANCE_RAW, start, start - DISTANCE_RAW, start]
    # 学习：判断条件any((not low <= point <= high for point in [start] + targets))；成立(True)时执行下方缩进代码。
    if any(not low <= point <= high for point in [start] + targets):
        # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError(
            # 学习：续写当前多行表达式中的一部分：f'起点 {start / 100:.2f} mm 无法进行双向各10 mm测试；'。
            f'起点 {start / 100:.2f} mm 无法进行双向各10 mm测试；'
            # 学习：续写当前多行表达式中的一部分：f'本次保守位置范围为 {low / 100:.2f}～{high / 100:.2f} mm。'。
            f'本次保守位置范围为 {low / 100:.2f}～{high / 100:.2f} mm。'
            # 学习：续写当前多行表达式中的一部分：'请先用厂家软件回零，并低速移到约25 mm，随后重新启动本程序。')。
            '请先用厂家软件回零，并低速移到约25 mm，随后重新启动本程序。')
    # 学习：return把targets（依次执行的目标列表）交回调用者，并立即结束当前函数。
    return targets


# 学习：定义confirm函数：历史独立电缸测试使用的输入确认；不是A–G连续任务的逐段确认；def本身只是定义，调用时才执行函数体。
def confirm(message, token):
    # 学习：把“去掉文字两端空白与换行”保存到response（操作员输入的文字）；单个=表示赋值。
    response = input(f'{message}\n输入 {token} 继续，其他输入退出：').strip()
    # 学习：判断条件response != token；成立(True)时执行下方缩进代码。
    if response != token:
        # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError('操作员取消；不继续执行下一段。')


# 学习：定义move函数：执行历史独立电缸方向测试中的一次低速定位，并记录反馈；def本身只是定义，调用时才执行函数体。
def move(device, target, expected_start, low, high, report):
    # 学习：把“读取电缸关键寄存器快照”保存到before；单个=表示赋值。
    before = device.snapshot()
    # 学习：把“调用validate，按本行参数取得结果”保存到errors（检测到的问题列表）；单个=表示赋值。
    errors = validate(before, device.id)
    # 学习：判断条件errors；成立(True)时执行下方缩进代码。
    if errors:
        # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError('; '.join(errors))
    # 学习：判断条件abs(before['position'] - expected_start) > TOLERANCE_RAW；成立(True)时执行下方缩进代码。
    if abs(before['position'] - expected_start) > TOLERANCE_RAW:
        # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError('等待期间位置发生变化，测试中止，请重新读取起点。')
    # 学习：把“建立字典，用字段名保存各项状态或参数”保存到segment；单个=表示赋值。
    segment = {'from_mm': before['position'] / 100, 'target_mm': target / 100,
               # 学习：字典字段complete：本动作是否确认完成；本行给出它的值。
               'feedback': [], 'complete': False}
    # 学习：把一个新元素追加到列表末尾。
    report['segments'].append(segment)
    # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到values（解码得到的数值集合）；单个=表示赋值。
    values = [target, 0, SPEED_RAW, ACCEL_PERCENT, 0, 0x20]
    # 学习：构造定位报文头：站号1、功能码16(0x10)、起始地址5632(0x1600)、连续6字、数据12字节。
    prefix = struct.pack('>BBHHB', device.id, 16, 0x1600, 6, 12)
    # Clear the start bit, then raise it. Both packets carry the complete slow parameters.
    # 学习：把“True”保存到report['write_attempted']；单个=表示赋值。
    report['write_attempted'] = True
    # 学习：发送一帧写报文并校验对应应答，不自动重发。
    device.write_ack(prefix + struct.pack('>6H', *values))
    # 学习：把“33”保存到values[-1]；单个=表示赋值。
    values[-1] = 0x21
    # 学习：发送一帧写报文并校验对应应答，不自动重发。
    device.write_ack(prefix + struct.pack('>6H', *values))
    # 学习：把“计算time.monotonic() + abs(target - before['position']) / SPEED_RAW + 15”保存到deadline（允许本操作完成的最晚时刻）；单个=表示赋值。
    deadline = time.monotonic() + abs(target - before['position']) / SPEED_RAW + 15
    # 学习：把“0”保存到settled（连续满足到位条件的反馈次数）；单个=表示赋值。
    settled = 0
    # 学习：把“0”保存到last_print；单个=表示赋值。
    last_print = 0
    # 学习：while循环：只要time.monotonic() < deadline成立就继续；完成、异常或超时会结束等待。
    while time.monotonic() < deadline:
        # 学习：把“读取一个电缸寄存器并返回有效数值”保存到alarm（电缸报警寄存器值）；单个=表示赋值。
        alarm = device.value(0x160F)
        # 学习：把“读取一个电缸寄存器并返回有效数值”保存到status（检查结果或状态字）；单个=表示赋值。
        status = device.value(0x1611)
        # 学习：把“读取一个电缸寄存器并返回有效数值”保存到position；单个=表示赋值。
        position = device.value(0x160C)
        # 学习：把一个新元素追加到列表末尾。
        segment['feedback'].append({'time': time.time(), 'position': position,
                                    # 学习：字典字段alarm：报警码，0代表无报警；本行给出它的值。
                                    'status': status, 'alarm': alarm})
        # 学习：判断条件alarm or status & 1 << 9 or (not status & 1 << 10) or (not status & 1 << 6)；成立(True)时执行下方缩进代码。
        if alarm or status & (1 << 9) or not status & (1 << 10) or not status & (1 << 6):
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('报警、失能或回零状态丢失，终止测试。')
        # 学习：判断条件not low <= position <= high；成立(True)时执行下方缩进代码。
        if not low <= position <= high:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('位置超出本次保守测试范围。')
        # 学习：判断条件position < min(target, before['position']) - 30 or position > max(target, before['position']) + 30；成立(True)时执行下方缩进代码。
        if position < min(target, before['position']) - 30 or position > max(target, before['position']) + 30:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('反馈位置偏离本段预期范围。')
        # 学习：判断条件time.monotonic() - last_print > 0.5；成立(True)时执行下方缩进代码。
        if time.monotonic() - last_print > 0.5:
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print(f'当前位置 {position / 100:.2f} mm → 目标 {target / 100:.2f} mm', flush=True)
            # 学习：把“取得不会被系统校时倒拨影响的计时时刻（秒）”保存到last_print；单个=表示赋值。
            last_print = time.monotonic()
        # 学习：把“abs(position - target) <= TOLERANCE_RAW and (not status & 1 << 8) and status & 1 << 5”保存到arrived；单个=表示赋值。
        arrived = abs(position - target) <= TOLERANCE_RAW and not status & (1 << 8) and status & (1 << 5)
        # 学习：本次到位就把稳定计数加1；任一条件不满足则清零，避免瞬时反馈误判。
        settled = settled + 1 if arrived else 0
        # 学习：连续3次反馈满足到位条件才确认电缸完成。
        if settled >= 3:
            # 学习：批量更新字典中的字段。
            segment.update(complete=True, final_mm=position / 100)
            # 学习：return把控制权交回调用者，并立即结束当前函数。
            return
        # 学习：暂停当前线程0.08秒。
        time.sleep(0.08)
    # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
    raise RuntimeError('运动超时；不自动返回或重试。')


# 学习：定义main函数：电缸通信与历史方向测试模块；联动程序只复用Device、validate、bounds，历史独立测试仍保留原确认。；def本身只是定义，调用时才执行函数体。
def main():
    # 学习：把“调用argparse.ArgumentParser，按本行参数取得结果”保存到parser（命令行参数解析器）；单个=表示赋值。
    parser = argparse.ArgumentParser(description=__doc__)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--port', default='COM3')
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--baud', type=int, default=115200)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--id', type=int, default=1)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--check', action='store_true', help='Read position and validate only; no writes')
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--center', action='store_true', help='Prepare at 25 mm before capturing the test start')
    # 学习：把“解析本次命令行参数”保存到args（用户启动参数）；单个=表示赋值。
    args = parser.parse_args()
    # 学习：判断条件not 1 <= args.id <= 247；成立(True)时执行下方缩进代码。
    if not 1 <= args.id <= 247:
        # 学习：打印参数错误并结束参数解析。
        parser.error('ID must be 1..247')
    # 学习：把“建立字典，用字段名保存各项状态或参数”保存到report（运行或测试报告字典）；单个=表示赋值。
    report = {'time': datetime.now().astimezone().isoformat(), 'connection': vars(args),
              # 学习：字典条目：'distance_mm'对应10。
              'speed_mm_s': 1, 'acceleration_percent': ACCEL_PERCENT, 'distance_mm': 10,
              # 学习：字典字段success：流程是否成功完成；本行给出它的值。
              'segments': [], 'write_attempted': False, 'success': False}
    # 学习：把“None”保存到device（电缸通信对象）；单个=表示赋值。
    device = None
    # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
    try:
        # 学习：把“调用Device，按本行参数取得结果”保存到device（电缸通信对象）；单个=表示赋值。
        device = Device(args.port, args.baud, args.id)
        # 学习：把“读取电缸关键寄存器快照”保存到initial（动作起点TCP位姿）；单个=表示赋值。
        initial = device.snapshot()
        # 学习：把“initial（动作起点TCP位姿）”保存到report['initial']（准备控制时的只读检查快照）；单个=表示赋值。
        report['initial'] = initial
        # 学习：把“initial['position']（电缸实际位置，单位0.01 mm）”保存到start（该段开始时的状态或位置）；单个=表示赋值。
        start = initial['position']
        # 学习：把“计算start / 100”保存到report['original_position_mm']；单个=表示赋值。
        report['original_position_mm'] = start / 100
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(f'当前原始位置：{start / 100:.2f} mm；回零状态：{initial["homing"]}')
        # 学习：把“按名义行程、配置行程和原点偏置计算方向测试保守范围，返回原始位置单位”保存到(low, high)；单个=表示赋值。
        low, high = bounds(initial)
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到report['allowed_range_mm']；单个=表示赋值。
        report['allowed_range_mm'] = [low / 100, high / 100]
        # 学习：把“调用validate，按本行参数取得结果”保存到errors（检测到的问题列表）；单个=表示赋值。
        errors = validate(initial, args.id)
        # 学习：判断条件args.center and (not 0 <= start <= high)；成立(True)时执行下方缩进代码。
        if args.center and not 0 <= start <= high:
            # 学习：把一个新元素追加到列表末尾。
            errors.append('当前位置不适合执行向中部的准备运动。')
        # 学习：把“条件args.center成立取前一值，否则取后一值”保存到planned_start；单个=表示赋值。
        planned_start = 2500 if args.center else start
        # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
        try:
            # 学习：把“生成历史电缸方向测试的往返目标，并核对其处于保守范围”保存到targets（依次执行的目标列表）；单个=表示赋值。
            targets = plan(planned_start, low, high)
        # 学习：except捕获RuntimeError，执行下面的错误记录或清理逻辑。
        except RuntimeError as exc:
            # 学习：把一个新元素追加到列表末尾。
            errors.append(str(exc))
        # 学习：判断条件errors；成立(True)时执行下方缩进代码。
        if errors:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('\n'.join(errors))
        # 学习：把“逐项遍历并计算，生成所需数据：[p / 100 for p in targets]”保存到report['targets_mm']；单个=表示赋值。
        report['targets_mm'] = [p / 100 for p in targets]
        # 学习：依次列出当前→A→B→缸48→C→缸13→A→D→E→缸48→F→G→缸0→A；循环按此顺序连续执行。
        route = ([start, planned_start] if args.center else [start]) + targets
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print('路线：' + ' → '.join(f'{p / 100:.2f} mm' for p in route))
        # 学习：判断条件args.check；成立(True)时执行下方缩进代码。
        if args.check:
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print('检查通过。此次只读检查，没有发送写入指令。')
            # 学习：把“True”保存到report['success']（流程是否成功完成）；单个=表示赋值。
            report['success'] = True
            # 学习：return把0交回调用者，并立即结束当前函数。
            return 0
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print('速度1 mm/s，每段约10秒。正负指位置值增减，实际上下方向由现场观察。')
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print('Ctrl+C 会尝试发送暂停；通信或程序故障时不保证停止，现场需能切断驱动运动。')
        # 学习：历史独立电缸测试使用的输入确认；不是A–G连续任务的逐段确认。
        confirm('确认上述完整路线无障碍、负载可靠固定，并关闭其他控制软件。', 'START')
        # 学习：判断条件args.center；成立(True)时执行下方缩进代码。
        if args.center:
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print('先以1 mm/s移到25 mm，之后重新记录位置作为双向测试起点。')
            # 学习：执行历史独立电缸方向测试中的一次低速定位，并记录反馈。
            move(device, planned_start, start, min(low, start), high, report)
            # 学习：把“读取电缸关键寄存器快照”保存到centered；单个=表示赋值。
            centered = device.snapshot()
            # 学习：把“调用validate，按本行参数取得结果”保存到errors（检测到的问题列表）；单个=表示赋值。
            errors = validate(centered, args.id)
            # 学习：判断条件errors；成立(True)时执行下方缩进代码。
            if errors:
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('; '.join(errors))
            # 学习：把“centered['position']（电缸实际位置，单位0.01 mm）”保存到start（该段开始时的状态或位置）；单个=表示赋值。
            start = centered['position']
            # 学习：判断条件abs(start - planned_start) > TOLERANCE_RAW；成立(True)时执行下方缩进代码。
            if abs(start - planned_start) > TOLERANCE_RAW:
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('准备运动未稳定在25 mm附近。')
            # 学习：把“生成历史电缸方向测试的往返目标，并核对其处于保守范围”保存到targets（依次执行的目标列表）；单个=表示赋值。
            targets = plan(start, low, high)
            # 学习：把“逐项遍历并计算，生成所需数据：[p / 100 for p in targets]”保存到report['targets_mm']；单个=表示赋值。
            report['targets_mm'] = [p / 100 for p in targets]
            # 学习：历史独立电缸测试使用的输入确认；不是A–G连续任务的逐段确认。
            confirm(f'准备完成，测试起点为{start/100:.2f} mm，下一段正向10 mm。', 'GO')
        # 学习：把“计算start / 100”保存到report['start_mm']；单个=表示赋值。
        report['start_mm'] = start / 100
        # 学习：把“start（该段开始时的状态或位置）”保存到previous（上一行的下标）；单个=表示赋值。
        previous = start
        # 学习：for逐项循环：从enumerate(targets)取一个元素放入(index, target)，再执行缩进部分。
        for index, target in enumerate(targets):
            # 学习：判断条件index；成立(True)时执行下方缩进代码。
            if index:
                # 学习：历史独立电缸测试使用的输入确认；不是A–G连续任务的逐段确认。
                confirm(f'下一段将移动到 {target / 100:.2f} mm。', 'GO')
            # 学习：执行历史独立电缸方向测试中的一次低速定位，并记录反馈。
            move(device, target, previous, low, high, report)
            # 学习：把“target（本段目标）”保存到previous（上一行的下标）；单个=表示赋值。
            previous = target
            # 学习：判断条件index in (0, 2)；成立(True)时执行下方缩进代码。
            if index in (0, 2):
                # 学习：把“条件index == 0成立取前一值，否则取后一值”保存到direction；单个=表示赋值。
                direction = '位置值增加方向' if index == 0 else '位置值减少方向'
                # 学习：把“去掉文字两端空白与换行”保存到note；单个=表示赋值。
                note = input(f'{direction}已经走完10 mm。请记录实际方向（上/下/伸出/缩回等）：').strip()
                # 学习：把“note”保存到report.setdefault('observed_directions', {})[direction]；单个=表示赋值。
                report.setdefault('observed_directions', {})[direction] = note
        # 学习：把“True”保存到report['success']（流程是否成功完成）；单个=表示赋值。
        report['success'] = True
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print('两方向各10 mm测试完成，已返回本次起点。')
        # 学习：return把0交回调用者，并立即结束当前函数。
        return 0
    # 学习：except捕获(Exception, KeyboardInterrupt)，执行下面的错误记录或清理逻辑。
    except (Exception, KeyboardInterrupt) as exc:
        # 学习：把“str(exc) or '操作员按下 Ctrl+C'”保存到report['error']（错误说明）；单个=表示赋值。
        report['error'] = str(exc) or '操作员按下 Ctrl+C'
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(f'测试停止：{report["error"]}')
        # 学习：判断条件device is not None and report['write_attempted']；成立(True)时执行下方缩进代码。
        if device is not None and report['write_attempted']:
            # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
            try:
                # 学习：请求电缸保持使能的软件暂停。
                device.pause()
                # 学习：把“True”保存到report['pause_acknowledged']；单个=表示赋值。
                report['pause_acknowledged'] = True
                # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
                print('已收到暂停指令应答；仍需现场确认停止。不自动返回起点。')
            # 学习：except捕获(Exception, KeyboardInterrupt)，执行下面的错误记录或清理逻辑。
            except (Exception, KeyboardInterrupt) as stop_error:
                # 学习：把“转换或创建str类型的数据”保存到report['pause_error']；单个=表示赋值。
                report['pause_error'] = str(stop_error)
                # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
                print('暂停未确认！请立即使用现场停止装置。')
        # 学习：return把1交回调用者，并立即结束当前函数。
        return 1
    # 学习：finally中的资源释放与日志保存，无论前面成功、报错或被键盘中断都尝试执行。
    finally:
        # 学习：判断条件device is not None；成立(True)时执行下方缩进代码。
        if device is not None:
            # 学习：把“device.log”保存到report['frames']（原始收发帧记录）；单个=表示赋值。
            report['frames'] = device.log
            # 学习：关闭当前资源或串口。
            device.close()
        # 学习：把“计算Path(__file__).resolve().parent / 'logs' / ('direction_test_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + …”保存到output（生成文件或测试内存输出）；单个=表示赋值。
        output = Path(__file__).resolve().parent / 'logs' / ('direction_test_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
        # 学习：创建输出目录，已存在时按参数决定是否允许。
        output.parent.mkdir(parents=True, exist_ok=True)
        # 学习：以UTF-8写入文本文件。
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(f'测试记录：{output}')


# 学习：只有直接启动该文件时才执行入口；被其他文件import时跳过。
if __name__ == '__main__':
    # 学习：运行入口函数并以其返回值结束进程；0通常表示成功，1表示失败。
    raise SystemExit(main())
