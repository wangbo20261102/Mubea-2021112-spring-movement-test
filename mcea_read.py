# 学习：电缸只读通信模块：打开串口并读取寄存器；本文件不发送运动命令。
# 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
"""MCEA Modbus RTU reader. Only function 03 is implemented; no motion writes."""
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

# 学习：导入serial，用于通过pyserial访问串口。
import serial


# 学习：定义crc16函数：计算Modbus报文CRC16校验，结果低字节在前，用于识别传输错误；def本身只是定义，调用时才执行函数体。
def crc16(data):
    # 学习：把“65535”保存到crc（逐位更新的16位CRC校验值）；单个=表示赋值。
    crc = 0xFFFF
    # 学习：for逐项循环：从data取一个元素放入byte，再执行缩进部分。
    for byte in data:
        # 学习：把当前字节与CRC逐位异或；^=表示计算异或后写回crc。
        crc ^= byte
        # 学习：每个字节有8个二进制位，因此重复8轮CRC计算；_表示不使用本轮序号。
        for _ in range(8):
            # 学习：CRC右移1位；原最低位为1时再与多项式0xA001异或，否则与0异或。
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    # 学习：把16位CRC转换为2字节并返回；<H是小端无符号整数，即低字节先发送。
    return struct.pack('<H', crc)


# 学习：定义Reader类：封装串口读取和通信日志；self表示当前创建的那个对象。
class Reader:
    # 学习：定义__init__函数：按串口参数创建读取对象并打开串口；def本身只是定义，调用时才执行函数体。
    def __init__(self, port, baud, device_id, parity='N', stopbits=1):
        # 学习：把“device_id”保存到self.id（Modbus设备站号）；单个=表示赋值。
        self.id = device_id
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到self.log（串口收发记录列表）；单个=表示赋值。
        self.log = []
        # 学习：把“创建串口对象；port=None时暂不打开实际端口”保存到self.serial（底层串口对象）；单个=表示赋值。
        self.serial = serial.Serial(port=None, baudrate=baud, bytesize=8,
                                    # 学习：续写上方赋值的数据/参数：parity=parity, stopbits=stopbits,；这些内容仍属于同一条赋值语句。
                                    parity=parity, stopbits=stopbits,
                                    # 学习：续写上方赋值的数据/参数：timeout=0.03, write_timeout=1)；这些内容仍属于同一条赋值语句。
                                    timeout=0.03, write_timeout=1)
        # 学习：明确关闭串口握手控制线，按本次转换器的配置通信。
        self.serial.dtr = False
        # 学习：明确关闭串口握手控制线，按本次转换器的配置通信。
        self.serial.rts = False
        # 学习：把“port”保存到self.serial.port；单个=表示赋值。
        self.serial.port = port
        # 学习：打开已配置的串口。
        self.serial.open()
        # 学习：暂停当前线程0.1秒。
        time.sleep(0.1)

    # 学习：定义read函数：读取电缸一个寄存器，解析有效应答并保留原始收发记录；def本身只是定义，调用时才执行函数体。
    def read(self, address, timeout=0.45):
        # 学习：构造03只读请求：站号self.id、功能码3、寄存器address、读取1个16位字；>表示大端字节顺序。
        body = struct.pack('>BBHH', self.id, 3, address, 1)
        # 学习：把“计算body + crc16(body)”保存到request（包含CRC的Modbus请求）；单个=表示赋值。
        request = body + crc16(body)
        # 学习：清除旧串口输入，避免把旧报文当作新应答。
        self.serial.reset_input_buffer()
        # 学习：向当前通信接口发送已构造的字节报文；请求类型由报文中的功能码决定。
        self.serial.write(request)
        # 学习：等待输出缓冲区的数据发送完成。
        self.serial.flush()
        # 学习：把“转换或创建bytearray类型的数据”保存到data（接收的数据缓冲区）；单个=表示赋值。
        data = bytearray()
        # 学习：把“计算time.monotonic() + timeout”保存到deadline（允许本操作完成的最晚时刻）；单个=表示赋值。
        deadline = time.monotonic() + timeout
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到record（当前一次通信的收发记录）；单个=表示赋值。
        record = {'address': f'0x{address:04X}', 'tx': request.hex(' ')}
        # 学习：while循环：只要time.monotonic() < deadline成立就继续；完成、异常或超时会结束等待。
        while time.monotonic() < deadline:
            # 学习：把另一组元素或字节追加到缓冲区末尾。
            data.extend(self.serial.read(self.serial.in_waiting or 1))
            # 学习：for逐项循环：从range(max(0, len(data) - 256), len(data) - 4)取一个元素放入offset，再执行缩进部分。
            for offset in range(max(0, len(data) - 256), len(data) - 4):
                # 学习：判断条件data[offset] != self.id；成立(True)时执行下方缩进代码。
                if data[offset] != self.id:
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：把“data[offset + 1]”保存到function（Modbus功能码）；单个=表示赋值。
                function = data[offset + 1]
                # 学习：把“条件function == 131成立取前一值，否则取后一值”保存到length（本帧预期字节数）；单个=表示赋值。
                length = 5 if function == 0x83 else 7
                # 学习：判断条件function not in (3, 131) or offset + length > len(data)；成立(True)时执行下方缩进代码。
                if function not in (3, 0x83) or offset + length > len(data):
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：把“转换或创建bytes类型的数据”保存到frame（截取的一帧完整通信数据）；单个=表示赋值。
                frame = bytes(data[offset:offset + length])
                # 学习：判断条件crc16(frame[:-2]) != frame[-2:]；成立(True)时执行下方缩进代码。
                if crc16(frame[:-2]) != frame[-2:]:
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：判断条件function == 3 and frame[2] != 2；成立(True)时执行下方缩进代码。
                if function == 3 and frame[2] != 2:
                    # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                    continue
                # 学习：把“把字节转为便于查看的十六进制文本”保存到record['rx']（接收报文的十六进制文本）；单个=表示赋值。
                record['rx'] = frame.hex(' ')
                # 学习：判断条件function == 131；成立(True)时执行下方缩进代码。
                if function == 0x83:
                    # 学习：把“f'Modbus exception {frame[2]}'”保存到record['error']（错误说明）；单个=表示赋值。
                    record['error'] = f'Modbus exception {frame[2]}'
                # 学习：else处理前面条件不成立的分支。
                else:
                    # 学习：把“调用int.from_bytes，按本行参数取得结果”保存到record['value']（有效的寄存器数值）；单个=表示赋值。
                    record['value'] = int.from_bytes(frame[3:5], 'big')
                # 学习：把一个新元素追加到列表末尾。
                self.log.append(record)
                # 学习：暂停当前线程0.02秒。
                time.sleep(0.02)
                # 学习：return把record（当前一次通信的收发记录）交回调用者，并立即结束当前函数。
                return record
        # 学习：批量更新字典中的字段。
        record.update(rx=bytes(data).hex(' '), error='No valid response before timeout')
        # 学习：把一个新元素追加到列表末尾。
        self.log.append(record)
        # 学习：return把record（当前一次通信的收发记录）交回调用者，并立即结束当前函数。
        return record

    # 学习：定义close函数：关闭串口，释放被程序占用的通信资源；def本身只是定义，调用时才执行函数体。
    def close(self):
        # 学习：关闭当前资源或串口。
        self.serial.close()


# Documented basic readable registers; unavailable registers are recorded as errors.
# 学习：把“建立字典，用字段名保存各项状态或参数”保存到REGISTERS（电缸可读寄存器地址与名称的对照表）；单个=表示赋值。
REGISTERS = {
    # 学习：寄存器0x0100：回零指令状态，结果以homing_command_state为字段名保存。
    0x0100: 'homing_command_state',
    # 学习：寄存器0x0101：推力百分比，结果以push_force_percent为字段名保存。
    0x0101: 'push_force_percent',
    # 学习：寄存器0x0102：推压距离，单位0.01 mm，结果以push_distance_0_01mm为字段名保存。
    0x0102: 'push_distance_0_01mm',
    # 学习：寄存器0x0103：目标位置，单位0.01 mm，结果以target_position_0_01mm为字段名保存。
    0x0103: 'target_position_0_01mm',
    # 学习：寄存器0x0104：基本速度百分比，结果以speed_percent为字段名保存。
    0x0104: 'speed_percent',
    # 学习：寄存器0x0105：加减速度百分比，结果以acceleration_percent为字段名保存。
    0x0105: 'acceleration_percent',
    # 学习：寄存器0x0106：相对位置原始值，结果以relative_position_raw为字段名保存。
    0x0106: 'relative_position_raw',
    # 学习：寄存器0x0107：点动命令原始值，结果以jog_command_raw为字段名保存。
    0x0107: 'jog_command_raw',
    # 学习：寄存器0x0200：回零状态，0未回零、1完成、2进行中，结果以homing_status为字段名保存。
    0x0200: 'homing_status',
    # 学习：寄存器0x0201：运动状态，0运动中、1到位、2堵转，结果以motion_status为字段名保存。
    0x0201: 'motion_status',
    # 学习：寄存器0x0202：实际位置，单位0.01 mm，结果以actual_position_0_01mm为字段名保存。
    0x0202: 'actual_position_0_01mm',
    # 学习：寄存器0x0204：电机电流原始值，结果以motor_current_raw为字段名保存。
    0x0204: 'motor_current_raw',
    # 学习：寄存器0x0301：回零方向配置，结果以homing_direction为字段名保存。
    0x0301: 'homing_direction',
    # 学习：寄存器0x0302：Modbus站号，结果以device_id为字段名保存。
    0x0302: 'device_id',
    # 学习：寄存器0x0303：波特率配置代码，结果以baud_code为字段名保存。
    0x0303: 'baud_code',
    # 学习：寄存器0x0304：串口停止位代码，结果以stopbits_code为字段名保存。
    0x0304: 'stopbits_code',
    # 学习：寄存器0x0305：串口校验方式代码，结果以parity_code为字段名保存。
    0x0305: 'parity_code',
    # 学习：寄存器0x0306：行程限制，单位0.01 mm，结果以stroke_limit_0_01mm为字段名保存。
    0x0306: 'stroke_limit_0_01mm',
    # 学习：寄存器0x0308：原点偏置，单位0.01 mm，结果以origin_offset_0_01mm为字段名保存。
    0x0308: 'origin_offset_0_01mm',
    # 学习：寄存器0x0309：推压速度百分比，结果以push_speed_percent为字段名保存。
    0x0309: 'push_speed_percent',
    # 学习：寄存器0x030A：推压方向，结果以push_direction为字段名保存。
    0x030A: 'push_direction',
    # 学习：寄存器0x0402：是否启用I/O控制模式，结果以io_mode_enabled为字段名保存。
    0x0402: 'io_mode_enabled',
    # 学习：寄存器0x0504：上电自动回零配置，结果以power_on_auto_home为字段名保存。
    0x0504: 'power_on_auto_home',
    # 学习：寄存器0x1600：扩展目标位置，单位0.01 mm，结果以extended_target_position_0_01mm为字段名保存。
    0x1600: 'extended_target_position_0_01mm',
    # 学习：寄存器0x1601：扩展推压距离，单位0.01 mm，结果以extended_push_distance_0_01mm为字段名保存。
    0x1601: 'extended_push_distance_0_01mm',
    # 学习：寄存器0x1602：扩展速度，单位0.01 mm/s，结果以extended_speed_0_01mm_s为字段名保存。
    0x1602: 'extended_speed_0_01mm_s',
    # 学习：寄存器0x1603：扩展加减速度百分比，结果以extended_acceleration_percent为字段名保存。
    0x1603: 'extended_acceleration_percent',
    # 学习：寄存器0x1604：扩展推力百分比，结果以extended_push_force_percent为字段名保存。
    0x1604: 'extended_push_force_percent',
    # 学习：寄存器0x1605：扩展控制字，位0定位、位3暂停、位5使能，结果以extended_control_word为字段名保存。
    0x1605: 'extended_control_word',
    # 学习：寄存器0x160C：扩展实际位置，单位0.01 mm，结果以extended_actual_position_0_01mm为字段名保存。
    0x160C: 'extended_actual_position_0_01mm',
    # 学习：寄存器0x160D：扩展电机电流原始值，字段按mA标记，结果以extended_motor_current_mA_raw为字段名保存。
    0x160D: 'extended_motor_current_mA_raw',
    # 学习：寄存器0x160E：扩展实际速度原始值，单位0.01 mm/s，结果以extended_actual_speed_0_01mm_s_raw为字段名保存。
    0x160E: 'extended_actual_speed_0_01mm_s_raw',
    # 学习：寄存器0x160F：扩展报警码，0表示无报警，结果以extended_alarm_code为字段名保存。
    0x160F: 'extended_alarm_code',
    # 学习：寄存器0x1610：保留/急停相关原始字段，不据此猜测全部状态，结果以extended_reserved_emergency_status_raw为字段名保存。
    0x1610: 'extended_reserved_emergency_status_raw',
    # 学习：寄存器0x1611：扩展状态字，用不同二进制位表示使能、报警、运动、回零和到位，结果以extended_status_word为字段名保存。
    0x1611: 'extended_status_word',
# 学习：关闭上方开始的括号/列表/字典；它和前面的多行内容合成一条完整表达式。
}
# 学习：for逐项循环：从range(4)取一个元素放入group，再执行缩进部分。
for group in range(4):
    # 学习：for逐项循环：从enumerate(('position_0_01mm', 'acceleration_percent', 'speed_percent', 'push_distance_0_01mm', 'push_force_per…取一个元素放入(field, suffix)，再执行缩进部分。
    for field, suffix in enumerate(('position_0_01mm', 'acceleration_percent',
                                    # 学习：续写for要遍历的数据范围或元素集合，本行不会单独启动新循环。
                                    'speed_percent', 'push_distance_0_01mm',
                                    # 学习：续写for要遍历的数据范围或元素集合，本行不会单独启动新循环。
                                    'push_force_percent')):
        # 学习：把“f'io_group_{group + 1}_{suffix}'”保存到REGISTERS[1029 + group * 5 + field]；单个=表示赋值。
        REGISTERS[0x0405 + group * 5 + field] = f'io_group_{group + 1}_{suffix}'


# 学习：定义main函数：电缸只读通信模块：打开串口并读取寄存器；本文件不发送运动命令。；def本身只是定义，调用时才执行函数体。
def main():
    # 学习：把“调用argparse.ArgumentParser，按本行参数取得结果”保存到parser（命令行参数解析器）；单个=表示赋值。
    parser = argparse.ArgumentParser(description=__doc__)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--port', default='COM3')
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--id', type=int, default=1)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--baud', type=int, default=115200)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--parity', choices=['N', 'E', 'O'], default='N')
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--stopbits', type=int, choices=[1, 2], default=1)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--probe', action='store_true', help='Only read ID and homing status')
    # 学习：把“解析本次命令行参数”保存到args（用户启动参数）；单个=表示赋值。
    args = parser.parse_args()
    # 学习：判断条件not 1 <= args.id <= 247；成立(True)时执行下方缩进代码。
    if not 1 <= args.id <= 247:
        # 学习：打印参数错误并结束参数解析。
        parser.error('Device ID must be 1..247; broadcast is not supported')
    # 学习：把“建立字典，用字段名保存各项状态或参数”保存到report（运行或测试报告字典）；单个=表示赋值。
    report = {'time': datetime.now().astimezone().isoformat(), 'connection': vars(args),
              # 学习：字典字段communication_confirmed：是否已确认有效设备应答；本行给出它的值。
              'protocol': 'Modbus RTU, function 03 only', 'communication_confirmed': False,
              # 学习：字典条目：'identity_note'对应'Register responses alone do not verify nameplate model or serial number.'。
              'identity_note': 'Register responses alone do not verify nameplate model or serial number.'}
    # 学习：把“None”保存到reader（电缸只读通信对象）；单个=表示赋值。
    reader = None
    # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
    try:
        # 学习：把“调用Reader，按本行参数取得结果”保存到reader（电缸只读通信对象）；单个=表示赋值。
        reader = Reader(args.port, args.baud, args.id, args.parity, args.stopbits)
        # A matching ID readback helps distinguish device replies from noise/echo.
        # 学习：把“读取电缸一个寄存器，解析有效应答并保留原始收发记录”保存到identity（读取的设备站号寄存器结果）；单个=表示赋值。
        identity = reader.read(0x0302)
        # 学习：把“读取电缸一个寄存器，解析有效应答并保留原始收发记录”保存到status（检查结果或状态字）；单个=表示赋值。
        status = reader.read(0x0200)
        # 学习：把“identity.get('value') == args.id and 'value' in status”保存到confirmed（是否收到站号和回零状态的有效应答）；单个=表示赋值。
        confirmed = identity.get('value') == args.id and 'value' in status
        # 学习：把“confirmed（是否收到站号和回零状态的有效应答）”保存到report['communication_confirmed']（是否已确认有效设备应答）；单个=表示赋值。
        report['communication_confirmed'] = confirmed
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到report['probe']；单个=表示赋值。
        report['probe'] = [identity, status]
        # 学习：判断条件confirmed；成立(True)时执行下方缩进代码。
        if confirmed:
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print('Communication confirmed: ID register and homing status returned valid CRCs.')
            # 学习：判断条件not args.probe；成立(True)时执行下方缩进代码。
            if not args.probe:
                # 学习：把“建立字典，用字段名保存各项状态或参数”保存到report['parameters']；单个=表示赋值。
                report['parameters'] = {}
                # 学习：for逐项循环：从REGISTERS.items()取一个元素放入(address, name)，再执行缩进部分。
                for address, name in REGISTERS.items():
                    # 学习：把“读取电缸一个寄存器，解析有效应答并保留原始收发记录”保存到result（当前函数累积的结果）；单个=表示赋值。
                    result = reader.read(address)
                    # 学习：把“result（当前函数累积的结果）”保存到report['parameters'][name]；单个=表示赋值。
                    report['parameters'][name] = result
                    # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
                    print(f"{result['address']} {name}: {result.get('value', result.get('error'))}")
                # 学习：把“逐项遍历并计算，生成所需数据：{name: item['value'] for name, item in report['parameters'].items() if 'value' in item}”保存到values（解码得到的数值集合）；单个=表示赋值。
                values = {name: item['value'] for name, item in report['parameters'].items() if 'value' in item}
                # 学习：把“建立字典，用字段名保存各项状态或参数”保存到report['decoded']；单个=表示赋值。
                report['decoded'] = {}
                # 学习：for逐项循环：从values.items()取一个元素放入(name, value)，再执行缩进部分。
                for name, value in values.items():
                    # 学习：判断条件name.endswith('_0_01mm')；成立(True)时执行下方缩进代码。
                    if name.endswith('_0_01mm'):
                        # 学习：把“计算value / 100”保存到report['decoded'][name.replace('_0_01mm', '_mm')]；单个=表示赋值。
                        report['decoded'][name.replace('_0_01mm', '_mm')] = value / 100
                # 学习：把“从字典读取字段，不存在时使用给定默认值”保存到word（当前读取或准备的16位状态/控制字）；单个=表示赋值。
                word = values.get('extended_status_word')
                # 学习：判断条件word is not None；成立(True)时执行下方缩进代码。
                if word is not None:
                    # 学习：把“逐项遍历并计算，生成所需数据：{label: bool(word & 1 << bit) for bit, label in [(15, 'controller_ready'), (10, 'motor_enabled'…”保存到report['decoded']['extended_status_bits']；单个=表示赋值。
                    report['decoded']['extended_status_bits'] = {
                        # 学习：续写上方赋值的数据/参数：label: bool(word & (1 << bit)) for bit, label in；这些内容仍属于同一条赋值语句。
                        label: bool(word & (1 << bit)) for bit, label in
                        # 学习：续写上方赋值的数据/参数：[(15, 'controller_ready'), (10, 'motor_enabled'), (9, 'alarm_active'),；这些内容仍属于同一条赋值语句。
                        [(15, 'controller_ready'), (10, 'motor_enabled'), (9, 'alarm_active'),
                         # 学习：续写上方赋值的数据/参数：(8, 'moving'), (6, 'homed'), (5, 'in_position')]；这些内容仍属于同一条赋值语句。
                         (8, 'moving'), (6, 'homed'), (5, 'in_position')]
                    # 学习：关闭上方开始的括号/列表/字典；它和前面的多行内容合成一条完整表达式。
                    }
        # 学习：else处理前面条件不成立的分支。
        else:
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print('Communication NOT confirmed. Inspect report before any further action.')
    # 学习：except捕获(serial.SerialException, OSError)，执行下面的错误记录或清理逻辑。
    except (serial.SerialException, OSError) as exc:
        # 学习：把“转换或创建str类型的数据”保存到report['error']（错误说明）；单个=表示赋值。
        report['error'] = str(exc)
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(f'Connection error: {exc}')
    # 学习：finally中的资源释放与日志保存，无论前面成功、报错或被键盘中断都尝试执行。
    finally:
        # 学习：判断条件reader is not None；成立(True)时执行下方缩进代码。
        if reader is not None:
            # 学习：把“reader.log”保存到report['frames']（原始收发帧记录）；单个=表示赋值。
            report['frames'] = reader.log
            # 学习：关闭当前资源或串口。
            reader.close()
        # 学习：把“计算Path(__file__).resolve().parent / 'logs' / ('mcea_read_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jso…”保存到destination（报告保存路径）；单个=表示赋值。
        destination = Path(__file__).resolve().parent / 'logs' / ('mcea_read_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
        # 学习：创建输出目录，已存在时按参数决定是否允许。
        destination.parent.mkdir(parents=True, exist_ok=True)
        # 学习：以UTF-8写入文本文件。
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(f'Report: {destination}')
    # 学习：return把条件report['communication_confirmed']成立取前一值，否则取后一值交回调用者，并立即结束当前函数。
    return 0 if report['communication_confirmed'] else 1


# 学习：只有直接启动该文件时才执行入口；被其他文件import时跳过。
if __name__ == '__main__':
    # 学习：运行入口函数并以其返回值结束进程；0通常表示成功，1表示失败。
    raise SystemExit(main())
