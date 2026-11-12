# 学习：独立连续运行主程序；各功能已在下方逐行说明，坐标m、姿态rad；启动后仍需一次RUN或RECOVER确认。
"""A-G manually started, continuous single-cycle hardware runner.
Robot 192.168.21.5; cylinder COM4,115200,8N1,id1.
Hold the orientation captured from hardware on 2026-11-12 13:10:55 UTC+8.
No rotation-to-zero stage. No joint lock. Robot30/cylinder1 mm/s.
After interruption, RECOVER confirms continuous return-to-start preparation and one new cycle.
Dependencies: pyserial==3.5, ur_rtde==1.6.5.
Importing this file never connects or moves hardware.
User launch: python ag_run.py (one RUN confirmation, then continuous execution).
Read only: --check. Offline preview (starting at A): --preview.
Generated from reviewed local trial modules; hardware motion not validated.
"""
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


# 学习：导入argparse，用于读取命令行参数。
import argparse


# 学习：导入csv，用于输出模拟采样表格。
import csv


# 学习：导入json，用于读写JSON数据文件。
import json


# 学习：导入math，用于计算距离、平方根和三角函数。
import math


# 学习：从pathlib导入Path，用于以Path对象处理文件与目录。
from pathlib import Path


# 学习：把“建立字典，用字段名保存各项状态或参数”保存到POINTS（底座坐标系实测A–G位置，单位mm）；单个=表示赋值。
POINTS = {
    # 学习：字典条目：'B'对应建立按顺序排列的列表，方括号内是各个元素。
    'A': [-337.9, -424.4, 382.0], 'B': [-340.0, -422.4, 443.0],
    # 学习：字典条目：'D'对应建立按顺序排列的列表，方括号内是各个元素。
    'C': [-330.6, -431.6, 445.1], 'D': [70.5, -436.3, 392.1],
    # 学习：字典条目：'F'对应建立按顺序排列的列表，方括号内是各个元素。
    'E': [79.4, -412.1, 216.7], 'F': [93.3, -456.2, 229.7],
    # 学习：字典条目：'G'对应建立按顺序排列的列表，方括号内是各个元素。
    'G': [67.9, -465.0, 410.0],
# 学习：关闭上方开始的括号/列表/字典；它和前面的多行内容合成一条完整表达式。
}


# 学习：把“30.0”保存到ROBOT_SPEED（模拟机械臂速度上限30 mm/s）；单个=表示赋值。
ROBOT_SPEED = 30.0  # mm/s; includes horizontal, vertical and diagonal moves


# 学习：把“1.0”保存到CYLINDER_SPEED（模拟电缸速度上限1 mm/s）；单个=表示赋值。
CYLINDER_SPEED = 1.0  # mm/s; unchanged


# 学习：把“1.0”保存到ACCEL（模拟加速度1 mm/s²）；单个=表示赋值。
ACCEL = 1.0  # mm/s², chosen for the virtual model only


# 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到TOOL_AXIS（工具自身Z轴在底座坐标系中的方向）；单个=表示赋值。
TOOL_AXIS = [-0.013476970671436439, -0.999907139719671, 0.0020206927394919583]


# 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到ROTATION（该旋转向量对应的3×3旋转矩阵）；单个=表示赋值。
ROTATION = [[0.7061936829343256, -0.7078904247446596, -0.013476970671436439], [-0.008087507651501125, 0.01096832519635993, -0.999907139719671], [0.7079725096383618, 0.7062371006943862, 0.0020206927394919583]]


# 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到ROTATION_VECTOR（模拟保持的旋转向量，单位rad）；单个=表示赋值。
ROTATION_VECTOR = [1.4747881408617243, -0.6236196873562871, 0.6049084354198352]


# 学习：定义motion_profile函数：按距离、速度和加速度计算三角形或梯形速度曲线各段时间；def本身只是定义，调用时才执行函数体。
def motion_profile(distance, speed, accel=ACCEL):
    # 学习：判断条件distance <= 1e-12；成立(True)时执行下方缩进代码。
    if distance <= 1e-12:
        # 学习：return把组成一组固定顺序的数据，便于成组赋值或返回交回调用者，并立即结束当前函数。
        return 0.0, 0.0, 0.0
    # 学习：把“取几个数值中的最小值”保存到ramp（加速或减速单段时间，单位秒）；单个=表示赋值。
    ramp = min(speed / accel, math.sqrt(distance / accel))
    # 学习：把“计算accel * ramp”保存到peak（短距离情况下实际能达到的模拟峰值速度）；单个=表示赋值。
    peak = accel * ramp
    # 学习：把“取几个数值中的最大值”保存到cruise（匀速阶段时间，单位秒）；单个=表示赋值。
    cruise = max(0.0, (distance - accel * ramp * ramp) / peak)
    # 学习：return把组成一组固定顺序的数据，便于成组赋值或返回交回调用者，并立即结束当前函数。
    return ramp, cruise, 2 * ramp + cruise


# 学习：定义displacement函数：计算给定时刻在速度曲线下已走过的距离，仅用于模拟；def本身只是定义，调用时才执行函数体。
def displacement(distance, elapsed, speed, accel=ACCEL):
    # 学习：把“按距离、速度和加速度计算三角形或梯形速度曲线各段时间”保存到(ramp, cruise, duration)；单个=表示赋值。
    ramp, cruise, duration = motion_profile(distance, speed, accel)
    # 学习：把“取几个数值中的最大值”保存到t（模拟时刻或当前协议字段类型）；单个=表示赋值。
    t = max(0.0, min(duration, elapsed))
    # 学习：判断条件t <= ramp；成立(True)时执行下方缩进代码。
    if t <= ramp:
        # 学习：return把计算0.5 * accel * t * t交回调用者，并立即结束当前函数。
        return 0.5 * accel * t * t
    # 学习：判断条件t <= ramp + cruise；成立(True)时执行下方缩进代码。
    if t <= ramp + cruise:
        # 学习：return把计算0.5 * accel * ramp * ramp + accel * ramp * (t - ramp)交回调用者，并立即结束当前函数。
        return 0.5 * accel * ramp * ramp + accel * ramp * (t-ramp)
    # 学习：return把计算distance - 0.5 * accel * (duration - t) ** 2交回调用者，并立即结束当前函数。
    return distance - 0.5 * accel * (duration-t)**2


# 学习：定义build_plan函数：按A–G顺序生成离线计划，并为每个到位动作插入1秒停顿；def本身只是定义，调用时才执行函数体。
def build_plan(start=None):
    # 学习：把“创建列表副本，数据来自POINTS['A'] if start is None else start”保存到xyz（模拟TCP位置，单位mm）；单个=表示赋值。
    xyz = list(POINTS['A'] if start is None else start)
    # 学习：把“组成一组固定顺序的数据，便于成组赋值或返回”保存到(cylinder, clock, phases)；单个=表示赋值。
    cylinder, clock, phases = 0.0, 0.0, []

    # 学习：定义add函数：给离线计划添加一个动作或停顿阶段，并推进模拟时间；def本身只是定义，调用时才执行函数体。
    def add(step, label, axis, target=None):
        # 学习：nonlocal允许当前内部函数更新外层的xyz, cylinder, clock，避免创建同名局部变量。
        nonlocal xyz, cylinder, clock
        # 学习：把“组成一组固定顺序的数据，便于成组赋值或返回”保存到(old_xyz, old_cyl)；单个=表示赋值。
        old_xyz, old_cyl = xyz[:], cylinder
        # 学习：把“0.0”保存到distance（本段位移长度）；单个=表示赋值。
        distance = 0.0
        # 学习：判断条件axis == 'robot'；成立(True)时执行下方缩进代码。
        if axis == 'robot':
            # 学习：把“创建列表副本，数据来自target”保存到xyz（模拟TCP位置，单位mm）；单个=表示赋值。
            xyz = list(target)
            # 学习：把“计算两个坐标向量之间的直线距离”保存到distance（本段位移长度）；单个=表示赋值。
            distance = math.dist(old_xyz, xyz)
        # 学习：判断条件axis == 'cylinder'；成立(True)时执行下方缩进代码。
        elif axis == 'cylinder':
            # 学习：把“转换或创建float类型的数据”保存到cylinder（电缸状态字典）；单个=表示赋值。
            cylinder = float(target)
            # 学习：把“取得绝对值，去掉正负方向只比较差距”保存到distance（本段位移长度）；单个=表示赋值。
            distance = abs(cylinder-old_cyl)
        # 学习：把“条件axis == 'robot'成立取前一值，否则取后一值”保存到speed（速度数据或本阶段速度上限）；单个=表示赋值。
        speed = ROBOT_SPEED if axis == 'robot' else CYLINDER_SPEED
        # 学习：把“条件axis == 'dwell'成立取前一值，否则取后一值”保存到duration（当前阶段预计用时，单位秒）；单个=表示赋值。
        duration = 1.0 if axis == 'dwell' else motion_profile(distance, speed)[2]
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到phase（模拟阶段数据）；单个=表示赋值。
        phase = dict(index=len(phases), step=step, label=label, axis=axis,
                     # 学习：续写上方赋值的数据/参数：start_s=clock, duration_s=duration, end_s=clock+duration,；这些内容仍属于同一条赋值语句。
                     start_s=clock, duration_s=duration, end_s=clock+duration,
                     # 学习：续写上方赋值的数据/参数：from_xyz_mm=old_xyz, to_xyz_mm=xyz[:],；这些内容仍属于同一条赋值语句。
                     from_xyz_mm=old_xyz, to_xyz_mm=xyz[:],
                     # 学习：续写上方赋值的数据/参数：from_cylinder_mm=old_cyl, to_cylinder_mm=cylinder,；这些内容仍属于同一条赋值语句。
                     from_cylinder_mm=old_cyl, to_cylinder_mm=cylinder,
                     # 学习：续写上方赋值的数据/参数：distance_mm=distance, rotation_vector_rad=ROTATION_VECTOR[:], max_speed_mm_s=speed,；这些内容仍属于同一条赋值语句。
                     distance_mm=distance, rotation_vector_rad=ROTATION_VECTOR[:], max_speed_mm_s=speed,
                     # 学习：续写上方赋值的数据/参数：virtual_acceleration_mm_s2=ACCEL)；这些内容仍属于同一条赋值语句。
                     virtual_acceleration_mm_s2=ACCEL)
        # 学习：把一个新元素追加到列表末尾。
        phases.append(phase)
        # 学习：在原值基础上更新clock（模拟累计时间，单位秒）：clock += duration。
        clock += duration

    # 学习：定义robot函数：给模拟计划添加机器人到点动作及1秒停顿；def本身只是定义，调用时才执行函数体。
    def robot(step, point):
        # 学习：把一个值加入集合，自动去重。
        add(step, f'机械臂 → {point}', 'robot', POINTS[point])
        # 学习：把一个值加入集合，自动去重。
        add(step, '到位停顿1秒', 'dwell')

    # 学习：定义cyl函数：为离线模拟添加一次电缸定位和随后的1秒停顿；def本身只是定义，调用时才执行函数体。
    def cyl(step, position):
        # 学习：把一个值加入集合，自动去重。
        add(step, f'电缸 → {position} mm', 'cylinder', position)
        # 学习：把一个值加入集合，自动去重。
        add(step, '到位停顿1秒', 'dwell')

    # 学习：判断条件math.dist(xyz, POINTS['A']) > 1e-09；成立(True)时执行下方缩进代码。
    if math.dist(xyz, POINTS['A']) > 1e-9:
        # 学习：执行一次机器人到点动作并确认到位。
        robot('准备', 'A')
    # 学习：把一个值加入集合，自动去重。
    add('1', '虚拟电缸已回零，位置0 mm', 'check')
    # 学习：执行一次机器人到点动作并确认到位。
    robot('1', 'B')
    # 学习：为离线模拟添加一次电缸定位和随后的1秒停顿。
    cyl('2', 48)
    # 学习：执行一次机器人到点动作并确认到位。
    robot('4', 'C')
    # 学习：为离线模拟添加一次电缸定位和随后的1秒停顿。
    cyl('5', 13)
    # 学习：执行一次机器人到点动作并确认到位。
    robot('6', 'A')
    # 学习：执行一次机器人到点动作并确认到位。
    robot('7', 'D')
    # 学习：执行一次机器人到点动作并确认到位。
    robot('8', 'E')
    # 学习：为离线模拟添加一次电缸定位和随后的1秒停顿。
    cyl('9', 48)
    # 学习：执行一次机器人到点动作并确认到位。
    robot('10', 'F')
    # 学习：执行一次机器人到点动作并确认到位。
    robot('11', 'G')
    # Explicit serial ordering chosen for offline simulation of step 12.
    # 学习：为离线模拟添加一次电缸定位和随后的1秒停顿。
    cyl('12a', 0)
    # 学习：执行一次机器人到点动作并确认到位。
    robot('12b', 'A')
    # 学习：return把创建一份键值数据字典，或复制已有字典交回调用者，并立即结束当前函数。
    return dict(mode='offline_geometric_only', hardware_connections=False,
                # 学习：续写当前多行表达式中的一部分：units='mm,s', points=POINTS, initial_xyz_mm=list(start or POINTS['A']),。
                units='mm,s', points=POINTS, initial_xyz_mm=list(start or POINTS['A']),
                # 学习：续写当前多行表达式中的一部分：initial_cylinder_mm=0, robot_max_speed_mm_s=ROBOT_SPEED,。
                initial_cylinder_mm=0, robot_max_speed_mm_s=ROBOT_SPEED,
                # 学习：续写当前多行表达式中的一部分：cylinder_max_speed_mm_s=CYLINDER_SPEED,。
                cylinder_max_speed_mm_s=CYLINDER_SPEED,
                # 学习：续写当前多行表达式中的一部分：virtual_acceleration_mm_s2=ACCEL,。
                virtual_acceleration_mm_s2=ACCEL,
                # 学习：续写当前多行表达式中的一部分：tool_axis_in_base=TOOL_AXIS,。
                tool_axis_in_base=TOOL_AXIS,
                # 学习：续写当前多行表达式中的一部分：orientation_frame='robot_base', orientation_representation='rotation_vector_rad',。
                orientation_frame='robot_base', orientation_representation='rotation_vector_rad',
                # 学习：续写当前多行表达式中的一部分：rotation_matrix=ROTATION, rotation_vector_rad=ROTATION_VECTOR,。
                rotation_matrix=ROTATION, rotation_vector_rad=ROTATION_VECTOR,
                # 学习：续写当前多行表达式中的一部分：target_tcp_poses_m_rad={k:[x/1000 for x in v]+ROTATION_VECTOR for k,v in POINTS.items()},。
                target_tcp_poses_m_rad={k:[x/1000 for x in v]+ROTATION_VECTOR for k,v in POINTS.items()},
                # 学习：续写当前多行表达式中的一部分：orientation_heading_policy='fixed full orientation captured from hardware',。
                orientation_heading_policy='fixed full orientation captured from hardware',
                # 学习：续写当前多行表达式中的一部分：locked_joints=[],。
                locked_joints=[],
                # 学习：续写当前多行表达式中的一部分：joint_trajectory_status='not_solved',。
                joint_trajectory_status='not_solved',
                # 学习：续写当前多行表达式中的一部分：timing_is_geometric_estimate=True,。
                timing_is_geometric_estimate=True,
                # 学习：续写当前多行表达式中的一部分：orientation_is_hardware_calibrated=False,。
                orientation_is_hardware_calibrated=False,
                # 学习：续写当前多行表达式中的一部分：cylinder_positive='down', cylinder_zero_is_virtual=True,。
                cylinder_positive='down', cylinder_zero_is_virtual=True,
                # 学习：续写当前多行表达式中的一部分：tcp_and_cylinder_are_separate_coordinates=True,。
                tcp_and_cylinder_are_separate_coordinates=True,
                # 学习：续写当前多行表达式中的一部分：final_return_order='retract cylinder to 0 at G, dwell, robot G to A, dwell',。
                final_return_order='retract cylinder to 0 at G, dwell, robot G to A, dwell',
                # 学习：续写当前多行表达式中的一部分：simulated_homing_search=False, simulated_ik_or_collision=False,。
                simulated_homing_search=False, simulated_ik_or_collision=False,
                # 学习：续写当前多行表达式中的一部分：total_duration_s=clock, phases=phases)。
                total_duration_s=clock, phases=phases)


# 学习：导入argparse，用于读取命令行参数。
import argparse


# 学习：导入json，用于读写JSON数据文件。
import json


# 学习：导入math，用于计算距离、平方根和三角函数。
import math


# 学习：导入re，用于匹配控制器版本文本。
import re


# 学习：导入socket，用于进行网络通信。
import socket


# 学习：导入struct，用于把数值与通信字节报文相互转换。
import struct


# 学习：导入sys，用于访问启动参数和退出状态。
import sys


# 学习：导入time，用于计时及短暂等待。
import time


# 学习：导入traceback，用于保留出错位置堆栈。
import traceback


# 学习：从datetime导入datetime, timezone, timedelta，用于处理日期、时间和时区。
from datetime import datetime, timezone, timedelta


# 学习：从pathlib导入Path，用于以Path对象处理文件与目录。
from pathlib import Path


# 学习：把“Path(__file__).resolve().parent”保存到ROOT（当前文件所在目录）；单个=表示赋值。
ROOT = Path(__file__).resolve().parent


# 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到FIXED_ORIENTATION_RAD（固定旋转向量，单位rad，不是欧拉角）；单个=表示赋值。
FIXED_ORIENTATION_RAD = [1.4747881408617243, -0.6236196873562871, 0.6049084354198352]


# 学习：把“'2026-11-12T13:10:55.171401+08:00'”保存到ORIENTATION_CAPTURED_AT（固定姿态的实际读取时间）；单个=表示赋值。
ORIENTATION_CAPTURED_AT = '2026-11-12T13:10:55.171401+08:00'


# 学习：把实测XYZ从mm除以1000换成m，再追加完整固定旋转向量rad，生成六维机器人目标。
POSES = {k: [x / 1000 for x in v] + FIXED_ORIENTATION_RAD[:] for k, v in POINTS.items()}


# 学习：把“0.0005”保存到POSITION_TOL（机械臂位置容差0.0005 m，即0.5 mm）；单个=表示赋值。
POSITION_TOL = 0.0005


# 学习：把“0.003”保存到ANGLE_TOL（姿态容差0.003 rad）；单个=表示赋值。
ANGLE_TOL = 0.003


# 学习：定义now函数：返回带UTC+8时区的当前时间字符串，用于日志记录；def本身只是定义，调用时才执行函数体。
def now():
    # 学习：return把调用datetime.now(timezone(timedelta(hours=8))).isoformat，按本行参数取得结果交回调用者，并立即结束当前函数。
    return datetime.now(timezone(timedelta(hours=8))).isoformat()


# 学习：定义exact函数：从网络连接反复接收，直到凑齐指定字节数；连接中断则报错；def本身只是定义，调用时才执行函数体。
def exact(s, n):
    # 学习：把“b''”保存到data（接收的数据缓冲区）；单个=表示赋值。
    data = b''
    # 学习：while循环：只要len(data) < n成立就继续；完成、异常或超时会结束等待。
    while len(data) < n:
        # 学习：把“从网络接收指定上限的字节数”保存到chunk（这次从网络接收的字节块）；单个=表示赋值。
        chunk = s.recv(n - len(data))
        # 学习：判断条件not chunk；成立(True)时执行下方缩进代码。
        if not chunk:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise EOFError('Controller closed state connection')
        # 学习：在原值基础上更新data（接收的数据缓冲区）：data += chunk。
        data += chunk
    # 学习：return把data（接收的数据缓冲区）交回调用者，并立即结束当前函数。
    return data


# 学习：定义read_robot函数：通过30012只读端口解析机械臂TCP、TCP偏置和控制器模式；def本身只是定义，调用时才执行函数体。
def read_robot(ip):
    # 学习：with限定资源或临时替换的有效范围：与指定IP和端口建立TCP连接，并设置连接超时；离开时自动执行对应清理。
    with socket.create_connection((ip, 30012), 3) as s:
        # 学习：设置通信超时时间，单位秒。
        s.settimeout(3)
        # 学习：for逐项循环：从range(30)取一个元素放入_，再执行缩进部分。
        for _ in range(30):
            # 学习：解析5字节状态报文头：4字节长度I和1字节类型B，均按大端读取。
            size, kind = struct.unpack('>IB', exact(s, 5))
            # 学习：判断条件not 5 <= size <= 100000；成立(True)时执行下方缩进代码。
            if not 5 <= size <= 100000:
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Invalid controller packet size')
            # 学习：把“从网络连接反复接收，直到凑齐指定字节数；连接中断则报错”保存到payload（去掉报文头后的数据主体）；单个=表示赋值。
            payload = exact(s, size - 5)
            # 学习：判断条件kind != 16；成立(True)时执行下方缩进代码。
            if kind != 16:
                # 学习：continue跳过本轮剩余代码，检查下一条数据或下一轮循环。
                continue
            # 学习：把“组成一组固定顺序的数据，便于成组赋值或返回”保存到(result, offset)；单个=表示赋值。
            result, offset = ({}, 0)
            # 学习：while循环：只要offset < len(payload)成立就继续；完成、异常或超时会结束等待。
            while offset < len(payload):
                # 学习：读取当前子报文的长度n和类型t，后面按类型解析TCP或控制器模式。
                n, t = struct.unpack_from('>IB', payload, offset)
                # 学习：判断条件n < 5 or offset + n > len(payload)；成立(True)时执行下方缩进代码。
                if n < 5 or offset + n > len(payload):
                    # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                    raise RuntimeError('Invalid state subpacket')
                # 学习：把“payload[offset + 5:offset + n]”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
                d = payload[offset + 5:offset + n]
                # 学习：类型4是笛卡尔位姿数据；至少96字节才够解码12个8字节浮点数。
                if t == 4 and len(d) >= 96:
                    # 学习：从数据块按大端顺序解码12个8字节浮点数：前6个TCP位姿，后6个TCP偏置。
                    values = list(struct.unpack_from('>12d', d))
                    # 学习：判断条件not all((math.isfinite(x) for x in values))；成立(True)时执行下方缩进代码。
                    if not all((math.isfinite(x) for x in values)):
                        # 学习：抛出异常：反馈含非有限数值，不能当作有效位姿或速度；交由上层错误处理，不继续本段正常流程。
                        raise RuntimeError('Nonfinite pose')
                    # 学习：前6项保存为TCP位姿，剩余6项保存为工具偏置，供后续核对实测坐标基准。
                    result.update(pose=values[:6], tcp_offset=values[6:])
                # 学习：类型0是机器人模式数据；长度足够时读取连接、上电、停止、程序状态和模式编号。
                if t == 0 and len(d) >= 17:
                    # 学习：把“组成一组固定顺序的数据，便于成组赋值或返回”保存到keys；单个=表示赋值。
                    keys = ('connected', 'enabled', 'power_on', 'emergency_stop', 'protective_stop', 'program_running', 'program_paused')
                    # 学习：批量更新字典中的字段。
                    result.update(dict(zip(keys, [bool(v) for v in d[8:15]])))
                    # 学习：把“d[15]”保存到result['mode']；单个=表示赋值。
                    result['mode'] = d[15]
                # 学习：跳过当前子报文的n个字节，把解析位置移到下一个子报文。
                offset += n
            # 学习：判断条件'pose' in result and 'mode' in result；成立(True)时执行下方缩进代码。
            if 'pose' in result and 'mode' in result:
                # 学习：return把result（当前函数累积的结果）交回调用者，并立即结束当前函数。
                return result
    # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
    raise RuntimeError('No pose/mode packet received')


# 学习：定义dashboard函数：通过29999接口查询控制器版本、就绪、安全和程序状态；def本身只是定义，调用时才执行函数体。
def dashboard(ip):
    # 学习：with限定资源或临时替换的有效范围：与指定IP和端口建立TCP连接，并设置连接超时；离开时自动执行对应清理。
    with socket.create_connection((ip, 29999), 3) as s:
        # 学习：设置通信超时时间，单位秒。
        s.settimeout(3)
        # 学习：把“把网络连接包装为可逐行读写的文件对象”保存到f；单个=表示赋值。
        f = s.makefile('rwb', buffering=0)
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到result（当前函数累积的结果）；单个=表示赋值。
        result = {'greeting': f.readline(4096).decode(errors='replace').strip()}
        # 学习：for逐项循环：从('PolyscopeVersion', 'robotmode', 'safetymode', 'running')取一个元素放入command，再执行缩进部分。
        for command in ('PolyscopeVersion', 'robotmode', 'safetymode', 'running'):
            # 学习：给Dashboard查询命令加换行、编码后发送；此处命令清单只包含读取状态的查询。
            f.write((command + '\n').encode())
            # 学习：把“去掉文字两端空白与换行”保存到result[command]；单个=表示赋值。
            result[command] = f.readline(4096).decode(errors='replace').strip()
        # 学习：return把result（当前函数累积的结果）交回调用者，并立即结束当前函数。
        return result


# 学习：定义orientation_error函数：把旋转向量换成四元数，计算两种姿态的实际夹角，单位rad；def本身只是定义，调用时才执行函数体。
def orientation_error(a, b):
    # 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
    """Physical angular error, independent of equivalent axis-angle branches."""

    # 学习：定义quat函数：把三元素旋转向量转换为四元素单位四元数；def本身只是定义，调用时才执行函数体。
    def quat(rv):
        # 学习：判断条件len(rv) != 3 or not all((math.isfinite(v) for v in rv))；成立(True)时执行下方缩进代码。
        if len(rv) != 3 or not all((math.isfinite(v) for v in rv)):
            # 学习：return把None交回调用者，并立即结束当前函数。
            return None
        # 学习：把“求平方根，用于计算向量长度或加速时间”保存到angle（旋转向量长度，即旋转角度rad）；单个=表示赋值。
        angle = math.sqrt(sum((v * v for v in rv)))
        # 学习：判断条件angle < 1e-12；成立(True)时执行下方缩进代码。
        if angle < 1e-12:
            # 学习：return把建立按顺序排列的列表，方括号内是各个元素交回调用者，并立即结束当前函数。
            return [1.0, 0.0, 0.0, 0.0]
        # 学习：return把计算[math.cos(angle / 2)] + [v * math.sin(angle / 2) / angle for v in rv]交回调用者，并立即结束当前函数。
        return [math.cos(angle / 2)] + [v * math.sin(angle / 2) / angle for v in rv]
    # 学习：把“组成一组固定顺序的数据，便于成组赋值或返回”保存到(qa, qb)；单个=表示赋值。
    qa, qb = (quat(a), quat(b))
    # 学习：判断条件qa is None or qb is None；成立(True)时执行下方缩进代码。
    if qa is None or qb is None:
        # 学习：return把math.inf交回调用者，并立即结束当前函数。
        return math.inf
    # 学习：四元数点积取绝对值以接受q与-q等价表示，再限制到[0,1]避免浮点误差使acos越界。
    dot = min(1.0, max(0.0, abs(sum((x * y for x, y in zip(qa, qb))))))
    # 学习：return把计算2 * math.acos(dot)交回调用者，并立即结束当前函数。
    return 2 * math.acos(dot)


# 学习：定义matches_fixed_orientation函数：核对六维TCP位姿中的姿态是否与保存值相符；def本身只是定义，调用时才执行函数体。
def matches_fixed_orientation(pose):
    # 学习：return把len(pose) == 6 and orientation_error(pose[3:], FIXED_ORIENTATION_RAD) <= ANGLE_TOL交回调用者，并立即结束当前函数。
    return len(pose) == 6 and orientation_error(pose[3:], FIXED_ORIENTATION_RAD) <= ANGLE_TOL


# 学习：定义require_target函数：检查单个六维目标是否有效，并请控制器核对可达性与安全限位；def本身只是定义，调用时才执行函数体。
def require_target(control, pose):
    # 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
    """One controller endpoint check; no path sampling or multi-point IK loop."""
    # 学习：判断条件len(pose) != 6 or not all((math.isfinite(v) for v in pose))；成立(True)时执行下方缩进代码。
    if len(pose) != 6 or not all((math.isfinite(v) for v in pose)):
        # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError('Invalid target pose')
    # 学习：判断条件not control.isPoseWithinSafetyLimits(pose)；成立(True)时执行下方缩进代码。
    if not control.isPoseWithinSafetyLimits(pose):
        # 学习：抛出异常：目标或运动指令被控制器拒绝；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError('Controller rejected target reachability or safety limits; no motion sent')


# 学习：定义cylinder_state_errors函数：收集电缸状态问题；恢复模式只特别识别0x28软件暂停；def本身只是定义，调用时才执行函数体。
def cylinder_state_errors(state, recovery=False):
    # 学习：把“创建一份键值数据字典，或复制已有字典”保存到check；单个=表示赋值。
    check = dict(state)
    # 学习：恢复检查只识别40(0x28)这一已知软件暂停状态；不能据此忽略其他控制模式。
    if recovery and check['control'] == 40:
        # 学习：仅在用于校验的字典副本中把0x28按0x20使能态评估；这行不写设备、不实际解除暂停。
        check['control'] = 32
    # 学习：把“调用validate，按本行参数取得结果”保存到errors（检测到的问题列表）；单个=表示赋值。
    errors = validate(check, 1)
    # 学习：判断条件recovery and (not 0 <= state['position'] <= bounds(state)[1])；成立(True)时执行下方缩进代码。
    if recovery and (not 0 <= state['position'] <= bounds(state)[1]):
        # 学习：把一个新元素追加到列表末尾。
        errors.append('Cylinder recovery position outside calibrated trial range')
    # 学习：return把errors（检测到的问题列表）交回调用者，并立即结束当前函数。
    return errors


# 学习：定义require_initial函数：核对两台设备是否满足正常或恢复启动条件，有问题则报错；def本身只是定义，调用时才执行函数体。
def require_initial(robot, dash, cylinder, recovery=False):
    # 学习：把“收集电缸状态问题；恢复模式只特别识别0x28软件暂停”保存到problems（启动条件问题列表）；单个=表示赋值。
    problems = cylinder_state_errors(cylinder, recovery)
    # 学习：判断条件not all((robot[k] for k in ('connected', 'enabled', 'power_on'))) or robot['mode'] != 7；成立(True)时执行下方缩进代码。
    if not all((robot[k] for k in ('connected', 'enabled', 'power_on'))) or robot['mode'] != 7:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Robot is not powered and ready in RUNNING mode')
    # 学习：判断条件robot['emergency_stop'] or robot['protective_stop']；成立(True)时执行下方缩进代码。
    if robot['emergency_stop'] or robot['protective_stop']:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Robot stop flag is active')
    # 学习：判断条件robot['program_running'] or robot['program_paused']；成立(True)时执行下方缩进代码。
    if robot['program_running'] or robot['program_paused']:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Existing robot program is running or paused; do not overwrite it')
    # 学习：判断条件dash.get('safetymode') != 'Safetymode: NORMAL'；成立(True)时执行下方缩进代码。
    if dash.get('safetymode') != 'Safetymode: NORMAL':
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Robot safety mode must be NORMAL')
    # 学习：判断条件dash.get('running') != 'Program running: false'；成立(True)时执行下方缩进代码。
    if dash.get('running') != 'Program running: false':
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Dashboard does not confirm idle program state')
    # 学习：把“调用re.search，按本行参数取得结果”保存到match；单个=表示赋值。
    match = re.search('URSoftware (\\d+)\\.(\\d+)\\.(\\d+)', dash.get('PolyscopeVersion', ''))
    # 学习：判断条件not match or not (3, 14, 3) <= tuple(map(int, match.groups())) < (4, 0, 0)；成立(True)时执行下方缩进代码。
    if not match or not (3, 14, 3) <= tuple(map(int, match.groups())) < (4, 0, 0):
        # 学习：把一个新元素追加到列表末尾。
        problems.append('This package targets CB3 PolyScope 3.14.3 through 3.x only')
    # 学习：判断条件not recovery and (not matches_fixed_orientation(robot['pose']))；成立(True)时执行下方缩进代码。
    if not recovery and (not matches_fixed_orientation(robot['pose'])):
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Current orientation differs from the captured fixed orientation; no automatic rotation or recapture')
    # 学习：判断条件math.dist(robot['tcp_offset'], [0.0] * 6) > 1e-06；成立(True)时执行下方缩进代码。
    if math.dist(robot['tcp_offset'], [0.0] * 6) > 1e-06:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Active TCP differs from the zero-offset TCP recorded for this task; revalidate coordinates')
    # 学习：判断条件not recovery and abs(cylinder['position']) > 5；成立(True)时执行下方缩进代码。
    if not recovery and abs(cylinder['position']) > 5:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Cylinder must start already homed at 0 mm (+/-0.05 mm)')
    # 学习：判断条件bounds(cylinder)[1] < 4800；成立(True)时执行下方缩进代码。
    if bounds(cylinder)[1] < 4800:
        # 学习：把一个新元素追加到列表末尾。
        problems.append('Cylinder configured stroke/offset leaves insufficient margin for 48 mm')
    # 学习：判断条件problems；成立(True)时执行下方缩进代码。
    if problems:
        # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
        raise RuntimeError('; '.join(problems))


# 学习：定义inspect函数：只读检查设备，分别判断普通运行和人工选择恢复是否可行；def本身只是定义，调用时才执行函数体。
def inspect(ip, port):
    # 学习：从serial.tools.list_ports导入comports，用于使用已有功能，避免重复实现。
    from serial.tools.list_ports import comports
    # 学习：把“逐项遍历并计算，生成所需数据：[{'port': p.device, 'vid': p.vid, 'pid': p.pid} for p in comports()]”保存到ports（当前电脑枚举出的串口清单）；单个=表示赋值。
    ports = [{'port': p.device, 'vid': p.vid, 'pid': p.pid} for p in comports()]
    # 学习：把“调用Device，按本行参数取得结果”保存到device（电缸通信对象）；单个=表示赋值。
    device = Device(port, 115200, 1)
    # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
    try:
        # 学习：把“读取电缸关键寄存器快照”保存到cylinder（电缸状态字典）；单个=表示赋值。
        cylinder = device.snapshot()
    # 学习：finally中的资源释放与日志保存，无论前面成功、报错或被键盘中断都尝试执行。
    finally:
        # 学习：关闭当前资源或串口。
        device.close()
    # 学习：把“通过30012只读端口解析机械臂TCP、TCP偏置和控制器模式”保存到robot（机械臂状态字典）；单个=表示赋值。
    robot = read_robot(ip)
    # 学习：把“通过29999接口查询控制器版本、就绪、安全和程序状态”保存到dash（Dashboard查询结果）；单个=表示赋值。
    dash = dashboard(ip)
    # 学习：把“创建一份键值数据字典，或复制已有字典”保存到result（当前函数累积的结果）；单个=表示赋值。
    result = dict(time=now(), ip=ip, port=port, robot=robot, dashboard=dash, cylinder=cylinder, serial_ports=ports, motion_commands_sent=False)
    # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
    try:
        # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
        require_initial(robot, dash, cylinder)
        # 学习：批量更新字典中的字段。
        result.update(trial_start_ready=True, blocking_reason=None, fixed_orientation_rad=FIXED_ORIENTATION_RAD[:], orientation_captured_at=ORIENTATION_CAPTURED_AT, orientation_initialization_required=False)
    # 学习：except捕获RuntimeError，执行下面的错误记录或清理逻辑。
    except RuntimeError as error:
        # 学习：批量更新字典中的字段。
        result.update(trial_start_ready=False, blocking_reason=str(error))
    # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
    try:
        # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
        require_initial(robot, dash, cylinder, recovery=True)
        # 学习：把“True”保存到result['manual_recovery_available']（是否允许操作员选择恢复）；单个=表示赋值。
        result['manual_recovery_available'] = True
        # 学习：把“None”保存到result['manual_recovery_blocking_reason']（恢复入口被阻止的原因）；单个=表示赋值。
        result['manual_recovery_blocking_reason'] = None
    # 学习：except捕获RuntimeError，执行下面的错误记录或清理逻辑。
    except RuntimeError as error:
        # 学习：把“False”保存到result['manual_recovery_available']（是否允许操作员选择恢复）；单个=表示赋值。
        result['manual_recovery_available'] = False
        # 学习：把“转换或创建str类型的数据”保存到result['manual_recovery_blocking_reason']（恢复入口被阻止的原因）；单个=表示赋值。
        result['manual_recovery_blocking_reason'] = str(error)
    # 学习：return把result（当前函数累积的结果）交回调用者，并立即结束当前函数。
    return result


# 学习：定义set_stage函数：记录当前执行阶段，立即保存日志并在终端显示；def本身只是定义，调用时才执行函数体。
def set_stage(report, label):
    # 学习：把“label”保存到report['stage']（当前执行阶段）；单个=表示赋值。
    report['stage'] = label
    # 学习：把“返回带UTC+8时区的当前时间字符串，用于日志记录”保存到report['stage_started']（当前阶段开始时间）；单个=表示赋值。
    report['stage_started'] = now()
    # 学习：保存当前运行日志。
    save(report)
    # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
    print('\n[阶段] ' + label, flush=True)


# 学习：定义Trial类：把当前一轮设备对象、动作方法和监测状态放在一起；self表示当前创建的那个对象。
class Trial:

    # 学习：定义__init__函数：保存传入的两台设备对象，并初始化本轮监测状态；def本身只是定义，调用时才执行函数体。
    def __init__(self, receive, control, device, report):
        # 学习：把“组成一组固定顺序的数据，便于成组赋值或返回”保存到(self.r, self.c, self.d, self.report)；单个=表示赋值。
        self.r, self.c, self.d, self.report = (receive, control, device, report)
        # 学习：把“None”保存到self.last_timestamp（最近一次RTDE反馈时间戳）；单个=表示赋值。
        self.last_timestamp = None
        # 学习：把“取得不会被系统校时倒拨影响的计时时刻（秒）”保存到self.last_fresh（最近一次收到新反馈的本机单调时刻）；单个=表示赋值。
        self.last_fresh = time.monotonic()
        # 学习：把“0”保存到self.last_cylinder_poll（最近一次轮询电缸的时刻）；单个=表示赋值。
        self.last_cylinder_poll = 0
        # 学习：把“0”保存到self.cylinder_expected（电缸应保持的位置，单位0.01 mm）；单个=表示赋值。
        self.cylinder_expected = 0
        # 学习：把“False”保存到self.cylinder_moving（电缸是否处于定位过程的标记）；单个=表示赋值。
        self.cylinder_moving = False
        # 学习：暂时取消静态位置保持比较，让机械臂可以进入本段运动；运动过程中另有直线偏离核验。
        self.expected_hold = None
        # 学习：把“False”保存到self.recovery_mode（是否处于中断恢复模式）；单个=表示赋值。
        self.recovery_mode = False
        # 学习：把“None”保存到self.recovery_orientation（恢复开始时实际姿态，用于回缩期间保持）；单个=表示赋值。
        self.recovery_orientation = None
        # 学习：把“False”保存到self.restoring_orientation（是否正在主动恢复原固定姿态）；单个=表示赋值。
        self.restoring_orientation = False

    # 学习：定义tick函数：执行一次实时反馈检查、速度记录和看门狗更新，不在这里决定任务路线；def本身只是定义，调用时才执行函数体。
    def tick(self):
        # 学习：判断条件not self.r.isConnected() or not self.c.isConnected()；成立(True)时执行下方缩进代码。
        if not self.r.isConnected() or not self.c.isConnected():
            # 学习：抛出异常：RTDE连接断开；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('RTDE disconnected')
        # 学习：把“读取控制器反馈时间戳，以识别数据是否更新”保存到stamp；单个=表示赋值。
        stamp = self.r.getTimestamp()
        # 学习：判断条件stamp != self.last_timestamp；成立(True)时执行下方缩进代码。
        if stamp != self.last_timestamp:
            # 学习：把“stamp”保存到self.last_timestamp（最近一次RTDE反馈时间戳）；单个=表示赋值。
            self.last_timestamp = stamp
            # 学习：把“取得不会被系统校时倒拨影响的计时时刻（秒）”保存到self.last_fresh（最近一次收到新反馈的本机单调时刻）；单个=表示赋值。
            self.last_fresh = time.monotonic()
        # 学习：判断条件time.monotonic() - self.last_fresh > 1；成立(True)时执行下方缩进代码。
        if time.monotonic() - self.last_fresh > 1:
            # 学习：抛出异常：机器人反馈超过允许时间未更新；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Stale RTDE state')
        # 学习：就绪条件：机器人模式7、安全模式1(NORMAL)、控制程序状态2(运行中)；任何一项变化都中断当前任务。
        if self.r.getRobotMode() != 7 or self.r.getSafetyMode() != 1 or self.r.getRuntimeState() != 2:
            # 学习：抛出异常：控制程序未处于预期运行状态；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Robot no longer ready / control program no longer running')
        # 学习：判断条件self.r.getSpeedScaling() <= 0；成立(True)时执行下方缩进代码。
        if self.r.getSpeedScaling() <= 0:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Robot execution speed scaling is zero; do not auto-resume')
        # 学习：把“创建列表副本，数据来自self.r.getActualTCPPose()”保存到pose（六维TCP位姿，前三项m、后三项rad）；单个=表示赋值。
        pose = list(self.r.getActualTCPPose())
        # 学习：判断条件not all((math.isfinite(v) for v in pose))；成立(True)时执行下方缩进代码。
        if not all((math.isfinite(v) for v in pose)):
            # 学习：抛出异常：反馈含非有限数值，不能当作有效位姿或速度；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Nonfinite TCP pose')
        # 学习：把“条件self.recovery_mode成立取前一值，否则取后一值”保存到required_orientation（当前阶段应该保持的姿态）；单个=表示赋值。
        required_orientation = self.recovery_orientation if self.recovery_mode else FIXED_ORIENTATION_RAD
        # 学习：判断条件not self.restoring_orientation and orientation_error(pose[3:], required_orientation) > ANGLE_TOL；成立(True)时执行下方缩进代码。
        if not self.restoring_orientation and orientation_error(pose[3:], required_orientation) > ANGLE_TOL:
            # 学习：抛出异常：实际姿态超出保存姿态容差；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Captured fixed tool orientation tolerance exceeded')
        # 学习：判断条件math.dist(self.c.getTCPOffset(), [0.0] * 6) > 1e-06；成立(True)时执行下方缩进代码。
        if math.dist(self.c.getTCPOffset(), [0.0] * 6) > 1e-06:
            # 学习：抛出异常：活动TCP偏置发生变化；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Active TCP changed')
        # 学习：若当前阶段要求机械臂保持位置，核对XYZ与预期位置的距离不超过0.5 mm。
        if self.expected_hold is not None and math.dist(pose[:3], self.expected_hold[:3]) > POSITION_TOL:
            # 学习：抛出异常：要求保持位置时机器人发生位移；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Robot moved during cylinder motion or dwell')
        # 学习：把“创建列表副本，数据来自self.r.getActualTCPSpeed()”保存到speed（速度数据或本阶段速度上限）；单个=表示赋值。
        speed = list(self.r.getActualTCPSpeed())
        # 学习：判断条件len(speed) != 6 or not all((math.isfinite(v) for v in speed))；成立(True)时执行下方缩进代码。
        if len(speed) != 6 or not all((math.isfinite(v) for v in speed)):
            # 学习：抛出异常：反馈含非有限数值，不能当作有效位姿或速度；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Nonfinite TCP speed feedback')
        # 学习：TCP速度前3项是XYZ线速度；平方和开根号得到整体线速度，单位m/s。
        linear = math.sqrt(sum((v * v for v in speed[:3])))
        # 学习：TCP速度后3项是角速度向量；平方和开根号得到整体角速度，单位rad/s。
        angular = math.sqrt(sum((v * v for v in speed[3:])))
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到self.report['last_robot_feedback']；单个=表示赋值。
        self.report['last_robot_feedback'] = dict(time=now(), pose=pose, tcp_speed=speed)
        # 学习：记录本轮见过的最大TCP线速度m/s；只记日志，不用额外瞬时速度阈值中断。
        self.report['max_observed_linear_speed_m_s'] = max(linear, self.report.get('max_observed_linear_speed_m_s', 0))
        # 学习：记录本轮见过的最大TCP角速度rad/s；不是欧拉角变化率或J6速度。
        self.report['max_observed_angular_speed_rad_s'] = max(angular, self.report.get('max_observed_angular_speed_rad_s', 0))
        # 学习：尝试更新控制器看门狗；接口返回失败则抛错，避免失去监测时继续正常流程。
        if not self.c.kickWatchdog():
            # 学习：抛出异常：看门狗更新失败；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Watchdog update failed')
        # 学习：判断条件not self.cylinder_moving and time.monotonic() - self.last_cylinder_poll > 0.2；成立(True)时执行下方缩进代码。
        if not self.cylinder_moving and time.monotonic() - self.last_cylinder_poll > 0.2:
            # 学习：把“读取电缸关键寄存器快照”保存到state（电缸寄存器状态字典）；单个=表示赋值。
            state = self.d.snapshot()
            # 学习：把“收集电缸状态问题；恢复模式只特别识别0x28软件暂停”保存到errors（检测到的问题列表）；单个=表示赋值。
            errors = cylinder_state_errors(state, self.recovery_mode)
            # 学习：判断条件errors or abs(state['position'] - self.cylinder_expected) > 5；成立(True)时执行下方缩进代码。
            if errors or abs(state['position'] - self.cylinder_expected) > 5:
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Stationary cylinder state changed: ' + str(errors))
            # 学习：把“取得不会被系统校时倒拨影响的计时时刻（秒）”保存到self.last_cylinder_poll（最近一次轮询电缸的时刻）；单个=表示赋值。
            self.last_cylinder_poll = time.monotonic()

    # 学习：定义dwell函数：到位后停顿1秒；等待期间仍持续检查设备状态；def本身只是定义，调用时才执行函数体。
    def dwell(self):
        # 学习：用单调计时设置1秒停顿终点；等待期间仍调用tick检查设备，不是无监控地睡1秒。
        until = time.monotonic() + 1
        # 学习：while循环：只要time.monotonic() < until成立就继续；完成、异常或超时会结束等待。
        while time.monotonic() < until:
            # 学习：检查一次实时状态并更新看门狗。
            self.tick()
            # 学习：短暂等待0.03秒(30毫秒)，避免空转占满CPU；循环下次醒来继续检查状态。
            time.sleep(0.03)

    # 学习：定义restore_fixed_orientation函数：恢复阶段保持当前XYZ，将姿态恢复到13:10:55保存值；def本身只是定义，调用时才执行函数体。
    def restore_fixed_orientation(self):
        # 学习：判断条件not self.recovery_mode；成立(True)时执行下方缩进代码。
        if not self.recovery_mode:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Orientation restoration requires recovery mode')
        # 学习：把“创建列表副本，数据来自self.r.getActualTCPPose()”保存到start（该段开始时的状态或位置）；单个=表示赋值。
        start = list(self.r.getActualTCPPose())
        # 学习：取当前位姿前三项XYZ，再拼接原固定旋转向量；目标仅改变姿态，不改变XYZ。
        target = start[:3] + FIXED_ORIENTATION_RAD[:]
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到event（当前动作的日志条目）；单个=表示赋值。
        event = dict(action='restore_captured_orientation', from_pose=start, target_pose=target, start=now(), complete=False)
        # 学习：把一个新元素追加到列表末尾。
        self.report['segments'].append(event)
        # 学习：判断条件not matches_fixed_orientation(start)；成立(True)时执行下方缩进代码。
        if not matches_fixed_orientation(start):
            # 学习：检查单个六维目标是否有效，并请控制器核对可达性与安全限位。
            require_target(self.c, target)
            # 学习：把“start[:]”保存到self.expected_hold（机械臂当前应保持的TCP位置）；单个=表示赋值。
            self.expected_hold = start[:]
            # 学习：标记正处于主动姿态恢复，允许这一段姿态向原保存值变化，仍监控当前XYZ。
            self.restoring_orientation = True
            # 学习：把“读取异步动作进度；新的负值标记用于识别完成”保存到old（启动动作前的异步操作标记）；单个=表示赋值。
            old = self.c.getAsyncOperationProgress()
            # 学习：批量更新字典中的字段。
            self.report.update(robot_motion_attempted=True, motion_commands_sent=True)
            # 学习：向控制器发送目标直线插补；v单位m/s、a单位m/s²，True为异步；本行数值不表示保证的角速度。
            if not self.c.moveL(target, 0.001, 0.001, True):
                # 学习：抛出异常：目标或运动指令被控制器拒绝；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Recovery orientation move rejected')
            # 学习：把“计算time.monotonic() + max(60, orientation_error(start[3:], FIXED_ORIENTATION_RAD) / 0.002 + 60)”保存到until（等待结束的单调时刻）；单个=表示赋值。
            until = time.monotonic() + max(60, orientation_error(start[3:], FIXED_ORIENTATION_RAD) / 0.002 + 60)
            # 学习：while循环：只要time.monotonic() < until成立就继续；完成、异常或超时会结束等待。
            while time.monotonic() < until:
                # 学习：检查一次实时状态并更新看门狗。
                self.tick()
                # 学习：把“读取异步动作进度；新的负值标记用于识别完成”保存到value；单个=表示赋值。
                value = self.c.getAsyncOperationProgress()
                # 学习：新的负值异步标记表示本次动作结束；还要与旧标记不同，防止把上次完成状态当成本次完成。
                if value < 0 and value != old:
                    # 学习：break立即退出当前这一层循环；本处表示已满足完成条件。
                    break
                # 学习：短暂等待0.03秒(30毫秒)，避免空转占满CPU；循环下次醒来继续检查状态。
                time.sleep(0.03)
            # 学习：循环正常耗尽而没有break时进入这里；等待循环中通常在此报告超时。
            else:
                # 学习：抛出异常：操作超过允许时间仍未完成；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Recovery orientation timeout')
        # 学习：把“创建列表副本，数据来自self.r.getActualTCPPose()”保存到actual（当前实际TCP位姿）；单个=表示赋值。
        actual = list(self.r.getActualTCPPose())
        # 学习：判断条件math.dist(actual[:3], start[:3]) > POSITION_TOL or not matches_fixed_orientation(actual)；成立(True)时执行下方缩进代码。
        if math.dist(actual[:3], start[:3]) > POSITION_TOL or not matches_fixed_orientation(actual):
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Recovery did not reach captured orientation at the held XYZ')
        # 学习：把“False”保存到self.restoring_orientation（是否正在主动恢复原固定姿态）；单个=表示赋值。
        self.restoring_orientation = False
        # 学习：把“False”保存到self.recovery_mode（是否处于中断恢复模式）；单个=表示赋值。
        self.recovery_mode = False
        # 学习：把“None”保存到self.recovery_orientation（恢复开始时实际姿态，用于回缩期间保持）；单个=表示赋值。
        self.recovery_orientation = None
        # 学习：把“target（本段目标）”保存到self.expected_hold（机械臂当前应保持的TCP位置）；单个=表示赋值。
        self.expected_hold = target
        # 学习：监控状态并完成到位后1秒停顿。
        self.dwell()
        # 学习：批量更新字典中的字段。
        event.update(complete=True, actual_pose=actual, end=now())

    # 学习：定义robot函数：让机械臂直线到指定A–G目标，监测并确认到位后停顿1秒；def本身只是定义，调用时才执行函数体。
    def robot(self, name):
        # 学习：暂时取消静态位置保持比较，让机械臂可以进入本段运动；运动过程中另有直线偏离核验。
        self.expected_hold = None
        # 学习：检查一次实时状态并更新看门狗。
        self.tick()
        # 学习：把“POSES[name]”保存到target（本段目标）；单个=表示赋值。
        target = POSES[name]
        # 学习：把“创建列表副本，数据来自self.r.getActualTCPPose()”保存到initial（动作起点TCP位姿）；单个=表示赋值。
        initial = list(self.r.getActualTCPPose())
        # 学习：把“计算两个坐标向量之间的直线距离”保存到distance（本段位移长度）；单个=表示赋值。
        distance = math.dist(initial[:3], target[:3])
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到event（当前动作的日志条目）；单个=表示赋值。
        event = dict(action='robot', target=name, pose=target, start=now(), complete=False)
        # 学习：把一个新元素追加到列表末尾。
        self.report['segments'].append(event)
        # 学习：只有当前位置离目标超过0.5 mm时才发送机器人动作。
        if distance > POSITION_TOL:
            # 学习：检查单个六维目标是否有效，并请控制器核对可达性与安全限位。
            require_target(self.c, target)
            # 学习：把“True”保存到self.report['robot_motion_attempted']（是否尝试发送机器人动作）；单个=表示赋值。
            self.report['robot_motion_attempted'] = True
            # 学习：把“True”保存到self.report['motion_commands_sent']（是否尝试发送过运动命令）；单个=表示赋值。
            self.report['motion_commands_sent'] = True
            # 学习：把“读取异步动作进度；新的负值标记用于识别完成”保存到old（启动动作前的异步操作标记）；单个=表示赋值。
            old = self.c.getAsyncOperationProgress()
            # 学习：向控制器发送目标直线插补；v单位m/s、a单位m/s²，True为异步；本行数值不表示保证的角速度。
            if not self.c.moveL(target, 0.03, 0.001, True):
                # 学习：抛出异常：目标或运动指令被控制器拒绝；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('moveL rejected: ' + name)
            # 学习：根据本段mm距离和30 mm/s计算估计用时，取3倍再加20秒作为超时上限；这不是运动速度设定。
            deadline = time.monotonic() + 3 * motion_profile(distance * 1000, 30)[2] + 20
            # 学习：把“False”保存到complete；单个=表示赋值。
            complete = False
            # 学习：while循环：只要time.monotonic() < deadline成立就继续；完成、异常或超时会结束等待。
            while time.monotonic() < deadline:
                # 学习：检查一次实时状态并更新看门狗。
                self.tick()
                # 学习：把“读取异步动作进度；新的负值标记用于识别完成”保存到progress（控制器异步动作进度标记）；单个=表示赋值。
                progress = self.c.getAsyncOperationProgress()
                # 学习：把“创建列表副本，数据来自self.r.getActualTCPPose()”保存到actual（当前实际TCP位姿）；单个=表示赋值。
                actual = list(self.r.getActualTCPPose())
                # 学习：计算起点指向目标的XYZ差向量，供后面的直线路径偏离检查使用。
                v = [target[i] - initial[i] for i in range(3)]
                # 学习：用点积求实际位置在线段上的投影比例：0在起点，1在终点，超出范围表示在线段延长线上。
                u = sum(((actual[i] - initial[i]) * v[i] for i in range(3))) / (distance * distance)
                # 学习：把投影比例限制在0到1，算出命令线段上离实际TCP最近的点。
                nearest = [initial[i] + min(1, max(0, u)) * v[i] for i in range(3)]
                # 学习：判断条件math.dist(actual[:3], nearest) > 0.002；成立(True)时执行下方缩进代码。
                if math.dist(actual[:3], nearest) > 0.002:
                    # 学习：抛出异常：实际位置偏离当前命令线段；交由上层错误处理，不继续本段正常流程。
                    raise RuntimeError('Robot deviated from expected straight segment')
                # 学习：新的负值异步标记表示本次动作结束；还要与旧标记不同，防止把上次完成状态当成本次完成。
                if progress < 0 and progress != old:
                    # 学习：把“True”保存到complete；单个=表示赋值。
                    complete = True
                    # 学习：break立即退出当前这一层循环；本处表示已满足完成条件。
                    break
                # 学习：短暂等待0.03秒(30毫秒)，避免空转占满CPU；循环下次醒来继续检查状态。
                time.sleep(0.03)
            # 学习：判断条件not complete；成立(True)时执行下方缩进代码。
            if not complete:
                # 学习：抛出异常：操作超过允许时间仍未完成；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Robot move timeout')
        # 学习：把“创建列表副本，数据来自self.r.getActualTCPPose()”保存到actual（当前实际TCP位姿）；单个=表示赋值。
        actual = list(self.r.getActualTCPPose())
        # 学习：判断条件math.dist(actual[:3], target[:3]) > POSITION_TOL or not matches_fixed_orientation(actual)；成立(True)时执行下方缩进代码。
        if math.dist(actual[:3], target[:3]) > POSITION_TOL or not matches_fixed_orientation(actual):
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Robot target verification failed')
        # 学习：核对实际TCP平移速度不超过0.0002 m/s(0.2 mm/s)，用于确认到位后已近似静止。
        if math.sqrt(sum((v * v for v in self.r.getActualTCPSpeed()[:3]))) > 0.0002:
            # 学习：抛出异常：反馈速度表明机器人尚未静止；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Robot has not stopped at target')
        # 学习：把“target（本段目标）”保存到self.expected_hold（机械臂当前应保持的TCP位置）；单个=表示赋值。
        self.expected_hold = target
        # 学习：监控状态并完成到位后1秒停顿。
        self.dwell()
        # 学习：批量更新字典中的字段。
        event.update(complete=True, actual_pose=actual, end=now())

    # 学习：定义cylinder函数：让电缸到0、13或48 mm绝对位置，确认稳定到位后停顿1秒；def本身只是定义，调用时才执行函数体。
    def cylinder(self, target_mm):
        # 学习：把电缸目标mm乘100并取整，得到0.01 mm单位的寄存器整数。
        target = round(target_mm * 100)
        # 学习：检查一次实时状态并更新看门狗。
        self.tick()
        # 学习：把“读取电缸关键寄存器快照”保存到before；单个=表示赋值。
        before = self.d.snapshot()
        # 学习：把“收集电缸状态问题；恢复模式只特别识别0x28软件暂停”保存到errors（检测到的问题列表）；单个=表示赋值。
        errors = cylinder_state_errors(before, self.recovery_mode)
        # 学习：判断条件errors or abs(before['position'] - self.cylinder_expected) > 5；成立(True)时执行下方缩进代码。
        if errors or abs(before['position'] - self.cylinder_expected) > 5:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Cylinder start invalid: ' + str(errors))
        # 学习：判断条件target not in (0, 1300, 4800) or target > bounds(before)[1]；成立(True)时执行下方缩进代码。
        if target not in (0, 1300, 4800) or target > bounds(before)[1]:
            # 学习：抛出异常：目标或反馈超出允许范围；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Cylinder target outside approved trial positions')
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到event（当前动作的日志条目）；单个=表示赋值。
        event = dict(action='cylinder', target_mm=target_mm, start=now(), complete=False)
        # 学习：把一个新元素追加到列表末尾。
        self.report['segments'].append(event)
        # 学习：把“before['position']（电缸实际位置，单位0.01 mm）”保存到start（该段开始时的状态或位置）；单个=表示赋值。
        start = before['position']
        # 学习：判断条件abs(start - target) > 5 or (self.recovery_mode and before['control'] == 40)；成立(True)时执行下方缩进代码。
        if abs(start - target) > 5 or (self.recovery_mode and before['control'] == 40):
            # 学习：把“True”保存到self.cylinder_moving（电缸是否处于定位过程的标记）；单个=表示赋值。
            self.cylinder_moving = True
            # 学习：把“True”保存到self.report['cylinder_motion_attempted']（是否尝试发送电缸动作）；单个=表示赋值。
            self.report['cylinder_motion_attempted'] = True
            # 学习：把“True”保存到self.report['motion_commands_sent']（是否尝试发送过运动命令）；单个=表示赋值。
            self.report['motion_commands_sent'] = True
            # 学习：构造定位报文头：站号1、功能码16(0x10)、起始地址5632(0x1600)、连续6字、数据12字节。
            prefix = struct.pack('>BBHHB', 1, 16, 5632, 6, 12)
            # 学习：依次使用32(0x20准备)与33(0x21触发)两个控制字；两帧都携带同一个全新目标和低速参数。
            for word in (32, 33):
                # 学习：完整写入6个16位参数：目标、推压0、速度100(1 mm/s)、加速度1%、推力0、当前准备/触发控制字，并验证应答。
                self.d.write_ack(prefix + struct.pack('>6H', target, 0, 100, 1, 0, word))
            # 学习：按原始位置差除以100得到1 mm/s预计行程时间，再加20秒作为定位超时上限。
            until = time.monotonic() + abs(target - start) / 100 + 20
            # 学习：把“0”保存到settled（连续满足到位条件的反馈次数）；单个=表示赋值。
            settled = 0
            # 学习：while循环：只要time.monotonic() < until成立就继续；完成、异常或超时会结束等待。
            while time.monotonic() < until:
                # 学习：检查一次实时状态并更新看门狗。
                self.tick()
                # 学习：读取报警寄存器5647(0x160F)；0代表无报警。
                alarm = self.d.value(5647)
                # 学习：读取状态字5649(0x1611)，下面按二进制位检查使能、回零、运动和到位。
                word = self.d.value(5649)
                # 学习：读取实际位置5644(0x160C)，原始整数除以100才是mm。
                pos = self.d.value(5644)
                # 学习：检查报警码和bit9报警位；bit10(1024)必须使能，bit6(64)必须已回零，否则报错。
                if alarm or word & 512 or (not word & 1024) or (not word & 64):
                    # 学习：抛出异常：电缸未完成回零或回零状态丢失；交由上层错误处理，不继续本段正常流程。
                    raise RuntimeError('Cylinder alarm, lost enable or lost homing')
                # 学习：核对电缸反馈在本段起终点范围内，仅留30原始值(0.30 mm)反馈余量，且绝不超出0～48 mm。
                if not max(0, min(start, target) - 30) <= pos <= min(4800, max(start, target) + 30):
                    # 学习：抛出异常：实际位置偏离当前命令线段；交由上层错误处理，不继续本段正常流程。
                    raise RuntimeError('Cylinder position deviated from segment')
                # 学习：到位条件同时满足：位置误差≤5原始值(0.05 mm)、运动位bit8为0、到位位bit5为1。
                done = abs(pos - target) <= 5 and (not word & 256) and word & 32
                # 学习：本次到位就把稳定计数加1；任一条件不满足则清零，避免瞬时反馈误判。
                settled = settled + 1 if done else 0
                # 学习：连续3次反馈满足到位条件才确认电缸完成。
                if settled >= 3:
                    # 学习：break立即退出当前这一层循环；本处表示已满足完成条件。
                    break
                # 学习：短暂等待0.03秒(30毫秒)，避免空转占满CPU；循环下次醒来继续检查状态。
                time.sleep(0.03)
            # 学习：循环正常耗尽而没有break时进入这里；等待循环中通常在此报告超时。
            else:
                # 学习：抛出异常：操作超过允许时间仍未完成；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Cylinder move timeout')
        # 学习：把“False”保存到self.cylinder_moving（电缸是否处于定位过程的标记）；单个=表示赋值。
        self.cylinder_moving = False
        # 学习：把“target（本段目标）”保存到self.cylinder_expected（电缸应保持的位置，单位0.01 mm）；单个=表示赋值。
        self.cylinder_expected = target
        # 学习：监控状态并完成到位后1秒停顿。
        self.dwell()
        # 学习：批量更新字典中的字段。
        event.update(complete=True, end=now())


# 学习：定义execute函数：连接控制接口，连续执行恢复或整套任务，并在异常时停止和释放资源；def本身只是定义，调用时才执行函数体。
def execute(ip, port, report, recovery=False):
    # 学习：导入rtde_receive，用于读取机械臂RTDE实时状态。
    import rtde_receive
    # 学习：导入rtde_control，用于发送机械臂RTDE控制指令。
    import rtde_control
    # 学习：先把接收接口、控制接口、电缸对象都置为None；连接中途失败时据此判断哪些资源需要释放。
    r = c = d = None
    # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
    try:
        # 学习：记录当前执行阶段，立即保存日志并在终端显示。
        set_stage(report, '重新核对两台设备的实时状态')
        # 学习：把“只读检查设备，分别判断普通运行和人工选择恢复是否可行”保存到current（当前设备检查结果）；单个=表示赋值。
        current = inspect(ip, port)
        # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
        require_initial(current['robot'], current['dashboard'], current['cylinder'], recovery=recovery)
        # 学习：把“current（当前设备检查结果）”保存到report['initial']（准备控制时的只读检查快照）；单个=表示赋值。
        report['initial'] = current
        # 学习：记录当前执行阶段，立即保存日志并在终端显示。
        set_stage(report, '连接机器人RTDE状态接口')
        # 学习：把“创建125 Hz机器人状态接收接口”保存到r（机器人状态接收对象）；单个=表示赋值。
        r = rtde_receive.RTDEReceiveInterface(ip, 125.0)
        # 学习：判断条件math.dist(r.getActualTCPSpeed(), [0.0] * 6) > 0.0002；成立(True)时执行下方缩进代码。
        if math.dist(r.getActualTCPSpeed(), [0.0] * 6) > 0.0002:
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Robot pose changed or robot is moving before control setup')
        # 学习：把“创建列表副本，数据来自r.getActualTCPPose()”保存到start（该段开始时的状态或位置）；单个=表示赋值。
        start = list(r.getActualTCPPose())
        # 学习：把“调用Device，按本行参数取得结果”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
        d = Device(port, 115200, 1)
        # 学习：把“读取电缸关键寄存器快照”保存到fresh_cylinder（准备控制前再次读取的电缸状态）；单个=表示赋值。
        fresh_cylinder = d.snapshot()
        # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
        require_initial(read_robot(ip), dashboard(ip), fresh_cylinder, recovery=recovery)
        # 学习：把“True”保存到report['control_upload_attempted']；单个=表示赋值。
        report['control_upload_attempted'] = True
        # 学习：记录当前执行阶段，立即保存日志并在终端显示。
        set_stage(report, '建立机器人控制连接并上传空闲控制程序；尚未移动')
        # 学习：把“创建控制接口并上传控制脚本；只在用户确认后执行”保存到c（机器人控制对象）；单个=表示赋值。
        c = rtde_control.RTDEControlInterface(ip, 125.0)
        # 学习：把“计算time.monotonic() + 5”保存到until（等待结束的单调时刻）；单个=表示赋值。
        until = time.monotonic() + 5
        # 学习：while循环：只要r.getRuntimeState() != 2成立就继续；完成、异常或超时会结束等待。
        while r.getRuntimeState() != 2:
            # 学习：判断条件time.monotonic() > until；成立(True)时执行下方缩进代码。
            if time.monotonic() > until:
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Control script did not enter running state')
            # 学习：短暂等待0.03秒(30毫秒)，避免空转占满CPU；循环下次醒来继续检查状态。
            time.sleep(0.03)
        # 学习：判断条件not c.setWatchdog(1.0)；成立(True)时执行下方缩进代码。
        if not c.setWatchdog(1.0):
            # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Cannot configure control watchdog')
        # 学习：把“调用Trial，按本行参数取得结果”保存到trial（管理当前一轮动作与反馈的Trial对象）；单个=表示赋值。
        trial = Trial(r, c, d, report)
        # 学习：把“recovery（是否选择恢复后重新开始）”保存到trial.recovery_mode；单个=表示赋值。
        trial.recovery_mode = recovery
        # 学习：把“条件recovery成立取前一值，否则取后一值”保存到trial.recovery_orientation；单个=表示赋值。
        trial.recovery_orientation = start[3:] if recovery else None
        # 学习：把“fresh_cylinder['position']（电缸实际位置，单位0.01 mm）”保存到trial.cylinder_expected；单个=表示赋值。
        trial.cylinder_expected = fresh_cylinder['position']
        # 学习：把“start（该段开始时的状态或位置）”保存到trial.expected_hold；单个=表示赋值。
        trial.expected_hold = start
        # 学习：检查一次实时状态并更新看门狗。
        trial.tick()
        # 学习：把“False”保存到report['full_path_sampling_enabled']（是否执行全路径预采样；当前关闭）；单个=表示赋值。
        report['full_path_sampling_enabled'] = False
        # 学习：把“0”保存到report['preflight_samples']（全路径预采样数；当前为0）；单个=表示赋值。
        report['preflight_samples'] = 0
        # 学习：把“FIXED_ORIENTATION_RAD[:]”保存到report['fixed_orientation_rad']（保存的固定旋转向量，单位rad）；单个=表示赋值。
        report['fixed_orientation_rad'] = FIXED_ORIENTATION_RAD[:]
        # 学习：把“ORIENTATION_CAPTURED_AT（固定姿态的实际读取时间）”保存到report['orientation_captured_at']（固定姿态读取时间）；单个=表示赋值。
        report['orientation_captured_at'] = ORIENTATION_CAPTURED_AT
        # 学习：把“创建列表副本，数据来自r.getActualTCPPose()”保存到actual（当前实际TCP位姿）；单个=表示赋值。
        actual = list(r.getActualTCPPose())
        # 学习：判断条件math.dist(actual[:3], start[:3]) > POSITION_TOL or orientation_error(actual[3:], start[3:]) > ANGLE_TOL；成立(True)时执行下方缩进代码。
        if math.dist(actual[:3], start[:3]) > POSITION_TOL or orientation_error(actual[3:], start[3:]) > ANGLE_TOL:
            # 学习：抛出异常：建立连接期间机器人位置或姿态发生变化；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Start moved during connection setup')
        # 学习：记录当前执行阶段，立即保存日志并在终端显示。
        set_stage(report, '进入连续恢复准备，尚未恢复固定姿态' if recovery else '保持读取时姿态，连续执行A–G；没有姿态归零动作')
        # 学习：已确认恢复时，先依序回缸0、恢复原姿态并回A；之后从头开始完整新一轮。
        if recovery:
            # 学习：记录当前执行阶段，立即保存日志并在终端显示。
            set_stage(report, '中断后连续重新准备，不续跑旧步骤')
            # 学习：记录当前执行阶段，立即保存日志并在终端显示。
            set_stage(report, '恢复1：机械臂保持当前XYZ和姿态，电缸以1 mm/s回到标定位置0 mm（同组新指令解除软件暂停）')
            # 学习：执行一次电缸绝对位置定位并确认到位。
            trial.cylinder(0)
            # 学习：记录当前执行阶段，立即保存日志并在终端显示。
            set_stage(report, '恢复2：保持当前XYZ，将末端恢复到13:10:55记录的固定姿态')
            # 学习：保持XYZ，恢复原固定姿态。
            trial.restore_fixed_orientation()
            # 学习：记录当前执行阶段，立即保存日志并在终端显示。
            set_stage(report, '恢复3：电缸保持0 mm，机械臂返回A点')
            # 学习：执行一次机器人到点动作并确认到位。
            trial.robot('A')
            # 学习：把“True”保存到report['recovered_to_start']（恢复准备是否回到起始状态）；单个=表示赋值。
            report['recovered_to_start'] = True
            # 学习：把“True”保存到report['restart_cycle_from_beginning']（是否从头开始新一轮）；单个=表示赋值。
            report['restart_cycle_from_beginning'] = True
            # 学习：记录当前执行阶段，立即保存日志并在终端显示。
            set_stage(report, '已回到起始状态；连续从B点开始新一轮，不续跑中断段')
        # 学习：依次列出当前→A→B→缸48→C→缸13→A→D→E→缸48→F→G→缸0→A；循环按此顺序连续执行。
        route = [('robot', 'A'), ('robot', 'B'), ('cylinder', 48), ('robot', 'C'), ('cylinder', 13), ('robot', 'A'), ('robot', 'D'), ('robot', 'E'), ('cylinder', 48), ('robot', 'F'), ('robot', 'G'), ('cylinder', 0), ('robot', 'A')]
        # 学习：已确认恢复时，先依序回缸0、恢复原姿态并回A；之后从头开始完整新一轮。
        if recovery:
            # 学习：恢复阶段已经回过A，因此删掉任务清单首项重复的回A，下一动作直接A→B。
            route = route[1:]
        # 学习：for逐项循环：从route取一个元素放入(axis, target)，再执行缩进部分。
        for axis, target in route:
            # 学习：记录当前执行阶段，立即保存日志并在终端显示。
            set_stage(report, f'执行：{axis} → {target}')
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print(f'{axis} -> {target}', flush=True)
            # 学习：axis为robot就调用trial.robot(target)，为cylinder就调用trial.cylinder(target)；本段到位停顿结束后才进入下一段。
            getattr(trial, axis)(target)
            # 学习：保存当前运行日志。
            save(report)
        # 学习：批量更新字典中的字段。
        report.update(success=True, final_robot_pose=list(r.getActualTCPPose()), final_cylinder=d.snapshot())
    # 学习：except捕获BaseException，执行下面的错误记录或清理逻辑。
    except BaseException as error:
        # 学习：把“str(error) or type(error).__name__”保存到report['error']（错误说明）；单个=表示赋值。
        report['error'] = str(error) or type(error).__name__
        # 学习：把“从字典读取字段，不存在时使用给定默认值”保存到report['error_stage']（失败发生在哪个阶段）；单个=表示赋值。
        report['error_stage'] = report.get('stage')
        # 学习：把“调用traceback.format_exc，按本行参数取得结果”保存到report['error_traceback']（错误调用堆栈）；单个=表示赋值。
        report['error_traceback'] = traceback.format_exc(limit=8)
        # 学习：判断条件c is not None；成立(True)时执行下方缩进代码。
        if c is not None:
            # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
            try:
                # 学习：请求机器人减速停止，参数为减速度，不是安全级急停。
                c.stopL(0.05)
                # 学习：把“True”保存到report['robot_stop_requested']（是否已请求机器人软件停止）；单个=表示赋值。
                report['robot_stop_requested'] = True
            # 学习：except捕获BaseException，执行下面的错误记录或清理逻辑。
            except BaseException as stop_error:
                # 学习：把“转换或创建str类型的数据”保存到report['robot_stop_error']；单个=表示赋值。
                report['robot_stop_error'] = str(stop_error)
        # 学习：判断条件d is not None and (report.get('cylinder_motion_attempted') or report.get('cylinder_control_attempted'))；成立(True)时执行下方缩进代码。
        if d is not None and (report.get('cylinder_motion_attempted') or report.get('cylinder_control_attempted')):
            # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
            try:
                # 学习：请求电缸保持使能的软件暂停。
                d.pause()
                # 学习：把“True”保存到report['cylinder_pause_acknowledged']（是否收到电缸软件暂停应答）；单个=表示赋值。
                report['cylinder_pause_acknowledged'] = True
            # 学习：except捕获BaseException，执行下面的错误记录或清理逻辑。
            except BaseException as stop_error:
                # 学习：把“转换或创建str类型的数据”保存到report['cylinder_pause_error']；单个=表示赋值。
                report['cylinder_pause_error'] = str(stop_error)
        # 学习：把刚才捕获的异常继续抛给上层，避免错误后继续执行后续动作。
        raise
    # 学习：finally中的资源释放与日志保存，无论前面成功、报错或被键盘中断都尝试执行。
    finally:
        # 学习：判断条件d is not None；成立(True)时执行下方缩进代码。
        if d is not None:
            # 学习：把“d.log”保存到report['cylinder_frames']；单个=表示赋值。
            report['cylinder_frames'] = d.log
            # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
            try:
                # 学习：关闭当前资源或串口。
                d.close()
            # 学习：except捕获Exception，执行下面的错误记录或清理逻辑。
            except Exception as error:
                # 学习：把“转换或创建str类型的数据”保存到report['cylinder_close_error']；单个=表示赋值。
                report['cylinder_close_error'] = str(error)
        # 学习：判断条件c is not None；成立(True)时执行下方缩进代码。
        if c is not None:
            # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
            try:
                # 学习：结束本程序上传的机器人控制脚本。
                c.stopScript()
            # 学习：except捕获Exception，执行下面的错误记录或清理逻辑。
            except Exception as error:
                # 学习：把“转换或创建str类型的数据”保存到report['control_cleanup_error']；单个=表示赋值。
                report['control_cleanup_error'] = str(error)
            # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
            try:
                # 学习：断开当前RTDE接口。
                c.disconnect()
            # 学习：except捕获Exception，执行下面的错误记录或清理逻辑。
            except Exception:
                # 学习：pass不做额外操作；这里用于清理失败时保持流程可继续释放其他资源。
                pass
        # 学习：判断条件r is not None；成立(True)时执行下方缩进代码。
        if r is not None:
            # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
            try:
                # 学习：断开当前RTDE接口。
                r.disconnect()
            # 学习：except捕获Exception，执行下面的错误记录或清理逻辑。
            except Exception as error:
                # 学习：把“转换或创建str类型的数据”保存到report['receive_cleanup_error']；单个=表示赋值。
                report['receive_cleanup_error'] = str(error)
        # 学习：把“返回带UTC+8时区的当前时间字符串，用于日志记录”保存到report['end']（结束时间）；单个=表示赋值。
        report['end'] = now()
        # 学习：保存当前运行日志。
        save(report)


# 学习：定义save函数：把当前运行报告保存为UTF-8格式JSON日志；def本身只是定义，调用时才执行函数体。
def save(report):
    # 学习：把“计算ROOT / 'logs'”保存到folder（输出目录）；单个=表示赋值。
    folder = ROOT / 'logs'
    # 学习：创建输出目录，已存在时按参数决定是否允许。
    folder.mkdir(exist_ok=True)
    # 学习：把“计算folder / report['log_file']”保存到path（输出文件路径）；单个=表示赋值。
    path = folder / report['log_file']
    # 学习：以UTF-8写入文本文件。
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')


# 学习：定义main函数：模块版联动程序：默认只读；--run经一次确认后连续运行。坐标使用m，姿态使用rad。；def本身只是定义，调用时才执行函数体。
def main():
    # 学习：把“调用argparse.ArgumentParser，按本行参数取得结果”保存到parser（命令行参数解析器）；单个=表示赋值。
    parser = argparse.ArgumentParser(description=__doc__)
    # 学习：把“规定同组运行模式参数不能同时选择”保存到mode（互斥运行模式参数组）；单个=表示赋值。
    mode = parser.add_mutually_exclusive_group()
    # 学习：声明一个可接受的启动参数及默认值。
    mode.add_argument('--check', action='store_true')
    # 学习：声明一个可接受的启动参数及默认值。
    mode.add_argument('--preview', action='store_true')
    # 学习：声明一个可接受的启动参数及默认值。
    mode.add_argument('--run', action='store_true')
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--ip', default='192.168.21.5')
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--port', default='COM4')
    # 学习：把“解析本次命令行参数”保存到args（用户启动参数）；单个=表示赋值。
    args = parser.parse_args()
    # 学习：判断条件args.preview；成立(True)时执行下方缩进代码。
    if args.preview:
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(json.dumps(build_plan(), ensure_ascii=False, indent=2))
        # 学习：return把0交回调用者，并立即结束当前函数。
        return 0
    # 学习：把“创建一份键值数据字典，或复制已有字典”保存到report（运行或测试报告字典）；单个=表示赋值。
    report = dict(start=now(), success=False, segments=[], motion_commands_sent=False, log_file=datetime.now().strftime('trial_%Y%m%d_%H%M%S_%f.json'))
    # 学习：try开始可能报错的操作；except负责处理错误，finally无论成功失败都用于清理/记录。
    try:
        # 学习：把“只读检查设备，分别判断普通运行和人工选择恢复是否可行”保存到report['check']（启动检查结果）；单个=表示赋值。
        report['check'] = inspect(args.ip, args.port)
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(json.dumps(report['check'], ensure_ascii=False, indent=2))
        # 学习：判断条件not args.run；成立(True)时执行下方缩进代码。
        if not args.run:
            # 学习：把“True”保存到report['success']（流程是否成功完成）；单个=表示赋值。
            report['success'] = True
            # 学习：return把0交回调用者，并立即结束当前函数。
            return 0
        # 学习：把“False”保存到recovery（是否选择恢复后重新开始）；单个=表示赋值。
        recovery = False
        # 学习：判断条件not report['check']['trial_start_ready']；成立(True)时执行下方缩进代码。
        if not report['check']['trial_start_ready']:
            # 学习：判断条件not report['check'].get('manual_recovery_available')；成立(True)时执行下方缩进代码。
            if not report['check'].get('manual_recovery_available'):
                # 学习：抛出异常：发现不满足要求的状态或结果，按本行错误说明中止当前流程；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError(report['check']['blocking_reason'])
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print('中断后尚未回到起始状态。确认后将连续执行：电缸回0（新定位指令解除软件暂停）→ 恢复固定姿态 → 回A → 从头完整运行一轮。')
            # 学习：判断条件input('输入 RECOVER 后连续恢复并运行完整一轮；其他输入退出：').strip() != 'RECOVER'；成立(True)时执行下方缩进代码。
            if input('输入 RECOVER 后连续恢复并运行完整一轮；其他输入退出：').strip() != 'RECOVER':
                # 学习：抛出异常：操作员未输入要求的启动确认；交由上层错误处理，不继续本段正常流程。
                raise RuntimeError('Cancelled before manual recovery')
            # 学习：把“True”保存到recovery（是否选择恢复后重新开始）；单个=表示赋值。
            recovery = True
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print('单次连续实际运行：不做全路径采样。各段到位停顿1秒后自动进入下一段，控制器状态和目标限位仍会核验。')
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print(f'保持读取时姿态 Rx/Ry/Rz={FIXED_ORIENTATION_RAD} rad；J6自由；机械臂30 mm/s，电缸1 mm/s。')
        # 学习：判断条件not recovery and input('输入 RUN 后回车连续运行完整一轮（无需逐段确认）：').strip() != 'RUN'；成立(True)时执行下方缩进代码。
        if not recovery and input('输入 RUN 后回车连续运行完整一轮（无需逐段确认）：').strip() != 'RUN':
            # 学习：抛出异常：操作员未输入要求的启动确认；交由上层错误处理，不继续本段正常流程。
            raise RuntimeError('Cancelled before control upload')
        # 学习：把“False”保存到report['motion_commands_sent']（是否尝试发送过运动命令）；单个=表示赋值。
        report['motion_commands_sent'] = False
        # 学习：把“False”保存到report['manual_steps']（是否逐段等待确认；当前连续版为False）；单个=表示赋值。
        report['manual_steps'] = False
        # 学习：把“'continuous_single_cycle'”保存到report['run_mode']（运行方式说明）；单个=表示赋值。
        report['run_mode'] = 'continuous_single_cycle'
        # 学习：把“recovery（是否选择恢复后重新开始）”保存到report['manual_recovery_selected']（操作员是否选择了恢复入口）；单个=表示赋值。
        report['manual_recovery_selected'] = recovery
        # 学习：连接控制接口，连续执行恢复或整套任务，并在异常时停止和释放资源。
        execute(args.ip, args.port, report, recovery=recovery)
        # 学习：return把0交回调用者，并立即结束当前函数。
        return 0
    # 学习：except捕获(Exception, KeyboardInterrupt)，执行下面的错误记录或清理逻辑。
    except (Exception, KeyboardInterrupt) as error:
        # 学习：把“str(error) or type(error).__name__”保存到report['error']（错误说明）；单个=表示赋值。
        report['error'] = str(error) or type(error).__name__
        # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
        print('Stopped: ' + report['error'])
        # 学习：判断条件report.get('error_stage')；成立(True)时执行下方缩进代码。
        if report.get('error_stage'):
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print('中断/失败阶段：' + report['error_stage'], flush=True)
        # 学习：判断条件isinstance(error, KeyboardInterrupt)；成立(True)时执行下方缩进代码。
        if isinstance(error, KeyboardInterrupt):
            # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
            print('收到键盘或调试器中断；程序已退出，不会继续下一段。', flush=True)
        # 学习：return把1交回调用者，并立即结束当前函数。
        return 1
    # 学习：finally中的资源释放与日志保存，无论前面成功、报错或被键盘中断都尝试执行。
    finally:
        # 学习：把“转换或创建bool类型的数据”保存到report['motion_commands_sent']（是否尝试发送过运动命令）；单个=表示赋值。
        report['motion_commands_sent'] = bool(report.get('robot_motion_attempted') or report.get('cylinder_motion_attempted'))
        # 学习：保存当前运行日志。
        save(report)

# 学习：只有直接启动本文件时才进入主流程；导入文件不会启动设备。
if __name__ == '__main__':
    # 学习：没有命令行参数时，为单文件程序选择实际连续运行入口；仍需手动输入RUN或RECOVER。
    if len(sys.argv) == 1:
        # 学习：添加--run参数；各段之间无需输入GO。
        sys.argv.extend(['--run'])
    # 学习：运行主函数并把成功/失败状态码交给操作系统。
    raise SystemExit(main())
