# 学习：离线几何模拟：只计算轨迹、时间并输出文件，不连接真实设备。
# 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
"""A-G offline geometric simulation. Standard library only; no device I/O.

Run: python simulate_ag.py
Optional synthetic initial position: --start-xyz X Y Z (mm)
Tool rotation vector is fixed at the hardware snapshot, relative to base. No joint is
locked. No IK, robot links, collisions or homing search simulated.
"""
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


# 学习：定义sample函数：根据模拟时刻取出该时刻TCP位置、电缸位置和固定姿态；def本身只是定义，调用时才执行函数体。
def sample(plan, t):
    # 学习：把“取几个数值中的最小值”保存到t（模拟时刻或当前协议字段类型）；单个=表示赋值。
    t = min(max(0.0, t), plan['total_duration_s'])
    # 学习：把“调用next，按本行参数取得结果”保存到phase（模拟阶段数据）；单个=表示赋值。
    phase = next((p for p in plan['phases'] if p['end_s'] > t), plan['phases'][-1])
    # 学习：把“phase['distance_mm']”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
    d = phase['distance_mm']
    # 学习：把“条件d成立取前一值，否则取后一值”保存到f；单个=表示赋值。
    f = displacement(d, t-phase['start_s'], phase['max_speed_mm_s'], phase['virtual_acceleration_mm_s2']) / d if d else 1.0
    # 学习：把“逐项遍历并计算，生成所需数据：[a + (b - a) * f for a, b in zip(phase['from_xyz_mm'], phase['to_xyz_mm'])]”保存到xyz（模拟TCP位置，单位mm）；单个=表示赋值。
    xyz = [a+(b-a)*f for a,b in zip(phase['from_xyz_mm'], phase['to_xyz_mm'])]
    # 学习：把“计算phase['from_cylinder_mm'] + (phase['to_cylinder_mm'] - phase['from_cylinder_mm']) * f”保存到cylinder（电缸状态字典）；单个=表示赋值。
    cylinder = phase['from_cylinder_mm'] + (phase['to_cylinder_mm']-phase['from_cylinder_mm'])*f
    # 学习：return把创建一份键值数据字典，或复制已有字典交回调用者，并立即结束当前函数。
    return dict(t_s=t, phase=phase['index'], step=phase['step'], label=phase['label'],
                # 学习：续写当前多行表达式中的一部分：x_mm=xyz[0], y_mm=xyz[1], z_mm=xyz[2], cylinder_mm=cylinder,。
                x_mm=xyz[0], y_mm=xyz[1], z_mm=xyz[2], cylinder_mm=cylinder,
                # 学习：续写当前多行表达式中的一部分：rx_rad=ROTATION_VECTOR[0], ry_rad=ROTATION_VECTOR[1], rz_rad=ROTATION_VECTOR[2],。
                rx_rad=ROTATION_VECTOR[0], ry_rad=ROTATION_VECTOR[1], rz_rad=ROTATION_VECTOR[2],
                # 学习：续写当前多行表达式中的一部分：tool_z_x=TOOL_AXIS[0], tool_z_y=TOOL_AXIS[1], tool_z_z=TOOL_AXIS[2])。
                tool_z_x=TOOL_AXIS[0], tool_z_y=TOOL_AXIS[1], tool_z_z=TOOL_AXIS[2])


# 学习：定义validate函数：检查模拟计划的顺序、边界、姿态与最终归位；def本身只是定义，调用时才执行函数体。
def validate(plan):
    # 学习：assert核对plan['robot_max_speed_mm_s'] == 30；不成立就使离线验证失败。
    assert plan['robot_max_speed_mm_s'] == 30
    # 学习：assert核对plan['cylinder_max_speed_mm_s'] == 1；不成立就使离线验证失败。
    assert plan['cylinder_max_speed_mm_s'] == 1
    # 学习：assert核对plan['locked_joints'] == []；不成立就使离线验证失败。
    assert plan['locked_joints'] == []
    # 学习：assert核对plan['rotation_vector_rad'] == ROTATION_VECTOR；不成立就使离线验证失败。
    assert plan['rotation_vector_rad'] == ROTATION_VECTOR
    # 学习：assert核对plan['tool_axis_in_base'] == TOOL_AXIS；不成立就使离线验证失败。
    assert plan['tool_axis_in_base'] == TOOL_AXIS
    # 学习：把“plan['phases']”保存到phases（模拟各阶段列表）；单个=表示赋值。
    phases = plan['phases']
    # 学习：assert核对[p['step'] for p in phases if p['axis'] in ('robot', 'cylinder') and p['step'] != '准备'] == ['1', '2', '4', '5', '6', '7', '8', '9', '10', '1…；不成立就使离线验证失败。
    assert [p['step'] for p in phases if p['axis'] in ('robot','cylinder') and p['step'] != '准备'] == [
        # 学习：续写当前多行表达式中的一部分：'1','2','4','5','6','7','8','9','10','11','12a','12b']。
        '1','2','4','5','6','7','8','9','10','11','12a','12b']
    # 学习：for逐项循环：从enumerate(phases)取一个元素放入(i, p)，再执行缩进部分。
    for i, p in enumerate(phases):
        # 学习：判断条件i；成立(True)时执行下方缩进代码。
        if i:
            # 学习：assert核对p['from_xyz_mm'] == phases[i - 1]['to_xyz_mm']；不成立就使离线验证失败。
            assert p['from_xyz_mm'] == phases[i-1]['to_xyz_mm']
            # 学习：assert核对p['from_cylinder_mm'] == phases[i - 1]['to_cylinder_mm']；不成立就使离线验证失败。
            assert p['from_cylinder_mm'] == phases[i-1]['to_cylinder_mm']
        # 学习：assert核对0 <= p['to_cylinder_mm'] <= 50；不成立就使离线验证失败。
        assert 0 <= p['to_cylinder_mm'] <= 50
        # 学习：判断条件p['axis'] in ('robot', 'cylinder')；成立(True)时执行下方缩进代码。
        if p['axis'] in ('robot','cylinder'):
            # 学习：assert核对phases[i + 1]['axis'] == 'dwell' and phases[i + 1]['duration_s'] == 1；不成立就使离线验证失败。
            assert phases[i+1]['axis'] == 'dwell' and phases[i+1]['duration_s'] == 1
        # 学习：for逐项循环：从range(101)取一个元素放入j，再执行缩进部分。
        for j in range(101):
            # 学习：把“根据模拟时刻取出该时刻TCP位置、电缸位置和固定姿态”保存到row；单个=表示赋值。
            row = sample(plan, p['start_s'] + p['duration_s']*j/100)
            # 学习：assert核对row['tool_z_z'] == TOOL_AXIS[2]；不成立就使离线验证失败。
            assert row['tool_z_z'] == TOOL_AXIS[2]
            # 学习：assert核对[row[k] for k in ('rx_rad', 'ry_rad', 'rz_rad')] == ROTATION_VECTOR；不成立就使离线验证失败。
            assert [row[k] for k in ('rx_rad','ry_rad','rz_rad')] == ROTATION_VECTOR
        # Analytic maximum speed is a*ramp, including triangular short moves.
        # 学习：把“组成一组固定顺序的数据，便于成组赋值或返回”保存到(speed, accel)；单个=表示赋值。
        speed, accel = p['max_speed_mm_s'], p['virtual_acceleration_mm_s2']
        # 学习：把“按距离、速度和加速度计算三角形或梯形速度曲线各段时间”保存到(ramp, _, duration)；单个=表示赋值。
        ramp, _, duration = motion_profile(p['distance_mm'], speed, accel)
        # 学习：assert核对accel * ramp <= speed + 1e-12；不成立就使离线验证失败。
        assert accel*ramp <= speed+1e-12
        # 学习：assert核对abs(displacement(p['distance_mm'], duration, speed, accel) - p['distance_mm']) < 1e-09；不成立就使离线验证失败。
        assert abs(displacement(p['distance_mm'], duration, speed, accel)-p['distance_mm']) < 1e-9
    # 学习：把“根据模拟时刻取出该时刻TCP位置、电缸位置和固定姿态”保存到last；单个=表示赋值。
    last = sample(plan, plan['total_duration_s'])
    # 学习：assert核对math.dist([last['x_mm'], last['y_mm'], last['z_mm']], POINTS['A']) < 1e-09；不成立就使离线验证失败。
    assert math.dist([last['x_mm'],last['y_mm'],last['z_mm']], POINTS['A']) < 1e-9
    # 学习：assert核对last['cylinder_mm'] == 0；不成立就使离线验证失败。
    assert last['cylinder_mm'] == 0
    # 学习：return把建立字典，用字段名保存各项状态或参数交回调用者，并立即结束当前函数。
    return {'passed': True, 'checks': ['updated sequence', 'exact XYZ endpoints',
            # 学习：续写当前多行表达式中的一部分：'continuous phase boundaries', 'robot max 30 mm/s, cylinder max 1 mm/s', '1 s arrival dwell',。
            'continuous phase boundaries', 'robot max 30 mm/s, cylinder max 1 mm/s', '1 s arrival dwell',
            # 学习：续写当前多行表达式中的一部分：'0..50 mm cylinder range', 'constant captured hardware orientation', 'no locked joints', 'final A and cylinder zero'],。
            '0..50 mm cylinder range', 'constant captured hardware orientation', 'no locked joints', 'final A and cylinder zero'],
            # 学习：字典条目：'limitation'对应'Geometric simulation only. No IK, solved joint trajectory, link geometry, collisions or real homing…。
            'limitation': 'Geometric simulation only. No IK, solved joint trajectory, link geometry, collisions or real homing.'}


# 学习：定义main函数：离线几何模拟：只计算轨迹、时间并输出文件，不连接真实设备。；def本身只是定义，调用时才执行函数体。
def main():
    # 学习：把“调用argparse.ArgumentParser，按本行参数取得结果”保存到parser（命令行参数解析器）；单个=表示赋值。
    parser = argparse.ArgumentParser(description=__doc__)
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--start-xyz', nargs=3, type=float, metavar=('X','Y','Z'))
    # 学习：声明一个可接受的启动参数及默认值。
    parser.add_argument('--out-dir', type=Path, default=Path(__file__).resolve().parent)
    # 学习：把“解析本次命令行参数”保存到args（用户启动参数）；单个=表示赋值。
    args = parser.parse_args()
    # 学习：判断条件args.start_xyz and (not all((math.isfinite(x) for x in args.start_xyz)))；成立(True)时执行下方缩进代码。
    if args.start_xyz and not all(math.isfinite(x) for x in args.start_xyz):
        # 学习：打印参数错误并结束参数解析。
        parser.error('Start coordinates must be finite')
    # 学习：把“按A–G顺序生成离线计划，并为每个到位动作插入1秒停顿”保存到plan；单个=表示赋值。
    plan = build_plan(args.start_xyz)
    # 学习：把“调用validate，按本行参数取得结果”保存到checks（模拟自检结果）；单个=表示赋值。
    checks = validate(plan)
    # 学习：创建输出目录，已存在时按参数决定是否允许。
    args.out_dir.mkdir(parents=True, exist_ok=True)
    # 学习：以UTF-8写入文本文件。
    (args.out_dir/'simulation_plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding='utf-8')
    # 学习：以UTF-8写入文本文件。
    (args.out_dir/'simulation_checks.json').write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding='utf-8')
    # 学习：把“转换或创建set类型的数据”保存到times（需要输出模拟采样的时刻集合）；单个=表示赋值。
    times = set([0.0, plan['total_duration_s']])
    # 学习：批量更新字典中的字段。
    times.update(i/10 for i in range(math.ceil(plan['total_duration_s']*10)))
    # 学习：批量更新字典中的字段。
    times.update(p['end_s'] for p in plan['phases'])
    # 学习：with限定资源或临时替换的有效范围：打开已配置的串口；离开时自动执行对应清理。
    with (args.out_dir/'simulation_trace.csv').open('w',newline='',encoding='utf-8-sig') as f:
        # 学习：把“调用csv.DictWriter，按本行参数取得结果”保存到writer（CSV表格写入器）；单个=表示赋值。
        writer = csv.DictWriter(f, fieldnames=list(sample(plan,0)))
        # 学习：写CSV表格列名。
        writer.writeheader()
        # 学习：for逐项循环：从sorted(times)取一个元素放入t，再执行缩进部分。
        for t in sorted(times):
            # 学习：写入一行CSV采样结果。
            writer.writerow(sample(plan,t))
    # 学习：把“在同一目录中选择另一个文件名”保存到template（模拟页面模板文件）；单个=表示赋值。
    template = Path(__file__).with_name('simulation_view.html')
    # 学习：判断条件template.exists()；成立(True)时执行下方缩进代码。
    if template.exists():
        # 学习：把“替换文本中的指定片段”保存到fragment（填入计划数据后的页面内容）；单个=表示赋值。
        fragment = template.read_text(encoding='utf-8').replace('/*PLAN_DATA*/{}', json.dumps(plan,ensure_ascii=False))
        # 学习：以UTF-8写入文本文件。
        (args.out_dir/'ag_simulation.html').write_text(
            # 学习：续写上方调用的参数或文本：'<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'。
            '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
            # 学习：续写上方调用的参数或文本：'<title>A–G 离线模拟</title><style>:root{color-scheme:light dark;--background:Canvas;--foreground:CanvasText;--border:GrayText;--muted:ButtonFace;--muted-foreground…。
            '<title>A–G 离线模拟</title><style>:root{color-scheme:light dark;--background:Canvas;--foreground:CanvasText;--border:GrayText;--muted:ButtonFace;--muted-foreground:GrayText;--viz-series-1:light-dark(#17654b,#62d4ad);--viz-series-2:light-dark(#3155a0,#9cbaff);font:16px system-ui}body{max-width:900px;margin:24px auto;padding:0 16px;background:var(--background);color:var(--foreground)}button,input,select{font:inherit}button{padding:8px 14px} .viz-controls{display:flex;gap:12px;flex-wrap:wrap;align-items:center}.form-range{width:100%}.text-small{font-size:13px}.text-muted{color:var(--muted-foreground)}</style>'
            # 学习：续写上方调用的参数或文本：+fragment+'</html>',encoding='utf-8')。
            +fragment+'</html>',encoding='utf-8')
        # 学习：以UTF-8写入文本文件。
        (args.out_dir/'ag_simulation_inline.html').write_text(fragment,encoding='utf-8')
    # 学习：向终端输出说明或结果，flush=True表示立即刷新显示。
    print(json.dumps({'simulation_completed': True, 'checks': checks['passed'],
                      # 学习：字典条目：'duration_s'对应把位置换算结果取整，供整数寄存器使用。
                      'duration_s': round(plan['total_duration_s'],3),
                      # 学习：字典条目：'duration_min'对应把位置换算结果取整，供整数寄存器使用。
                      'duration_min': round(plan['total_duration_s']/60,3),
                      # 学习：字典条目：'robot_path_mm'对应把位置换算结果取整，供整数寄存器使用。
                      'robot_path_mm': round(sum(p['distance_mm'] for p in plan['phases'] if p['axis']=='robot'),3),
                      # 学习：字典条目：'cylinder_travel_mm'对应把各项数值相加。
                      'cylinder_travel_mm': sum(p['distance_mm'] for p in plan['phases'] if p['axis']=='cylinder'),
                      # 学习：字典条目：'final'对应根据模拟时刻取出该时刻TCP位置、电缸位置和固定姿态。
                      'final':sample(plan,plan['total_duration_s'])},ensure_ascii=False,indent=2))


# 学习：只有直接启动该文件时才执行入口；被其他文件import时跳过。
if __name__ == '__main__':
    # 学习：调用main，按本行参数取得结果。
    main()
