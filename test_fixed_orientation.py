# 学习：离线测试：用模拟对象代替机器人和电缸，检查动作和异常处理。
# 学习：下面是模块/函数的说明文字docstring，不是设备指令；保留原文字不改字符串内容。
"""Offline tests for the captured-orientation continuous program; no hardware I/O."""
# 学习：导入ast，用于解析Python语法结构，不执行被解析代码。
import ast
# 学习：导入json，用于读写JSON数据文件。
import json
# 学习：导入unittest，用于组织并运行离线测试。
import unittest
# 学习：从pathlib导入Path，用于以Path对象处理文件与目录。
from pathlib import Path
# 学习：从unittest.mock导入Mock, patch，用于用Mock和patch模拟设备及临时替换函数。
from unittest.mock import Mock,patch
# 学习：导入ag_trial，用于调用该模块提供的功能。
import ag_trial as app


# 学习：定义CapturedOrientation类：把每项离线测试组织成unittest测试类；self表示当前创建的那个对象。
class CapturedOrientation(unittest.TestCase):
    # 学习：定义setUp函数：为每个测试准备独立、正常就绪的模拟机器人和电缸状态；def本身只是定义，调用时才执行函数体。
    def setUp(self):
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到self.robot；单个=表示赋值。
        self.robot=dict(connected=True,enabled=True,power_on=True,mode=7,
                        # 学习：续写上方赋值的数据/参数：emergency_stop=False,protective_stop=False,program_running=False,；这些内容仍属于同一条赋值语句。
                        emergency_stop=False,protective_stop=False,program_running=False,
                        # 学习：续写上方赋值的数据/参数：program_paused=False,pose=app.POSES['A'][:],tcp_offset=[0.]*6)；这些内容仍属于同一条赋值语句。
                        program_paused=False,pose=app.POSES['A'][:],tcp_offset=[0.]*6)
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到self.dash；单个=表示赋值。
        self.dash={'PolyscopeVersion':'URSoftware 3.15.8.106339',
                   # 学习：字典条目：'running'对应'Program running: false'。
                   'safetymode':'Safetymode: NORMAL','running':'Program running: false'}
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到self.cyl；单个=表示赋值。
        self.cyl=dict(id=1,position=0,homing=1,status=17504,alarm=0,control=33,io=0,limit=5084,offset=100)

    # 学习：定义test_targets_reproduce_hardware_snapshot函数：验证全部目标沿用实际读取的完整精度姿态；def本身只是定义，调用时才执行函数体。
    def test_targets_reproduce_hardware_snapshot(self):
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到expected（本测试期望得到的值）；单个=表示赋值。
        expected=[1.4747881408617243,-.6236196873562871,.6049084354198352]
        # 学习：核对实际值与期望值完全相等。
        self.assertEqual(app.FIXED_ORIENTATION_RAD,expected)
        # 学习：for逐项循环：从app.POSES.values()取一个元素放入pose，再执行缩进部分。
        for pose in app.POSES.values():
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(pose[3:],expected)
        # 学习：核对实际值与期望值完全相等。
        self.assertEqual(app.build_plan()['rotation_vector_rad'],expected)

    # 学习：定义test_start_requires_captured_orientation_not_zero函数：验证普通启动要求保存姿态，不能直接使用零姿态；def本身只是定义，调用时才执行函数体。
    def test_start_requires_captured_orientation_not_zero(self):
        # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
        app.require_initial(self.robot,self.dash,self.cyl)
        # 学习：把“计算self.robot['pose'][:3] + [0.0, 0.0, 0.0]”保存到self.robot['pose']（TCP位姿：XYZ为m，姿态为rad）；单个=表示赋值。
        self.robot['pose']=self.robot['pose'][:3]+[0.,0.,0.]
        # 学习：with限定资源或临时替换的有效范围：核对抛出的异常类型和错误文字符合预期；离开时自动执行对应清理。
        with self.assertRaisesRegex(RuntimeError,'captured fixed orientation'):
            # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
            app.require_initial(self.robot,self.dash,self.cyl)

    # 学习：定义test_equivalent_rotation_representation_is_accepted函数：验证同一物理姿态的等价旋转向量不会被误拒绝；def本身只是定义，调用时才执行函数体。
    def test_equivalent_rotation_representation_is_accepted(self):
        # 学习：把“app.FIXED_ORIENTATION_RAD”保存到rv（三元素旋转向量，单位rad）；单个=表示赋值。
        rv=app.FIXED_ORIENTATION_RAD
        # 学习：把“调用app.math.sqrt，按本行参数取得结果”保存到n；单个=表示赋值。
        n=app.math.sqrt(sum(v*v for v in rv))
        # 学习：把“逐项遍历并计算，生成所需数据：[v * (n + 2 * app.math.pi) / n for v in rv]”保存到equivalent（同一实际旋转的另一种旋转向量表示）；单个=表示赋值。
        equivalent=[v*(n+2*app.math.pi)/n for v in rv]
        # 学习：核对结果为真。
        self.assertTrue(app.matches_fixed_orientation(self.robot['pose'][:3]+equivalent))

    # 学习：定义test_existing_program_and_stop_flags_reject函数：验证已有程序及停止状态会阻止启动；def本身只是定义，调用时才执行函数体。
    def test_existing_program_and_stop_flags_reject(self):
        # 学习：for逐项循环：从('program_running', 'program_paused', 'emergency_stop', 'protective_stop')取一个元素放入key，再执行缩进部分。
        for key in ('program_running','program_paused','emergency_stop','protective_stop'):
            # 学习：with限定资源或临时替换的有效范围：为一组测试数据单独标记结果，便于定位失败；核对这段操作应当抛出指定异常；离开时自动执行对应清理。
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
                app.require_initial(dict(self.robot,**{key:True}),self.dash,self.cyl)

    # 学习：定义test_cylinder_faults_and_unhomed_reject函数：验证电缸报警、未回零和异常状态会阻止启动；def本身只是定义，调用时才执行函数体。
    def test_cylinder_faults_and_unhomed_reject(self):
        # 学习：for逐项循环：从({'homing': 0}, {'alarm': 1}, {'position': 1300}, {'io': 1}, {'status': 17504 | 256})取一个元素放入change，再执行缩进部分。
        for change in ({'homing':0},{'alarm':1},{'position':1300},{'io':1},{'status':17504|256}):
            # 学习：with限定资源或临时替换的有效范围：为一组测试数据单独标记结果，便于定位失败；核对这段操作应当抛出指定异常；离开时自动执行对应清理。
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
                app.require_initial(self.robot,self.dash,dict(self.cyl,**change))

    # 学习：定义test_interrupted_state_is_recoverable_but_not_normal_ready函数：验证中断状态可进入恢复，但不能直接按普通任务运行；def本身只是定义，调用时才执行函数体。
    def test_interrupted_state_is_recoverable_but_not_normal_ready(self):
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到stopped（测试准备的中断状态）；单个=表示赋值。
        stopped=dict(self.cyl,position=4800,control=0x28,status=50272)
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到robot（机械臂状态字典）；单个=表示赋值。
        robot=dict(self.robot,pose=self.robot['pose'][:])
        # 学习：在原值基础上更新robot['pose'][3]：robot['pose'][3] += 0.02。
        robot['pose'][3]+=.02
        # 学习：with限定资源或临时替换的有效范围：核对这段操作应当抛出指定异常；离开时自动执行对应清理。
        with self.assertRaises(RuntimeError):
            # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
            app.require_initial(robot,self.dash,stopped)
        # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
        app.require_initial(robot,self.dash,stopped,recovery=True)

    # 学习：定义test_recovery_never_bypasses_alarm_estop_or_unhomed_state函数：验证恢复入口不能绕过报警、急停或未回零；def本身只是定义，调用时才执行函数体。
    def test_recovery_never_bypasses_alarm_estop_or_unhomed_state(self):
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到stopped（测试准备的中断状态）；单个=表示赋值。
        stopped=dict(self.cyl,position=4800,control=0x28,status=50272)
        # 学习：for逐项循环：从({'alarm': 1}, {'homing': 0}, {'io': 1}, {'control': 44})取一个元素放入change，再执行缩进部分。
        for change in ({'alarm':1},{'homing':0},{'io':1},{'control':0x2C}):
            # 学习：with限定资源或临时替换的有效范围：为一组测试数据单独标记结果，便于定位失败；核对这段操作应当抛出指定异常；离开时自动执行对应清理。
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
                app.require_initial(self.robot,self.dash,dict(stopped,**change),recovery=True)
        # 学习：with限定资源或临时替换的有效范围：核对这段操作应当抛出指定异常；离开时自动执行对应清理。
        with self.assertRaises(RuntimeError):
            # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
            app.require_initial(dict(self.robot,emergency_stop=True),self.dash,stopped,recovery=True)

    # 学习：定义test_paused_retraction_uses_new_zero_target_and_slow_parameters函数：验证解除软件暂停时发送全新0 mm低速定位参数；def本身只是定义，调用时才执行函数体。
    def test_paused_retraction_uses_new_zero_target_and_slow_parameters(self):
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到paused（测试准备的软件暂停状态）；单个=表示赋值。
        paused=dict(self.cyl,position=4800,control=0x28,status=50272)
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。；把“paused（测试准备的软件暂停状态）”保存到d.snapshot.return_value；单个=表示赋值。
        d=Mock();d.snapshot.return_value=paused
        # 学习：把“lambda address: {5647: 0, 5649: 17504, 5644: 0}[address]”保存到d.value.side_effect；单个=表示赋值。
        d.value.side_effect=lambda address:{0x160F:0,0x1611:17504,0x160C:0}[address]
        # 学习：把“调用app.Trial，按本行参数取得结果”保存到t（模拟时刻或当前协议字段类型）；单个=表示赋值。
        t=app.Trial(Mock(),Mock(),d,{'segments':[]})
        # 学习：把“True”保存到t.recovery_mode；单个=表示赋值。；把“4800”保存到t.cylinder_expected；单个=表示赋值。
        t.recovery_mode=True;t.cylinder_expected=4800
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
        with patch.object(t,'tick'),patch.object(t,'dwell'),patch.object(app.time,'sleep'):
            # 学习：执行一次电缸绝对位置定位并确认到位。
            t.cylinder(0)
        # 学习：把“逐项遍历并计算，生成所需数据：[call.args[0] for call in d.write_ack.call_args_list]”保存到frames（测试收集到的写报文字节序列）；单个=表示赋值。
        frames=[call.args[0] for call in d.write_ack.call_args_list]
        # 学习：核对实际值与期望值完全相等。
        self.assertEqual(len(frames),2)
        # 学习：for逐项循环：从zip(frames, (32, 33))取一个元素放入(packet, control)，再执行缩进部分。
        for packet,control in zip(frames,(0x20,0x21)):
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(app.struct.unpack('>BBHHB',packet[:7]),(1,16,0x1600,6,12))
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(app.struct.unpack('>6H',packet[7:]),(0,0,100,1,0,control))
        # 学习：核对实际值与期望值完全相等。
        self.assertEqual(t.cylinder_expected,0)

    # 学习：定义test_failed_recovery_command_does_not_retry函数：验证恢复写入失败后不会重复发出运动指令；def本身只是定义，调用时才执行函数体。
    def test_failed_recovery_command_does_not_retry(self):
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。；把“创建一份键值数据字典，或复制已有字典”保存到d.snapshot.return_value；单个=表示赋值。
        d=Mock();d.snapshot.return_value=dict(self.cyl,position=4800,control=0x28,status=50272)
        # 学习：把“调用RuntimeError，按本行参数取得结果”保存到d.write_ack.side_effect；单个=表示赋值。
        d.write_ack.side_effect=RuntimeError('missing acknowledgement')
        # 学习：把“调用app.Trial，按本行参数取得结果”保存到t（模拟时刻或当前协议字段类型）；单个=表示赋值。
        t=app.Trial(Mock(),Mock(),d,{'segments':[]})
        # 学习：把“True”保存到t.recovery_mode；单个=表示赋值。；把“4800”保存到t.cylinder_expected；单个=表示赋值。
        t.recovery_mode=True;t.cylinder_expected=4800
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；核对抛出的异常类型和错误文字符合预期；离开时自动执行对应清理。
        with patch.object(t,'tick'),self.assertRaisesRegex(RuntimeError,'acknowledgement'):
            # 学习：执行一次电缸绝对位置定位并确认到位。
            t.cylinder(0)
        # 学习：核对结果为真。
        self.assertTrue(t.report['cylinder_motion_attempted'])
        # 学习：核对该模拟函数恰好调用一次。
        d.write_ack.assert_called_once()

    # 学习：定义test_recovery_restores_original_snapshot_not_new_capture函数：验证恢复使用原姿态快照，而不是重新采集覆盖；def本身只是定义，调用时才执行函数体。
    def test_recovery_restores_original_snapshot_not_new_capture(self):
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到r（机器人状态接收对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到c（机器人控制对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
        r=Mock();c=Mock();d=Mock()
        # 学习：把“app.POSES['A'][:]”保存到start（该段开始时的状态或位置）；单个=表示赋值。；在原值基础上更新start[3]：start[3] += 0.02。
        start=app.POSES['A'][:];start[3]+=.02
        # 学习：把“计算start[:3] + app.FIXED_ORIENTATION_RAD[:]”保存到target（本段目标）；单个=表示赋值。
        target=start[:3]+app.FIXED_ORIENTATION_RAD[:]
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到r.getActualTCPPose.side_effect；单个=表示赋值。
        r.getActualTCPPose.side_effect=[start,target]
        # 学习：把“True”保存到c.isPoseWithinSafetyLimits.return_value；单个=表示赋值。；把“True”保存到c.moveL.return_value；单个=表示赋值。
        c.isPoseWithinSafetyLimits.return_value=True;c.moveL.return_value=True
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到c.getAsyncOperationProgress.side_effect；单个=表示赋值。
        c.getAsyncOperationProgress.side_effect=[-1,-2]
        # 学习：把“调用app.Trial，按本行参数取得结果”保存到t（模拟时刻或当前协议字段类型）；单个=表示赋值。；把“True”保存到t.recovery_mode；单个=表示赋值。
        t=app.Trial(r,c,d,{'segments':[]});t.recovery_mode=True
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
        with patch.object(t,'tick'),patch.object(t,'dwell'):
            # 学习：保持XYZ，恢复原固定姿态。
            t.restore_fixed_orientation()
        # 学习：核对模拟函数恰好调用一次且参数完全一致。
        c.moveL.assert_called_once_with(target,.001,.001,True)
        # 学习：核对结果为假。
        self.assertFalse(t.recovery_mode)

    # 学习：定义test_recovery_restarts_at_A_and_B_instead_of_resuming_old_step函数：验证恢复后回A再去B，从头开始而非续跑旧段；def本身只是定义，调用时才执行函数体。
    def test_recovery_restarts_at_A_and_B_instead_of_resuming_old_step(self):
        # 学习：把“app.POSES['C'][:]”保存到pose（六维TCP位姿，前三项m、后三项rad）；单个=表示赋值。；在原值基础上更新pose[3]：pose[3] += 0.02。
        pose=app.POSES['C'][:];pose[3]+=.02
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到robot（机械臂状态字典）；单个=表示赋值。
        robot=dict(self.robot,pose=pose[:])
        # 学习：把“创建一份键值数据字典，或复制已有字典”保存到cyl；单个=表示赋值。
        cyl=dict(self.cyl,position=4800,control=0x28,status=50272)
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到r（机器人状态接收对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到c（机器人控制对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
        r=Mock();c=Mock();d=Mock()
        # 学习：把“lambda: pose[:]”保存到r.getActualTCPPose.side_effect；单个=表示赋值。
        r.getActualTCPPose.side_effect=lambda:pose[:]
        # 学习：把“计算[0.0] * 6”保存到r.getActualTCPSpeed.return_value；单个=表示赋值。；把“2”保存到r.getRuntimeState.return_value；单个=表示赋值。
        r.getActualTCPSpeed.return_value=[0.]*6;r.getRuntimeState.return_value=2
        # 学习：把“True”保存到c.setWatchdog.return_value；单个=表示赋值。；把“lambda: dict(cyl)”保存到d.snapshot.side_effect；单个=表示赋值。；把“建立按顺序排列的列表，方括号内是各个元素”保存到d.log；单个=表示赋值。
        c.setWatchdog.return_value=True;d.snapshot.side_effect=lambda:dict(cyl);d.log=[]
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到modules（测试时替换真实RTDE接口的模块字典）；单个=表示赋值。
        modules={'rtde_receive':Mock(RTDEReceiveInterface=Mock(return_value=r)),
                 # 学习：字典条目：'rtde_control'对应创建可记录调用情况的模拟对象，不连接真实设备。
                 'rtde_control':Mock(RTDEControlInterface=Mock(return_value=c))}
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到state（电缸寄存器状态字典）；单个=表示赋值。
        state={'robot':robot,'dashboard':self.dash,'cylinder':dict(cyl)}
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到report（运行或测试报告字典）；单个=表示赋值。
        report={'segments':[]}
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；…；离开时自动执行对应清理。
        with patch.dict('sys.modules',modules),patch('mcea_direction_test.Device',return_value=d),patch.object(app,'Device',return_value=d,create=True),patch.object(app,'inspect',return_value=state),patch.object(app,'read_robot',return_value=robot),patch.object(app,'dashboard',return_value=self.dash),patch.object(app,'save'),patch('builtins.input',side_effect=AssertionError('Unexpected per-segment input')),patch.object(app,'Trial') as tc:
            # 学习：把“tc.return_value”保存到trial（管理当前一轮动作与反馈的Trial对象）；单个=表示赋值。
            trial=tc.return_value
            # 学习：定义retract函数：测试用的回缩替身：只修改模拟位置与控制字，不发送真实指令；def本身只是定义，调用时才执行函数体。
            def retract(mm):
                # 学习：把“把位置换算结果取整，供整数寄存器使用”保存到cyl['position']（电缸实际位置，单位0.01 mm）；单个=表示赋值。
                cyl['position']=round(mm*100)
                # 学习：把“33”保存到cyl['control']（电缸控制字）；单个=表示赋值。
                cyl['control']=0x21
            # 学习：定义restore函数：测试用的姿态恢复替身：只修改模拟姿态；def本身只是定义，调用时才执行函数体。；把“app.FIXED_ORIENTATION_RAD[:]”保存到pose[3:]；单个=表示赋值。
            def restore():pose[3:]=app.FIXED_ORIENTATION_RAD[:]
            # 学习：定义move函数：测试动作替身：只把模拟TCP改为给定目标，不操作设备；def本身只是定义，调用时才执行函数体。；把“app.POSES[name][:]”保存到pose[:]；单个=表示赋值。
            def move(name):pose[:]=app.POSES[name][:]
            # 学习：把“retract”保存到trial.cylinder.side_effect；单个=表示赋值。
            trial.cylinder.side_effect=retract
            # 学习：把“restore”保存到trial.restore_fixed_orientation.side_effect；单个=表示赋值。
            trial.restore_fixed_orientation.side_effect=restore
            # 学习：把“move”保存到trial.robot.side_effect；单个=表示赋值。
            trial.robot.side_effect=move
            # 学习：连接控制接口，连续执行恢复或整套任务，并在异常时停止和释放资源。
            app.execute('unused','unused',report,recovery=True)
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual([call.args[0] for call in trial.robot.call_args_list[:2]],['A','B'])
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(trial.cylinder.call_args_list[0].args,(0,))
            # 学习：核对结果为真。
            self.assertTrue(report['recovered_to_start'])
            # 学习：核对结果为真。
            self.assertTrue(report['restart_cycle_from_beginning'])

    # 学习：定义test_changed_tcp_rejects函数：验证TCP偏置变化后会阻止使用旧实测坐标；def本身只是定义，调用时才执行函数体。
    def test_changed_tcp_rejects(self):
        # 学习：把“0.01”保存到self.robot['tcp_offset'][2]；单个=表示赋值。
        self.robot['tcp_offset'][2]=.01
        # 学习：with限定资源或临时替换的有效范围：核对抛出的异常类型和错误文字符合预期；离开时自动执行对应清理。
        with self.assertRaisesRegex(RuntimeError,'TCP differs'):
            # 学习：核对两台设备是否满足正常或恢复启动条件，有问题则报错。
            app.require_initial(self.robot,self.dash,self.cyl)

    # 学习：定义test_target_limit_rejection_does_not_move函数：验证控制器拒绝目标后不发送运动；def本身只是定义，调用时才执行函数体。
    def test_target_limit_rejection_does_not_move(self):
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到c（机器人控制对象）；单个=表示赋值。；把“False”保存到c.isPoseWithinSafetyLimits.return_value；单个=表示赋值。
        c=Mock();c.isPoseWithinSafetyLimits.return_value=False
        # 学习：with限定资源或临时替换的有效范围：核对抛出的异常类型和错误文字符合预期；离开时自动执行对应清理。
        with self.assertRaisesRegex(RuntimeError,'Controller rejected'):
            # 学习：检查单个六维目标是否有效，并请控制器核对可达性与安全限位。
            app.require_target(c,app.POSES['A'])
        # 学习：核对该模拟函数没有被调用。
        c.moveL.assert_not_called()

    # 学习：定义test_check_mode_does_not_enter_execution函数：验证只读模式不会进入实际动作流程；def本身只是定义，调用时才执行函数体。
    def test_check_mode_does_not_enter_execution(self):
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
        with patch.object(app,'inspect',return_value={'trial_start_ready':True}),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('sys.argv',['ag_run.py','--check']):
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(app.main(),0)
            # 学习：核对该模拟函数没有被调用。
            execute.assert_not_called()

    # 学习：定义test_recover_prompt_selects_continuous_restart函数：验证一次RECOVER确认后选择连续恢复；def本身只是定义，调用时才执行函数体。
    def test_recover_prompt_selects_continuous_restart(self):
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到status（检查结果或状态字）；单个=表示赋值。
        status={'trial_start_ready':False,'manual_recovery_available':True,'blocking_reason':'paused cylinder'}
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
        with patch.object(app,'inspect',return_value=status),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('builtins.input',return_value='RECOVER') as prompt,patch('sys.argv',['ag_run.py','--run']):
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(app.main(),0)
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(prompt.call_count,1)
            # 学习：核对结果为真。
            self.assertTrue(execute.call_args.kwargs['recovery'])
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(execute.call_args.kwargs,{'recovery':True})
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(execute.call_args.args[2]['run_mode'],'continuous_single_cycle')

    # 学习：定义test_nonrecoverable_fault_never_prompts_or_uploads函数：验证不可恢复故障不进入控制上传和输入确认；def本身只是定义，调用时才执行函数体。
    def test_nonrecoverable_fault_never_prompts_or_uploads(self):
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到status（检查结果或状态字）；单个=表示赋值。
        status={'trial_start_ready':False,'manual_recovery_available':False,'blocking_reason':'emergency stop active'}
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
        with patch.object(app,'inspect',return_value=status),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('builtins.input') as prompt,patch('sys.argv',['ag_run.py','--run']):
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(app.main(),1)
            # 学习：核对该模拟函数没有被调用。；核对该模拟函数没有被调用。
            prompt.assert_not_called();execute.assert_not_called()

    # 学习：定义test_robot_command_uses_saved_orientation_and_30_mm_s函数：验证运动命令的保存姿态和30 mm/s速度；def本身只是定义，调用时才执行函数体。
    def test_robot_command_uses_saved_orientation_and_30_mm_s(self):
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到r（机器人状态接收对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到c（机器人控制对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。；把“app.POSES['A'][:]”保存到target（本段目标）；单个=表示赋值。
        r=Mock();c=Mock();d=Mock();target=app.POSES['A'][:]
        # 学习：把“target[:]”保存到pose（六维TCP位姿，前三项m、后三项rad）；单个=表示赋值。；在原值基础上更新pose[0]：pose[0] += 0.01。
        pose=target[:];pose[0]+=.01
        # 学习：把“lambda: pose[:]”保存到r.getActualTCPPose.side_effect；单个=表示赋值。
        r.getActualTCPPose.side_effect=lambda:pose[:]
        # 学习：把“计算[0.0] * 6”保存到r.getActualTCPSpeed.return_value；单个=表示赋值。
        r.getActualTCPSpeed.return_value=[0.]*6
        # 学习：把“True”保存到c.isPoseWithinSafetyLimits.return_value；单个=表示赋值。
        c.isPoseWithinSafetyLimits.return_value=True
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到c.getAsyncOperationProgress.side_effect；单个=表示赋值。
        c.getAsyncOperationProgress.side_effect=[-1,-2]
        # 学习：定义move函数：测试动作替身：只把模拟TCP改为给定目标，不操作设备；def本身只是定义，调用时才执行函数体。
        def move(p,*args):
            # 学习：把“p”保存到pose[:]；单个=表示赋值。
            pose[:]=p
            # 学习：return把True交回调用者，并立即结束当前函数。
            return True
        # 学习：把“move”保存到c.moveL.side_effect；单个=表示赋值。
        c.moveL.side_effect=move
        # 学习：把“调用app.Trial，按本行参数取得结果”保存到t（模拟时刻或当前协议字段类型）；单个=表示赋值。
        t=app.Trial(r,c,d,{'segments':[]})
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
        with patch.object(t,'tick'),patch.object(t,'dwell'):
            # 学习：执行一次机器人到点动作并确认到位。
            t.robot('A')
        # 学习：核对模拟函数恰好调用一次且参数完全一致。
        c.moveL.assert_called_once_with(target,.03,.001,True)
        # 学习：核对结果为真。
        self.assertTrue(t.report['segments'][0]['complete'])

    # 学习：定义test_feedback_retains_orientation_and_only_records_speed函数：验证姿态监控仍生效，速度只记录不触发额外阈值；def本身只是定义，调用时才执行函数体。
    def test_feedback_retains_orientation_and_only_records_speed(self):
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到r（机器人状态接收对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到c（机器人控制对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
        r=Mock();c=Mock();d=Mock()
        # 学习：把“True”保存到r.isConnected.return_value、c.isConnected.return_value；单个=表示赋值。
        r.isConnected.return_value=c.isConnected.return_value=True
        # 学习：把“1”保存到r.getTimestamp.return_value；单个=表示赋值。；把“7”保存到r.getRobotMode.return_value；单个=表示赋值。
        r.getTimestamp.return_value=1;r.getRobotMode.return_value=7
        # 学习：把“1”保存到r.getSafetyMode.return_value；单个=表示赋值。；把“2”保存到r.getRuntimeState.return_value；单个=表示赋值。；把“1”保存到r.getSpeedScaling.return_value；单个=表示赋值。
        r.getSafetyMode.return_value=1;r.getRuntimeState.return_value=2;r.getSpeedScaling.return_value=1
        # 学习：把“app.POSES['A'][:]”保存到r.getActualTCPPose.return_value；单个=表示赋值。
        r.getActualTCPPose.return_value=app.POSES['A'][:]
        # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到r.getActualTCPSpeed.return_value；单个=表示赋值。
        r.getActualTCPSpeed.return_value=[.04,0,0,.03,0,0]
        # 学习：把“计算[0.0] * 6”保存到c.getTCPOffset.return_value；单个=表示赋值。；把“True”保存到c.kickWatchdog.return_value；单个=表示赋值。
        c.getTCPOffset.return_value=[0.]*6;c.kickWatchdog.return_value=True
        # 学习：把“self.cyl”保存到d.snapshot.return_value；单个=表示赋值。
        d.snapshot.return_value=self.cyl
        # 学习：把“调用app.Trial，按本行参数取得结果”保存到t（模拟时刻或当前协议字段类型）；单个=表示赋值。；检查一次实时状态并更新看门狗。
        t=app.Trial(r,c,d,{'segments':[]});t.tick()
        # 学习：核对实际值与期望值完全相等。
        self.assertEqual(t.report['max_observed_linear_speed_m_s'],.04)
        # 学习：把“计算app.POSES['A'][:3] + [0.0, 0.0, 0.0]”保存到r.getActualTCPPose.return_value；单个=表示赋值。
        r.getActualTCPPose.return_value=app.POSES['A'][:3]+[0.,0.,0.]
        # 学习：with限定资源或临时替换的有效范围：核对抛出的异常类型和错误文字符合预期；离开时自动执行对应清理。
        with self.assertRaisesRegex(RuntimeError,'fixed tool orientation'):
            # 学习：检查一次实时状态并更新看门狗。
            t.tick()

    # 学习：定义test_execution_has_no_rotation_initialization_or_sampling函数：验证连续路线顺序，且无常规姿态归零或全路径预采样；def本身只是定义，调用时才执行函数体。
    def test_execution_has_no_rotation_initialization_or_sampling(self):
        # 学习：核对结果为假。
        self.assertFalse(hasattr(app.Trial,'initialize_orientation'))
        # 学习：核对结果为假。
        self.assertFalse(hasattr(app,'preflight_orientation'))
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到r（机器人状态接收对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到c（机器人控制对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
        r=Mock();c=Mock();d=Mock()
        # 学习：把“app.POSES['A'][:]”保存到r.getActualTCPPose.return_value；单个=表示赋值。
        r.getActualTCPPose.return_value=app.POSES['A'][:]
        # 学习：把“计算[0.0] * 6”保存到r.getActualTCPSpeed.return_value；单个=表示赋值。；把“2”保存到r.getRuntimeState.return_value；单个=表示赋值。
        r.getActualTCPSpeed.return_value=[0.]*6;r.getRuntimeState.return_value=2
        # 学习：把“True”保存到c.setWatchdog.return_value；单个=表示赋值。；把“self.cyl”保存到d.snapshot.return_value；单个=表示赋值。；把“建立按顺序排列的列表，方括号内是各个元素”保存到d.log；单个=表示赋值。
        c.setWatchdog.return_value=True;d.snapshot.return_value=self.cyl;d.log=[]
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到status（检查结果或状态字）；单个=表示赋值。
        status={'robot':self.robot,'dashboard':self.dash,'cylinder':self.cyl}
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到modules（测试时替换真实RTDE接口的模块字典）；单个=表示赋值。
        modules={'rtde_receive':Mock(RTDEReceiveInterface=Mock(return_value=r)),
                 # 学习：字典条目：'rtde_control'对应创建可记录调用情况的模拟对象，不连接真实设备。
                 'rtde_control':Mock(RTDEControlInterface=Mock(return_value=c))}
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到report（运行或测试报告字典）；单个=表示赋值。
        report={'segments':[]}
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；…；离开时自动执行对应清理。
        with patch.dict('sys.modules',modules),patch('mcea_direction_test.Device',return_value=d),patch.object(app,'Device',return_value=d,create=True),patch.object(app,'inspect',return_value=status),patch.object(app,'read_robot',return_value=self.robot),patch.object(app,'dashboard',return_value=self.dash),patch.object(app,'save'),patch('builtins.input',side_effect=AssertionError('Unexpected per-segment input')),patch.object(app,'Trial') as trial:
            # 学习：连接控制接口，连续执行恢复或整套任务，并在异常时停止和释放资源。
            app.execute('unused','unused',report)
            # 学习：把“逐项遍历并计算，生成所需数据：[(call[0], call.args[0]) for call in trial.return_value.method_calls if call[0] in ('robot', 'c…”保存到actions；单个=表示赋值。
            actions=[(call[0],call.args[0]) for call in trial.return_value.method_calls if call[0] in ('robot','cylinder')]
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(actions,[('robot','A'),('robot','B'),('cylinder',48),('robot','C'),('cylinder',13),('robot','A'),('robot','D'),('robot','E'),('cylinder',48),('robot','F'),('robot','G'),('cylinder',0),('robot','A')])
            # 学习：核对该模拟函数没有被调用。
            c.getInverseKinematics.assert_not_called()
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(report['fixed_orientation_rad'],app.FIXED_ORIENTATION_RAD)
            # 学习：核对结果为真。
            self.assertTrue(report['success'])

    # 学习：定义test_run_requires_only_one_start_confirmation函数：验证整轮实际任务只要求一次RUN确认；def本身只是定义，调用时才执行函数体。
    def test_run_requires_only_one_start_confirmation(self):
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；离开时自动执行对应清理。
        with patch.object(app,'inspect',return_value={'trial_start_ready':True}),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('builtins.input',return_value='RUN') as prompt,patch('sys.argv',['ag_run.py','--run']):
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(app.main(),0)
            # 学习：核对该模拟函数恰好调用一次。
            prompt.assert_called_once()
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual(execute.call_args.kwargs,{'recovery':False})
            # 学习：核对结果为假。
            self.assertFalse(execute.call_args.args[2]['manual_steps'])

    # 学习：定义test_continuous_route_stops_after_action_failure函数：验证某段失败后不继续下一段，并请求软件停止；def本身只是定义，调用时才执行函数体。
    def test_continuous_route_stops_after_action_failure(self):
        # 学习：把“创建可记录调用情况的模拟对象，不连接真实设备”保存到r（机器人状态接收对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到c（机器人控制对象）；单个=表示赋值。；把“创建可记录调用情况的模拟对象，不连接真实设备”保存到d（当前函数使用的数据块或电缸对象）；单个=表示赋值。
        r=Mock();c=Mock();d=Mock()
        # 学习：把“app.POSES['A'][:]”保存到r.getActualTCPPose.return_value；单个=表示赋值。
        r.getActualTCPPose.return_value=app.POSES['A'][:]
        # 学习：把“计算[0.0] * 6”保存到r.getActualTCPSpeed.return_value；单个=表示赋值。；把“2”保存到r.getRuntimeState.return_value；单个=表示赋值。
        r.getActualTCPSpeed.return_value=[0.]*6;r.getRuntimeState.return_value=2
        # 学习：把“True”保存到c.setWatchdog.return_value；单个=表示赋值。；把“self.cyl”保存到d.snapshot.return_value；单个=表示赋值。；把“建立按顺序排列的列表，方括号内是各个元素”保存到d.log；单个=表示赋值。
        c.setWatchdog.return_value=True;d.snapshot.return_value=self.cyl;d.log=[]
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到status（检查结果或状态字）；单个=表示赋值。
        status={'robot':self.robot,'dashboard':self.dash,'cylinder':self.cyl}
        # 学习：把“建立字典，用字段名保存各项状态或参数”保存到modules（测试时替换真实RTDE接口的模块字典）；单个=表示赋值。
        modules={'rtde_receive':Mock(RTDEReceiveInterface=Mock(return_value=r)),
                 # 学习：字典条目：'rtde_control'对应创建可记录调用情况的模拟对象，不连接真实设备。
                 'rtde_control':Mock(RTDEControlInterface=Mock(return_value=c))}
        # 学习：with限定资源或临时替换的有效范围：临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；临时替换真实函数或模块，离开with后恢复，供离线测试使用；…；离开时自动执行对应清理。
        with patch.dict('sys.modules',modules),patch('mcea_direction_test.Device',return_value=d),patch.object(app,'Device',return_value=d,create=True),patch.object(app,'inspect',return_value=status),patch.object(app,'read_robot',return_value=self.robot),patch.object(app,'dashboard',return_value=self.dash),patch.object(app,'save'),patch('builtins.input',side_effect=AssertionError('Unexpected per-segment input')),patch.object(app,'Trial') as trial:
            # 学习：把“建立按顺序排列的列表，方括号内是各个元素”保存到trial.return_value.robot.side_effect；单个=表示赋值。
            trial.return_value.robot.side_effect=[None,RuntimeError('operator stop')]
            # 学习：with限定资源或临时替换的有效范围：核对抛出的异常类型和错误文字符合预期；离开时自动执行对应清理。
            with self.assertRaisesRegex(RuntimeError,'operator stop'):
                # 学习：连接控制接口，连续执行恢复或整套任务，并在异常时停止和释放资源。
                app.execute('unused','unused',{'segments':[]})
            # 学习：核对实际值与期望值完全相等。
            self.assertEqual([call.args[0] for call in trial.return_value.robot.call_args_list],['A','B'])
            # 学习：核对该模拟函数没有被调用。
            trial.return_value.cylinder.assert_not_called()
            # 学习：核对模拟函数恰好调用一次且参数完全一致。
            c.stopL.assert_called_once_with(.05)
            # 学习：核对该模拟函数恰好调用一次。
            c.stopScript.assert_called_once()



# 学习：只有直接启动该文件时才执行入口；被其他文件import时跳过。
if __name__=='__main__':
    # 学习：调用unittest.main，按本行参数取得结果。
    unittest.main()
