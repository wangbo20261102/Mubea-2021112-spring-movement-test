"""Single-cycle UR CB3 / MCEA trial. Default is READ ONLY.

--preview: offline plan; --check: receive-only robot and FC03 cylinder query.
--run: explicitly confirmed hardware trial; NEVER invoked by preparation.
No homing search, enable, brake release, alarm reset, TCP or slider changes.
"""
import argparse
import json
import math
import re
import socket
import struct
import sys
import time
import traceback
from datetime import datetime, timezone, timedelta
from pathlib import Path
from simulate_ag import POINTS, build_plan, motion_profile
ROOT = Path(__file__).resolve().parent
FIXED_ORIENTATION_RAD = [1.4747881408617243, -0.6236196873562871, 0.6049084354198352]
ORIENTATION_CAPTURED_AT = '2026-11-12T13:10:55.171401+08:00'
POSES = {k: [x / 1000 for x in v] + FIXED_ORIENTATION_RAD[:] for k, v in POINTS.items()}
POSITION_TOL = 0.0005
ANGLE_TOL = 0.003

def now():
    return datetime.now(timezone(timedelta(hours=8))).isoformat()

def exact(s, n):
    data = b''
    while len(data) < n:
        chunk = s.recv(n - len(data))
        if not chunk:
            raise EOFError('Controller closed state connection')
        data += chunk
    return data

def read_robot(ip):
    with socket.create_connection((ip, 30012), 3) as s:
        s.settimeout(3)
        for _ in range(30):
            size, kind = struct.unpack('>IB', exact(s, 5))
            if not 5 <= size <= 100000:
                raise RuntimeError('Invalid controller packet size')
            payload = exact(s, size - 5)
            if kind != 16:
                continue
            result, offset = ({}, 0)
            while offset < len(payload):
                n, t = struct.unpack_from('>IB', payload, offset)
                if n < 5 or offset + n > len(payload):
                    raise RuntimeError('Invalid state subpacket')
                d = payload[offset + 5:offset + n]
                if t == 4 and len(d) >= 96:
                    values = list(struct.unpack_from('>12d', d))
                    if not all((math.isfinite(x) for x in values)):
                        raise RuntimeError('Nonfinite pose')
                    result.update(pose=values[:6], tcp_offset=values[6:])
                if t == 0 and len(d) >= 17:
                    keys = ('connected', 'enabled', 'power_on', 'emergency_stop', 'protective_stop', 'program_running', 'program_paused')
                    result.update(dict(zip(keys, [bool(v) for v in d[8:15]])))
                    result['mode'] = d[15]
                offset += n
            if 'pose' in result and 'mode' in result:
                return result
    raise RuntimeError('No pose/mode packet received')

def dashboard(ip):
    with socket.create_connection((ip, 29999), 3) as s:
        s.settimeout(3)
        f = s.makefile('rwb', buffering=0)
        result = {'greeting': f.readline(4096).decode(errors='replace').strip()}
        for command in ('PolyscopeVersion', 'robotmode', 'safetymode', 'running'):
            f.write((command + '\n').encode())
            result[command] = f.readline(4096).decode(errors='replace').strip()
        return result

def orientation_error(a, b):
    """Physical angular error, independent of equivalent axis-angle branches."""

    def quat(rv):
        if len(rv) != 3 or not all((math.isfinite(v) for v in rv)):
            return None
        angle = math.sqrt(sum((v * v for v in rv)))
        if angle < 1e-12:
            return [1.0, 0.0, 0.0, 0.0]
        return [math.cos(angle / 2)] + [v * math.sin(angle / 2) / angle for v in rv]
    qa, qb = (quat(a), quat(b))
    if qa is None or qb is None:
        return math.inf
    dot = min(1.0, max(0.0, abs(sum((x * y for x, y in zip(qa, qb))))))
    return 2 * math.acos(dot)

def matches_fixed_orientation(pose):
    return len(pose) == 6 and orientation_error(pose[3:], FIXED_ORIENTATION_RAD) <= ANGLE_TOL

def require_target(control, pose):
    """One controller endpoint check; no path sampling or multi-point IK loop."""
    if len(pose) != 6 or not all((math.isfinite(v) for v in pose)):
        raise RuntimeError('Invalid target pose')
    if not control.isPoseWithinSafetyLimits(pose):
        raise RuntimeError('Controller rejected target reachability or safety limits; no motion sent')

def cylinder_state_errors(state, recovery=False):
    from mcea_direction_test import validate, bounds
    check = dict(state)
    if recovery and check['control'] == 40:
        check['control'] = 32
    errors = validate(check, 1)
    if recovery and (not 0 <= state['position'] <= bounds(state)[1]):
        errors.append('Cylinder recovery position outside calibrated trial range')
    return errors

def require_initial(robot, dash, cylinder, recovery=False):
    from mcea_direction_test import validate, bounds
    problems = cylinder_state_errors(cylinder, recovery)
    if not all((robot[k] for k in ('connected', 'enabled', 'power_on'))) or robot['mode'] != 7:
        problems.append('Robot is not powered and ready in RUNNING mode')
    if robot['emergency_stop'] or robot['protective_stop']:
        problems.append('Robot stop flag is active')
    if robot['program_running'] or robot['program_paused']:
        problems.append('Existing robot program is running or paused; do not overwrite it')
    if dash.get('safetymode') != 'Safetymode: NORMAL':
        problems.append('Robot safety mode must be NORMAL')
    if dash.get('running') != 'Program running: false':
        problems.append('Dashboard does not confirm idle program state')
    match = re.search('URSoftware (\\d+)\\.(\\d+)\\.(\\d+)', dash.get('PolyscopeVersion', ''))
    if not match or not (3, 14, 3) <= tuple(map(int, match.groups())) < (4, 0, 0):
        problems.append('This package targets CB3 PolyScope 3.14.3 through 3.x only')
    if not recovery and (not matches_fixed_orientation(robot['pose'])):
        problems.append('Current orientation differs from the captured fixed orientation; no automatic rotation or recapture')
    if math.dist(robot['tcp_offset'], [0.0] * 6) > 1e-06:
        problems.append('Active TCP differs from the zero-offset TCP recorded for this task; revalidate coordinates')
    if not recovery and abs(cylinder['position']) > 5:
        problems.append('Cylinder must start already homed at 0 mm (+/-0.05 mm)')
    if bounds(cylinder)[1] < 4800:
        problems.append('Cylinder configured stroke/offset leaves insufficient margin for 48 mm')
    if problems:
        raise RuntimeError('; '.join(problems))

def inspect(ip, port):
    from mcea_direction_test import Device
    from serial.tools.list_ports import comports
    ports = [{'port': p.device, 'vid': p.vid, 'pid': p.pid} for p in comports()]
    device = Device(port, 115200, 1)
    try:
        cylinder = device.snapshot()
    finally:
        device.close()
    robot = read_robot(ip)
    dash = dashboard(ip)
    result = dict(time=now(), ip=ip, port=port, robot=robot, dashboard=dash, cylinder=cylinder, serial_ports=ports, motion_commands_sent=False)
    try:
        require_initial(robot, dash, cylinder)
        result.update(trial_start_ready=True, blocking_reason=None, fixed_orientation_rad=FIXED_ORIENTATION_RAD[:], orientation_captured_at=ORIENTATION_CAPTURED_AT, orientation_initialization_required=False)
    except RuntimeError as error:
        result.update(trial_start_ready=False, blocking_reason=str(error))
    try:
        require_initial(robot, dash, cylinder, recovery=True)
        result['manual_recovery_available'] = True
        result['manual_recovery_blocking_reason'] = None
    except RuntimeError as error:
        result['manual_recovery_available'] = False
        result['manual_recovery_blocking_reason'] = str(error)
    return result

def set_stage(report, label):
    report['stage'] = label
    report['stage_started'] = now()
    save(report)
    print('\n[阶段] ' + label, flush=True)

class Trial:

    def __init__(self, receive, control, device, report):
        self.r, self.c, self.d, self.report = (receive, control, device, report)
        self.last_timestamp = None
        self.last_fresh = time.monotonic()
        self.last_cylinder_poll = 0
        self.cylinder_expected = 0
        self.cylinder_moving = False
        self.expected_hold = None
        self.recovery_mode = False
        self.recovery_orientation = None
        self.restoring_orientation = False

    def tick(self):
        if not self.r.isConnected() or not self.c.isConnected():
            raise RuntimeError('RTDE disconnected')
        stamp = self.r.getTimestamp()
        if stamp != self.last_timestamp:
            self.last_timestamp = stamp
            self.last_fresh = time.monotonic()
        if time.monotonic() - self.last_fresh > 1:
            raise RuntimeError('Stale RTDE state')
        if self.r.getRobotMode() != 7 or self.r.getSafetyMode() != 1 or self.r.getRuntimeState() != 2:
            raise RuntimeError('Robot no longer ready / control program no longer running')
        if self.r.getSpeedScaling() <= 0:
            raise RuntimeError('Robot execution speed scaling is zero; do not auto-resume')
        pose = list(self.r.getActualTCPPose())
        if not all((math.isfinite(v) for v in pose)):
            raise RuntimeError('Nonfinite TCP pose')
        required_orientation = self.recovery_orientation if self.recovery_mode else FIXED_ORIENTATION_RAD
        if not self.restoring_orientation and orientation_error(pose[3:], required_orientation) > ANGLE_TOL:
            raise RuntimeError('Captured fixed tool orientation tolerance exceeded')
        if math.dist(self.c.getTCPOffset(), [0.0] * 6) > 1e-06:
            raise RuntimeError('Active TCP changed')
        if self.expected_hold is not None and math.dist(pose[:3], self.expected_hold[:3]) > POSITION_TOL:
            raise RuntimeError('Robot moved during cylinder motion or dwell')
        speed = list(self.r.getActualTCPSpeed())
        if len(speed) != 6 or not all((math.isfinite(v) for v in speed)):
            raise RuntimeError('Nonfinite TCP speed feedback')
        linear = math.sqrt(sum((v * v for v in speed[:3])))
        angular = math.sqrt(sum((v * v for v in speed[3:])))
        self.report['last_robot_feedback'] = dict(time=now(), pose=pose, tcp_speed=speed)
        self.report['max_observed_linear_speed_m_s'] = max(linear, self.report.get('max_observed_linear_speed_m_s', 0))
        self.report['max_observed_angular_speed_rad_s'] = max(angular, self.report.get('max_observed_angular_speed_rad_s', 0))
        if not self.c.kickWatchdog():
            raise RuntimeError('Watchdog update failed')
        if not self.cylinder_moving and time.monotonic() - self.last_cylinder_poll > 0.2:
            from mcea_direction_test import validate
            state = self.d.snapshot()
            errors = cylinder_state_errors(state, self.recovery_mode)
            if errors or abs(state['position'] - self.cylinder_expected) > 5:
                raise RuntimeError('Stationary cylinder state changed: ' + str(errors))
            self.last_cylinder_poll = time.monotonic()

    def dwell(self):
        until = time.monotonic() + 1
        while time.monotonic() < until:
            self.tick()
            time.sleep(0.03)

    def restore_fixed_orientation(self):
        if not self.recovery_mode:
            raise RuntimeError('Orientation restoration requires recovery mode')
        start = list(self.r.getActualTCPPose())
        target = start[:3] + FIXED_ORIENTATION_RAD[:]
        event = dict(action='restore_captured_orientation', from_pose=start, target_pose=target, start=now(), complete=False)
        self.report['segments'].append(event)
        if not matches_fixed_orientation(start):
            require_target(self.c, target)
            self.expected_hold = start[:]
            self.restoring_orientation = True
            old = self.c.getAsyncOperationProgress()
            self.report.update(robot_motion_attempted=True, motion_commands_sent=True)
            if not self.c.moveL(target, 0.001, 0.001, True):
                raise RuntimeError('Recovery orientation move rejected')
            until = time.monotonic() + max(60, orientation_error(start[3:], FIXED_ORIENTATION_RAD) / 0.002 + 60)
            while time.monotonic() < until:
                self.tick()
                value = self.c.getAsyncOperationProgress()
                if value < 0 and value != old:
                    break
                time.sleep(0.03)
            else:
                raise RuntimeError('Recovery orientation timeout')
        actual = list(self.r.getActualTCPPose())
        if math.dist(actual[:3], start[:3]) > POSITION_TOL or not matches_fixed_orientation(actual):
            raise RuntimeError('Recovery did not reach captured orientation at the held XYZ')
        self.restoring_orientation = False
        self.recovery_mode = False
        self.recovery_orientation = None
        self.expected_hold = target
        self.dwell()
        event.update(complete=True, actual_pose=actual, end=now())

    def robot(self, name):
        self.expected_hold = None
        self.tick()
        target = POSES[name]
        initial = list(self.r.getActualTCPPose())
        distance = math.dist(initial[:3], target[:3])
        event = dict(action='robot', target=name, pose=target, start=now(), complete=False)
        self.report['segments'].append(event)
        if distance > POSITION_TOL:
            require_target(self.c, target)
            self.report['robot_motion_attempted'] = True
            self.report['motion_commands_sent'] = True
            old = self.c.getAsyncOperationProgress()
            if not self.c.moveL(target, 0.03, 0.001, True):
                raise RuntimeError('moveL rejected: ' + name)
            deadline = time.monotonic() + 3 * motion_profile(distance * 1000, 30)[2] + 20
            complete = False
            while time.monotonic() < deadline:
                self.tick()
                progress = self.c.getAsyncOperationProgress()
                actual = list(self.r.getActualTCPPose())
                v = [target[i] - initial[i] for i in range(3)]
                u = sum(((actual[i] - initial[i]) * v[i] for i in range(3))) / (distance * distance)
                nearest = [initial[i] + min(1, max(0, u)) * v[i] for i in range(3)]
                if math.dist(actual[:3], nearest) > 0.002:
                    raise RuntimeError('Robot deviated from expected straight segment')
                if progress < 0 and progress != old:
                    complete = True
                    break
                time.sleep(0.03)
            if not complete:
                raise RuntimeError('Robot move timeout')
        actual = list(self.r.getActualTCPPose())
        if math.dist(actual[:3], target[:3]) > POSITION_TOL or not matches_fixed_orientation(actual):
            raise RuntimeError('Robot target verification failed')
        if math.sqrt(sum((v * v for v in self.r.getActualTCPSpeed()[:3]))) > 0.0002:
            raise RuntimeError('Robot has not stopped at target')
        self.expected_hold = target
        self.dwell()
        event.update(complete=True, actual_pose=actual, end=now())

    def cylinder(self, target_mm):
        from mcea_direction_test import validate, bounds
        target = round(target_mm * 100)
        self.tick()
        before = self.d.snapshot()
        errors = cylinder_state_errors(before, self.recovery_mode)
        if errors or abs(before['position'] - self.cylinder_expected) > 5:
            raise RuntimeError('Cylinder start invalid: ' + str(errors))
        if target not in (0, 1300, 4800) or target > bounds(before)[1]:
            raise RuntimeError('Cylinder target outside approved trial positions')
        event = dict(action='cylinder', target_mm=target_mm, start=now(), complete=False)
        self.report['segments'].append(event)
        start = before['position']
        if abs(start - target) > 5 or (self.recovery_mode and before['control'] == 40):
            self.cylinder_moving = True
            self.report['cylinder_motion_attempted'] = True
            self.report['motion_commands_sent'] = True
            prefix = struct.pack('>BBHHB', 1, 16, 5632, 6, 12)
            for word in (32, 33):
                self.d.write_ack(prefix + struct.pack('>6H', target, 0, 100, 1, 0, word))
            until = time.monotonic() + abs(target - start) / 100 + 20
            settled = 0
            while time.monotonic() < until:
                self.tick()
                alarm = self.d.value(5647)
                word = self.d.value(5649)
                pos = self.d.value(5644)
                if alarm or word & 512 or (not word & 1024) or (not word & 64):
                    raise RuntimeError('Cylinder alarm, lost enable or lost homing')
                if not max(0, min(start, target) - 30) <= pos <= min(4800, max(start, target) + 30):
                    raise RuntimeError('Cylinder position deviated from segment')
                done = abs(pos - target) <= 5 and (not word & 256) and word & 32
                settled = settled + 1 if done else 0
                if settled >= 3:
                    break
                time.sleep(0.03)
            else:
                raise RuntimeError('Cylinder move timeout')
        self.cylinder_moving = False
        self.cylinder_expected = target
        self.dwell()
        event.update(complete=True, end=now())

def execute(ip, port, report, recovery=False):
    import rtde_receive
    import rtde_control
    from mcea_direction_test import Device
    r = c = d = None
    try:
        set_stage(report, '重新核对两台设备的实时状态')
        current = inspect(ip, port)
        require_initial(current['robot'], current['dashboard'], current['cylinder'], recovery=recovery)
        report['initial'] = current
        set_stage(report, '连接机器人RTDE状态接口')
        r = rtde_receive.RTDEReceiveInterface(ip, 125.0)
        if math.dist(r.getActualTCPSpeed(), [0.0] * 6) > 0.0002:
            raise RuntimeError('Robot pose changed or robot is moving before control setup')
        start = list(r.getActualTCPPose())
        d = Device(port, 115200, 1)
        fresh_cylinder = d.snapshot()
        require_initial(read_robot(ip), dashboard(ip), fresh_cylinder, recovery=recovery)
        report['control_upload_attempted'] = True
        set_stage(report, '建立机器人控制连接并上传空闲控制程序；尚未移动')
        c = rtde_control.RTDEControlInterface(ip, 125.0)
        until = time.monotonic() + 5
        while r.getRuntimeState() != 2:
            if time.monotonic() > until:
                raise RuntimeError('Control script did not enter running state')
            time.sleep(0.03)
        if not c.setWatchdog(1.0):
            raise RuntimeError('Cannot configure control watchdog')
        trial = Trial(r, c, d, report)
        trial.recovery_mode = recovery
        trial.recovery_orientation = start[3:] if recovery else None
        trial.cylinder_expected = fresh_cylinder['position']
        trial.expected_hold = start
        trial.tick()
        report['full_path_sampling_enabled'] = False
        report['preflight_samples'] = 0
        report['fixed_orientation_rad'] = FIXED_ORIENTATION_RAD[:]
        report['orientation_captured_at'] = ORIENTATION_CAPTURED_AT
        actual = list(r.getActualTCPPose())
        if math.dist(actual[:3], start[:3]) > POSITION_TOL or orientation_error(actual[3:], start[3:]) > ANGLE_TOL:
            raise RuntimeError('Start moved during connection setup')
        set_stage(report, '进入连续恢复准备，尚未恢复固定姿态' if recovery else '保持读取时姿态，连续执行A–G；没有姿态归零动作')
        if recovery:
            set_stage(report, '中断后连续重新准备，不续跑旧步骤')
            set_stage(report, '恢复1：机械臂保持当前XYZ和姿态，电缸以1 mm/s回到标定位置0 mm（同组新指令解除软件暂停）')
            trial.cylinder(0)
            set_stage(report, '恢复2：保持当前XYZ，将末端恢复到13:10:55记录的固定姿态')
            trial.restore_fixed_orientation()
            set_stage(report, '恢复3：电缸保持0 mm，机械臂返回A点')
            trial.robot('A')
            report['recovered_to_start'] = True
            report['restart_cycle_from_beginning'] = True
            set_stage(report, '已回到起始状态；连续从B点开始新一轮，不续跑中断段')
        route = [('robot', 'A'), ('robot', 'B'), ('cylinder', 48), ('robot', 'C'), ('cylinder', 13), ('robot', 'A'), ('robot', 'D'), ('robot', 'E'), ('cylinder', 48), ('robot', 'F'), ('robot', 'G'), ('cylinder', 0), ('robot', 'A')]
        if recovery:
            route = route[1:]
        for axis, target in route:
            set_stage(report, f'执行：{axis} → {target}')
            print(f'{axis} -> {target}', flush=True)
            getattr(trial, axis)(target)
            save(report)
        report.update(success=True, final_robot_pose=list(r.getActualTCPPose()), final_cylinder=d.snapshot())
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        report['error_stage'] = report.get('stage')
        report['error_traceback'] = traceback.format_exc(limit=8)
        if c is not None:
            try:
                c.stopL(0.05)
                report['robot_stop_requested'] = True
            except BaseException as stop_error:
                report['robot_stop_error'] = str(stop_error)
        if d is not None and (report.get('cylinder_motion_attempted') or report.get('cylinder_control_attempted')):
            try:
                d.pause()
                report['cylinder_pause_acknowledged'] = True
            except BaseException as stop_error:
                report['cylinder_pause_error'] = str(stop_error)
        raise
    finally:
        if d is not None:
            report['cylinder_frames'] = d.log
            try:
                d.close()
            except Exception as error:
                report['cylinder_close_error'] = str(error)
        if c is not None:
            try:
                c.stopScript()
            except Exception as error:
                report['control_cleanup_error'] = str(error)
            try:
                c.disconnect()
            except Exception:
                pass
        if r is not None:
            try:
                r.disconnect()
            except Exception as error:
                report['receive_cleanup_error'] = str(error)
        report['end'] = now()
        save(report)

def save(report):
    folder = ROOT / 'logs'
    folder.mkdir(exist_ok=True)
    path = folder / report['log_file']
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--preview', action='store_true')
    mode.add_argument('--run', action='store_true')
    parser.add_argument('--ip', default='192.168.21.5')
    parser.add_argument('--port', default='COM4')
    args = parser.parse_args()
    if args.preview:
        print(json.dumps(build_plan(), ensure_ascii=False, indent=2))
        return 0
    report = dict(start=now(), success=False, segments=[], motion_commands_sent=False, log_file=datetime.now().strftime('trial_%Y%m%d_%H%M%S_%f.json'))
    try:
        report['check'] = inspect(args.ip, args.port)
        print(json.dumps(report['check'], ensure_ascii=False, indent=2))
        if not args.run:
            report['success'] = True
            return 0
        recovery = False
        if not report['check']['trial_start_ready']:
            if not report['check'].get('manual_recovery_available'):
                raise RuntimeError(report['check']['blocking_reason'])
            print('中断后尚未回到起始状态。确认后将连续执行：电缸回0（新定位指令解除软件暂停）→ 恢复固定姿态 → 回A → 从头完整运行一轮。')
            if input('输入 RECOVER 后连续恢复并运行完整一轮；其他输入退出：').strip() != 'RECOVER':
                raise RuntimeError('Cancelled before manual recovery')
            recovery = True
        print('单次连续实际运行：不做全路径采样。各段到位停顿1秒后自动进入下一段，控制器状态和目标限位仍会核验。')
        print(f'保持读取时姿态 Rx/Ry/Rz={FIXED_ORIENTATION_RAD} rad；J6自由；机械臂30 mm/s，电缸1 mm/s。')
        if not recovery and input('输入 RUN 后回车连续运行完整一轮（无需逐段确认）：').strip() != 'RUN':
            raise RuntimeError('Cancelled before control upload')
        report['motion_commands_sent'] = False
        report['manual_steps'] = False
        report['run_mode'] = 'continuous_single_cycle'
        report['manual_recovery_selected'] = recovery
        execute(args.ip, args.port, report, recovery=recovery)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error) or type(error).__name__
        print('Stopped: ' + report['error'])
        if report.get('error_stage'):
            print('中断/失败阶段：' + report['error_stage'], flush=True)
        if isinstance(error, KeyboardInterrupt):
            print('收到键盘或调试器中断；程序已退出，不会继续下一段。', flush=True)
        return 1
    finally:
        report['motion_commands_sent'] = bool(report.get('robot_motion_attempted') or report.get('cylinder_motion_attempted'))
        save(report)
if __name__ == '__main__':
    raise SystemExit(main())
