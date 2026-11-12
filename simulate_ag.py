"""A-G offline geometric simulation. Standard library only; no device I/O.

Run: python simulate_ag.py
Optional synthetic initial position: --start-xyz X Y Z (mm)
Tool rotation vector is fixed at the hardware snapshot, relative to base. No joint is
locked. No IK, robot links, collisions or homing search simulated.
"""
import argparse
import csv
import json
import math
from pathlib import Path

POINTS = {
    'A': [-337.9, -424.4, 382.0], 'B': [-340.0, -422.4, 443.0],
    'C': [-330.6, -431.6, 445.1], 'D': [70.5, -436.3, 392.1],
    'E': [79.4, -412.1, 216.7], 'F': [93.3, -456.2, 229.7],
    'G': [67.9, -465.0, 410.0],
}
ROBOT_SPEED = 30.0  # mm/s; includes horizontal, vertical and diagonal moves
CYLINDER_SPEED = 1.0  # mm/s; unchanged
ACCEL = 1.0  # mm/s², chosen for the virtual model only
TOOL_AXIS = [-0.013476970671436439, -0.999907139719671, 0.0020206927394919583]
ROTATION = [[0.7061936829343256, -0.7078904247446596, -0.013476970671436439], [-0.008087507651501125, 0.01096832519635993, -0.999907139719671], [0.7079725096383618, 0.7062371006943862, 0.0020206927394919583]]
ROTATION_VECTOR = [1.4747881408617243, -0.6236196873562871, 0.6049084354198352]


def motion_profile(distance, speed, accel=ACCEL):
    if distance <= 1e-12:
        return 0.0, 0.0, 0.0
    ramp = min(speed / accel, math.sqrt(distance / accel))
    peak = accel * ramp
    cruise = max(0.0, (distance - accel * ramp * ramp) / peak)
    return ramp, cruise, 2 * ramp + cruise


def displacement(distance, elapsed, speed, accel=ACCEL):
    ramp, cruise, duration = motion_profile(distance, speed, accel)
    t = max(0.0, min(duration, elapsed))
    if t <= ramp:
        return 0.5 * accel * t * t
    if t <= ramp + cruise:
        return 0.5 * accel * ramp * ramp + accel * ramp * (t-ramp)
    return distance - 0.5 * accel * (duration-t)**2


def build_plan(start=None):
    xyz = list(POINTS['A'] if start is None else start)
    cylinder, clock, phases = 0.0, 0.0, []

    def add(step, label, axis, target=None):
        nonlocal xyz, cylinder, clock
        old_xyz, old_cyl = xyz[:], cylinder
        distance = 0.0
        if axis == 'robot':
            xyz = list(target)
            distance = math.dist(old_xyz, xyz)
        elif axis == 'cylinder':
            cylinder = float(target)
            distance = abs(cylinder-old_cyl)
        speed = ROBOT_SPEED if axis == 'robot' else CYLINDER_SPEED
        duration = 1.0 if axis == 'dwell' else motion_profile(distance, speed)[2]
        phase = dict(index=len(phases), step=step, label=label, axis=axis,
                     start_s=clock, duration_s=duration, end_s=clock+duration,
                     from_xyz_mm=old_xyz, to_xyz_mm=xyz[:],
                     from_cylinder_mm=old_cyl, to_cylinder_mm=cylinder,
                     distance_mm=distance, rotation_vector_rad=ROTATION_VECTOR[:], max_speed_mm_s=speed,
                     virtual_acceleration_mm_s2=ACCEL)
        phases.append(phase)
        clock += duration

    def robot(step, point):
        add(step, f'机械臂 → {point}', 'robot', POINTS[point])
        add(step, '到位停顿1秒', 'dwell')

    def cyl(step, position):
        add(step, f'电缸 → {position} mm', 'cylinder', position)
        add(step, '到位停顿1秒', 'dwell')

    if math.dist(xyz, POINTS['A']) > 1e-9:
        robot('准备', 'A')
    add('1', '虚拟电缸已回零，位置0 mm', 'check')
    robot('1', 'B')
    cyl('2', 48)
    robot('4', 'C')
    cyl('5', 13)
    robot('6', 'A')
    robot('7', 'D')
    robot('8', 'E')
    cyl('9', 48)
    robot('10', 'F')
    robot('11', 'G')
    # Explicit serial ordering chosen for offline simulation of step 12.
    cyl('12a', 0)
    robot('12b', 'A')
    return dict(mode='offline_geometric_only', hardware_connections=False,
                units='mm,s', points=POINTS, initial_xyz_mm=list(start or POINTS['A']),
                initial_cylinder_mm=0, robot_max_speed_mm_s=ROBOT_SPEED,
                cylinder_max_speed_mm_s=CYLINDER_SPEED,
                virtual_acceleration_mm_s2=ACCEL,
                tool_axis_in_base=TOOL_AXIS,
                orientation_frame='robot_base', orientation_representation='rotation_vector_rad',
                rotation_matrix=ROTATION, rotation_vector_rad=ROTATION_VECTOR,
                target_tcp_poses_m_rad={k:[x/1000 for x in v]+ROTATION_VECTOR for k,v in POINTS.items()},
                orientation_heading_policy='fixed full orientation captured from hardware',
                locked_joints=[],
                joint_trajectory_status='not_solved',
                timing_is_geometric_estimate=True,
                orientation_is_hardware_calibrated=False,
                cylinder_positive='down', cylinder_zero_is_virtual=True,
                tcp_and_cylinder_are_separate_coordinates=True,
                final_return_order='retract cylinder to 0 at G, dwell, robot G to A, dwell',
                simulated_homing_search=False, simulated_ik_or_collision=False,
                total_duration_s=clock, phases=phases)


def sample(plan, t):
    t = min(max(0.0, t), plan['total_duration_s'])
    phase = next((p for p in plan['phases'] if p['end_s'] > t), plan['phases'][-1])
    d = phase['distance_mm']
    f = displacement(d, t-phase['start_s'], phase['max_speed_mm_s'], phase['virtual_acceleration_mm_s2']) / d if d else 1.0
    xyz = [a+(b-a)*f for a,b in zip(phase['from_xyz_mm'], phase['to_xyz_mm'])]
    cylinder = phase['from_cylinder_mm'] + (phase['to_cylinder_mm']-phase['from_cylinder_mm'])*f
    return dict(t_s=t, phase=phase['index'], step=phase['step'], label=phase['label'],
                x_mm=xyz[0], y_mm=xyz[1], z_mm=xyz[2], cylinder_mm=cylinder,
                rx_rad=ROTATION_VECTOR[0], ry_rad=ROTATION_VECTOR[1], rz_rad=ROTATION_VECTOR[2],
                tool_z_x=TOOL_AXIS[0], tool_z_y=TOOL_AXIS[1], tool_z_z=TOOL_AXIS[2])


def validate(plan):
    assert plan['robot_max_speed_mm_s'] == 30
    assert plan['cylinder_max_speed_mm_s'] == 1
    assert plan['locked_joints'] == []
    assert plan['rotation_vector_rad'] == ROTATION_VECTOR
    assert plan['tool_axis_in_base'] == TOOL_AXIS
    phases = plan['phases']
    assert [p['step'] for p in phases if p['axis'] in ('robot','cylinder') and p['step'] != '准备'] == [
        '1','2','4','5','6','7','8','9','10','11','12a','12b']
    for i, p in enumerate(phases):
        if i:
            assert p['from_xyz_mm'] == phases[i-1]['to_xyz_mm']
            assert p['from_cylinder_mm'] == phases[i-1]['to_cylinder_mm']
        assert 0 <= p['to_cylinder_mm'] <= 50
        if p['axis'] in ('robot','cylinder'):
            assert phases[i+1]['axis'] == 'dwell' and phases[i+1]['duration_s'] == 1
        for j in range(101):
            row = sample(plan, p['start_s'] + p['duration_s']*j/100)
            assert row['tool_z_z'] == TOOL_AXIS[2]
            assert [row[k] for k in ('rx_rad','ry_rad','rz_rad')] == ROTATION_VECTOR
        # Analytic maximum speed is a*ramp, including triangular short moves.
        speed, accel = p['max_speed_mm_s'], p['virtual_acceleration_mm_s2']
        ramp, _, duration = motion_profile(p['distance_mm'], speed, accel)
        assert accel*ramp <= speed+1e-12
        assert abs(displacement(p['distance_mm'], duration, speed, accel)-p['distance_mm']) < 1e-9
    last = sample(plan, plan['total_duration_s'])
    assert math.dist([last['x_mm'],last['y_mm'],last['z_mm']], POINTS['A']) < 1e-9
    assert last['cylinder_mm'] == 0
    return {'passed': True, 'checks': ['updated sequence', 'exact XYZ endpoints',
            'continuous phase boundaries', 'robot max 30 mm/s, cylinder max 1 mm/s', '1 s arrival dwell',
            '0..50 mm cylinder range', 'constant captured hardware orientation', 'no locked joints', 'final A and cylinder zero'],
            'limitation': 'Geometric simulation only. No IK, solved joint trajectory, link geometry, collisions or real homing.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start-xyz', nargs=3, type=float, metavar=('X','Y','Z'))
    parser.add_argument('--out-dir', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    if args.start_xyz and not all(math.isfinite(x) for x in args.start_xyz):
        parser.error('Start coordinates must be finite')
    plan = build_plan(args.start_xyz)
    checks = validate(plan)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir/'simulation_plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding='utf-8')
    (args.out_dir/'simulation_checks.json').write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding='utf-8')
    times = set([0.0, plan['total_duration_s']])
    times.update(i/10 for i in range(math.ceil(plan['total_duration_s']*10)))
    times.update(p['end_s'] for p in plan['phases'])
    with (args.out_dir/'simulation_trace.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(sample(plan,0)))
        writer.writeheader()
        for t in sorted(times):
            writer.writerow(sample(plan,t))
    template = Path(__file__).with_name('simulation_view.html')
    if template.exists():
        fragment = template.read_text(encoding='utf-8').replace('/*PLAN_DATA*/{}', json.dumps(plan,ensure_ascii=False))
        (args.out_dir/'ag_simulation.html').write_text(
            '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>A–G 离线模拟</title><style>:root{color-scheme:light dark;--background:Canvas;--foreground:CanvasText;--border:GrayText;--muted:ButtonFace;--muted-foreground:GrayText;--viz-series-1:light-dark(#17654b,#62d4ad);--viz-series-2:light-dark(#3155a0,#9cbaff);font:16px system-ui}body{max-width:900px;margin:24px auto;padding:0 16px;background:var(--background);color:var(--foreground)}button,input,select{font:inherit}button{padding:8px 14px} .viz-controls{display:flex;gap:12px;flex-wrap:wrap;align-items:center}.form-range{width:100%}.text-small{font-size:13px}.text-muted{color:var(--muted-foreground)}</style>'
            +fragment+'</html>',encoding='utf-8')
        (args.out_dir/'ag_simulation_inline.html').write_text(fragment,encoding='utf-8')
    print(json.dumps({'simulation_completed': True, 'checks': checks['passed'],
                      'duration_s': round(plan['total_duration_s'],3),
                      'duration_min': round(plan['total_duration_s']/60,3),
                      'robot_path_mm': round(sum(p['distance_mm'] for p in plan['phases'] if p['axis']=='robot'),3),
                      'cylinder_travel_mm': sum(p['distance_mm'] for p in plan['phases'] if p['axis']=='cylinder'),
                      'final':sample(plan,plan['total_duration_s'])},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
