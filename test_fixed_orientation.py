"""Offline tests for the captured-orientation continuous program; no hardware I/O."""
import ast
import json
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
import ag_trial as app


class CapturedOrientation(unittest.TestCase):
    def setUp(self):
        self.robot=dict(connected=True,enabled=True,power_on=True,mode=7,
                        emergency_stop=False,protective_stop=False,program_running=False,
                        program_paused=False,pose=app.POSES['A'][:],tcp_offset=[0.]*6)
        self.dash={'PolyscopeVersion':'URSoftware 3.15.8.106339',
                   'safetymode':'Safetymode: NORMAL','running':'Program running: false'}
        self.cyl=dict(id=1,position=0,homing=1,status=17504,alarm=0,control=33,io=0,limit=5084,offset=100)

    def test_targets_reproduce_hardware_snapshot(self):
        expected=[1.4747881408617243,-.6236196873562871,.6049084354198352]
        self.assertEqual(app.FIXED_ORIENTATION_RAD,expected)
        for pose in app.POSES.values():
            self.assertEqual(pose[3:],expected)
        self.assertEqual(app.build_plan()['rotation_vector_rad'],expected)

    def test_start_requires_captured_orientation_not_zero(self):
        app.require_initial(self.robot,self.dash,self.cyl)
        self.robot['pose']=self.robot['pose'][:3]+[0.,0.,0.]
        with self.assertRaisesRegex(RuntimeError,'captured fixed orientation'):
            app.require_initial(self.robot,self.dash,self.cyl)

    def test_equivalent_rotation_representation_is_accepted(self):
        rv=app.FIXED_ORIENTATION_RAD
        n=app.math.sqrt(sum(v*v for v in rv))
        equivalent=[v*(n+2*app.math.pi)/n for v in rv]
        self.assertTrue(app.matches_fixed_orientation(self.robot['pose'][:3]+equivalent))

    def test_existing_program_and_stop_flags_reject(self):
        for key in ('program_running','program_paused','emergency_stop','protective_stop'):
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                app.require_initial(dict(self.robot,**{key:True}),self.dash,self.cyl)

    def test_cylinder_faults_and_unhomed_reject(self):
        for change in ({'homing':0},{'alarm':1},{'position':1300},{'io':1},{'status':17504|256}):
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                app.require_initial(self.robot,self.dash,dict(self.cyl,**change))

    def test_interrupted_state_is_recoverable_but_not_normal_ready(self):
        stopped=dict(self.cyl,position=4800,control=0x28,status=50272)
        robot=dict(self.robot,pose=self.robot['pose'][:])
        robot['pose'][3]+=.02
        with self.assertRaises(RuntimeError):
            app.require_initial(robot,self.dash,stopped)
        app.require_initial(robot,self.dash,stopped,recovery=True)

    def test_recovery_never_bypasses_alarm_estop_or_unhomed_state(self):
        stopped=dict(self.cyl,position=4800,control=0x28,status=50272)
        for change in ({'alarm':1},{'homing':0},{'io':1},{'control':0x2C}):
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                app.require_initial(self.robot,self.dash,dict(stopped,**change),recovery=True)
        with self.assertRaises(RuntimeError):
            app.require_initial(dict(self.robot,emergency_stop=True),self.dash,stopped,recovery=True)

    def test_paused_retraction_uses_new_zero_target_and_slow_parameters(self):
        paused=dict(self.cyl,position=4800,control=0x28,status=50272)
        d=Mock();d.snapshot.return_value=paused
        d.value.side_effect=lambda address:{0x160F:0,0x1611:17504,0x160C:0}[address]
        t=app.Trial(Mock(),Mock(),d,{'segments':[]})
        t.recovery_mode=True;t.cylinder_expected=4800
        with patch.object(t,'tick'),patch.object(t,'dwell'),patch.object(app.time,'sleep'):
            t.cylinder(0)
        frames=[call.args[0] for call in d.write_ack.call_args_list]
        self.assertEqual(len(frames),2)
        for packet,control in zip(frames,(0x20,0x21)):
            self.assertEqual(app.struct.unpack('>BBHHB',packet[:7]),(1,16,0x1600,6,12))
            self.assertEqual(app.struct.unpack('>6H',packet[7:]),(0,0,100,1,0,control))
        self.assertEqual(t.cylinder_expected,0)

    def test_failed_recovery_command_does_not_retry(self):
        d=Mock();d.snapshot.return_value=dict(self.cyl,position=4800,control=0x28,status=50272)
        d.write_ack.side_effect=RuntimeError('missing acknowledgement')
        t=app.Trial(Mock(),Mock(),d,{'segments':[]})
        t.recovery_mode=True;t.cylinder_expected=4800
        with patch.object(t,'tick'),self.assertRaisesRegex(RuntimeError,'acknowledgement'):
            t.cylinder(0)
        self.assertTrue(t.report['cylinder_motion_attempted'])
        d.write_ack.assert_called_once()

    def test_recovery_restores_original_snapshot_not_new_capture(self):
        r=Mock();c=Mock();d=Mock()
        start=app.POSES['A'][:];start[3]+=.02
        target=start[:3]+app.FIXED_ORIENTATION_RAD[:]
        r.getActualTCPPose.side_effect=[start,target]
        c.isPoseWithinSafetyLimits.return_value=True;c.moveL.return_value=True
        c.getAsyncOperationProgress.side_effect=[-1,-2]
        t=app.Trial(r,c,d,{'segments':[]});t.recovery_mode=True
        with patch.object(t,'tick'),patch.object(t,'dwell'):
            t.restore_fixed_orientation()
        c.moveL.assert_called_once_with(target,.001,.001,True)
        self.assertFalse(t.recovery_mode)

    def test_recovery_restarts_at_A_and_B_instead_of_resuming_old_step(self):
        pose=app.POSES['C'][:];pose[3]+=.02
        robot=dict(self.robot,pose=pose[:])
        cyl=dict(self.cyl,position=4800,control=0x28,status=50272)
        r=Mock();c=Mock();d=Mock()
        r.getActualTCPPose.side_effect=lambda:pose[:]
        r.getActualTCPSpeed.return_value=[0.]*6;r.getRuntimeState.return_value=2
        c.setWatchdog.return_value=True;d.snapshot.side_effect=lambda:dict(cyl);d.log=[]
        modules={'rtde_receive':Mock(RTDEReceiveInterface=Mock(return_value=r)),
                 'rtde_control':Mock(RTDEControlInterface=Mock(return_value=c))}
        state={'robot':robot,'dashboard':self.dash,'cylinder':dict(cyl)}
        report={'segments':[]}
        with patch.dict('sys.modules',modules),patch('mcea_direction_test.Device',return_value=d),patch.object(app,'Device',return_value=d,create=True),patch.object(app,'inspect',return_value=state),patch.object(app,'read_robot',return_value=robot),patch.object(app,'dashboard',return_value=self.dash),patch.object(app,'save'),patch('builtins.input',side_effect=AssertionError('Unexpected per-segment input')),patch.object(app,'Trial') as tc:
            trial=tc.return_value
            def retract(mm):
                cyl['position']=round(mm*100)
                cyl['control']=0x21
            def restore():pose[3:]=app.FIXED_ORIENTATION_RAD[:]
            def move(name):pose[:]=app.POSES[name][:]
            trial.cylinder.side_effect=retract
            trial.restore_fixed_orientation.side_effect=restore
            trial.robot.side_effect=move
            app.execute('unused','unused',report,recovery=True)
            self.assertEqual([call.args[0] for call in trial.robot.call_args_list[:2]],['A','B'])
            self.assertEqual(trial.cylinder.call_args_list[0].args,(0,))
            self.assertTrue(report['recovered_to_start'])
            self.assertTrue(report['restart_cycle_from_beginning'])

    def test_changed_tcp_rejects(self):
        self.robot['tcp_offset'][2]=.01
        with self.assertRaisesRegex(RuntimeError,'TCP differs'):
            app.require_initial(self.robot,self.dash,self.cyl)

    def test_target_limit_rejection_does_not_move(self):
        c=Mock();c.isPoseWithinSafetyLimits.return_value=False
        with self.assertRaisesRegex(RuntimeError,'Controller rejected'):
            app.require_target(c,app.POSES['A'])
        c.moveL.assert_not_called()

    def test_check_mode_does_not_enter_execution(self):
        with patch.object(app,'inspect',return_value={'trial_start_ready':True}),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('sys.argv',['ag_run.py','--check']):
            self.assertEqual(app.main(),0)
            execute.assert_not_called()

    def test_recover_prompt_selects_continuous_restart(self):
        status={'trial_start_ready':False,'manual_recovery_available':True,'blocking_reason':'paused cylinder'}
        with patch.object(app,'inspect',return_value=status),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('builtins.input',return_value='RECOVER') as prompt,patch('sys.argv',['ag_run.py','--run']):
            self.assertEqual(app.main(),0)
            self.assertEqual(prompt.call_count,1)
            self.assertTrue(execute.call_args.kwargs['recovery'])
            self.assertEqual(execute.call_args.kwargs,{'recovery':True})
            self.assertEqual(execute.call_args.args[2]['run_mode'],'continuous_single_cycle')

    def test_nonrecoverable_fault_never_prompts_or_uploads(self):
        status={'trial_start_ready':False,'manual_recovery_available':False,'blocking_reason':'emergency stop active'}
        with patch.object(app,'inspect',return_value=status),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('builtins.input') as prompt,patch('sys.argv',['ag_run.py','--run']):
            self.assertEqual(app.main(),1)
            prompt.assert_not_called();execute.assert_not_called()

    def test_robot_command_uses_saved_orientation_and_30_mm_s(self):
        r=Mock();c=Mock();d=Mock();target=app.POSES['A'][:]
        pose=target[:];pose[0]+=.01
        r.getActualTCPPose.side_effect=lambda:pose[:]
        r.getActualTCPSpeed.return_value=[0.]*6
        c.isPoseWithinSafetyLimits.return_value=True
        c.getAsyncOperationProgress.side_effect=[-1,-2]
        def move(p,*args):
            pose[:]=p
            return True
        c.moveL.side_effect=move
        t=app.Trial(r,c,d,{'segments':[]})
        with patch.object(t,'tick'),patch.object(t,'dwell'):
            t.robot('A')
        c.moveL.assert_called_once_with(target,.03,.001,True)
        self.assertTrue(t.report['segments'][0]['complete'])

    def test_feedback_retains_orientation_and_only_records_speed(self):
        r=Mock();c=Mock();d=Mock()
        r.isConnected.return_value=c.isConnected.return_value=True
        r.getTimestamp.return_value=1;r.getRobotMode.return_value=7
        r.getSafetyMode.return_value=1;r.getRuntimeState.return_value=2;r.getSpeedScaling.return_value=1
        r.getActualTCPPose.return_value=app.POSES['A'][:]
        r.getActualTCPSpeed.return_value=[.04,0,0,.03,0,0]
        c.getTCPOffset.return_value=[0.]*6;c.kickWatchdog.return_value=True
        d.snapshot.return_value=self.cyl
        t=app.Trial(r,c,d,{'segments':[]});t.tick()
        self.assertEqual(t.report['max_observed_linear_speed_m_s'],.04)
        r.getActualTCPPose.return_value=app.POSES['A'][:3]+[0.,0.,0.]
        with self.assertRaisesRegex(RuntimeError,'fixed tool orientation'):
            t.tick()

    def test_execution_has_no_rotation_initialization_or_sampling(self):
        self.assertFalse(hasattr(app.Trial,'initialize_orientation'))
        self.assertFalse(hasattr(app,'preflight_orientation'))
        r=Mock();c=Mock();d=Mock()
        r.getActualTCPPose.return_value=app.POSES['A'][:]
        r.getActualTCPSpeed.return_value=[0.]*6;r.getRuntimeState.return_value=2
        c.setWatchdog.return_value=True;d.snapshot.return_value=self.cyl;d.log=[]
        status={'robot':self.robot,'dashboard':self.dash,'cylinder':self.cyl}
        modules={'rtde_receive':Mock(RTDEReceiveInterface=Mock(return_value=r)),
                 'rtde_control':Mock(RTDEControlInterface=Mock(return_value=c))}
        report={'segments':[]}
        with patch.dict('sys.modules',modules),patch('mcea_direction_test.Device',return_value=d),patch.object(app,'Device',return_value=d,create=True),patch.object(app,'inspect',return_value=status),patch.object(app,'read_robot',return_value=self.robot),patch.object(app,'dashboard',return_value=self.dash),patch.object(app,'save'),patch('builtins.input',side_effect=AssertionError('Unexpected per-segment input')),patch.object(app,'Trial') as trial:
            app.execute('unused','unused',report)
            actions=[(call[0],call.args[0]) for call in trial.return_value.method_calls if call[0] in ('robot','cylinder')]
            self.assertEqual(actions,[('robot','A'),('robot','B'),('cylinder',48),('robot','C'),('cylinder',13),('robot','A'),('robot','D'),('robot','E'),('cylinder',48),('robot','F'),('robot','G'),('cylinder',0),('robot','A')])
            c.getInverseKinematics.assert_not_called()
            self.assertEqual(report['fixed_orientation_rad'],app.FIXED_ORIENTATION_RAD)
            self.assertTrue(report['success'])

    def test_run_requires_only_one_start_confirmation(self):
        with patch.object(app,'inspect',return_value={'trial_start_ready':True}),patch.object(app,'save'),patch.object(app,'execute') as execute,patch('builtins.input',return_value='RUN') as prompt,patch('sys.argv',['ag_run.py','--run']):
            self.assertEqual(app.main(),0)
            prompt.assert_called_once()
            self.assertEqual(execute.call_args.kwargs,{'recovery':False})
            self.assertFalse(execute.call_args.args[2]['manual_steps'])

    def test_continuous_route_stops_after_action_failure(self):
        r=Mock();c=Mock();d=Mock()
        r.getActualTCPPose.return_value=app.POSES['A'][:]
        r.getActualTCPSpeed.return_value=[0.]*6;r.getRuntimeState.return_value=2
        c.setWatchdog.return_value=True;d.snapshot.return_value=self.cyl;d.log=[]
        status={'robot':self.robot,'dashboard':self.dash,'cylinder':self.cyl}
        modules={'rtde_receive':Mock(RTDEReceiveInterface=Mock(return_value=r)),
                 'rtde_control':Mock(RTDEControlInterface=Mock(return_value=c))}
        with patch.dict('sys.modules',modules),patch('mcea_direction_test.Device',return_value=d),patch.object(app,'Device',return_value=d,create=True),patch.object(app,'inspect',return_value=status),patch.object(app,'read_robot',return_value=self.robot),patch.object(app,'dashboard',return_value=self.dash),patch.object(app,'save'),patch('builtins.input',side_effect=AssertionError('Unexpected per-segment input')),patch.object(app,'Trial') as trial:
            trial.return_value.robot.side_effect=[None,RuntimeError('operator stop')]
            with self.assertRaisesRegex(RuntimeError,'operator stop'):
                app.execute('unused','unused',{'segments':[]})
            self.assertEqual([call.args[0] for call in trial.return_value.robot.call_args_list],['A','B'])
            trial.return_value.cylinder.assert_not_called()
            c.stopL.assert_called_once_with(.05)
            c.stopScript.assert_called_once()



if __name__=='__main__':
    unittest.main()
