"""Run both delivery variants offline; never connect to hardware."""
import contextlib
import importlib.util
import io
import json
import py_compile
import socket
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
import serial
import test_fixed_orientation as tests

ROOT = Path(__file__).resolve().parent


def no_hardware(*args, **kwargs):
    raise AssertionError('Hardware communication is forbidden during offline tests')


def main():
    results = []
    with patch.object(socket, 'create_connection', side_effect=no_hardware), patch.object(serial, 'Serial', side_effect=no_hardware):
        for name, filename in [('module', 'ag_trial.py'), ('single_file', 'ag_run.py')]:
            spec = importlib.util.spec_from_file_location('offline_' + name, ROOT / filename)
            app = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(app)
            tests.app = app
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                result = unittest.TextTestRunner(stream=output, verbosity=2).run(
                    unittest.defaultTestLoader.loadTestsFromModule(tests))
            py_compile.compile(str(ROOT / filename), doraise=True)
            results.append(dict(target=name, tests=result.testsRun, success=result.wasSuccessful(),
                                failures=len(result.failures), errors=len(result.errors)))
            print(f'{name}: {result.testsRun} tests; success={result.wasSuccessful()}')
            if not result.wasSuccessful():
                print(output.getvalue())
    report = dict(mode='offline_only', hardware_connected=False, motion_commands_sent=False,
                  results=results)
    folder = ROOT / 'reports'
    folder.mkdir(exist_ok=True)
    (folder / 'validation_continuous.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return 0 if all(x['success'] for x in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
