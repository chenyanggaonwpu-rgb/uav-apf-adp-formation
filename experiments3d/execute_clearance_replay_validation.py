"""Wait for audited postprocessing, validate the frozen replay and make an MP4."""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import time
from pathlib import Path


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('study', type=Path)
    args = parser.parse_args()
    study = args.study.resolve()
    root = Path(__file__).resolve().parents[1]
    print('REPLAY_WAITING_FOR_AUDITED_POSTPROCESSING', flush=True)
    while not (study / 'postprocessing_complete.json').exists():
        time.sleep(2)
    assert json.loads((study / 'postprocessing_complete.json').read_text())['passed']
    assert not (study / 'replay_validation.json').exists(), 'Preserve prior replay validation'
    runtime = root / '.runtime/margin_study_20261006/replay_validation'
    runtime.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['MATLAB_PREFDIR'] = str(runtime / 'prefs')
    Path(env['MATLAB_PREFDIR']).mkdir(exist_ok=True)
    code = (f"cd('{root.as_posix()}'); addpath('config','src','src3d','experiments3d'); "
            f"validate_clearance_replay3d('{study.as_posix()}');")
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    with (runtime / 'stdout.log').open('wb') as stream:
        worker = subprocess.Popen(
            ['C:/Program Files/MATLAB/R2024a/bin/matlab.exe', '-singleCompThread',
             '-batch', code, '-logfile', str(runtime / 'matlab.log')],
            cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT,
            startupinfo=startup, creationflags=subprocess.CREATE_NO_WINDOW)
        last = 0
        started = time.monotonic()
        while worker.poll() is None:
            time.sleep(2)
            elapsed = time.monotonic() - started
            if elapsed - last >= 30:
                print(f'REPLAY_VALIDATION_RUNNING elapsed={elapsed:.0f}s', flush=True)
                last = elapsed
        assert worker.returncode == 0, 'Replay failed; retained MATLAB log has diagnostics'
    report_source = study / 'postprocessing_snapshot/analyze_clearance_study.py'
    spec = importlib.util.spec_from_file_location('frozen_report_after_replay', report_source)
    report = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(report)
    report.build(study)
    print('REPLAY_VIDEO_AND_REPORT_COMPLETE', flush=True)
