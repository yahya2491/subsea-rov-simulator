"""Run the hover-controller validation campaign and build the report.

    ros2 run subsea_rov_bringup run_validation                       # all scenarios
    ros2 run subsea_rov_bringup run_validation --scenarios calm_hold impulse_push
    ros2 run subsea_rov_bringup run_validation --output ~/hover_validation --seed 7

Each scenario gets a fresh, headless simulator so runs are independent and
reproducible (fixed seed). Runs are isolated on their own Gazebo partition
and ROS domain, so a GUI session you have open neither disturbs nor is
disturbed by them. Exits non-zero if any scenario fails its limits.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import yaml

from . import analysis


def session_pids(sid: int):
    """Return the PIDs of every live process in session ``sid`` (Linux /proc)."""
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
        except OSError:
            continue
        if int(fields[3]) == sid and fields[0] != 'Z':   # fields[3] = session id
            pids.append(int(entry))
    return pids


def kill_session(sid: int, grace: float = 10.0) -> int:
    """Terminate a launch and everything it spawned, orphans included.

    Stopping `gz sim` can leave its server process orphaned (re-parented to
    init) but still in the launch's session. Left alone it keeps simulating
    and the next scenario would connect to it.
    """
    pids = session_pids(sid)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline and session_pids(sid):
            time.sleep(0.2)
        pids = session_pids(sid)
        if not pids:
            break
    return len(pids)


def main(argv=None):
    share = get_package_share_directory('subsea_rov_bringup')
    default_file = os.path.join(share, 'config', 'scenarios.yaml')
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--scenarios', nargs='*', default=None,
                   help='scenario names (default: all in the scenarios file)')
    p.add_argument('--scenarios-file', default=default_file)
    p.add_argument('--output', default='validation_results')
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--timeout', type=float, default=300.0, help='per-scenario timeout, s')
    p.add_argument('--domain-id', type=int, default=77,
                   help='ROS_DOMAIN_ID for the runs, keeps them off your normal domain')
    a = p.parse_args(argv)

    with open(a.scenarios_file) as f:
        available = list(yaml.safe_load(f)['scenarios'])
    names = a.scenarios or available
    unknown = sorted(set(names) - set(available))
    if unknown:
        p.error(f'unknown scenarios {unknown}; available: {available}')

    out = os.path.abspath(os.path.expanduser(a.output))
    os.makedirs(out, exist_ok=True)
    base_env = dict(os.environ, ROS_DOMAIN_ID=str(a.domain_id))

    for i, name in enumerate(names, 1):
        print(f'[{i}/{len(names)}] {name} ...', flush=True)
        for stale in (f'{name}.json', f'{name}.csv', f'{name}.png'):
            if os.path.exists(os.path.join(out, stale)):
                os.remove(os.path.join(out, stale))
        cmd = ['ros2', 'launch', 'subsea_rov_bringup', 'sim.launch.py', 'gui:=false',
               f'scenario:={name}', f'scenarios_file:={a.scenarios_file}',
               f'output_dir:={out}', f'seed:={a.seed}']
        log_path = os.path.join(out, f'{name}.log')
        start = time.monotonic()
        # Fresh Gazebo partition per run, and the run in its own session so it
        # can be torn down completely afterwards.
        env = dict(base_env, GZ_PARTITION=f'hover_validation_{os.getpid()}_{i}')
        with open(log_path, 'w') as log:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env,
                                    start_new_session=True)
            try:
                proc.wait(timeout=a.timeout)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    pass
                print(f'    timed out after {a.timeout:.0f} s (log: {log_path})')
            left = kill_session(proc.pid)
            if left:
                print(f'    warning: {left} processes from this run could not be stopped')
        result_path = os.path.join(out, f'{name}.json')
        if os.path.exists(result_path):
            r = yaml.safe_load(open(result_path))
            verdict = 'PASS' if r['passed'] else 'FAIL'
            print(f'    {verdict} in {time.monotonic() - start:.0f} s'
                  + ('' if r['passed'] else ': ' + '; '.join(r['failures'])), flush=True)
        else:
            print(f'    no result written (log: {log_path})', flush=True)

    ok = analysis.write_report(out)
    produced = {n for n in names if os.path.exists(os.path.join(out, f'{n}.json'))}
    ok = ok and produced == set(names)
    print(f'\nreport: {os.path.join(out, "report.md")}')
    print('OVERALL: ' + ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
