"""Metrics, plots and the validation report for hover scenarios.

Independent of ROS so it can be unit-tested and re-run on saved CSVs:

    python3 -m subsea_rov_bringup.analysis <results_dir>
"""

import csv
import json
import math
import os
import sys
from typing import Dict, List

import numpy as np

COLUMNS = (
    ['t']
    + [f'gt_{k}' for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
    + [f'nav_{k}' for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
    + ['sp_x', 'sp_y', 'sp_z', 'sp_yaw']
    + ['cur_x', 'cur_y', 'cur_z', 'push_x', 'push_y', 'push_z']
    + [f'u{i}' for i in range(8)]
    + ['w_fx', 'w_fy', 'w_fz', 'w_mx', 'w_my', 'w_mz']
)
SATURATION_N = 14.7

METRIC_UNITS = {
    'horizontal_rms': 'm', 'horizontal_max': 'm', 'final_horizontal_rms': 'm',
    'depth_rms': 'm', 'depth_max': 'm',
    'yaw_rms_deg': 'deg', 'yaw_max_deg': 'deg', 'final_yaw_max_deg': 'deg',
    'tilt_max_deg': 'deg', 'nav_position_rms': 'm', 'nav_yaw_rms_deg': 'deg',
    'saturation_fraction': '', 'recovery_time': 's',
}


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def write_csv(path: str, rows: List[List[float]]) -> None:
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        w.writerows(rows)


def load_csv(path: str) -> Dict[str, np.ndarray]:
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        data = np.array([[float(v) for v in row] for row in reader])
    if data.size == 0:
        data = np.zeros((0, len(header)))
    return {name: data[:, i] for i, name in enumerate(header)}


def errors(d: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    return {
        'horizontal': np.hypot(d['gt_x'] - d['sp_x'], d['gt_y'] - d['sp_y']),
        'depth': np.abs(d['gt_z'] - d['sp_z']),
        'yaw': np.abs(wrap(d['gt_yaw'] - d['sp_yaw'])),
        'tilt': np.maximum(np.abs(d['gt_roll']), np.abs(d['gt_pitch'])),
        'nav_position': np.sqrt((d['nav_x'] - d['gt_x']) ** 2 + (d['nav_y'] - d['gt_y']) ** 2
                                + (d['nav_z'] - d['gt_z']) ** 2),
        'nav_yaw': np.abs(wrap(d['nav_yaw'] - d['gt_yaw'])),
    }


def _rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float('nan')


def _max(x):
    return float(np.max(x)) if len(x) else float('nan')


def recovery_times(d, spec) -> List[dict]:
    """Time from the end of each `recover` event until errors stay in tolerance."""
    tol = spec['recovery_tolerance']
    e = errors(d)
    bad = (e['horizontal'] > tol) | (e['depth'] > tol)
    t = d['t']
    events = sorted(spec.get('events', []), key=lambda ev: ev['t'])
    out = []
    for i, ev in enumerate(events):
        if not ev.get('recover'):
            continue
        dur = ev.get('push', {}).get('duration', 0.0) if 'push' in ev else 0.0
        start = ev['t'] + dur
        stop = events[i + 1]['t'] if i + 1 < len(events) else spec['duration']
        win = (t >= start) & (t < stop)
        if not np.any(win):
            continue
        tw, bw = t[win], bad[win]
        if not np.any(bw):
            rec = 0.0
        elif bw[-1]:
            rec = math.inf            # never got back inside tolerance
        else:
            rec = float(tw[np.where(bw)[0][-1]] - start)
        out.append({'event_t': ev['t'], 'recovery_time': rec})
    return out


def evaluate(d: Dict[str, np.ndarray], spec: dict) -> dict:
    t = d['t']
    run = (t >= 0.0) & (t <= spec['duration'])
    final = run & (t >= spec['duration'] - spec['final_window'])
    e = errors(d)
    thrust = np.column_stack([d[f'u{i}'] for i in range(8)])
    deg = math.degrees
    m = {
        'horizontal_rms': _rms(e['horizontal'][run]),
        'horizontal_max': _max(e['horizontal'][run]),
        'final_horizontal_rms': _rms(e['horizontal'][final]),
        'depth_rms': _rms(e['depth'][run]),
        'depth_max': _max(e['depth'][run]),
        'yaw_rms_deg': deg(_rms(e['yaw'][run])),
        'yaw_max_deg': deg(_max(e['yaw'][run])),
        'final_yaw_max_deg': deg(_max(e['yaw'][final])),
        'tilt_max_deg': deg(_max(e['tilt'][run])),
        'nav_position_rms': _rms(e['nav_position'][run]),
        'nav_yaw_rms_deg': deg(_rms(e['nav_yaw'][run])),
        'saturation_fraction': float(np.mean(np.any(np.abs(thrust[run]) > SATURATION_N, axis=1)))
        if np.any(run) else float('nan'),
    }
    rec = recovery_times(d, spec)
    if rec:
        m['recovery_time'] = max(r['recovery_time'] for r in rec)

    failures = []
    for key, limit in spec.get('limits', {}).items():
        value = m.get(key)
        if value is None or not (value <= limit):     # NaN and inf fail too
            failures.append(f'{key} = {value:.3f} > {limit}' if value is not None
                            else f'{key} not measured')
    covered = float(t[run][-1] - t[run][0]) if np.count_nonzero(run) > 1 else 0.0
    if covered < 0.95 * spec['duration']:
        failures.append(f'only {covered:.1f} s of {spec["duration"]} s recorded')
    return {'metrics': m, 'recoveries': rec, 'limits': spec.get('limits', {}),
            'passed': not failures, 'failures': failures}


def plot(d: Dict[str, np.ndarray], spec: dict, title: str, path: str) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t = d['t']
    e = errors(d)
    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    fig.suptitle(f'{title}: {spec.get("description", "")}', fontsize=10)

    a = ax[0]
    a.plot(t, e['horizontal'], label='horizontal error (truth vs setpoint)', color='C0')
    a.plot(t, e['nav_position'], label='navigation error (estimate vs truth)', color='C1', lw=0.8)
    a.axhline(spec['recovery_tolerance'], color='0.6', ls=':', lw=0.8, label='recovery tolerance')
    a.set_ylabel('m')
    a.legend(loc='upper right', fontsize=8)

    a = ax[1]
    a.plot(t, d['sp_z'], 'k--', lw=0.9, label='setpoint z')
    a.plot(t, d['gt_z'], color='C0', label='true z')
    a.plot(t, d['nav_z'], color='C1', lw=0.8, label='estimated z')
    a.set_ylabel('z (m, up)')
    a.legend(loc='upper right', fontsize=8)

    a = ax[2]
    a.plot(t, np.degrees(wrap(d['sp_yaw'])), 'k--', lw=0.9, label='setpoint yaw')
    a.plot(t, np.degrees(d['gt_yaw']), color='C0', label='true yaw')
    a.plot(t, np.degrees(d['nav_yaw']), color='C1', lw=0.8, label='estimated yaw')
    a.plot(t, np.degrees(d['gt_roll']), color='C2', lw=0.8, label='roll')
    a.plot(t, np.degrees(d['gt_pitch']), color='C3', lw=0.8, label='pitch')
    a.set_ylabel('deg')
    a.legend(loc='upper right', fontsize=8, ncol=2)

    a = ax[3]
    for k, c in zip(('x', 'y', 'z'), ('C0', 'C1', 'C2')):
        a.plot(t, d[f'cur_{k}'], color=c, label=f'current {k} (m/s)')
    a.set_ylabel('m/s')
    b = a.twinx()
    for k, c in zip(('x', 'y', 'z'), ('C0', 'C1', 'C2')):
        b.plot(t, d[f'push_{k}'], color=c, ls='--', lw=0.9, label=f'push {k} (N)')
    thrust = np.max(np.abs(np.column_stack([d[f'u{i}'] for i in range(8)])), axis=1)
    b.plot(t, thrust, color='0.4', lw=0.7, label='max |thrust| (N)')
    b.set_ylabel('N')
    h1, l1 = a.get_legend_handles_labels()
    h2, l2 = b.get_legend_handles_labels()
    a.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=7, ncol=2)
    a.set_xlabel('time since disturbance phase start (s)')

    for a in ax:
        a.axvspan(t[0] if len(t) else 0, 0, color='0.92', zorder=0)
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def fmt(value: float, key: str) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return 'n/a'
    if math.isinf(value):
        return 'never'
    if key == 'saturation_fraction':
        return f'{100 * value:.1f} %'
    return f'{value:.3f}' if METRIC_UNITS.get(key) == 'm' else f'{value:.2f}'


def write_report(out_dir: str) -> bool:
    """Collect every <scenario>.json in out_dir into report.md; True if all passed."""
    results = {}
    for name in sorted(os.listdir(out_dir)):
        if name.endswith('.json') and name != 'summary.json':
            with open(os.path.join(out_dir, name)) as f:
                results[name[:-5]] = json.load(f)
    cols = ['horizontal_rms', 'horizontal_max', 'final_horizontal_rms', 'depth_max',
            'yaw_max_deg', 'tilt_max_deg', 'recovery_time', 'nav_position_rms',
            'saturation_fraction']
    lines = ['# Hover controller validation', '',
             'Errors are ground truth vs. setpoint over the disturbance phase. The controller '
             'runs on the navigation estimate only (IMU + magnetometer AHRS, DVL, pressure '
             'depth and USBL fused by the Kalman filter).', '',
             '| scenario | result | ' + ' | '.join(
                 f'{c} ({METRIC_UNITS[c]})' if METRIC_UNITS[c] else c for c in cols) + ' |',
             '|' + '---|' * (len(cols) + 2)]
    all_ok = bool(results)
    for name, r in results.items():
        m = r['metrics']
        ok = r['passed']
        all_ok &= ok
        lines.append(f'| {name} | {"PASS" if ok else "**FAIL**"} | '
                     + ' | '.join(fmt(m.get(c), c) for c in cols) + ' |')
    lines += ['', f'**Overall: {"PASS" if all_ok else "FAIL"}** ({len(results)} scenarios)', '']
    for name, r in results.items():
        lines += [f'## {name}', '', r.get('description', ''), '']
        if r['failures']:
            lines += ['Failed limits:', ''] + [f'- {x}' for x in r['failures']] + ['']
        lines += ['| metric | value | limit |', '|---|---|---|']
        for key, value in r['metrics'].items():
            lim = r['limits'].get(key)
            lines.append(f'| {key} | {fmt(value, key)} | {"" if lim is None else lim} |')
        lines += ['', f'![{name}]({name}.png)', '']
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write('\n'.join(lines))
    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump({'passed': all_ok, 'scenarios': {k: v['passed'] for k, v in results.items()}},
                  f, indent=2)
    return all_ok


if __name__ == '__main__':
    sys.exit(0 if write_report(sys.argv[1]) else 1)
