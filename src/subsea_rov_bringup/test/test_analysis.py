import math
import os

import numpy as np
import pytest

from subsea_rov_bringup import analysis
from subsea_rov_bringup.scenario_runner import load_scenario

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')

SPEC = {'duration': 30.0, 'final_window': 10.0, 'recovery_tolerance': 0.1,
        'events': [{'t': 10.0, 'push': {'force': [0, 40, 0], 'duration': 1.0}, 'recover': True}],
        'limits': {'horizontal_max': 0.5, 'recovery_time': 8.0}}


def synthetic(offset_fn, n=601):
    t = np.linspace(-5.0, 30.0, n)
    d = {c: np.zeros(n) for c in analysis.COLUMNS}
    d['t'] = t
    d['sp_z'] = d['gt_z'] = d['nav_z'] = np.full(n, 5.0)
    d['gt_y'] = np.array([offset_fn(x) for x in t])
    d['nav_y'] = d['gt_y'] + 0.01
    return d


def push_response(t):
    # 0.3 m excursion after the push ending at t = 11, decaying to 0 by t = 16
    if 11.0 <= t < 16.0:
        return 0.3 * (1.0 - (t - 11.0) / 5.0)
    return 0.0


def test_metrics_and_recovery_time():
    r = analysis.evaluate(synthetic(push_response), SPEC)
    m = r['metrics']
    assert m['horizontal_max'] == pytest.approx(0.3, abs=0.01)
    # Back inside 0.1 m at t = 11 + 5 * (2/3) = 14.33 s, i.e. 3.33 s after the push.
    assert m['recovery_time'] == pytest.approx(3.33, abs=0.1)
    assert m['nav_position_rms'] == pytest.approx(0.01, abs=1e-6)
    assert r['passed']


def test_never_recovering_fails():
    r = analysis.evaluate(synthetic(lambda t: 0.3 if t > 11 else 0.0), SPEC)
    assert math.isinf(r['metrics']['recovery_time'])
    assert not r['passed']


def test_short_recording_fails():
    d = synthetic(lambda t: 0.0)
    keep = d['t'] < 10.0
    r = analysis.evaluate({k: v[keep] for k, v in d.items()}, SPEC)
    assert not r['passed']
    assert any('recorded' in f for f in r['failures'])


def test_yaw_error_wraps():
    d = synthetic(lambda t: 0.0)
    d['gt_yaw'][:] = math.radians(179.0)
    d['sp_yaw'][:] = math.radians(-179.0)
    m = analysis.evaluate(d, SPEC)['metrics']
    assert m['yaw_max_deg'] == pytest.approx(2.0, abs=1e-6)


def test_csv_roundtrip_and_report(tmp_path):
    d = synthetic(push_response)
    rows = np.column_stack([d[c] for c in analysis.COLUMNS]).tolist()
    analysis.write_csv(tmp_path / 'demo.csv', rows)
    back = analysis.load_csv(tmp_path / 'demo.csv')
    np.testing.assert_allclose(back['gt_y'], d['gt_y'])
    import json
    result = analysis.evaluate(back, SPEC)
    (tmp_path / 'demo.json').write_text(json.dumps(result))
    assert analysis.write_report(str(tmp_path))
    assert 'demo' in (tmp_path / 'report.md').read_text()


def test_every_shipped_scenario_loads():
    import yaml
    with open(os.path.join(CONFIG, 'scenarios.yaml')) as f:
        names = list(yaml.safe_load(f)['scenarios'])
    assert len(names) >= 5
    known_metrics = set(analysis.METRIC_UNITS)
    for name in names:
        spec = load_scenario(os.path.join(CONFIG, 'scenarios.yaml'), name)
        assert spec['duration'] > 0 and spec['settle'] > 0
        assert set(spec.get('limits', {})) <= known_metrics, name
        for ev in spec['events']:
            assert {'current', 'wave', 'push', 'setpoint'} & set(ev), (name, ev)
