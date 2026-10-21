"""Check saved measured execution evidence; never infer GUI visibility from headless data."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args()
summary=json.loads((args.run/'summary.json').read_text())
assert summary['success'], 'Run reported failure'
for item in summary['algorithms']:
    alg=item['algorithm'];route=json.loads((args.run/f'{alg}.json').read_text());execution=route['execution']
    assert execution['success']
    files=list((args.run/alg).rglob('*telemetry.csv'))
    assert len(files)==1, f'Expected one measured telemetry file, got {files}'
    with files[0].open() as f:rows=list(csv.DictReader(f))
    points=np.array([[float(r[k]) for k in ('x_enu_m','y_enu_m','z_enu_m')] for r in rows])
    assert np.isfinite(points).all() and len(points)>2
    traveled=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
    assert traveled>0.1
    assert abs(traveled-execution['actual_traveled_m'])<1e-7
    if execution['mode']=='replay':
        np.testing.assert_allclose(points[0],route['start'],atol=.015)
        np.testing.assert_allclose(points[-1],route['goal'],atol=.015)
        assert execution['px4_flight'] is False
    else:
        assert execution['route_completed'] and execution['status']=='LANDED_AFTER_ROUTE'
        commands={a['command'] for a in execution['acknowledgements'] if a['result']==0}
        assert {176,400,21} <= commands
        assert rows[-1]['armed']=='False'
        assert any(r['armed']=='True' for r in rows)
        assert execution.get('goal_error_m',0)<.30
    screenshots=list((args.run/alg).rglob('*.png'))
    print(f'{alg}: PASS, {len(rows)} simulator samples, {traveled:.3f} m actual total; '
          f'{len(screenshots)} window captures available (require visual inspection)')
