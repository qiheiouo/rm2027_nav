"""Prespecified 36 episodes using the frozen pre-acquired reserve lifecycle."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
from reserve_run import trial, FIXTURE
from world import scene

ROOT = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--build', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--confirm', nargs=3, metavar=('SCENE', 'SEED', 'ARM'))
    args = ap.parse_args()
    protocol = json.loads((ROOT/'preacquired_pilot_protocol.json').read_text())
    configs = protocol['scenario_configs']
    assert len(configs) == 12 and protocol['episodes'] == 36
    assert scene('head_on', 13) == FIXTURE['scene']
    for cfg in configs:
        assert cfg == scene(cfg['name'], cfg['seed'])
    jobs = [(cfg, arm) for cfg in configs for arm in protocol['arms']]
    if args.confirm:
        s, n, a = args.confirm
        jobs = [(cfg, arm) for cfg, arm in jobs if (cfg['name'], cfg['seed'], arm) == (s, int(n), a)]
        assert len(jobs) == 1
    args.output.mkdir(parents=True, exist_ok=False)
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    # Snapshot all scenarios before the first measured episode, including confirmations.
    (args.output/'protocol.json').write_text(json.dumps(protocol, indent=2)+'\n')
    environment = dict(platform=platform.platform(), python=platform.python_version(),
                       affinity=sorted(os.sched_getaffinity(0)),
                       compiler=subprocess.check_output(['g++', '--version'], text=True).splitlines()[0],
                       git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip())
    (args.output/'environment.json').write_text(json.dumps(environment, indent=2)+'\n')
    results = []
    for cfg, arm in jobs:
        name = f"{cfg['name']}_{cfg['seed']}_{arm}"
        out = args.output/name
        out.mkdir()
        print(name, flush=True)
        r = trial(args.build, out, arm, 'normal', full_episode=True, cfg=cfg)
        r.update(id=name, scene=cfg['name'], seed=cfg['seed'], config=cfg)
        results.append(r)
        (args.output/'results.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
