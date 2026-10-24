#!/usr/bin/env python3
"""Rebuild the upstream file coverage index from the fixed checkout and matrix."""
import argparse
import csv
import io
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = ROOT / 'external/z_localization'
PIN = '76e126328b670adc92981d9b4900cc059a6f2343'


def owners(path):
    if path.startswith('src/livox_ros_driver2/'):
        return 'L09,S02'
    if '/test/' in path:
        return 'T02' if '/evaluation/' in path else 'T01'
    if path in ('LICENSE', 'README.md') or path.startswith('src/localization_interfaces/LICENSE'):
        return 'S01'
    if path.startswith('docs/assets/experiments/'):
        return 'T02'
    if path.startswith(('config/', 'docs/', 'src/frontend/docs/', 'src/backend/docs/')) or path.endswith('README.md'):
        return 'T03'
    if 'CMakeLists.txt' in path or path.startswith(('cmake/', '.')) or '/cmake/' in path:
        return 'S02'
    if path.startswith('src/localization_interfaces/'):
        return 'F10,F11,F13'
    if '/launch/' in path:
        return {'lio.launch.py': 'L01', 'frontend.launch.py': 'L02',
                'bringup.launch.py': 'L03', 'backend.launch.py': 'L04',
                'body_calibration.launch.py': 'L05'}[Path(path).name]
    if '/rviz/' in path:
        return 'L06'
    if path.endswith('package.xml'):
        return 'S02'
    if path.startswith('src/bringup/'):
        return 'L03'
    name = Path(path).stem
    if path.startswith('src/frontend/'):
        if '/apps/' in path:
            return 'L02' if name == 'visualization' else 'F07,F08,F09,F10,F11,F12,F13'
        if name in ('initialization', 'window', 'bootstrap', 'uncertainty', 'preparation'):
            return 'F02,F05'
        if name in ('input', 'sampling', 'execution'):
            return 'F06'
        if name in ('trajectory', 'prediction', 'preintegration', 'inertial'):
            return 'F03'
        if name in ('map', 'plane', 'voxel', 'moments', 'plane_factor', 'registration', 'likelihood', 'storage'):
            return 'F04'
        return 'F01'
    if path.startswith('src/backend/tools/'):
        if '/cleanup/' in path or name == 'cleanup':
            return 'M02'
        return {'filter': 'M03,M04', 'transform': 'M05,M06', 'alignment': 'M06',
                'build': 'M07,M08', 'generate': 'M07,M12', 'describe': 'M09',
                'calibrate': 'L05', 'body_calibration': 'L05,B18',
                'bundle': 'M01,M08,M10', 'arguments': 'T03'}[name]
    if path.startswith('src/backend/apps/'):
        if name == 'publisher_state': return 'D01,D02,D06'
        if name == 'publisher_view': return 'D04'
        if name == 'publisher_legacy': return 'D04,D05'
        if name == 'publisher': return 'B03,B04,D04'
        if name in ('frontend_connection', 'frontend_session', 'reset_request'): return 'F13,F14,F15,B02'
        if name == 'node_input': return 'B02'
        if name == 'node_map': return 'B09,M01'
        if name in ('main', 'node'): return 'S03,B01,B02'
        if name in ('parameters', 'fusion_parameters', 'config'): return 'T03'
        if name == 'logger': return 'D02'
        return 'B01,B02,L04'
    if path.startswith(('src/backend/src/', 'src/backend/include/')):
        if '/gprobe/' in path or name == 'gprobe': return 'B14'
        if '/scan_context/' in path or name == 'scan_context': return 'B13'
        if name.startswith('uwb') or name in ('graph_uwb', 'graph_calibration'): return 'B16,B17'
        if name.startswith('loop'): return 'B15'
        if name.startswith('relocalization') or name == 'search': return 'B11'
        if name in ('recovery_matcher', 'localization_recovery', 'localization_state'): return 'B12'
        if name.startswith('registration') or name in ('tracking_matcher', 'target_cache', 'observation', 'matching'): return 'B08'
        if name in ('graph_innovation', 'pose_conditioning', 'quality'): return 'B07'
        if name in ('graph_noise', 'covariance', 'measurement', 'noise'): return 'B06'
        if name in ('cloud_storage', 'directory', 'map_bundle'): return 'M01,M10'
        if name == 'body': return 'B18'
        if name in ('localization_trajectory', 'keyframe_map'): return 'M01,B09'
        if name in ('reset_coordinator', 'ports'): return 'F13,F14'
        if name in ('localization_output', 'pose_output', 'frame_history'): return 'B03,D01'
        if name == 'logger': return 'D02'
        if name == 'config': return 'T03,B19'
        return 'B05'
    raise ValueError('Unclassified upstream file: ' + path)


def inventory():
    head = subprocess.check_output(['git', '-C', str(UPSTREAM), 'rev-parse', 'HEAD'], text=True).strip()
    assert head == PIN, f'expected {PIN}, got {head}'
    rows = list(csv.DictReader((ROOT / 'docs/zlocalization_upstream_parity_matrix.tsv').open(), delimiter='\t'))
    ids = {r['id'] for r in rows}
    paths = subprocess.check_output(['git', '-C', str(UPSTREAM), 'ls-tree', '-r', '--name-only', PIN], text=True).splitlines()
    out = io.StringIO()
    writer = csv.writer(out, delimiter='\t', lineterminator='\n')
    writer.writerow(('upstream_path', 'matrix_ids'))
    for path in paths:
        linked = owners(path)
        assert set(linked.split(',')) <= ids
        writer.writerow((path, linked))
    return out.getvalue(), len(paths)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    content, count = inventory()
    target = ROOT / 'docs/zlocalization_upstream_file_inventory.tsv'
    if args.check:
        assert target.read_text() == content, 'upstream coverage index needs regeneration'
    else:
        target.write_text(content)
    print(f'{count} upstream tracked files mapped to migration matrix; pin verified')


if __name__ == '__main__':
    main()
