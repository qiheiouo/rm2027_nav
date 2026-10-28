#!/usr/bin/env python3
"""Check the installed Jazzy simulation profile without starting ROS or a GUI."""
from collections import Counter
import importlib.util
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_packages_with_prefixes, get_package_share_directory
from catkin_pkg.package import parse_package

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('rm_description', 'rm_nav_config', 'rm_new_car_simulation',
            'rm_navigation_bringup', 'rm_navigation_launch',
            'rm_competition_interfaces', 'rm_competition_mission', 'rm_pursuit',
            'rm_localization_adapters', 'rm_relocalization_bridge')
ASSETS = {
    'rm_description': ['urdf/rm_sentry_2027.urdf.xacro'],
    'rm_nav_config': ['config/nav2_phase1_5_mppi.yaml', 'behavior_trees/event_driven_replanning.xml'],
    'rm_competition_interfaces': ['msg/PursuitCandidate.msg', 'msg/LioSourceStatus.msg'],
    'rm_localization_adapters': ['config/lio_adapter.yaml'],
    'rm_relocalization_bridge': ['config/map_odom_from_global_pose.yaml'],
    'rm_competition_mission': ['trees/competition_default.xml'],
    'rm_pursuit': ['launch/pursuit.launch.py'],
    'rm_navigation_bringup': ['launch/simulation.launch.py'],
    'rm_navigation_launch': ['launch/field_geometry_simulation.launch.py'],
    'rm_new_car_simulation': ['launch/baseline.launch.py', 'config/baseline.yaml',
                              'config/new_car_cad_baseline.yaml', 'config/simulation.rviz'],
}


def check():
    import os
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise RuntimeError('Source the Jazzy simulation install/setup.bash first')
    with tempfile.TemporaryDirectory(prefix='rm2027-package-check-') as logs:
        rows = subprocess.check_output(
            ['colcon', '--log-base', logs, 'list', '--topological-order'],
            cwd=ROOT, text=True).splitlines()
    parsed = [line.split('\t') for line in rows if line.strip()]
    duplicates = [name for name, count in Counter(row[0] for row in parsed).items() if count > 1]
    if duplicates:
        raise RuntimeError(f'Duplicate packages: {duplicates}')
    sources = {}
    for name, path, kind in parsed:
        source = (ROOT / path).resolve()
        if not source.is_relative_to(ROOT / 'src'):
            raise RuntimeError(f'Package outside src/: {name}: {path}')
        sources[name] = source
    installed = get_packages_with_prefixes()
    for name in PACKAGES:
        source = sources.get(name)
        if source is None:
            raise RuntimeError(f'Missing source package: {name}')
        package = parse_package(str(source / 'package.xml'))
        if package.name != name:
            raise RuntimeError(f'package.xml name mismatch: {name}')
        cmake = source / 'CMakeLists.txt'
        if cmake.exists() and not re.search(r'project\(\s*' + re.escape(name) + r'\s*\)', cmake.read_text()):
            raise RuntimeError(f'CMake project mismatch: {name}')
        expected = ROOT / 'install' / name
        if name not in installed or Path(installed[name]).resolve() != expected.resolve():
            raise RuntimeError(f'{name}: expected prefix {expected}, got {installed.get(name)}')
        package.evaluate_conditions(os.environ)
        for dependency in package.build_depends + package.buildtool_depends:
            if dependency.evaluated_condition is False:
                continue
            if dependency.name not in installed:
                raise RuntimeError(f'{name}: missing build dependency {dependency.name}')
        share = Path(get_package_share_directory(name))
        for asset in ASSETS[name]:
            if not (share / asset).is_file():
                raise RuntimeError(f'Missing installed asset: {share / asset}')
    # Check the executable profile's runtime closure. The two launch bundles
    # also contain unselected Humble hardware compositions, intentionally not built.
    system_modules = {'python3-numpy': 'numpy', 'python3-yaml': 'yaml', 'python3-scipy': 'scipy'}
    for name in ('rm_new_car_simulation', 'rm_nav_config', 'rm_description'):
        package = parse_package(str(sources[name] / 'package.xml'))
        for dependency in package.exec_depends:
            key = dependency.name
            if key in system_modules:
                if importlib.util.find_spec(system_modules[key]) is None:
                    raise RuntimeError(f'Missing Python dependency: {key}')
            elif key not in installed:
                raise RuntimeError(f'{name}: missing runtime dependency {key}')
    prefix = Path(installed['rm_new_car_simulation'])
    for path in ('lib/librm_omni_wheel_drive.so', 'lib/rm_new_car_simulation/simulation_tf.py',
                 'lib/rm_new_car_simulation/baseline_scene.py', 'lib/rm_new_car_simulation/smoke_baseline.py'):
        if not (prefix / path).is_file():
            raise RuntimeError(f'Missing installed executable: {prefix / path}')
    # Maps are generated from the installed scene source at launch, not copied
    # from another workspace. Check the generation contract without starting Gazebo.
    script = prefix / 'lib/rm_new_car_simulation/baseline_scene.py'
    spec = importlib.util.spec_from_file_location('simulation_scene_check', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    share = Path(get_package_share_directory('rm_new_car_simulation'))
    with tempfile.TemporaryDirectory(prefix='rm2027-scene-check-') as tmp:
        output = Path(tmp)
        manifest = module.generate(share, output, field='rules2026_course', spawn=[-3, -3, .002, 0, 0, 0])
        for name in ('world.sdf', 'robot.urdf', 'field.yaml', 'field.pgm', 'field.pcd', 'manifest.json'):
            if not (output / name).is_file() or (output / name).stat().st_size == 0:
                raise RuntimeError(f'Missing generated scene asset: {name}')
        world = ET.parse(output / 'world.sdf')
        if world.find(".//model[@name='rules2026_course']") is None:
            raise RuntimeError('Static course missing from generated world')
        print(f'Scene: {manifest["revision"]}; world / URDF / occupancy / PCD generated')
    print(f'PASS: {len(sources)} source packages, no duplicates; {len(PACKAGES)} expected install prefixes; '
          'launch/assets/build dependencies and simulation runtime dependencies present')
    print('Scope: Jazzy simulation only; legacy Humble/hardware launches are not runtime-validated')


if __name__ == '__main__':
    try:
        check()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        raise SystemExit(f'FAIL: {error}')
