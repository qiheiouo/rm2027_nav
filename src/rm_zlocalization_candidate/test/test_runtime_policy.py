"""Meaningful configuration/owner regression, using the actual upstream loader."""
import importlib.util
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))
from runtime_policy import backend_parameters, native_parameters


@pytest.fixture
def backend_share(tmp_path):
    upstream = ROOT / 'external/z_localization'
    (tmp_path / 'launch').mkdir()
    (tmp_path / 'config').mkdir()
    (tmp_path / 'launch/backend.launch.py').symlink_to(upstream / 'src/backend/launch/backend.launch.py')
    (tmp_path / 'config/backend.yaml').symlink_to(upstream / 'config/backend.yaml')
    return tmp_path


def parameters(share, tmp_path, **kwargs):
    overlay = tmp_path / 'overlay.yaml'
    overlay.write_text(yaml.safe_dump({'uwb': {'enabled': True},
        'common': {'body': {'frame': 'base_link'}},
        'output': {'legacy': {'enabled': True, 'publish_tf': True}}}))
    return backend_parameters(share, overlay, **kwargs)


def test_overlays_cannot_override_owners_and_native_input(backend_share, tmp_path):
    p = parameters(backend_share, tmp_path, mode='mapping', frontend='zlio')
    assert p['common']['input_cloud_in_local_frame'] is True
    assert p['common']['odom_frame'] == 'zlio_odom'
    assert p['common']['body']['frame'] == 'lio_imu_link'
    assert not p['uwb']['enabled'] and not p['output']['legacy']['enabled']
    assert not p['trajectory']['save_on_shutdown']
    assert p['reset']['service'] == '/zlocalization/reset_internal'
    assert p['common']['prediction_topic'].endswith('/unused_native_prediction')


def test_gprobe_loop_and_display_are_explicit(backend_share, tmp_path):
    p = parameters(backend_share, tmp_path, mode='mapping', shadow=True,
                   descriptor='gprobe', loop=True, visualization=True, world_cloud=True)
    assert p['loop']['enabled'] and p['common']['descriptor_type'] == 'gprobe'
    assert 'scan_context' not in p['common']
    assert p['visualization']['publish_runtime_map'] and p['visualization']['publish_paths']
    assert p['output']['world_cloud']['topic'] == '/zlocalization/shadow/world_cloud'
    assert p['services']['save_map'] == '/zlocalization/shadow/save_map'


def test_prior_and_loop_have_exclusive_correction_ownership(backend_share, tmp_path):
    with pytest.raises(ValueError, match='loop closure'):
        parameters(backend_share, tmp_path, mode='localization', prior_map=str(tmp_path), loop=True)
    with pytest.raises(ValueError, match='prior-map'):
        parameters(backend_share, tmp_path, mode='localization', prior_map='')
    p = parameters(backend_share, tmp_path, mode='localization', prior_map=str(tmp_path))
    assert p['prior_map']['enabled']


def test_dotted_keys_cannot_bypass_constraints(backend_share, tmp_path):
    overlay = tmp_path / 'dotted.yaml'
    overlay.write_text('output.legacy.enabled: true\n')
    with pytest.raises(ValueError, match='nested YAML'):
        backend_parameters(backend_share, overlay, mode='mapping')


@pytest.mark.parametrize('scale', ['', 'nan', 'inf', '-1', '0'])
def test_native_unit_scale_requires_explicit_finite_value(tmp_path, scale):
    config = tmp_path / 'lio.yaml'
    config.write_text('{}')
    with pytest.raises(ValueError):
        native_parameters(str(config), scale)


def test_native_no_tf_and_measurement_time_controls(tmp_path):
    config = tmp_path / 'lio.yaml'
    config.write_text('{}')
    p = native_parameters(str(config), '1', input_type='pointcloud2',
                          point_time_field='timestamp', point_time_scale='1e-9',
                          point_time_absolute=True)
    assert not p['publish_tf'] and p['coordinated_reset']
    assert p['point_time_field'] == 'timestamp' and p['point_time_absolute']
    assert p['world_frame'] == 'zlio_odom' and p['body_frame'] == 'lio_imu_link'


@pytest.mark.parametrize('name', ['z_localization_candidate.launch.py', 'zlio_shadow.launch.py'])
def test_disabled_entry_needs_no_upstream_install(monkeypatch, name):
    from launch import LaunchContext
    path = Path(__file__).resolve().parents[1] / 'launch' / name
    spec = importlib.util.spec_from_file_location('entry_under_test', path)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    monkeypatch.setattr(entry, 'share', lambda _: pytest.fail('disabled entry resolved optional package'))
    context = LaunchContext()
    context.launch_configurations['enabled'] = 'false'
    assert entry.setup(context) == []
