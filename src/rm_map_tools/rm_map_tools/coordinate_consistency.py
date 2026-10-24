"""Offline Z prior / Nav2 raster consistency. Never grants map or hardware approval."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .map_bundle import MapBundleError
from .map_quality import (load_occupancy_image, world_to_pgm, pgm_cell_centers_world,
                          euclidean_distance_to_mask, sha256_file)


def _vector(value, size, label):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise MapBundleError(f'{label}: expected {size} finite numbers') from exc
    if result.shape != (size,) or not np.isfinite(result).all():
        raise MapBundleError(f'{label}: expected {size} finite numbers')
    return result


def _transform(value, label):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise MapBundleError(f'{label}: expected rigid 4x4 matrix') from exc
    if (result.shape != (4, 4) or not np.isfinite(result).all() or
            not np.allclose(result[3], [0, 0, 0, 1], atol=1e-7, rtol=0) or
            not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-6, rtol=0) or
            abs(np.linalg.det(result[:3, :3])-1.) > 1e-6):
        raise MapBundleError(f'{label}: invalid SE(3); scale/reflection/nonrigid matrix forbidden')
    return result


def _yaml(path):
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise MapBundleError(f'{path}: YAML root must be a mapping')
    return value


def read_xyz_pcd(path):
    """PCL ASCII / uncompressed little-endian binary, including extra/count fields."""
    with Path(path).open('rb') as stream:
        header = {}
        for line in stream:
            text = line.decode('ascii').strip()
            if not text or text.startswith('#'):
                continue
            key, *values = text.split()
            header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        mode = header['DATA'][0].lower()
        if mode not in ('ascii', 'binary'):
            raise MapBundleError(f'PCD DATA {mode}: convert with PCL to binary/ASCII and rebind hashes')
        fields = header['FIELDS']
        if len(set(fields)) != len(fields) or not {'x', 'y', 'z'} <= set(fields):
            raise MapBundleError('PCD FIELDS must be unique and include x/y/z')
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1']*len(fields))))
        if not len(fields) == len(sizes) == len(types) == len(counts) or min(counts) < 1:
            raise MapBundleError('PCD field SIZE/TYPE/COUNT lengths or counts invalid')
        dtype = []
        for i, (size, kind, count) in enumerate(zip(sizes, types, counts)):
            if (kind == 'F' and size in (4, 8)) or (kind in ('I', 'U') and size in (1, 2, 4, 8)):
                dtype.append((f'f{i}', f'<{kind.lower()}{size}', (count,)))
            else:
                raise MapBundleError('unsupported PCD SIZE/TYPE')
        indexes = [fields.index(axis) for axis in ('x', 'y', 'z')]
        if any(counts[i] != 1 for i in indexes):
            raise MapBundleError('PCD XYZ must be scalar')
        points = int(header['POINTS'][0])
        if points <= 0 or int(header['WIDTH'][0])*int(header['HEIGHT'][0]) != points:
            raise MapBundleError('PCD POINTS / WIDTH / HEIGHT inconsistent or empty')
        payload = stream.read()
    if mode == 'binary':
        dt = np.dtype(dtype)
        if len(payload) != points*dt.itemsize:
            raise MapBundleError('PCD binary payload length does not match POINTS/fields')
        records = np.frombuffer(payload, dtype=dt, count=points)
        xyz = np.column_stack([records[f'f{i}'][:, 0] for i in indexes]).astype(float)
    else:
        values = np.asarray(payload.decode('ascii').split(), dtype=float)
        if len(values) != points*sum(counts):
            raise MapBundleError('PCD ASCII payload length does not match POINTS/fields')
        offsets = np.cumsum([0]+counts[:-1])
        xyz = values.reshape(points, sum(counts))[:, [offsets[i] for i in indexes]]
    if not np.isfinite(xyz).all():
        raise MapBundleError('PCD XYZ contains nonfinite values')
    return xyz, {'points': points, 'data_mode': mode, 'fields': fields}


def _apply(transform, points):
    return points @ transform[:3, :3].T + transform[:3, 3]


def check_coordinates(prior_pcd, map_yaml, transform_metadata, occupancy_image=None):
    report = {'schema_version': 1, 'software_consistency_pass': False,
              'real_map_accepted': False, 'hardware_go': False, 'checks': {}, 'diagnostics': []}

    def check(name, passed, reason, **metrics):
        report['checks'][name] = {'passed': bool(passed), **metrics}
        if not passed:
            report['diagnostics'].append({'code': name, 'reason': reason})

    try:
        m = _yaml(transform_metadata)
        report['dataset_kind'] = m['dataset_kind']
        if m['schema_version'] != 1 or m['dataset_kind'] not in ('synthetic', 'new_car_real'):
            raise MapBundleError('unsupported metadata schema_version/dataset_kind')
        if not all(isinstance(m[key], str) and m[key].strip() for key in ('map_id', 'revision')):
            raise MapBundleError('metadata map_id/revision required')
        report.update(map_id=m['map_id'], revision=m['revision'])
        if m['units'] != 'm' or m['axes'] != 'right_handed_z_up':
            raise MapBundleError('units must be m and axes right_handed_z_up')
        if m['pixel_convention'] != 'top_left_rows_down_cell_centers':
            raise MapBundleError('pixel_convention mismatch; do not silently flip/shift raster')
        frames = m['frames']
        if not isinstance(frames, dict) or not all(isinstance(frames[k], str) and frames[k] for k in ('mapping', 'prior', 'nav2')):
            raise MapBundleError('frames.mapping/prior/nav2 must be explicit')
        check('FRAME_MISMATCH', frames['prior'] == frames['nav2'] == 'map',
              'Current runtime requires prior=Nav2=map; bake transform into entire bundle first', frames=frames)
        t_pm = _transform(m['T_prior_mapping'], 'T_prior_mapping')
        t_np = _transform(m['T_map_prior'], 'T_map_prior')
        check('RUNTIME_TRANSFORM_NOT_BAKED', np.allclose(t_np, np.eye(4), atol=1e-7, rtol=0),
              'T_map_prior is nonidentity; transform whole bundle, rebuild descriptors, recheck')
        check('TRANSFORMS_SE3', True, '')
        occupancy = load_occupancy_image(map_yaml)
        raster = _yaml(map_yaml)
        if occupancy_image is not None and Path(occupancy_image).resolve() != occupancy.image_path:
            raise MapBundleError('explicit occupancy image differs from map.yaml image reference')
        check('ARTIFACT_IDENTITY', all(
            m['artifacts'][key] == sha256_file(path) for key, path in
            [('prior_pcd_sha256', prior_pcd), ('map_yaml_sha256', map_yaml),
             ('occupancy_image_sha256', occupancy.image_path)]),
            'Metadata hashes do not bind these exact prior / YAML / image files')
        expected = _vector(m['expected_grid_origin'], 3, 'expected_grid_origin')
        origin = np.array(occupancy.origin)
        xy_error = float(np.linalg.norm(expected[:2]-origin[:2]))
        yaw_error = abs(math.atan2(math.sin(expected[2]-origin[2]), math.cos(expected[2]-origin[2])))
        resolution = float(m['expected_resolution'])
        if not math.isfinite(resolution) or resolution <= 0:
            raise MapBundleError('expected_resolution must be positive finite')
        check('ORIGIN_ORIENTATION', xy_error <= 1e-6 and yaw_error <= 1e-6 and
              abs(resolution-occupancy.resolution) <= 1e-9,
              'YAML origin/yaw/resolution differs from independent raster generation metadata',
              xy_error_m=xy_error, yaw_error_rad=yaw_error, yaml_origin=origin.tolist())
        xyz, pcd_info = read_xyz_pcd(prior_pcd)
        mapped = _apply(t_np, xyz)
        report['pcd'] = pcd_info
        band = _vector(m['occupancy_z_band_m'], 2, 'occupancy_z_band_m')
        if band[0] >= band[1]:
            raise MapBundleError('occupancy_z_band_m must be ordered [min,max) in map coordinates')
        cloud = mapped[(mapped[:, 2] >= band[0]) & (mapped[:, 2] < band[1])]
        if not len(cloud):
            raise MapBundleError('no PCD points in map occupancy height band')
        acceptance = m['acceptance']
        ratio_keys = ('min_points_inside_ratio', 'min_bounds_overlap_ratio', 'min_sample_occupied_ratio')
        for key in ratio_keys:
            value = float(acceptance[key])
            if not math.isfinite(value) or not 0 < value <= 1:
                raise MapBundleError(f'{key} must be in (0,1]')
        distances = ('max_landmark_error_m', 'max_pcd_support_error_m', 'sample_max_distance_m')
        for key in distances:
            if not math.isfinite(float(acceptance[key])) or float(acceptance[key]) < 0:
                raise MapBundleError(f'{key} must be finite and nonnegative')
        min_landmarks = int(acceptance['min_landmarks'])
        if min_landmarks < 3:
            raise MapBundleError('at least three noncollinear landmarks required')
        rows, cols, inside = world_to_pgm(occupancy, cloud[:, 0], cloud[:, 1])
        # Work in raster-local coordinates; AABB in map coordinates is wrong for yawed grids.
        angle = occupancy.origin[2]
        rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        local = (cloud[:, :2]-origin[:2]) @ rot
        low, high = local.min(axis=0), local.max(axis=0)
        extent = np.array([occupancy.width, occupancy.height])*occupancy.resolution
        intersection = np.maximum(0., np.minimum(high, extent)-np.maximum(low, 0.))
        area = float(np.prod(high-low))
        overlap = float(np.prod(intersection)/area) if area > 0 else 0.
        inside_ratio = float(inside.mean())
        check('BOUNDS_OVERLAP', inside_ratio >= float(acceptance['min_points_inside_ratio']) and
              overlap >= float(acceptance['min_bounds_overlap_ratio']),
              'Insufficient prior/raster overlap; check units, crop, origin, yaw and transform direction',
              points_inside_ratio=inside_ratio, prior_bbox_overlap_ratio=overlap,
              prior_bounds_map=[mapped.min(axis=0).tolist(), mapped.max(axis=0).tolist()],
              prior_bounds_grid=[low.tolist(), high.tolist()], grid_extent_m=extent.tolist())
        # Match Nav2 Humble strict threshold comparisons, including exact-threshold unknown cells.
        probability = occupancy.pixels.astype(float)/255.
        if not int(raster.get('negate', 0)):
            probability = 1.-probability
        occupied = probability > float(raster['occupied_thresh'])
        free = probability < float(raster['free_thresh'])
        unknown = ~(occupied | free)
        distance = euclidean_distance_to_mask(occupied, occupancy.resolution)
        sample_index = np.linspace(0, len(cloud)-1, min(len(cloud), 10000), dtype=int)
        supported = np.zeros(len(sample_index), dtype=bool)
        ok = inside[sample_index]
        supported[ok] = distance[rows[sample_index][ok], cols[sample_index][ok]] <= float(acceptance['sample_max_distance_m'])
        sample_ratio = float(supported.mean())
        check('SAMPLE_CORRESPONDENCE', sample_ratio >= float(acceptance['min_sample_occupied_ratio']),
              'Height-band prior samples do not correspond to occupied raster cells within declared tolerance',
              deterministic_samples=len(sample_index), occupied_support_ratio=sample_ratio)
        landmarks = m['landmarks']
        if not isinstance(landmarks, list) or len(landmarks) < min_landmarks:
            raise MapBundleError('insufficient independent landmarks')
        xy = np.array([_vector(item['map_xy'], 2, 'landmark map_xy') for item in landmarks])
        if np.linalg.matrix_rank(xy-xy[0], tol=1e-6) < 2:
            raise MapBundleError('landmarks are collinear; orientation cannot be accepted')
        if not isinstance(m['landmark_source'], str) or not m['landmark_source'].strip():
            raise MapBundleError('landmark_source required; numeric self-consistency is not independent evidence')
        check('LANDMARK_INDEPENDENCE', m['dataset_kind'] == 'synthetic' or m['landmarks_independent'] is True,
              'Real map needs independently measured landmark coordinates; do not generate labels from tested transform',
              source=m['landmark_source'], synthetic_only=m['dataset_kind'] == 'synthetic',
              real_independent_evidence=m['dataset_kind'] == 'new_car_real' and m['landmarks_independent'] is True)
        wx, wy = pgm_cell_centers_world(occupancy)
        ids = set()
        for item in landmarks:
            name = item['id']
            if not isinstance(name, str) or not name or name in ids:
                raise MapBundleError('landmark IDs must be nonempty and unique')
            ids.add(name)
            prior = _vector(item['prior_xyz'], 3, f'{name}.prior_xyz')
            target = _vector(item['map_xy'], 2, f'{name}.map_xy')
            pixel = _vector(item['pixel'], 2, f'{name}.pixel')
            if np.any(pixel != np.floor(pixel)):
                raise MapBundleError('landmark pixel must be integer [col,row]')
            u, v = map(int, pixel)
            point = _apply(t_np, prior)
            geometric_error = float(np.linalg.norm(point[:2]-target))
            support_error = float(np.linalg.norm(xyz-prior, axis=1).min())
            pixel_inside = 0 <= u < occupancy.width and 0 <= v < occupancy.height
            pixel_error = float(np.linalg.norm(np.array([wx[v,u],wy[v,u]])-target)) if pixel_inside else None
            label = item['occupancy']
            if label not in ('occupied', 'free', 'unknown'):
                raise MapBundleError('landmark occupancy must be occupied/free/unknown')
            pixel_class_ok = pixel_inside and {'occupied': occupied, 'free': free, 'unknown': unknown}[label][v,u]
            mapping_error = None
            if 'mapping_xyz' in item:
                mapping_error = float(np.linalg.norm(_apply(t_pm, _vector(item['mapping_xyz'], 3, 'mapping_xyz'))-prior))
            tolerance = float(acceptance['max_landmark_error_m'])
            check('LANDMARK:'+name, geometric_error <= tolerance and
                  support_error <= float(acceptance['max_pcd_support_error_m']) and
                  pixel_error is not None and pixel_error <= tolerance and pixel_class_ok and
                  (mapping_error is None or mapping_error <= tolerance),
                  f'{name}: transform/map landmark, image pixel/class, PCD support or mapping chain mismatch',
                  map_error_m=geometric_error, pcd_support_error_m=support_error,
                  pixel_center_error_m=pixel_error, pixel_class_ok=bool(pixel_class_ok),
                  mapping_chain_error_m=mapping_error)
    except KeyError as exc:
        report['diagnostics'].append({'code': 'INVALID_INPUT', 'reason': f'Missing required field: {exc}'})
    except (MapBundleError, OSError, UnicodeError, yaml.YAMLError, TypeError, ValueError, IndexError) as exc:
        report['diagnostics'].append({'code': 'INVALID_INPUT', 'reason': str(exc)})
    report['software_consistency_pass'] = not report['diagnostics']
    report['scope'] = 'Input coordinate consistency only; synthetic fixtures never validate a real new-car map'
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prior-pcd', required=True)
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--occupancy-image')
    parser.add_argument('--transform-metadata', required=True)
    args = parser.parse_args(argv)
    report = check_coordinates(args.prior_pcd, args.map_yaml, args.transform_metadata, args.occupancy_image)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['software_consistency_pass'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
