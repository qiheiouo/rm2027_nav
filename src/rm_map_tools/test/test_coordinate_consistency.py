"""Asymmetric synthetic geometry tests checker behavior, never a real-map acceptance."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from rm_map_tools.coordinate_consistency import check_coordinates, main, read_xyz_pcd
from rm_map_tools.map_export import write_ascii_pcd


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_metadata(f):
    f['meta']['artifacts'] = dict(prior_pcd_sha256=digest(f['pcd']),
        map_yaml_sha256=digest(f['yaml']), occupancy_image_sha256=digest(f['image']))
    f['metadata'].write_text(yaml.safe_dump(f['meta']))


def fixture(tmp_path):
    width, height, res = 6, 4, .5
    origin = [10., -7., math.pi/2]
    pixels = [(0, 3), (4, 2), (1, 0), (3, 1)]
    xyz = np.array([[10.-(height-1-v+.5)*res, -7.+(u+.5)*res, .5] for u,v in pixels])
    pcd, image, yml, metadata = [tmp_path/x for x in ('prior.pcd', 'map.pgm', 'map.yaml', 'coordinates.yaml')]
    write_ascii_pcd(pcd, xyz)
    raster = np.full((height,width), 254, dtype=np.uint8)
    for u,v in pixels:raster[v,u] = 0
    raster[0, 5] = 205
    image.write_bytes(f'P5\n{width} {height}\n255\n'.encode()+raster.tobytes())
    yml.write_text(yaml.safe_dump(dict(image=image.name, resolution=res, origin=origin,
        negate=0, occupied_thresh=.65, free_thresh=.196, mode='trinary')))
    t_pm = np.eye(4);t_pm[:3,3] = [1.,2.,.1]
    meta = dict(schema_version=1, dataset_kind='synthetic', map_id='checker_only', revision='test1',
        units='m', axes='right_handed_z_up', frames=dict(mapping='mapping_raw',prior='map',nav2='map'),
        T_prior_mapping=t_pm.tolist(), T_map_prior=np.eye(4).tolist(),
        expected_grid_origin=origin, expected_resolution=res,
        pixel_convention='top_left_rows_down_cell_centers', occupancy_z_band_m=[.1,1.8],
        acceptance=dict(min_landmarks=3, min_points_inside_ratio=.9, min_bounds_overlap_ratio=.9,
            min_sample_occupied_ratio=.9, max_landmark_error_m=.01,
            max_pcd_support_error_m=.01, sample_max_distance_m=.01),
        landmark_source='synthetic independent annotations: tests checker only', landmarks_independent=False,
        landmarks=[dict(id=f'L{i}',prior_xyz=p.tolist(),map_xy=p[:2].tolist(),pixel=list(pixel),
            occupancy='occupied',mapping_xyz=(p-t_pm[:3,3]).tolist()) for i,(p,pixel) in enumerate(zip(xyz,pixels))])
    f=dict(pcd=pcd,image=image,yaml=yml,metadata=metadata,meta=meta,xyz=xyz)
    save_metadata(f)
    return f


def run(f, **kwargs):
    return check_coordinates(f['pcd'],f['yaml'],f['metadata'],**kwargs)


def codes(result):
    return {x['code'] for x in result['diagnostics']}


def test_nonzero_origin_yaw_row_flip_and_chain_are_consistent(tmp_path):
    f=fixture(tmp_path);r=run(f,occupancy_image=f['image'])
    assert r['software_consistency_pass'],r
    assert r['checks']['BOUNDS_OVERLAP']['points_inside_ratio']==1.
    assert r['checks']['SAMPLE_CORRESPONDENCE']['occupied_support_ratio']==1.
    assert not r['real_map_accepted'] and not r['hardware_go']


@pytest.mark.parametrize('field,value,code',[
    ('expected_grid_origin',[10.,-7.,0.],'ORIGIN_ORIENTATION'),
    ('expected_grid_origin',[11.,-7.,math.pi/2],'ORIGIN_ORIENTATION'),
    ('expected_resolution',1.,'ORIGIN_ORIENTATION'),
    ('units','mm','INVALID_INPUT'),
    ('pixel_convention','bottom_left','INVALID_INPUT'),
    ('T_map_prior',np.diag([2.,1.,1.,1.]).tolist(),'INVALID_INPUT'),
    ('T_map_prior',np.diag([-1.,1.,1.,1.]).tolist(),'INVALID_INPUT'),
])
def test_explicit_coordinate_mismatches_fail(tmp_path,field,value,code):
    f=fixture(tmp_path);f['meta'][field]=value;save_metadata(f)
    assert code in codes(run(f))


def test_nonidentity_prior_transform_must_be_baked_even_if_geometry_matches(tmp_path):
    f=fixture(tmp_path)
    shifted=f['xyz']-np.array([2.,3.,0.])
    write_ascii_pcd(f['pcd'],shifted)
    t=np.eye(4);t[:3,3]=[2.,3.,0.]
    f['meta']['T_map_prior']=t.tolist();f['meta']['frames']['prior']='prior_local'
    for item,point in zip(f['meta']['landmarks'],shifted):
        item['prior_xyz']=point.tolist();item['mapping_xyz']=(point-[1.,2.,.1]).tolist()
    save_metadata(f);r=run(f)
    assert {'FRAME_MISMATCH','RUNTIME_TRANSFORM_NOT_BAKED'}<=codes(r)
    assert all(r['checks']['LANDMARK:'+x['id']]['passed'] for x in f['meta']['landmarks'])


def test_wrong_transform_direction_bounds_and_landmarks_fail(tmp_path):
    f=fixture(tmp_path);f['meta']['T_map_prior'][0][3]=100.;save_metadata(f);r=run(f)
    assert {'BOUNDS_OVERLAP','SAMPLE_CORRESPONDENCE','LANDMARK:L0'}<=codes(r)


@pytest.mark.parametrize('fault',['flip_row','wrong_landmark','unsupported_point','wrong_mapping','collinear','too_few'])
def test_landmark_faults_cannot_pass_only_on_overlapping_bounds(tmp_path,fault):
    f=fixture(tmp_path);items=f['meta']['landmarks']
    if fault=='flip_row':items[0]['pixel'][1]=0
    elif fault=='wrong_landmark':items[0]['map_xy'][0]+=1.
    elif fault=='unsupported_point':items[0]['prior_xyz'][2]+=1.
    elif fault=='wrong_mapping':items[0]['mapping_xyz'][0]+=1.
    elif fault=='collinear':
        for i,item in enumerate(items):item['map_xy']=[float(i),0.]
    else:f['meta']['landmarks']=items[:2]
    save_metadata(f);r=run(f)
    assert r['checks']['BOUNDS_OVERLAP']['passed']
    assert not r['software_consistency_pass']
    assert 'LANDMARK:L0' in codes(r) or 'INVALID_INPUT' in codes(r)


def test_free_or_unknown_raster_does_not_support_obstacle_samples(tmp_path):
    f=fixture(tmp_path);data=bytearray(f['image'].read_bytes());offset=len(b'P5\n6 4\n255\n')
    data[offset:]=bytes([205]*24);f['image'].write_bytes(data);save_metadata(f)
    r=run(f)
    assert 'SAMPLE_CORRESPONDENCE' in codes(r) and 'LANDMARK:L0' in codes(r)


def test_negate_and_exact_threshold_follow_nav2(tmp_path):
    f=fixture(tmp_path);raster=yaml.safe_load(f['yaml'].read_text())
    data=bytearray(f['image'].read_bytes());offset=len(b'P5\n6 4\n255\n')
    data[offset:]=bytes(255-v for v in data[offset:]);f['image'].write_bytes(data)
    raster['negate']=1;f['yaml'].write_text(yaml.safe_dump(raster));save_metadata(f)
    assert run(f)['software_consistency_pass']
    # At the exact occupied threshold Nav2 yields unknown, not occupied.
    data[offset+3*6]=128;f['image'].write_bytes(data)
    raster['occupied_thresh']=128/255.;f['yaml'].write_text(yaml.safe_dump(raster))
    f['meta']['landmarks'][0]['occupancy']='unknown'
    f['meta']['acceptance']['min_sample_occupied_ratio']=.7
    save_metadata(f);r=run(f)
    assert r['checks']['LANDMARK:L0']['passed'] and r['software_consistency_pass']


def test_artifact_or_explicit_image_mismatch_fails(tmp_path):
    f=fixture(tmp_path);f['pcd'].write_text(f['pcd'].read_text()+'\n')
    assert 'ARTIFACT_IDENTITY' in codes(run(f))
    assert 'INVALID_INPUT' in codes(run(f,occupancy_image=tmp_path/'other.pgm'))


def test_pcl_binary_extra_vector_fields_and_truncation(tmp_path):
    f=fixture(tmp_path);xyz=f['xyz']
    header=(f'VERSION .7\nFIELDS intensity x y z normal\nSIZE 4 4 4 4 4\nTYPE F F F F F\n'
        f'COUNT 1 1 1 1 3\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n').encode()
    data=np.column_stack([np.ones(len(xyz)),xyz,np.zeros((len(xyz),3))]).astype('<f4').tobytes()
    f['pcd'].write_bytes(header+data);save_metadata(f)
    assert run(f)['software_consistency_pass']
    loaded,_=read_xyz_pcd(f['pcd']);assert np.allclose(loaded,xyz)
    f['pcd'].write_bytes(header+data[:-1]);save_metadata(f)
    assert 'INVALID_INPUT' in codes(run(f))


@pytest.mark.parametrize('bad_header',[b'DATA binary_compressed\n',b'DATA ascii\n'])
def test_unsupported_or_malformed_pcd_fails_with_reason(tmp_path,bad_header):
    f=fixture(tmp_path);f['pcd'].write_bytes(bad_header);save_metadata(f)
    r=run(f);assert 'INVALID_INPUT' in codes(r) and r['diagnostics'][-1]['reason']


def test_no_height_band_or_nonfinite_input_fails(tmp_path):
    f=fixture(tmp_path);f['meta']['occupancy_z_band_m']=[3.,4.];save_metadata(f)
    assert 'no PCD points' in run(f)['diagnostics'][-1]['reason']
    f['meta']['occupancy_z_band_m']=[.1,1.8]
    f['meta']['expected_grid_origin'][0]=float('nan');save_metadata(f)
    assert 'INVALID_INPUT' in codes(run(f))


def test_real_flag_cannot_approve_map_and_needs_independent_landmarks(tmp_path):
    f=fixture(tmp_path);f['meta']['dataset_kind']='new_car_real';save_metadata(f)
    assert 'LANDMARK_INDEPENDENCE' in codes(run(f))
    f['meta']['landmarks_independent']=True;save_metadata(f);r=run(f)
    assert r['software_consistency_pass'] and not r['real_map_accepted'] and not r['hardware_go']


def test_cli_json_exit_codes(tmp_path,capsys):
    f=fixture(tmp_path);args=['--prior-pcd',str(f['pcd']),'--map-yaml',str(f['yaml']),
        '--occupancy-image',str(f['image']),'--transform-metadata',str(f['metadata'])]
    assert main(args)==0
    assert json.loads(capsys.readouterr().out)['dataset_kind']=='synthetic'
    f['meta']['frames']['nav2']='odom';save_metadata(f)
    assert main(args)==2
    assert 'FRAME_MISMATCH' in codes(json.loads(capsys.readouterr().out))
