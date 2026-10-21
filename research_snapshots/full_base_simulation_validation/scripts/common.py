import numpy as np
from scipy.spatial.transform import Rotation
from adapter import Calibration, HeadingSample, PoseSample, Startup
CLOCK='gazebo_sim_clock'
REFERENCE='sim_gravity_heading'
def stamp(t): return t.sec*1_000_000_000+t.nanosec

def matrix(p):
    T=np.eye(4);q=p.orientation;T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    T[:3,3]=[p.position.x,p.position.y,p.position.z];return T

def set_pose(p,T):
    p.position.x,p.position.y,p.position.z=map(float,T[:3,3])
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,Rotation.from_matrix(T[:3,:3]).as_quat())

def calibration(manifest):
    F=np.eye(4);F[:3,3]=manifest['gimbal_center_m']
    K=np.eye(4);K[:3,3]=manifest['sensor_offset_m']
    if 'synthetic_T_base_joint' in manifest:
        F=np.array(manifest['synthetic_T_base_joint']);K=np.array(manifest['synthetic_T_joint_sensor_home'])
    return Calibration(F,K,0.,'sim_CAD_'+manifest['lidar_selection'],'base_link','gimbal_yaw_link','sim_lidar_link')

def samples(raw,h):
    return (PoseSample(matrix(raw.pose.pose),stamp(raw.header.stamp),CLOCK,raw.header.frame_id,
                       raw.header.frame_id,raw.child_frame_id),
            HeadingSample(h.yaw_rad,stamp(h.header.stamp),CLOCK,f'{h.source_boot_id}:{h.reset_counter}',
                          h.header.frame_id,h.valid,h.online))

def startup(raw,h,cal,roll_pitch,missing=False,home_confirmed=False):
    r,hs=samples(raw,h)
    # roll/pitch are declared fixture attitude inputs. Never read scoring truth.
    R=Rotation.from_euler('xyz',[*roll_pitch,h.yaw_rad]).as_matrix()
    if missing:R[:]=np.nan
    gravity=R.T@np.array([0.,0.,-1.])
    return Startup(r,hs,R,gravity,r.stamp_ns,r.stamp_ns,CLOCK,home_confirmed,0.,cal.calibration_id,'shadow_full_odom')
