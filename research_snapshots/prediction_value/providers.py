"""Causal sources. Input is observations only; no scene/truth/robot state."""
import math
from vendor.tracker_core import Detection, MultiObjectTracker, Point2D, TrackState

class Sources:
    def __init__(self):
        # Existing canonical public-v2 KF, parameter values from its frozen YAML.
        self.tracker=MultiObjectTracker(association_gate=.60,process_noise=3.,measurement_noise=.08,
            initial_variance=1.,min_hits_to_confirm=3,min_displacement_to_confirm=.15,
            use_global_assignment=False,tentative_max_misses=1,max_coast_time_sec=.60,
            prediction_steps=30,prediction_dt=.1,velocity_decay_tau=0.,max_prediction_speed=0.,
            public_anchor_mode="last_observation_cv")
        self.history=[]
        self.tracks=()
    def update(self, position, stamp):
        detections=[] if position is None else [Detection(Point2D(*position),.72,.72,20)]
        self.tracks=self.tracker.update(detections,stamp+100.).tracks
        if position is not None:
            self.history.append((stamp,position))
    def prediction(self, mode, now, observed):
        if observed is None or not self.history:
            return None
        stamp,pos=self.history[-1]
        age=now-stamp
        if age>.4+1e-9:
            return None
        velocity=(0.,0.)
        if mode == "CV" and len(self.history)>1:
            prev,prevpos=self.history[-2]
            velocity=tuple((pos[k]-prevpos[k])/(stamp-prev) for k in (0,1))
        if mode == "C_KF":
            valid=[t for t in self.tracks if t.state == TrackState.CONFIRMED or t.state == TrackState.COASTING]
            if not valid:
                return None
            track=valid[0]
            pos=(track.position.x,track.position.y)
            velocity=(track.velocity.x,track.velocity.y)
        return [[pos[k]+velocity[k]*(age+(i+1)*.1) for k in (0,1)] for i in range(30)]
