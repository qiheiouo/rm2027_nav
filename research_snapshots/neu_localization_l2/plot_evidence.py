#!/usr/bin/env python3
"""Static research figure from selected evidence; no runtime decisions."""
from pathlib import Path
import os
import json
import numpy as np
root=Path(__file__).resolve().parents[2]
os.environ.setdefault("MPLCONFIGDIR",str(root/"build/l2_matplotlib"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

evidence=Path(__file__).with_name("evidence")
q=json.loads((evidence/"sc_only_global_q_repeat_A.json").read_text())
truth=json.loads((evidence/"scoring_labels.json").read_text())[q["query_id"]]
T=np.asarray(truth["T_map_query"])
fig,axes=plt.subplots(1,2,figsize=(10,3.6),layout="constrained")
for place,color in [("repeat_A","#18735d"),("repeat_B","#c44136"),("corner","#7d6a9d")]:
    hs=[h for h in q["hypotheses"] if h["place_id"]==place and h["local_eligible"]]
    axes[0].scatter([np.linalg.norm(np.asarray(h["fine_pose_map_query"])[:2,3]-T[:2,3]) for h in hs],
                    [h["gicp_score"] for h in hs],label=place,color=color,s=45)
axes[0].set(xlabel="Global XY error against scoring label (m)",ylabel="GICP fitness (m²)",
            title="Repeated places both pass local quality")
axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
values=json.loads((evidence/"summary.json").read_text())["by_method_mode"]
x=np.arange(2)
axes[1].bar(x-.17,[values[k+"/global"]["correct_registered_basin"] for k in ["sc_only","fpfh_ransac"]],width=.34,label="Fine pose within truth tolerance",color="#548eae")
axes[1].bar(x+.17,[values[k+"/global"]["correct_basin_captured"] for k in ["sc_only","fpfh_ransac"]],width=.34,label="Also passes fixed research qualifiers",color="#18735d")
axes[1].set(xticks=x,xticklabels=["SC yaw + GICP","+ FPFH / RANSAC"],ylim=(0,8.9),
            ylabel="Queries out of 8 analytic holdouts",title="Coarse capture helps; end-to-end limits remain")
axes[1].legend(fontsize=8,loc="upper left");axes[1].grid(axis="y",alpha=.2)
fig.savefig(evidence/"candidate_ambiguity.png",dpi=170);plt.close(fig)
