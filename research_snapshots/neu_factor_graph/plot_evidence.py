#!/usr/bin/env python3
"""One static scientific figure, synthetic scope stated explicitly."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from backend import inv
from run_experiment import trajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=Path("build/neu_factor_graph"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for name, ax in zip(["drift_noise", "alias_12m", "weak_longitudinal", "gicp_connectivity"], axes.flat):
        data = json.loads((args.input/f"{name}_input.json").read_text())
        n = len(data["samples"])
        truth = trajectory(n, straight=name=="weak_longitudinal")
        O = np.array([s["T_odom_base"] for s in data["samples"]])
        scan = {f["node_id"]:np.array(f["T_map_base"]) for f in data["scan_factors"]}
        C, base = None, []
        for i in range(n):
            if i in scan: C = scan[i]@inv(O[i])
            base.append(C@O[i])
        causal = np.array(json.loads((args.input/f"{name}_causal.json").read_text())["T_map_base"])
        for X, label, color in [(np.array(base), "Single correction", "#bd4e38"), (causal, "Causal prefix graph", "#2466a0")]:
            errors = abs(X[:,0,3]-truth[:,0,3]) if name=="weak_longitudinal" else np.linalg.norm(X[:,:3,3]-truth[:,:3,3], axis=1)
            ax.plot(np.arange(n)*.2, errors, color=color, label=label)
        ax.set_title({"drift_noise":"Correct basin + designed noisy factors", "alias_12m":"All map factors shifted 12 m: not detectable", "weak_longitudinal":"Corridor: no new longitudinal information", "gicp_connectivity":"Existing GICP: fixed nominal weights, no accuracy gain"}[name], fontsize=10)
        ax.set_xlabel("Source time (s)")
        ax.set_ylabel("Longitudinal error (m)" if name=="weak_longitudinal" else "Position error (m)")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle("Task 1 offline controls - synthetic data, one fixed realization; no hardware claim", fontsize=12)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)


if __name__ == "__main__":
    main()
