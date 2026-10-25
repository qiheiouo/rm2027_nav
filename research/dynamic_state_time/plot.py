"""Export one research figure from retained raw traces; no new trials."""
import argparse
import gzip
import json
import pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from world import actors_at,clearance,scene

def main():
    p=argparse.ArgumentParser();p.add_argument("--data",type=pathlib.Path,required=True);p.add_argument("--output",type=pathlib.Path,required=True);a=p.parse_args()
    fig,axes=plt.subplots(1,3,figsize=(13,3.7),layout="constrained")
    colors={"simple":"#2070b4","search":"#c93636","native":"#32925c"}
    cfg=scene("head_on",13)
    for arm,color in colors.items():
        rows=[json.loads(x) for x in gzip.open(a.data/"pilot_01"/f"head_on_13_{arm}.jsonl.gz","rt")]
        rows=[x for x in rows if x["t"]<=7.]
        axes[0].plot([r["state"][0] for r in rows],[r["state"][1] for r in rows],color=color,label=arm)
        axes[1].plot([r["t"] for r in rows],[clearance(r["state"],actors_at(cfg,r["t"])) for r in rows],color=color,label=arm)
    axes[0].plot([5.09,5.09-.45*6.82],[0,0],"--",color="#777777",alpha=.6,label="actor")
    axes[0].scatter([1.39757],[-.00552],color=colors["search"],marker="x",s=70,zorder=10)
    axes[0].set(xlabel="x (m)",ylabel="y (m)",title="Head-on seed 13: first 7 s",ylim=(-1,1))
    axes[0].legend(fontsize=8,loc="upper right");axes[0].grid(alpha=.2)
    axes[1].axhline(.05,color="#444444",ls=":",label="5 cm gate")
    axes[1].axhline(0,color="#222222",lw=.8)
    axes[1].set(xlabel="time (s)",ylabel="physical clearance (m)",title="Search stops, then is struck")
    axes[1].grid(alpha=.2)
    rows=json.loads((a.data/"pilot_01/results.json").read_text())
    labels=["static","crossing","head-on","multiple"]
    for j,arm in enumerate(colors):
        vals=[sum(r["safe_completion"] for r in rows if r["scene"]==sc and r["arm"]==arm) for sc in ("static","crossing","head_on","multiple")]
        axes[2].bar([k+(j-1)*.24 for k in range(4)],vals,width=.23,color=colors[arm],label=arm)
    axes[2].set(xticks=range(4),xticklabels=labels,yticks=range(4),ylim=(0,3.5),ylabel="safe completed / 3",title="Pre-registered pilot: Stop")
    axes[2].tick_params(axis="x",labelrotation=20)
    fig.suptitle("RM2027 independent harness — exact CV input, delayed plant; NOT a real STVL A/B",fontsize=11)
    fig.savefig(a.output,dpi=150)
if __name__=="__main__":main()
