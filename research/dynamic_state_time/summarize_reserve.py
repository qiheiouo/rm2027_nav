"""Small summaries only; never copy full traces or logs into the repository."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True); args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []; timeline = []; files = []
    for run in ("fixture_01", "fixture_confirmation"):
        directory = args.data/run
        results.extend(dict(run=run, **r) for r in json.loads((directory/"results.json").read_text()))
        for path in sorted(directory.iterdir()):
            files.append(dict(path=str(path.relative_to(args.data)), size=path.stat().st_size,
                              sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        for path in sorted(directory.glob("*.jsonl.gz")):
            rows = [json.loads(line) for line in gzip.open(path, "rt")]
            if run != "fixture_01": continue
            matches = [("initial", rows[0]),
                       ("first_admission", next((r for r in rows if r["accepted"]), None)),
                       ("first_late", next((r for r in rows if r["reason"]=="planner_deadline_rejected"), None)),
                       ("first_parked", next((r for r in rows if sum(v*v for v in r["state"][2:])<1e-10), None)),
                       ("last_certificate", next((r for r in reversed(rows) if r["candidate_available"]), None)),
                       ("collision", rows[-1])]
            for label, row in matches:
                if row is None: continue
                timeline.append(dict(run=run, arm_condition=path.name.removesuffix(".jsonl.gz"), event=label,
                                     t=row["t"], state=row["state"],
                                     proposed=row["proposed"]["command"] if row["proposed"] else None,
                                     admitted=row["accepted"], committed=row["committed_action"],
                                     backup=row["active_backup"], candidate_available=row["candidate_available"],
                                     reason=row["reason"], actually_executed=row["executed"][-1][1:3],
                                     planner_ms=row["planner_ms"], validation_ms=row["backup_validation_ms"],
                                     response_ms=row["response_ms"]))
    audit = json.loads((args.data/"fixture_01/sanity.json").read_text())
    summary = dict(verdict="Stop", reason="Initial reserve unavailable with frozen nine-action library at exact fixture state",
                   scope="8 B/C0 rollouts; 4 C1 initialization attempts without rollout; repeated same fixture, not independent scenarios",
                   persistence_result="NOT_EVALUATED; initialization gate failed", episode_36="NOT_RUN",
                   audit=audit, results=results)
    (args.output/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    (args.output/"timeline.json").write_text(json.dumps(timeline, indent=2)+"\n")
    columns = ["run","arm","condition","collision","completion","minimum_clearance_m","end_s",
               "wait_s","accepted","rejected","timeout_brakes","intrinsic_deadline_misses","response_deadline_misses"]
    with (args.output/"trials.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=columns+["intrinsic_p99_ms","intrinsic_max_ms"], lineterminator="\n")
        writer.writeheader()
        for r in results:
            writer.writerow({k:r.get(k) for k in columns} | dict(intrinsic_p99_ms=r["intrinsic_ms"]["p99"],
                                                                 intrinsic_max_ms=r["intrinsic_ms"]["max"]))
    manifest = dict(dataset_id="rm2027-persistent-backup-fixture-20261026", root=str(args.data.resolve()),
                    filename_convention="<fixture_01|fixture_confirmation>/<B|C0|C1>_<normal|late>.{json,jsonl.gz}; *_planner.log, *_reserve.log; sanity.json and results.json",
                    origin="local independent Python/C++ harness, GCC -O3 -DNDEBUG C++17, one logical CPU",
                    purpose="exact saved initial-reserve failure and unchanged confirmation; B/C0 counterfactual comparison",
                    generated_or_real="generated ideal CV; consumers receive no future truth",
                    required_experiment="run_reserve.sh fixture_01 and fixture_confirmation",
                    files=files, total_bytes=sum(f["size"] for f in files),
                    excluded="build_cache; preliminary diagnostics remain external and are superseded by reproducible sanity.json")
    (args.output/"dataset_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    for r in results:
        print(r["run"], r["arm"], r["condition"], r.get("end_s"),
              "latency", "/".join(f'{r["intrinsic_ms"][k]:.3f}' for k in ("mean","p50","p95","p99","max")))


if __name__ == "__main__": main()
