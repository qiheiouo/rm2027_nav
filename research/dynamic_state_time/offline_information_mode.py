"""Retrospective information-mode analysis; no planner/executor entry points.

ModeEstimator consumes a whitelisted observation prefix. Case/event/truth labels
exist only in the evaluators. Detailed mode decisions remain outside Git.
"""
import argparse
from collections import Counter, defaultdict
import copy
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import time

from forensic_certificate_failures import write_csv, schedule_states, same_queued_prefix, projection
from world import stats

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / 'information_mode_protocol.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def observed(row):
    m = row['observation']
    return dict(t=row['t'], source=m['source_epoch'], receive=m['receive_time'],
                source_version=m['source_version'], set_version=m['obstacle_set_version'],
                ids=list(m['track_ids']), states=copy.deepcopy(m['source_states']))


class ModeEstimator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = {}
        self.visible = None
        self.missing = set()
        self.last = None
        self.clean = 0

    def step(self, o):
        c = self.cfg
        t, source = o['t'], o['source']
        assert source <= o['receive'] + 1e-8 <= t + 1e-8
        assert len(o['ids']) == len(o['states']) == len(set(o['ids']))
        ids = set(o['ids'])
        states = dict(zip(o['ids'], o['states']))
        reasons = []
        renamed = []
        new_source = self.last is None or source > self.last['source'] + 1e-8
        if self.last and (source < self.last['source'] - 1e-8 or o['source_version'] < self.last['source_version'] or o['set_version'] < self.last['set_version']):
            reasons.append('version_regression')
        if self.visible is not None:
            removed, added = self.visible - ids, ids - self.visible
            # Unique causal spatial rename only. Ambiguity never deletes a ghost.
            matches = {}
            for old_id in removed:
                old = self.tracks[old_id]
                dt = source - old['source']
                pos = [old['state'][j] + dt * old['state'][j+2] for j in (0, 1)]
                options = [i for i in added if math.dist(pos, states[i][:2]) <= c['relabel_position_tolerance_m']]
                if len(options) == 1:
                    matches[old_id] = options[0]
            unique = Counter(matches.values())
            renamed = [(i, j) for i, j in matches.items() if unique[j] == 1]
            renamed_old = {i for i, _ in renamed}
            self.missing.update(removed - renamed_old)
            if removed or added:
                reasons.append('identity_or_visibility_change')
            if ids & self.missing:
                reasons.append('reappearance')
            self.missing.difference_update(ids)
            self.missing.difference_update(renamed_old)
            for i in renamed_old:
                self.tracks.pop(i)
            if o['set_version'] != self.last['set_version']:
                reasons.append('set_version_change')
        residuals, jumps, accelerations = [], [], []
        has_difference = not ids
        for i, state in states.items():
            old = self.tracks.get(i)
            if old and source > old['source'] + 1e-8:
                dt = source - old['source']
                fd = [(state[j] - old['state'][j]) / dt for j in (0, 1)]
                residuals.append(math.dist(state[2:4], fd))
                jump = math.dist(state[2:4], old['state'][2:4])
                jumps.append(jump)
                accelerations.append(jump / dt)
                has_difference = True
            elif old is None:
                reasons.append('cold_track')
            if old is None or source >= old['source'] - 1e-8:
                self.tracks[i] = dict(state=list(state), source=source)
        residual = max(residuals, default=0.)
        jump = max(jumps, default=0.)
        if residual > c['residual_threshold_mps'] + 1e-8:
            reasons.append('velocity_position_incoherence')
        if jump > c['velocity_jump_mps'] + 1e-8:
            reasons.append('velocity_jump')
        if reasons:
            self.clean = 0
        elif new_source and has_difference:
            self.clean = min(self.clean + 1, c['recovery_clean_source_updates'])
        missing = [dict(id=i, source=self.tracks[i]['source'],
                        age_s=t-self.tracks[i]['source'],
                        unresolved_timeout=t-self.tracks[i]['source'] > c['missing_timeout_s'])
                   for i in sorted(self.missing)]
        age = t - source
        if age > c['freshness_s'] + 1e-9:
            mode = 'STALE'
        elif missing:
            mode = 'OCCLUDED'
        elif reasons or self.clean < c['recovery_clean_source_updates']:
            mode = 'CHANGE_UNCERTAIN'
        else:
            mode = 'STABLE_TRACK'
        self.last = copy.deepcopy(o)
        self.visible = ids
        return dict(t=t, mode=mode, reasons=sorted(set(reasons)), clean=self.clean,
                    age_s=age, missing=missing, aliases=renamed,
                    velocity_residual_mps=residual, velocity_jump_mps=jump,
                    acceleration_mps2=max(accelerations, default=0.))


def read_gz(path):
    with gzip.open(path, 'rt') as f:
        return [json.loads(x) for x in f]


def certificate_samples():
    manifest = json.loads((ROOT/'results/uncertainty_certificate/dataset_manifest.json').read_text())
    data = Path(manifest['root'])
    samples = read_gz(data/'calibration_samples.jsonl.gz') + read_gz(data/'evaluation_samples.jsonl.gz')
    return samples, manifest


def restriction_runs(modes):
    lengths = []
    run = 0
    for m in modes + [dict(mode='STABLE_TRACK')]:
        if m['mode'] != 'STABLE_TRACK':
            run += 1
        elif run:
            lengths.append(.05 * run)
            run = 0
    return lengths


def event_labels(case, rows, modes):
    """Evaluation only. Truth-event times never enter ModeEstimator."""
    events = [(e['t'], 'motion_change') for e in case['motion_events']]
    d = case['input']
    if d['kind'] in ('occlusion', 'id_relabel', 'id_misassociate', 'velocity_bias', 'position_bias'):
        events.append((d['start_s'], d['kind']))
    result = []
    for epoch, kind in events:
        # Match the unchanged observation generator's strict event comparison.
        # A source epoch 2.6999999999999997 has not observed the event at 2.7.
        index = next((i for i, r in enumerate(rows) if r['observation']['source_epoch'] >= epoch), None)
        if index is None:
            result.append(dict(case_id=case['id'], category=case['category'], event=kind, source_event_s=epoch,
                               first_delivered_s=None, mode=None, detected=None, detection_delay_from_delivery_s=None))
            continue
        required = 'OCCLUDED' if kind == 'occlusion' else 'CHANGE_UNCERTAIN'
        hit = next((j for j in range(index, len(modes)) if modes[j]['mode'] == required), None)
        result.append(dict(case_id=case['id'], category=case['category'], event=kind, source_event_s=epoch,
                           first_delivered_s=rows[index]['t'], mode=modes[index]['mode'],
                           detected=modes[index]['mode'] == required,
                           detection_delay_from_delivery_s=None if hit is None else modes[hit]['t']-rows[index]['t']))
    return result


def phase1(args, cfg):
    assert not (args.derived/'modes.jsonl.gz').exists(), 'Do not overwrite sealed mode replay'
    args.derived.mkdir(parents=True, exist_ok=True)
    args.output.mkdir(parents=True, exist_ok=True)
    protocol = json.loads((args.raw/'holdout_120/protocol.json').read_text())
    assert protocol == json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    samples, sample_manifest = certificate_samples()
    by_case = defaultdict(list)
    for s in samples:
        by_case[s['case_id']].append(s)
    episodes, transitions, events, costs = [], [], [], []
    mode_counts = Counter()
    with gzip.open(args.derived/'modes.jsonl.gz', 'wt') as sink:
        for case in protocol['cases']:
            cid = case['id']
            rows = read_gz(args.raw/'holdout_120'/f'{cid}_C1/C1_normal.jsonl.gz')
            estimator = ModeEstimator(cfg)
            modes = []
            previous = None
            edges = []
            for r in rows:
                begin = time.perf_counter_ns()
                m = estimator.step(observed(r))
                costs.append((time.perf_counter_ns()-begin)/1e6)
                modes.append(m)
                mode_counts[m['mode']] += 1
                sink.write(json.dumps(dict(case_id=cid, **m), separators=(',', ':'))+'\n')
                if m['mode'] != previous:
                    transitions.append(dict(case_id=cid, t=m['t'], old=previous, new=m['mode'], reasons=m['reasons']))
                    if previous is not None:
                        edges.append(m['t'])
                    previous = m['mode']
            by_t = {m['t']: m for m in modes}
            unsafe = sorted((s for s in by_case[cid] if not s['safe']), key=lambda s:s['t'])
            first = unsafe[0] if unsafe else None
            # Fixed warmup ends at 3 nominal source differences, not fitted per case.
            post = [m for m in modes if m['t'] >= .15-1e-9]
            stale_mismatch = sum((m['mode']=='STALE') != (r['observation']['age_s']>.15+1e-9) for r, m in zip(rows, modes))
            native_stale_mismatch = sum((m['mode']=='STALE') != (r['validity']=='stale_observation') for r, m in zip(rows, modes))
            runs = restriction_runs(modes)
            episode = dict(case_id=cid, category=case['category'], input_mode=case['mode'], cycles=len(modes),
                           mode_counts=dict(Counter(m['mode'] for m in modes)),
                           stable_post_warmup=sum(m['mode']=='STABLE_TRACK' for m in post)/len(post),
                           unexpected_ideal_cycles=sum(m['mode']!='STABLE_TRACK' for m in post) if case['mode']=='ideal_cv' else None,
                           transitions=len(edges), transitions_per_s=len(edges)/(.05*len(rows)),
                           max_transitions_in_1s=max((sum(t<=u<t+1 for u in edges) for t in edges), default=0),
                           stale_mismatch=stale_mismatch, native_stale_mismatch=native_stale_mismatch,
                           first_unsafe_s=None if first is None else first['t'],
                           first_unsafe_mode=None if first is None else by_t[first['t']]['mode'],
                           first_unsafe_restricted=None if first is None else by_t[first['t']]['mode']!='STABLE_TRACK',
                           restriction_fraction=sum(m['mode']!='STABLE_TRACK' for m in modes)/len(modes),
                           max_restriction_s=max(runs, default=0.),
                           missing_timeout_cycles=sum(any(x['unresolved_timeout'] for x in m['missing']) for m in modes))
            episodes.append(episode)
            events.extend(event_labels(case, rows, modes))
    category = []
    required = cfg['phase1_gate']['required_categories']
    for cat in dict.fromkeys(c['category'] for c in protocol['cases']):
        er = [e for e in episodes if e['category']==cat]
        ev = [e for e in events if e['category']==cat and e['detected'] is not None]
        dangerous = [e for e in er if e['first_unsafe_s'] is not None]
        category.append(dict(category=cat, episodes=len(er),
                             event_onsets=len(ev), onsets_detected=sum(e['detected'] for e in ev),
                             onset_recall=sum(e['detected'] for e in ev)/len(ev) if ev else None,
                             unsafe_episodes=len(dangerous), first_unsafe_flagged=sum(e['first_unsafe_restricted'] for e in dangerous),
                             first_unsafe_recall=sum(e['first_unsafe_restricted'] for e in dangerous)/len(dangerous) if dangerous else None,
                             stable_retention=stats([e['stable_post_warmup'] for e in er]),
                             restriction_fraction=stats([e['restriction_fraction'] for e in er]),
                             max_restriction_s=stats([e['max_restriction_s'] for e in er])))
    by_cat = {x['category']: x for x in category}
    gate = cfg['phase1_gate']
    passed = (all(by_cat[c]['onset_recall']>=gate['onset_detection_each_required_min'] and by_cat[c]['first_unsafe_recall']>=gate['first_unsafe_certificate_flagged_each_required_min'] for c in required)
              and all(by_cat[c]['stable_retention']['min']>=gate['stable_crossing_multiple_retention_after_warmup_min'] for c in ('crossing','multiple'))
              and sum(e['unexpected_ideal_cycles'] or 0 for e in episodes)==0
              and sum(e['stale_mismatch']+e['native_stale_mismatch'] for e in episodes)==0)
    summary = dict(protocol_sha256=digest(CONFIG), scope=cfg['exposure'], episodes=len(episodes), cycles=sum(e['cycles'] for e in episodes),
                   phase1_passed=passed, mode_counts=dict(mode_counts), categories=category,
                   mode_latency_ms=stats(costs), transition_rate_per_s=stats([e['transitions_per_s'] for e in episodes]),
                   max_transitions_1s=stats([e['max_transitions_in_1s'] for e in episodes]),
                   stale_mismatches=sum(e['stale_mismatch'] for e in episodes),
                   native_stale_mismatches=sum(e['native_stale_mismatch'] for e in episodes))
    write_csv(args.output/'phase1_episodes.csv', episodes)
    write_csv(args.output/'phase1_events.csv', events)
    write_csv(args.output/'mode_transitions.csv', transitions)
    (args.output/'phase1_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    manifest = dict(dataset_id='rm2027-information-mode-replay-20261027', root=str(args.derived),
                    origin=str(args.raw), source_manifest='../certificate_holdout_rerun/dataset_manifest.json',
                    certificate_label_manifest='../uncertainty_certificate/dataset_manifest.json',
                    generated_or_real='derived from unchanged synthetic causal observations; no closed-loop experiment',
                    purpose='causal mode discrimination and conditional authority replay', filename_convention='modes.jsonl.gz; replay detail if Phase 1 passes',
                    protocol_sha256=digest(CONFIG), required_experiment='offline_information_mode.py test -> phase1 -> phase2 only if passed',
                    files=[dict(path='modes.jsonl.gz', size_bytes=(args.derived/'modes.jsonl.gz').stat().st_size, sha256=digest(args.derived/'modes.jsonl.gz'))])
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


def old_authority(row, mode, old, owned_authorized, old_ids, old_target, cfg):
    """Local retention support under OLD exact-CV assumptions, never truth.

    A missing observation may leave an old dependency uncontradicted briefly;
    its own source clock is not refreshed by unrelated visible observations.
    No resurrection of an object already unauthorized at the preceding snapshot.
    """
    if old is None or not owned_authorized:
        return False, 'no_previously_authorized_backup'
    if not old['safe_rest']['certified']:
        return False, 'uncertified_rest'
    if row['t'] < old['epoch']-1e-8:
        return False, 'time_regression'
    if mode['mode']=='STALE':
        return False, 'stale_dependency'
    _, states = schedule_states(old['start_state'], old_target,
                                old['controls'], old['epoch'], row['t'])
    if math.dist(states[-1], row['state']) > cfg['numerical_tolerance']:
        return False, 'state_deviation'
    if not same_queued_prefix(row, old):
        return False, 'immutable_prefix_conflict'
    o = observed(row)
    current = dict(zip(o['ids'], o['states']))
    aliases = dict(mode['aliases'])
    missing = {x['id']:x for x in mode['missing']}
    actors = old['prediction']['actors']
    assert len(old_ids)==len(actors)
    if set(missing)-set(old_ids):
        return False, 'missing_actor_absent_from_certificate'
    covered = set()
    for i, a in zip(old_ids, actors):
        j = aliases.get(i, i)
        if j not in current:
            if i not in missing:
                return False, 'uncovered_track_dependency'
            if missing[i]['age_s'] > cfg['freshness_s']+1e-9:
                return False, 'missing_dependency_stale'
            continue
        covered.add(j)
        dt = o['source'] - old['prediction']['epoch']
        expected = [a[0]+dt*a[2], a[1]+dt*a[3], *a[2:]]
        if any(abs(x-y)>cfg['numerical_tolerance'] for x, y in zip(expected, current[j])):
            return False, 'old_model_contradicted'
    if covered != set(current):
        return False, 'new_uncovered_actor'
    return True, 'old_assumptions_supported'


def scheduled_command(cert, stamp):
    command = cert.get('initial_target', [0., 0.])
    for event in cert['controls']:
        if event[0] <= stamp+1e-8:
            command = event[1:3]
    return list(command)


def phase2(args, cfg):
    summary1 = json.loads((args.output/'phase1_summary.json').read_text())
    assert summary1['phase1_passed'], 'Phase 2 requires the Phase 1 gate'
    assert summary1['protocol_sha256']==digest(CONFIG)
    output = args.derived/'authority_local.jsonl.gz'
    assert not output.exists(), 'Do not overwrite sealed authority replay'
    protocol = json.loads((args.raw/'holdout_120/protocol.json').read_text())
    assert protocol == json.loads((ROOT/'preacquired_holdout_protocol.json').read_text())
    modes = defaultdict(list)
    for m in read_gz(args.derived/'modes.jsonl.gz'):
        modes[m['case_id']].append(m)
    samples, _ = certificate_samples()
    labels = {(s['case_id'], s['cert_id']): s for s in samples}
    raw_results = json.loads((args.raw/'holdout_120/results.json').read_text())
    original = {r['case_id']: r for r in raw_results if r['arm']=='C1'}
    old29 = [r['case_id'] for r in csv.DictReader((ROOT/'results/certificate_failure_forensics/failure_mechanisms.csv').open()) if int(r['optimistic_certificates'])>0]
    episodes, failures, timelines = [], [], {}
    count_keys = ['safe_admissions', 'unsafe_admissions', 'safe_vetoed', 'unsafe_vetoed',
                  'safe_admitted', 'unsafe_admitted', 'stable_success_safe_admissions',
                  'stable_success_safe_vetoed', 'restriction_cycles', 'proposals_vetoed',
                  'old_retention_supported', 'retained_truth_safe', 'retained_truth_unsafe']
    with gzip.open(output, 'wt') as sink:
        for case in protocol['cases']:
            cid = case['id']
            directory = args.raw/'holdout_120'/f'{cid}_C1'
            rows = read_gz(directory/'C1_normal.jsonl.gz')
            life = json.loads((directory/'C1_normal_lifecycle.json').read_text())
            initial = next(x['certificate'] for x in life if 'certificate' in x)
            old, was_authorized = initial, True
            targets = {initial['id']:[0., 0.]}
            count = Counter({k:0 for k in count_keys})
            reasons = Counter()
            first_veto, first_supported, divergence = None, None, None
            detail = []
            for row, m in zip(rows, modes[cid]):
                assert row['t']==m['t']
                allow = m['mode']=='STABLE_TRACK'
                accepted = row['accepted']
                cert = row['active_backup']
                label = labels[(cid, cert['id'])] if accepted else None
                if accepted:
                    targets[cert['id']] = list(row['target'])
                    assert label['t']==row['t']
                    key = 'safe' if label['safe'] else 'unsafe'
                    count[key+'_admissions'] += 1
                    count[key+'_vetoed'] += not allow
                    if allow:
                        count[key+'_admitted'] += 1
                    if label['safe'] and row['t']>=.15-1e-9 and case['category'] in ('crossing','multiple'):
                        count['stable_success_safe_admissions'] += 1
                        count['stable_success_safe_vetoed'] += not allow
                if not allow:
                    count['restriction_cycles'] += 1
                    if row['proposed'] is not None:
                        count['proposals_vetoed'] += 1
                old_ids = [t['id'] for t in labels[(cid, old['id'])]['features']['tracks']] if old else []
                supported, why = old_authority(row, m, old, was_authorized, old_ids, targets[old['id']] if old else None, cfg) if not allow else (False, 'no_mode_veto')
                truth_min = None
                if not allow:
                    reasons[why] += 1
                if supported:
                    count['old_retention_supported'] += 1
                    # Label only after the causal authority decision.
                    audit, _, _ = projection(case, row['state'], row['target'], old['controls'], row['t'],
                                              max(row['t']+.94, old['safe_rest']['epoch']),
                                              old['prediction']['actors'], old['prediction']['epoch'])
                    truth_min = audit['truth_schedule_min_m']
                    count['retained_truth_safe'] += truth_min>=.05
                    count['retained_truth_unsafe'] += truth_min<.05
                if label is not None and not label['safe'] and not allow:
                    if first_veto is None:
                        first_veto = row['t']
                    if supported and truth_min>=.05 and first_supported is None:
                        first_supported = row['t']
                if divergence is None:
                    # Prefix comparison is valid only until the first changed dispatch.
                    original_command = row.get('committed_action')
                    if not allow:
                        alternative = scheduled_command(old, row['t']+.1) if supported else None
                        if alternative is None or original_command is None or math.dist(alternative, original_command)>1e-7:
                            divergence = row['t']
                item = dict(t=row['t'], mode=m['mode'], proposed=row['proposed'].get('command') if row['proposed'] else None,
                            original_accepted=accepted, mode_allows=allow, local_would_admit=bool(accepted and allow),
                            issued_cert_id=None if label is None else cert['id'], issued_truth_safe=None if label is None else label['safe'],
                            old_cert_id=None if old is None else old['id'], retention_supported=supported,
                            retention_reason=why, retained_truth_clearance_m=truth_min,
                            original_committed=row.get('committed_action'), prefix_diverged=divergence is not None)
                sink.write(json.dumps(dict(case_id=cid, **item), separators=(',', ':'))+'\n')
                # Every following snapshot is conditioned on ORIGINAL execution,
                # never a fictitious evolving counterfactual robot/queue.
                old = row['active_backup'] or row['retained_invalid_backup']
                was_authorized = bool(row['active_backup'])
                if m['mode']!='STABLE_TRACK' and len(detail)<12:
                    detail.append(item)
                elif not allow or (row['t'] in (5.05, 5.10, 5.15)):
                    if cid=='head_on_10201' and row['t'] in (5.05, 5.10, 5.15):
                        detail.append(item)
            ep = dict(case_id=cid, category=case['category'], original_collision=original[cid]['collision'],
                      original_completion=original[cid]['completion'], first_unsafe_veto_s=first_veto,
                      first_veto_with_supported_safe_backup_s=first_supported, first_prefix_divergence_s=divergence,
                      unsafe_veto_rate=count['unsafe_vetoed']/count['unsafe_admissions'] if count['unsafe_admissions'] else None,
                      safe_rejection_rate=count['safe_vetoed']/count['safe_admissions'] if count['safe_admissions'] else None,
                      counts=dict(count), retention_failure_reasons=dict(reasons))
            episodes.append(ep)
            if cid in old29:
                failures.append(dict(case_id=cid, category=case['category'], first_unsafe_veto_s=first_veto,
                                     veto_with_supported_safe_backup=first_supported is not None,
                                     first_supported_s=first_supported, prefix_divergence_s=divergence,
                                     interpretation='local opportunity only; changed execution outcome Unknown'))
            if cid in ('crossing_10100','head_on_10201','stop_start_10300','stop_start_10302','occlusion_reappearance_10502','id_switch_10608','stale_observation_10801','multiple_10900'):
                timelines[cid] = detail
    total = Counter({k:0 for k in count_keys})
    categories = []
    for ep in episodes:
        total.update(ep['counts'])
    for cat in dict.fromkeys(c['category'] for c in protocol['cases']):
        er = [e for e in episodes if e['category']==cat]
        counts = Counter({k:0 for k in count_keys})
        for e in er: counts.update(e['counts'])
        categories.append(dict(category=cat, **dict(counts),
                               unsafe_veto_rate=counts['unsafe_vetoed']/counts['unsafe_admissions'] if counts['unsafe_admissions'] else None,
                               safe_rejection_rate=counts['safe_vetoed']/counts['safe_admissions'] if counts['safe_admissions'] else None))
    restrictions = list(csv.DictReader((args.output/'phase1_episodes.csv').open()))
    successful = [r for r in restrictions if original[r['case_id']]['completion'] and original[r['case_id']]['minimum_clearance_m']>=.05]
    by_cat = {x['category']:x for x in categories}
    g = cfg['phase2_gate']
    rates = dict(unsafe_veto_rate=total['unsafe_vetoed']/total['unsafe_admissions'],
                 safe_rejection_rate=total['safe_vetoed']/total['safe_admissions'],
                 stable_success_safe_rejection=total['stable_success_safe_vetoed']/total['stable_success_safe_admissions'],
                 known29_veto_with_supported_safe_backup=sum(e['veto_with_supported_safe_backup'] for e in failures)/len(failures))
    passed = (rates['unsafe_veto_rate']>=g['unsafe_admission_veto_min']
              and all(by_cat[c]['unsafe_veto_rate']>=g['unsafe_admission_veto_each_required_min'] for c in cfg['phase1_gate']['required_categories'])
              and rates['stable_success_safe_rejection']<=g['stable_safe_admission_rejection_max']
              and rates['known29_veto_with_supported_safe_backup']>=g['known29_with_veto_and_causally_authorized_backup_min']
              and all(float(e['restriction_fraction'])<=g['successful_episode_restriction_fraction_max'] and float(e['max_restriction_s'])<=g['successful_episode_max_contiguous_restriction_s']+1e-8 for e in successful)
              and summary1['mode_latency_ms']['p99']<=g['mode_cost_p99_ms_max'])
    summary = dict(protocol_sha256=digest(CONFIG), phase2_passed=passed, verdict='Go-to-prototype' if passed else 'Modify',
                   reason='Mode discrimination is useful; veto is insufficient without assumption-supported executable reserve. No prototype.' if not passed else 'Offline gates passed only; no automatic prototype.',
                   scope='Local conditional replay only. Collision reduction, actual added waiting and completion Unknown.',
                   episodes=120, counts=dict(total), rates=rates, categories=categories,
                   known29_any_unsafe_veto=sum(e['first_unsafe_veto_s'] is not None for e in failures),
                   known29_supported_chain_opportunities=sum(e['veto_with_supported_safe_backup'] for e in failures),
                   first_divergence_s=stats([e['first_prefix_divergence_s'] for e in episodes if e['first_prefix_divergence_s'] is not None]),
                   successful_restriction_fraction=stats([float(e['restriction_fraction']) for e in successful]),
                   successful_max_restriction_s=stats([float(e['max_restriction_s']) for e in successful]),
                   additional_waiting_s=None, closed_loop_prevented_collisions=None)
    write_csv(args.output/'authority_episodes.csv', episodes)
    write_csv(args.output/'authority_categories.csv', categories)
    write_csv(args.output/'known29_authority.csv', failures)
    (args.output/'representative_authority.json').write_text(json.dumps(timelines, indent=2)+'\n')
    (args.output/'phase2_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    manifest = json.loads((args.output/'dataset_manifest.json').read_text())
    manifest['files'] = [f for f in manifest['files'] if f['path']!=output.name]
    manifest['files'].append(dict(path=output.name, size_bytes=output.stat().st_size, sha256=digest(output)))
    (args.output/'dataset_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


def self_test(cfg):
    def obs(t, ids=(0,), states=None, source=None):
        return dict(t=t, source=t-.1 if source is None else source, receive=t,
                    source_version=round((10+(t-.1 if source is None else source))*1e6), set_version=0,
                    ids=list(ids), states=states if states is not None else [[t-.1, 1., 1., 0., .3]])
    est = ModeEstimator(cfg)
    assert [est.step(obs(k*.05))['mode'] for k in range(5)] == ['CHANGE_UNCERTAIN']*3 + ['STABLE_TRACK']*2
    changed = est.step(obs(.25, states=[[.15, 1., -1., 0., .3]]))
    assert changed['mode']=='CHANGE_UNCERTAIN' and 'velocity_jump' in changed['reasons']
    lost = est.step(obs(.3, ids=(), states=[]))
    assert lost['mode']=='OCCLUDED' and len(lost['missing'])==1
    late = est.step(obs(1., ids=(), states=[]))
    assert late['mode']=='OCCLUDED' and late['missing'][0]['unresolved_timeout']
    stale = est.step(obs(1.1, ids=(), states=[], source=.8))
    assert stale['mode']=='STALE' and stale['missing']
    e = ModeEstimator(cfg)
    for k in range(5): e.step(obs(k*.05))
    assert e.step(obs(.25, ids=(99,)))['mode']=='CHANGE_UNCERTAIN'
    assert not e.missing and e.clean==0
    # Identical causal inputs under two distinct physical causes cannot be separated.
    observation = dict(t=.1, observation=dict(source_epoch=0., receive_time=.1, source_version=1,
                       obstacle_set_version=0, track_ids=[0], source_states=[[1, 1, .2, 0, .3]], kind='motion', evaluation_actor_indices=[0]))
    other = copy.deepcopy(observation); other['observation']['kind']='misassociation'; other['observation']['evaluation_actor_indices']=[999]
    assert observed(observation)==observed(other)
    prefix = ModeEstimator(cfg)
    m = prefix.step(observed(observation)); frozen=copy.deepcopy(m)
    prefix.step(obs(5.))
    assert m==frozen
    # Repeated packets cannot acquire three clean source updates.
    repeat = ModeEstimator(cfg)
    for t in (0., .05, .1): r=repeat.step(obs(t, states=[[-.1,1.,1.,0.,.3]], source=-.1))
    assert r['clean']==0 and r['mode']=='STALE'
    old = dict(epoch=0., start_state=[0.,0.,0.,0.], controls=[[.1,0.,0.]],
               safe_rest=dict(certified=True), prediction=dict(epoch=0.,actors=[[2.,0.,0.,0.,.3]]))
    row = dict(t=.05, state=[0.,0.,0.,0.], target=[0.,0.], immutable_queue=[[.1,0.,0.]],
               observation=dict(source_epoch=-.05,receive_time=.05,source_version=1,obstacle_set_version=0,
                                track_ids=[0],source_states=[[2.,0.,0.,0.,.3]]))
    m = dict(mode='CHANGE_UNCERTAIN', aliases=[], missing=[])
    assert old_authority(row,m,old,True,[0],[0.,0.],cfg)[0]
    assert not old_authority(row,m,old,False,[0],[0.,0.],cfg)[0]
    changed = copy.deepcopy(row);changed['observation']['source_states'][0][2]=.5
    assert old_authority(changed,m,old,True,[0],[0.,0.],cfg)[1]=='old_model_contradicted'
    lost = copy.deepcopy(row);lost['observation']['track_ids']=[];lost['observation']['source_states']=[]
    m['missing']=[dict(id=0,age_s=.15)]
    assert old_authority(lost,m,old,True,[0],[0.,0.],cfg)[0]
    m['missing'][0]['age_s']=.2
    assert old_authority(lost,m,old,True,[0],[0.,0.],cfg)[1]=='missing_dependency_stale'
    empty = copy.deepcopy(old);empty['prediction']['actors']=[]
    assert old_authority(lost,m,empty,True,[],[0.,0.],cfg)[1]=='missing_actor_absent_from_certificate'
    print('PASS: causal prefix/metadata isolation, warmup/recovery, motion/association ambiguity, missing persistence/timeout, freshness, repeated-source semantics, old-model contradiction, no resurrection, no missing-clock refresh')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('command', choices=['test', 'phase1', 'phase2'])
    ap.add_argument('--raw', type=Path)
    ap.add_argument('--derived', type=Path)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    cfg = json.loads(CONFIG.read_text())
    if args.command=='test':
        self_test(cfg)
        return
    assert not args.derived.resolve().is_relative_to(ROOT.parents[1])
    {'phase1':phase1, 'phase2':phase2}[args.command](args, cfg)


if __name__=='__main__':
    main()
