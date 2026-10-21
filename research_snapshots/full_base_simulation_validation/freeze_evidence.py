#!/usr/bin/env python3
"""Freeze the additive contract revision; never overwrite the 20261007 snapshot."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'docs/research/full_base_simulation_validation/evidence'
TARGET=BASE/'contracts_20261008/evidence_sha256.json'
paths=list((ROOT/'experiments/full_base_simulation_validation').rglob('*'))+list(BASE.rglob('*'))
paths += [ROOT/p for p in ['src/rm_localization_adapters/src/lio_adapter.cpp',
                         'src/rm_localization_adapters/README.md',
                         'docs/research/full_base_input_contract_20261008.md',
                         'docs/research/full_base_simulation_validation_20261007.md',
                         'docs/research/navigation_validation_status_20261007.md']]
entries={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)
         if p.is_file() and '__pycache__' not in p.parts and p!=TARGET}
TARGET.write_text(json.dumps(entries,indent=2)+'\n')
print(f'{len(entries)} current source/report/evidence hashes; historical manifest unchanged')
