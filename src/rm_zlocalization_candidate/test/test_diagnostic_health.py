from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from diagnostic_health import health, parse_state

TEXT = ('status=NORMAL;pose_trusted=true;tf_frozen=false;tf_published=true;'
        'epoch=0;generation=1;pose_revision=2;stamp_ns=1000000000;'
        'estimator_failure_count=0;last_estimator_failure=;reason=a%3Bb%25c')


def test_failure_reason_and_versions_survive_wire_encoding():
    fields = parse_state(TEXT)
    assert fields['reason'] == 'a;b%c'
    assert fields['generation'] == '1' and fields['pose_revision'] == '2'
    assert health(fields, now_ns=1_100_000_000, receive_age=.1,
                  source_receive_age=.1, timeout=.5)[0] == 0


def test_receipt_heartbeats_cannot_renew_source_or_paused_clock():
    fields = parse_state(TEXT)
    assert health(fields, now_ns=1_000_000_000, receive_age=0,
                  source_receive_age=.6, timeout=.5)[0] == 3
    assert health(fields, now_ns=1_600_000_000, receive_age=0,
                  source_receive_age=.1, timeout=.5)[0] == 3
    assert health(fields, now_ns=500_000_000, receive_age=0,
                  source_receive_age=.1, timeout=.5)[0] == 3


@pytest.mark.parametrize('text', [TEXT.replace('NORMAL', 'LOST'),
    TEXT.replace('epoch=0', 'epoch=x'), TEXT.replace('tf_frozen=false', 'tf_frozen=unknown')])
def test_malformed_or_contradictory_state_rejected(text):
    with pytest.raises(ValueError):
        parse_state(text)
