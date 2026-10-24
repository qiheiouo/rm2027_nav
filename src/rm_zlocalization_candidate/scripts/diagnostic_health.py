"""Read-only interpretation of upstream state; never localization authority."""
from urllib.parse import unquote

STATUSES = {'INITIALIZING': False, 'NORMAL': True, 'OBS_DEGRADED': True,
            'SUSPECT': False, 'LOST': False}


def parse_state(text):
    fields = dict(item.split('=', 1) for item in text.split(';') if '=' in item)
    fields = {key: unquote(value) for key, value in fields.items()}
    if fields.get('status') not in STATUSES:
        raise ValueError('unknown or missing backend status')
    for key in ('epoch', 'generation', 'pose_revision', 'stamp_ns', 'estimator_failure_count'):
        if int(fields[key]) < 0:
            raise ValueError('negative backend counter/time')
    for key in ('pose_trusted', 'tf_frozen', 'tf_published'):
        if fields[key] not in ('true', 'false'):
            raise ValueError('invalid backend boolean')
    if (fields['pose_trusted'] == 'true') != STATUSES[fields['status']]:
        raise ValueError('backend trust/status contradiction')
    return fields


def health(fields, *, now_ns, receive_age, source_receive_age, timeout):
    if fields is None:
        return 3, 'waiting for backend state'
    if receive_age > timeout or source_receive_age > timeout:
        return 3, 'backend or source input stale'
    if not STATUSES[fields['status']] or fields['tf_frozen'] == 'true':
        return 2, fields['status'] + ': ' + fields.get('reason', '')
    ns = int(fields['stamp_ns'])
    if ns <= 0 or ns > now_ns + 100_000_000 or now_ns - ns > timeout * 1e9:
        return 3, 'source stamp missing, stale or in the future'
    if fields['status'] == 'OBS_DEGRADED':
        return 1, 'global observations degraded; upstream still trusts prediction'
    return 0, 'upstream NORMAL; diagnostic only'
