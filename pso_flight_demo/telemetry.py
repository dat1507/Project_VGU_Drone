"""Pure native snapshot parsing. Never flatten multi-instance uORB messages."""

import re


def single_fields(output):
    pairs = re.findall(r"^[ \t]+(\w+):[ \t]+([^\n]+)", output, re.M)
    if len({k for k, _ in pairs}) != len(pairs):
        return {}  # ambiguous repeated fields fail closed, never last-value wins
    return dict(pairs)


def topic_instances(output):
    parts = re.split(r"^Instance (\d+):\s*$", output, flags=re.M)
    if len(parts) == 1:
        explicit = re.search(r"TOPIC: \w+ instance (\d+)", output)
        fields = single_fields(output)
        return {int(explicit[1]) if explicit else 0: fields} if fields else {}
    result = {}
    for i in range(1, len(parts), 2):
        index = int(parts[i])
        if index in result:
            return {}
        result[index] = single_fields(parts[i+1])
    return result


def simulation_us(stats):
    body = re.search(r"sim_time\s*\{([^}]*)\}", stats, re.S)
    if not body:
        return None
    def value(key):
        match = re.search(r"\b"+key+r": (\d+)", body[1])
        return int(match[1]) if match else 0
    return value("sec")*1_000_000 + value("nsec")//1000


def gcs_snapshot(output, status, clock_us, max_age_us=1_000_000):
    """Match Commander semantics: ANY PX4-classified GCS link, not link index 0.

    heartbeat_type_gcs is populated by MAVLink heartbeat type classification.
    Retain link instance/type/mode/RX for audit; require source age and received
    traffic, then vehicle_status's aggregate GCS state. This proves a snapshot
    only, not receipt/progression during a running-clock interval.
    """
    instances = topic_instances(output)
    selected = []
    for index, fields in instances.items():
        try:
            stamp = int(fields['timestamp'].split()[0])
            rx = int(fields['rx_message_count'])
            if (clock_us is not None and stamp > 0 and
                    0 <= clock_us-stamp <= max_age_us and rx > 0 and
                    fields.get('heartbeat_type_gcs') == 'True'):
                selected.append(index)
        except (ValueError, KeyError, TypeError):
            continue
    try:
        status_stamp = int(status['timestamp'].split()[0])
        status_fresh = clock_us is not None and 0 < status_stamp <= clock_us and clock_us-status_stamp <= max_age_us
    except (ValueError, KeyError, TypeError):
        status_fresh = False
    return dict(snapshot_pass=bool(selected and status_fresh and status.get('gcs_connection_lost') == 'False'),
                selected_instances=selected, instances=instances, clock_us=clock_us,
                heartbeat_progression_proven=False,
                selection_basis='PX4 MAVLink heartbeat_type_gcs + fresh timestamp + RX; Commander any-link semantics')


def gcs_progress(before, after):
    if not before['snapshot_pass'] or not after['snapshot_pass']:
        return False
    if after['clock_us'] <= before['clock_us']:
        return False
    for index in set(before['selected_instances']) & set(after['selected_instances']):
        a, b = before['instances'][index], after['instances'][index]
        if (int(b['timestamp'].split()[0]) > int(a['timestamp'].split()[0]) and
                int(b['rx_message_count']) > int(a['rx_message_count'])):
            return True  # RX + classified heartbeat; not a per-heartbeat sequence
    return False
