"""Summarize real patrol JSONL metrics without inventing performance data."""

import argparse
import json
import math
import os
import statistics


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1))
    return ordered[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'metrics', nargs='?', default='~/.ros/raspi_car/logs/patrol_metrics.jsonl')
    args = parser.parse_args()
    path = os.path.abspath(os.path.expanduser(args.metrics))
    records = []
    with open(path, encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise SystemExit('invalid JSON on line %d: %s' % (line_number, exc))
    if not records:
        raise SystemExit('no patrol records found in %s' % path)
    successes = [record for record in records if record.get('success') is True]
    durations = [float(record['duration_sec']) for record in successes]
    summary = {
        'source_file': path,
        'route': records[-1].get('route'),
        'laps_total': len(records),
        'laps_succeeded': len(successes),
        'success_rate_percent': round(100.0 * len(successes) / len(records), 2),
        'successful_lap_mean_sec': round(statistics.fmean(durations), 3) if durations else None,
        'successful_lap_p95_sec': round(percentile(durations, 0.95), 3) if durations else None,
        'first_timestamp': records[0].get('timestamp'),
        'last_timestamp': records[-1].get('timestamp'),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
