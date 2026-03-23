#!/usr/bin/env python3
import json
import argparse
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser(description="Validate CPM TX misbehaviorType vs START_CPM_ATTACK")
    p.add_argument("json_path", type=Path, nargs="+", help="Paths to logjson_cpm_tx*.json files")
    p.add_argument("--start-attack", type=float, default=5.0, help="START_CPM_ATTACK in seconds")
    p.add_argument("--expected-attack", type=str, default="LocalAttacker", help="Expected attack type after start")
    return p.parse_args()


def main():
    args = parse_args()
    for json_file in args.json_path:
        if not json_file.exists():
            raise SystemExit(f"ERROR: {json_file} does not exist")

        raw = json_file.read_text()
        data = json.loads(raw)

        if "CPM_TX" not in data:
            raise SystemExit("ERROR: top-level key 'CPM_TX' missing")

        cpm_list = data["CPM_TX"]
        if not isinstance(cpm_list, list):
            raise SystemExit("ERROR: 'CPM_TX' is not a list")

        n_pre = n_post = n_bad = 0
        first_bad = None
        for i, rec in enumerate(cpm_list):
            t = float(rec.get("tx_timestamp", float("nan")))
            mb = rec.get("misbehaviorType", "<MISSING>")
            expected = "Genuine" if t < args.start_attack else args.expected_attack
            if mb != expected:
                n_bad += 1
                if first_bad is None:
                    first_bad = (i, t, mb, expected)
            if t < args.start_attack:
                n_pre += 1
            else:
                n_post += 1

        print(f"file: {json_file}")
        print(f" records: {len(cpm_list)}  pre-attack (<{args.start_attack}s): {n_pre}  post-attack: {n_post}")
        print(f" bad labels: {n_bad}")
        if first_bad:
            idx, t, mb, exp = first_bad
            print(f" first mismatch idx {idx}: tx_timestamp={t}, misbehaviorType={mb}, expected={exp}")
        print()


if __name__ == "__main__":
    main()
