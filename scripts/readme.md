# Dataset Scripts Guide

This folder contains utility scripts for generating and validating F2MD IRTS datasets.

Run commands from repository root:

```bash
cd ~/artery
```

## Files

- `scripts/gen_dataset.sh`: run simulation(s), collect outputs, organize dataset folders, compute hashes.
- `scripts/check-reproducibility.sh`: compare two dataset runs by MD5 hashes of JSON files.
- `scripts/check-cpm-labeling.py`: validate CPM TX labels against attack start time.

## 1) Generate Dataset

Example:

```bash
bash ./scripts/gen_dataset.sh -t 60s -a 19 -p 0.25
```

Default output location:

```text
~/artery/dataset/F2MD_IRTS_output_dataset/
```

Useful options:

- `-t <time>`: simulation time limit (example: `60s`)
- `-a <id>`: attack type id (example: `19`)
- `-p <prob>`: attacker probability (example: `0.25`)
- `-k <true|false>`: KeepSameID filter (if omitted, both are generated)
- `-s <seed>`: RNG seed (default `42`)
- `-o <dir>`: output base directory
- `-d <dir>`: scenario directory override (default `scenarios/F2MD_IRTS`)
- `-r <path>`: run script override (default `~/artery-build/run_artery.sh`)

Custom output example:

```bash
bash ./scripts/gen_dataset.sh -t 60s -a 19 -p 0.25 -o ./dataset/my_out
```

## 2) Reproducibility Check

Run the same setup twice with identical parameters, then compare:

```bash
rm -rf ./dataset/output_dataset_run1 ./dataset/output_dataset_run2
mkdir -p ./dataset/output_dataset_run1 ./dataset/output_dataset_run2

bash ./scripts/gen_dataset.sh -t 60s -a 19 -p 0.25 -o ./dataset/output_dataset_run1
bash ./scripts/gen_dataset.sh -t 60s -a 19 -p 0.25 -o ./dataset/output_dataset_run2

bash ./scripts/check-reproducibility.sh \
  ./dataset/output_dataset_run1 \
  ./dataset/output_dataset_run2
```

Expected success line:

```text
✓ Reproducible: No differences found in JSON file hashes.
```

## 3) CPM Labeling Check

Manual expectation for attacker TX stream:

- before attack start (`< 5.0s`): `misbehaviorType = Genuine`
- after attack start (`>= 5.0s`): `misbehaviorType = LocalAttacker`

Script check example:

```bash
python3 ./scripts/check-cpm-labeling.py \
  ./dataset/F2MD_IRTS_output_dataset/rate_0.25/attack_19_SingleConstSpeed/keepSameID_true/tx/json/logjson_cpm_tx18.json \
  --start-attack=5.0 \
  --expected-attack=LocalAttacker
```

Notes:

- Non-attacker pseudonyms typically remain `Genuine`.
- Some logs may contain a trailing empty record (`{}`), which can appear as one mismatch (`tx_timestamp=nan`, missing `misbehaviorType`).
