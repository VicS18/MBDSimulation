#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# -------------------------------------------
# Configuration defaults
# -------------------------------------------
SCENARIO_DIR="${REPO_ROOT}/scenarios/F2MD_IRTS"
RUNNER="${REPO_ROOT}-build/run_artery.sh"
BASE_DATASET_DIR="${REPO_ROOT}/dataset/F2MD_IRTS_output_dataset"
SIM_TIME="2h"
CONFIG="CPM-attack"
SEED=42
ATTACK_TYPE=""          # run all if empty
ATTACK_PROB=""          # run rates array if empty
KEEP_SAME=""
LABEL_CHECK_SCRIPT="${SCRIPT_DIR}/check-cpm-labeling.py"

# Add rate values here (sub-datasets per rate)
rates=(0.05 0.10 0.15 0.25)

# attack mapping to a run folder name matching cpAttackTypes::AttackNames
declare -A attack_name=(
    [1]="DropObj" [2]="DropAllObj" [3]="AddObj" [4]="SingleRandomDist" [5]="SingleRandomSpeed"
    [6]="MultiConstDist" [7]="MultiConstSpeed" [8]="MultiRandomDist" [9]="MultiRandomSpeed"
    [10]="SingleConstDistOffset" [11]="SingleConstSpeedOffset" [12]="SingleRandomDistOffset"
    [13]="SingleRandomSpeedOffset" [14]="MultiConstDistOffset" [15]="MultiConstSpeedOffset"
    [16]="MultiRandomDistOffset" [17]="MultiRandomSpeedOffset" [18]="SingleConstDist"
    [19]="SingleConstSpeed" [20]="SingleConstSpeed" [21]="SingleRandomDistOffset_KeepSameID"
    [22]="SingleRandomSpeedOffset_KeepSameID" [23]="DropObj_set"
)

usage() {
    cat <<EOF
Usage: $0 [options]
  -t SIM_TIME        (default 2h)
  -a ATTACK_TYPE     (int, e.g. 19; empty => all attacks)
  -p ATTACK_PROB     (float, or rate set; empty => use rates array)
  -k KEEP_SAME       (false|true)
  -s SEED            (default 42)
  -o OUTPUT_DIR      (default: ${BASE_DATASET_DIR})
  -d SCENARIO_DIR    (default: ${SCENARIO_DIR})
  -r RUNNER_PATH     (default: ${RUNNER})
  -h                 print help

Example:
  $0 -t 60s -a 19 -p 0.25 -k false
  $0 -p 0.10      # run all attacks at 10% attacker probability
EOF
    exit 1
}

while getopts "t:a:p:k:s:o:d:r:h" opt; do
    case "${opt}" in
        t) SIM_TIME="$OPTARG";;
        a) ATTACK_TYPE="$OPTARG";;
        p) ATTACK_PROB="$OPTARG";;
        k) KEEP_SAME="$OPTARG";;
        s) SEED="$OPTARG";;
        o) BASE_DATASET_DIR="$OPTARG";;
        d) SCENARIO_DIR="$OPTARG";;
        r) RUNNER="$OPTARG";;
        h|*) usage;;
    esac
done

if [ ! -d "${SCENARIO_DIR}" ]; then
    echo "Error: scenario directory not found: ${SCENARIO_DIR}"
    exit 1
fi

if [ ! -x "${RUNNER}" ]; then
    echo "Error: runner not found or not executable: ${RUNNER}"
    exit 1
fi

# If empty ATTACK_PROB, use rates array. Else use that value only.
if [ -n "$ATTACK_PROB" ]; then
    rates=("$ATTACK_PROB")
fi

# Determine attack rotation list.
attack_index_list=(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23)
if [ -n "$ATTACK_TYPE" ]; then
    attack_index_list=("$ATTACK_TYPE")
fi

mkdir -p "${BASE_DATASET_DIR}"

copy_output() {
    local run_folder="$1"
    mkdir -p "${run_folder}"
    cp -r "${SCENARIO_DIR}/output/"* "${run_folder}/" 2>/dev/null || true
}

check_labeling() {
    local run_folder="$1" attackname="$2"
    if command -v python3 >/dev/null 2>&1 && [ -f "${LABEL_CHECK_SCRIPT}" ]; then
        # Current dataset layout writes tx logs under tx/json.
        local globpath="${run_folder}/tx/json/logjson_cpm_tx*.json"
        python3 "${LABEL_CHECK_SCRIPT}" ${globpath} --start-attack=5.0 --expected-attack="LocalAttacker" || true
    fi
}

compute_hashes() {
    local run_folder="$1"
    find "${run_folder}" -type f -name "*.json" -exec md5sum {} \; | sort > "${run_folder}/hashes.txt"
}

run_one() {
    local probability="$1" attack_id="$2" keep="$3"
    local attack_name_short="${attack_name[$attack_id]:-attack${attack_id}}"
    local run_tag="rate_${probability}/attack_${attack_id}_${attack_name_short}/keepSameID_${keep}"
    local run_folder="${BASE_DATASET_DIR}/${run_tag}"

    echo "[RUN] ${run_tag} (sim-time=${SIM_TIME} seed=${SEED})"

    rm -rf "${SCENARIO_DIR}/output"
    mkdir -p "${SCENARIO_DIR}/output"

    pushd "${SCENARIO_DIR}" > /dev/null
    bash "${RUNNER}" -u Cmdenv -c "${CONFIG}" \
        --sim-time-limit=${SIM_TIME} \
        --seed-0-mt=${SEED} \
        --*.node[*].middleware.F2MD_CPService.CP_LOCAL_ATTACKER_PROB=${probability} \
        --*.node[*].middleware.F2MD_CPService.CP_LOCAL_ATTACK_TYPE=${attack_id} \
        --*.node[*].middleware.F2MD_CPService.KeepSameID=${keep}
    popd > /dev/null

    copy_output "${run_folder}"
    check_labeling "${run_folder}" "${attack_name_short}"
    compute_hashes "${run_folder}"

    echo "[DONE] ${run_tag}"
}

# Main loop
for probability in "${rates[@]}"; do
    for attack_id in "${attack_index_list[@]}"; do
        for keep_flag in false true; do
            if [ -n "$KEEP_SAME" ]; then
                case "$KEEP_SAME" in
                    false|true) ;;
                    *) echo "Invalid -k value: ${KEEP_SAME}. Use false or true."; exit 1;;
                esac
                if [ "$keep_flag" != "$KEEP_SAME" ]; then
                    continue
                fi
            fi
            run_one "$probability" "$attack_id" "$keep_flag"
        done
    done
done

echo "=== All dataset runs completed under ${BASE_DATASET_DIR} ==="
