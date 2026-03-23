#!/usr/bin/env bash
set -euo pipefail

# Check reproducibility between two run folders by comparing MD5 hashes of JSON files.
# Usage: $0 <runA_folder> <runB_folder>

if [ $# -ne 2 ]; then
    echo "Usage: $0 <runA_folder> <runB_folder>"
    echo "Compares MD5 hashes of JSON files between two simulation run folders."
    exit 1
fi

runA="$1"
runB="$2"

if [ ! -d "$runA" ]; then
    echo "Error: $runA is not a directory"
    exit 1
fi

if [ ! -d "$runB" ]; then
    echo "Error: $runB is not a directory"
    exit 1
fi

compute_hashes() {
    local run_folder="$1"
    local hash_file="${run_folder}/hashes.txt"
    echo "Computing hashes for $run_folder..."
    # Hash lists must be comparable across different run directories.
    # Therefore we hash from within the run folder and store relative paths.
    # (Avoids false mismatches caused by absolute path differences.)
    (
        cd "$run_folder"
        find . -type f -name "*.json" -exec md5sum {} \; | sort
    ) > "$hash_file"
}

compute_hashes "$runA"
compute_hashes "$runB"

hashA="${runA}/hashes.txt"
hashB="${runB}/hashes.txt"

echo "Comparing hashes between:"
echo "  A: $hashA"
echo "  B: $hashB"
echo ""

if diff -q "$hashA" "$hashB" > /dev/null; then
    echo "✓ Reproducible: No differences found in JSON file hashes."
else
    echo "✗ Not reproducible: Differences found in JSON file hashes."
    echo "Diff output:"
    diff -u "$hashA" "$hashB" || true
fi
