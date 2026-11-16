#!/usr/bin/env bash
# The pre-PR gate: runs everything CI runs. Must be green before a PR is opened.
set -euo pipefail
cd "$(dirname "$0")/.."

step() { printf '\n== %s\n' "$*"; }

step "rustfmt"
cargo fmt --all --check

step "clippy"
cargo clippy --workspace --all-targets -- -D warnings

step "rust tests (golden model, G1)"
cargo test --workspace -q

step "vectors are up to date (regenerate + diff)"
make -C hw vectors >/dev/null
if ! git diff --quiet -- hw/vectors; then
    git --no-pager diff --stat -- hw/vectors
    echo "hw/vectors changed after regeneration: commit the regenerated files" >&2
    exit 1
fi
untracked=$(git ls-files --others --exclude-standard -- hw/vectors)
if [ -n "$untracked" ]; then
    echo "untracked vector files: $untracked — commit them" >&2
    exit 1
fi

step "RTL lint + G3 gates"
make -C hw all

printf '\nall gates green\n'
