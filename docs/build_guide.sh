#!/bin/bash
# Builds docs/quick_guide.pdf from docs/quick_guide.md. Run from any cwd.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$0")/..")"

pandoc "$ROOT/docs/quick_guide.md" -o "$ROOT/docs/quick_guide.pdf" --pdf-engine=xelatex \
    --resource-path="$ROOT/docs"
