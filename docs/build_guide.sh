#!/bin/bash
# Builds the printable one-page guides docs/guia_rapida.pdf (Spanish) and docs/quick_guide.pdf
# (English) from their .md sources. Run from any cwd.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || echo "$(dirname "$0")/..")"

for name in guia_rapida quick_guide; do
    pandoc "$ROOT/docs/$name.md" -o "$ROOT/docs/$name.pdf" --pdf-engine=xelatex \
        --resource-path="$ROOT/docs"
done
