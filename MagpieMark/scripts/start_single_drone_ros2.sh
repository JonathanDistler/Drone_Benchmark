#!/usr/bin/env bash
# Compatibility entry point; use the shared world-selectable launcher
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
    blank) shift; world=blank ;;
    test) shift; world=obstacles ;;
    *) world=blank ;;
esac
exec /usr/bin/python3 "$PROJECT_DIR/scripts/start_simulation.py" --ros system --n 1 --world "$world" "$@"
