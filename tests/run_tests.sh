#!/usr/bin/env bash
# run_tests.sh — run the full test suite (Python + JavaScript)
#
# Usage:
#   cd /path/to/mecanumbot_gui
#   bash tests/run_tests.sh
#
# Prerequisites (Python):
#   pip install pytest flask
#
# Prerequisites (JavaScript):
#   node (v14+) — standard on most Linux systems

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "================================================================"
echo "  MecanumBot GUI — Test Suite"
echo "================================================================"
echo ""

# ── Python tests ─────────────────────────────────────────────────────────────

echo "── Python tests (pytest) ───────────────────────────────────────"
cd "${REPO_ROOT}"
python -m pytest tests/test_database.py tests/test_app_api.py -v --tb=short
PYTHON_EXIT=$?

echo ""

# ── JavaScript tests ──────────────────────────────────────────────────────────

echo "── JavaScript tests (node) ─────────────────────────────────────"
if command -v node &>/dev/null; then
    node "${SCRIPT_DIR}/test_js_map_controls.js"
    JS_EXIT=$?
else
    echo "  node not found — skipping JS tests"
    JS_EXIT=0
fi

echo ""
echo "================================================================"
if [ "$PYTHON_EXIT" -eq 0 ] && [ "$JS_EXIT" -eq 0 ]; then
    echo "  All tests PASSED ✓"
    exit 0
else
    echo "  Some tests FAILED ✗  (python=$PYTHON_EXIT  js=$JS_EXIT)"
    exit 1
fi
