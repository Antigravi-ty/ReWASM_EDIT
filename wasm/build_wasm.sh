#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v em++ &> /dev/null; then
    if [ -f "/tmp/env.sh" ]; then
        # shellcheck source=/dev/null
        source /tmp/env.sh
    elif [ -n "${EMSDK:-}" ] && [ -f "${EMSDK}/emsdk_env.sh" ]; then
        # shellcheck source=/dev/null
        source "${EMSDK}/emsdk_env.sh"
    else
        echo "Error: Emscripten SDK (em++) not found in PATH."
        exit 1
    fi
fi

python3 "${SCRIPT_DIR}/compile.py"
