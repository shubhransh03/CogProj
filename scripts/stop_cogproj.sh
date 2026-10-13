#!/usr/bin/env bash
# ==============================================================================
# CogProj Clean Shutdown Wrapper
# Invokes ./scripts/run_cogproj.sh --stop
# ==============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$SCRIPT_DIR/run_cogproj.sh" --stop
