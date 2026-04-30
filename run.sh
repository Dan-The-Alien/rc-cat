#!/usr/bin/env bash
set -euo pipefail
# Startup-friendly launcher for the app.
# Uses the repository relative venv and runs streamlit in foreground
# so it can be managed by systemd. Logs are written to ./logs/streamlit.log.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$SCRIPT_DIR/venv"
APP="$SCRIPT_DIR/app.py"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
# Prefer the streamlit binary from the venv; fall back to system streamlit or
# to running via the venv python module.
if [ -x "$VENV/bin/streamlit" ]; then
	exec "$VENV/bin/streamlit" run "$APP" --server.port 8501 --server.address 0.0.0.0 --server.headless=true >>"$LOG_DIR/streamlit.log" 2>&1
elif command -v streamlit >/dev/null 2>&1; then
	exec streamlit run "$APP" --server.port 8501 --server.address 0.0.0.0 --server.headless=true >>"$LOG_DIR/streamlit.log" 2>&1
else
	exec "$VENV/bin/python" -m streamlit run "$APP" --server.port 8501 --server.address 0.0.0.0 --server.headless=true >>"$LOG_DIR/streamlit.log" 2>&1
fi