#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

RUN_GUI=false
RUN_CAMERA=false

# Default → run both
if [ $# -eq 0 ]; then
    RUN_GUI=true
    RUN_CAMERA=true
fi

# Parse args
for arg in "$@"; do
    case "$arg" in
        gui)
            RUN_GUI=true
            ;;
        camera)
            RUN_CAMERA=true
            ;;
        *)
            echo "Unknown argument: $arg"
            ;;
    esac
done

PIDS=()

cleanup() {
    echo ""
    echo "Stopping base station processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    exit
}

trap cleanup SIGINT SIGTERM

echo "Launching base station systems..."

# GUI
if $RUN_GUI; then
    echo "Starting Control GUI..."
    python3 "$ROOT_DIR/control_gui_system.py" &
    PIDS+=($!)
fi

# Camera receiver
if $RUN_CAMERA; then
    echo "Starting Camera Receiver..."
    bash "$ROOT_DIR/Camera/run_receiver_camera.sh" &
    PIDS+=($!)
fi

echo "Processes running: ${PIDS[*]}"

wait