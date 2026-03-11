#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

RUN_CAMERA=false
RUN_WHEEL=false
RUN_ARM=false

# If no args → run everything
if [ $# -eq 0 ]; then
    RUN_CAMERA=true
    RUN_WHEEL=true
    RUN_ARM=true
fi

# Parse arguments (order independent)
for arg in "$@"; do
    case "$arg" in
        camera)
            RUN_CAMERA=true
            ;;
        wheel)
            RUN_WHEEL=true
            ;;
        arm)
            RUN_ARM=true
            ;;
        *)
            echo "Unknown argument: $arg"
            ;;
    esac
done

PIDS=()

cleanup() {
    echo ""
    echo "Stopping rover processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    exit
}

trap cleanup SIGINT SIGTERM

echo "Launching rover systems..."

# Wheel
if $RUN_WHEEL; then
    echo "Starting Wheel Receiver..."
    python3 "$ROOT_DIR/Wheel/wheel_receiver.py" &
    PIDS+=($!)
fi

# Arm
if $RUN_ARM; then
    echo "Starting Arm Receiver..."
    python3 "$ROOT_DIR/Arm/arm_receiver.py" &
    PIDS+=($!)
fi

# Camera transmitter
if $RUN_CAMERA; then
    echo "Starting Camera Transmitter..."
    bash "$ROOT_DIR/Camera/run_transmitter_camera.sh" &
    PIDS+=($!)
fi

echo "Processes running: ${PIDS[*]}"

wait