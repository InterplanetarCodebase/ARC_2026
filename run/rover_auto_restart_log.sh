#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$ROOT_DIR/logs"

mkdir -p "$LOG_DIR"

GREEN="\033[1;32m"
RED="\033[1;31m"
YELLOW="\033[1;33m"
RESET="\033[0m"

RUN_CAMERA=false
RUN_WHEEL=false
RUN_ARM=false

if [ $# -eq 0 ]; then
    RUN_CAMERA=true
    RUN_WHEEL=true
    RUN_ARM=true
fi

for arg in "$@"; do
    case "$arg" in
        camera) RUN_CAMERA=true ;;
        wheel) RUN_WHEEL=true ;;
        arm) RUN_ARM=true ;;
        *) echo -e "${RED}Unknown argument: $arg${RESET}" ;;
    esac
done


run_supervised () {

    NAME=$1
    CMD=$2
    LOG=$3

    (
        while true
        do
            echo -e "${GREEN}[$NAME] starting...${RESET}"

            echo "===== $(date) =====" >> "$LOG"

            bash -c "$CMD" >> "$LOG" 2>&1

            echo -e "${RED}[$NAME] crashed — restarting in 2s${RESET}"
            echo "[CRASH] $(date)" >> "$LOG"

            sleep 2
        done
    ) &
    
    PIDS+=($!)
}

PIDS=()

cleanup() {

    echo -e "${YELLOW}Stopping rover processes...${RESET}"

    for pid in "${PIDS[@]}"
    do
        kill "$pid" 2>/dev/null
    done

    exit
}

trap cleanup SIGINT SIGTERM

echo -e "${GREEN}Launching Rover System${RESET}"

if $RUN_WHEEL; then
    run_supervised \
        "wheel" \
        "python3 $ROOT_DIR/Wheel/wheel_receiver.py" \
        "$LOG_DIR/wheel.log"
fi

if $RUN_ARM; then
    run_supervised \
        "arm" \
        "python3 $ROOT_DIR/Arm/arm_receiver.py" \
        "$LOG_DIR/arm.log"
fi

if $RUN_CAMERA; then
    run_supervised \
        "camera_tx" \
        "bash $ROOT_DIR/Camera/run_transmitter_camera.sh" \
        "$LOG_DIR/camera_tx.log"
fi

wait