#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$ROOT_DIR/logs"

mkdir -p "$LOG_DIR"

GREEN="\033[1;32m"
RED="\033[1;31m"
YELLOW="\033[1;33m"
RESET="\033[0m"

RUN_GUI=false
RUN_CAMERA=false

if [ $# -eq 0 ]; then
    RUN_GUI=true
    RUN_CAMERA=true
fi

for arg in "$@"; do
    case "$arg" in
        gui) RUN_GUI=true ;;
        camera) RUN_CAMERA=true ;;
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

    echo -e "${YELLOW}Stopping base processes...${RESET}"

    for pid in "${PIDS[@]}"
    do
        kill "$pid" 2>/dev/null
    done

    exit
}

trap cleanup SIGINT SIGTERM

echo -e "${GREEN}Launching Base Station${RESET}"

if $RUN_GUI; then
    run_supervised \
        "gui" \
        "python3 $ROOT_DIR/control_gui_system.py" \
        "$LOG_DIR/gui.log"
fi

if $RUN_CAMERA; then
    run_supervised \
        "camera_rx" \
        "bash $ROOT_DIR/Camera/run_receiver_camera.sh" \
        "$LOG_DIR/camera_rx.log"
fi

wait