#!/bin/bash

cd "$ROOT_DIR/Camera"

python3 camera_transmitter.py -c /dev/video0 /dev/video2 /dev/video4 -i 192.168.1.100 -p 5000 -w 320 -H 240 -b 500000