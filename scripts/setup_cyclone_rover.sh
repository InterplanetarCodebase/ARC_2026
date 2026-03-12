#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash

WS_INSTALL="$(cd "$PKG_DIR/../.." && pwd)/install/setup.bash"
if [[ -f "$WS_INSTALL" ]]; then
    source "$WS_INSTALL"
else
    echo "[WARN] Workspace not yet built — run 'colcon build' first"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI="file://$PKG_DIR/config/cyclonedds_rover.xml"

echo "[ROVER] ROS2 ready — domain=$ROS_DOMAIN_ID"
echo "[ROVER] DDS config: $CYCLONEDDS_URI"
```

The path logic assumes the standard colcon layout:
```
<anywhere>/
└── src/
    └── interplanetar_rover/   ← PKG_DIR
        └── scripts/           ← SCRIPT_DIR
        └── config/
└── install/                   ← WS_INSTALL (3 levels up from PKG_DIR)
