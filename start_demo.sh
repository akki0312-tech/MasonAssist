#!/bin/bash
# Start the full MasonAssist demo: Gazebo construction site + robot + VFH+
# navigation, moving workers, and the mission-control dashboard backend.
#   ./start_demo.sh             start (refuses if anything is already running)
#   ./start_demo.sh --restart   stop everything first, then start
#   ./start_demo.sh --headless  no Gazebo 3D window (much faster in this VM;
#                               the dashboard still shows everything)
#   --odometry=drift|perfect    odometry source (default drift = realistic
#                               track odometry errors, see odom_drift_core.py;
#                               perfect = the simulator's true pose)
#   --localization=amcl|none|slam (default amcl)
#                               none = pose is odometry; slam = build a new map
#                               (slam_toolbox); amcl = localise on the saved map
#   --no-workers                don't start the moving workers (used for mapping)
# Each component runs in its own process group so stop_demo.sh can kill the
# whole tree. Logs: /tmp/masonassist_demo/*.log
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR=/tmp/masonassist_demo
PATTERN='gz sim|vfh_plus_node\.py|safety_supervisor\.py|odom_drift\.py|map_pose_publisher\.py|async_slam_toolbox_node|nav2_map_server/map_server|nav2_amcl/amcl|nav2_lifecycle_manager/lifecycle_manager|tf2_ros/static_transform_publisher|mission_server\.py|worker_mover\.py|scan_frame_fix\.py|odom_to_tf_broadcaster\.py|robot_state_publisher|joint_state_publisher|ros_gz_bridge/parameter_bridge|ros2 launch masonassist_description'

running() {
    ps -eo pid=,args= | grep -E "$PATTERN" | grep -v -E "grep|stop_demo|start_demo"
}

RESTART=false; HEADLESS=false; ODOMETRY=drift; LOCALIZATION=amcl; WORKERS=true
for arg in "$@"; do
    case "$arg" in
        --restart) RESTART=true ;;
        --headless) HEADLESS=true ;;
        --odometry=perfect|--odometry=drift) ODOMETRY=${arg#*=} ;;
        --localization=none|--localization=slam|--localization=amcl) LOCALIZATION=${arg#*=} ;;
        --no-workers) WORKERS=false ;;
        *) echo "unknown option: $arg"; exit 2 ;;
    esac
done

if $RESTART; then
    "$WS/stop_demo.sh" || exit 1
fi

if [ -n "$(running)" ]; then
    echo "MasonAssist (or another ROS/Gazebo stack) is already running:"
    running
    echo "Use ./start_demo.sh --restart or ./stop_demo.sh first."
    exit 1
fi

mkdir -p "$RUN_DIR"
: > "$RUN_DIR/pgids"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$WS"

launch() {  # launch <name> <command...>
    local name=$1; shift
    setsid nohup "$@" > "$RUN_DIR/$name.log" 2>&1 < /dev/null &
    echo $! >> "$RUN_DIR/pgids"   # setsid child is its own group leader
}

fail() {  # a half-started stack is worse than none: stop it all
    echo "$1"
    echo "Stopping the partly started demo..."
    "$WS/stop_demo.sh" > /dev/null 2>&1
    exit 1
}

echo "[1/3] Gazebo + robot + VFH+ navigation..."
launch sim ros2 launch masonassist_description gazebo.launch.py \
    world_file:=construction_site.world enable_vfh_plus:=true headless:=$HEADLESS \
    odometry:=$ODOMETRY localization:=$LOCALIZATION
for i in $(seq 1 90); do
    grep -q "Entity creation successful" "$RUN_DIR/sim.log" 2>/dev/null && break
    sleep 1
done
if ! grep -q "Entity creation successful" "$RUN_DIR/sim.log"; then
    fail "Robot did not spawn within 90 s - see $RUN_DIR/sim.log"
fi
sleep 3

if $WORKERS; then
    echo "[2/3] Moving workers..."
    launch workers python3 src/masonassist_description/scripts/worker_mover.py
else
    echo "[2/3] Moving workers: skipped (--no-workers)"
fi

echo "[3/3] Mission server + dashboard..."
launch mission_server python3 src/masonassist_description/scripts/mission_server.py \
    --ros-args -p pose_topic:=/robot_pose -p odometry:=$ODOMETRY -p localization:=$LOCALIZATION
for i in $(seq 1 30); do
    curl -s -m 2 -o /dev/null -w '%{http_code}' localhost:8080/api/state 2>/dev/null | grep -q 200 && break
    sleep 1
done
if ! curl -s -m 2 -o /dev/null -w '%{http_code}' localhost:8080/api/state | grep -q 200; then
    fail "Dashboard backend did not come up - see $RUN_DIR/mission_server.log"
fi

# Health check after a settle time, so a node that starts and then crashes
# (or a Gazebo GUI that exits and takes the server with it) is caught:
# every component must still be running and data must be flowing.
sleep 5
COMPONENTS="gazebo-server:^gz sim server|^gz sim -r -s
vfh_plus_node:vfh_plus_node\.py
safety_supervisor:safety_supervisor\.py
odom_drift:odom_drift\.py
map_pose_publisher:map_pose_publisher\.py
robot_state_publisher:robot_state_publisher
joint_state_publisher:joint_state_publisher
clock_bridge:__node:=clock_bridge
world_service_bridge:__node:=world_service_bridge
masonassist_bridge:__node:=masonassist_bridge
lidar_bridge:__node:=lidar_bridge
scan_frame_fix:scan_frame_fix\.py
odom_to_tf_broadcaster:odom_to_tf_broadcaster\.py
mission_server:mission_server\.py"
$HEADLESS || COMPONENTS="$COMPONENTS
gazebo-gui:gz sim gui"
$WORKERS && COMPONENTS="$COMPONENTS
worker_mover:worker_mover\.py"
case $LOCALIZATION in
    none) COMPONENTS="$COMPONENTS
map_odom_identity:__node:=map_odom_identity" ;;
    slam) COMPONENTS="$COMPONENTS
slam_toolbox:async_slam_toolbox_node" ;;
    amcl) COMPONENTS="$COMPONENTS
map_server:nav2_map_server/map_server
amcl:nav2_amcl/amcl
lifecycle_manager:__node:=lifecycle_manager_localization" ;;
esac
MISSING=""
N=0
while IFS= read -r line; do
    if ps -eo args= | grep -v grep | grep -qE "${line#*:}"; then
        N=$((N + 1))
    else
        MISSING="$MISSING ${line%%:*}"
    fi
done <<< "$COMPONENTS"
[ -z "$MISSING" ] || fail "Not running after startup:$MISSING - see $RUN_DIR/*.log"

DATA=$(curl -s -m 2 localhost:8080/api/state | python3 -c "
import json, sys
d = json.load(sys.stdin)
gone = [k for k, v in (('odom', d['odom']), ('scan', d['scan']), ('supervisor', d['supervisor'])) if not v]
if d['mission_status'] == 'unknown':
    gone.append('mission_status')
print(' '.join(gone))" 2>&1)
[ -z "$DATA" ] || fail "No data from: $DATA - see $RUN_DIR/*.log"

if $HEADLESS; then
    echo "Running: all $N components (headless, no Gazebo GUI, odometry=$ODOMETRY, localization=$LOCALIZATION); odometry, LiDAR, supervisor and navigation status flowing"
else
    echo "Running: all $N components incl. Gazebo GUI (odometry=$ODOMETRY, localization=$LOCALIZATION); odometry, LiDAR, supervisor and navigation status flowing"
fi
echo "Dashboard: http://localhost:8080"
echo "Stop with: ./stop_demo.sh"
