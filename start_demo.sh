#!/bin/bash
# Start the full MasonAssist demo: Gazebo construction site + robot + VFH+
# navigation, moving workers, and the mission-control dashboard backend.
#   ./start_demo.sh            start (refuses if anything is already running)
#   ./start_demo.sh --restart  stop everything first, then start
# Each component runs in its own process group so stop_demo.sh can kill the
# whole tree. Logs: /tmp/masonassist_demo/*.log
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR=/tmp/masonassist_demo
PATTERN='gz sim|vfh_plus_node\.py|mission_server\.py|worker_mover\.py|scan_frame_fix\.py|odom_to_tf_broadcaster\.py|robot_state_publisher|joint_state_publisher|ros_gz_bridge/parameter_bridge|ros2 launch masonassist_description'

running() {
    ps -eo pid=,args= | grep -E "$PATTERN" | grep -v -E "grep|stop_demo|start_demo"
}

if [ "${1:-}" = "--restart" ]; then
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

echo "[1/3] Gazebo + robot + VFH+ navigation..."
launch sim ros2 launch masonassist_description gazebo.launch.py \
    world_file:=construction_site.world enable_vfh_plus:=true
for i in $(seq 1 90); do
    grep -q "Entity creation successful" "$RUN_DIR/sim.log" 2>/dev/null && break
    sleep 1
done
if ! grep -q "Entity creation successful" "$RUN_DIR/sim.log"; then
    echo "Robot did not spawn within 90 s - see $RUN_DIR/sim.log"; exit 1
fi
sleep 3

echo "[2/3] Moving workers..."
launch workers python3 src/masonassist_description/scripts/worker_mover.py

echo "[3/3] Mission server + dashboard..."
launch mission_server python3 src/masonassist_description/scripts/mission_server.py
for i in $(seq 1 30); do
    curl -s -m 2 -o /dev/null -w '%{http_code}' localhost:8080/api/state 2>/dev/null | grep -q 200 && break
    sleep 1
done
if ! curl -s -m 2 -o /dev/null -w '%{http_code}' localhost:8080/api/state | grep -q 200; then
    echo "Dashboard backend did not come up - see $RUN_DIR/mission_server.log"; exit 1
fi

N=$(ps -eo args= | grep -E 'gz sim server|vfh_plus_node\.py|robot_state_publisher|joint_state_publisher|__node:=clock_bridge|__node:=masonassist_bridge|__node:=lidar_bridge|scan_frame_fix\.py|odom_to_tf_broadcaster\.py|worker_mover\.py|mission_server\.py' | grep -v grep | wc -l)
echo "Running processes: $N (expected 11)"
echo "Dashboard: http://localhost:8080"
echo "Stop with: ./stop_demo.sh"
