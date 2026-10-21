#!/bin/bash
# Stop every MasonAssist demo process: the process groups started by
# start_demo.sh first, then a sweep by exact process name for anything left
# over from older/manual launches. Only MasonAssist/ROS-Gazebo processes are
# matched - browsers, editors etc. are never touched.
RUN_DIR=/tmp/masonassist_demo
PATTERN='gz sim|ruby .*gz sim|vfh_plus_node\.py|safety_supervisor\.py|mission_server\.py|worker_mover\.py|scan_frame_fix\.py|odom_to_tf_broadcaster\.py|robot_state_publisher|joint_state_publisher|ros_gz_bridge/parameter_bridge|ros2 launch masonassist_description|ros_gz_sim/create'

masonassist_pids() {
    ps -eo pid=,args= | grep -E "$PATTERN" | grep -v -E "grep|stop_demo|start_demo" | awk '{print $1}'
}

curl -s -m 2 -X POST localhost:8080/api/cmd -H 'Content-Type: application/json' \
    -d '{"cmd":"cancel"}' >/dev/null 2>&1

if [ -f "$RUN_DIR/pgids" ]; then
    while read -r pgid; do
        [ -n "$pgid" ] && kill -TERM -- "-$pgid" 2>/dev/null
    done < "$RUN_DIR/pgids"
    sleep 3
    while read -r pgid; do
        [ -n "$pgid" ] && kill -KILL -- "-$pgid" 2>/dev/null
    done < "$RUN_DIR/pgids"
    rm -f "$RUN_DIR/pgids"
fi

LEFT=$(masonassist_pids)
if [ -n "$LEFT" ]; then
    kill -KILL $LEFT 2>/dev/null
    sleep 2
fi

LEFT=$(masonassist_pids)
if [ -n "$LEFT" ]; then
    echo "STILL RUNNING (could not stop):"
    ps -o pid=,args= -p $LEFT
    exit 1
fi
echo "MasonAssist stopped - no demo processes running."
