#!/usr/bin/env bash
# End-to-end eng loading dock localization replay verification.
set -eo pipefail
MAP=/home/agxorin/autoware_data/maps/eng_loading_dock_map
BAG=/home/agxorin/Desktop/engg_loading_dock_01_gnss
AW=/home/agxorin/autoware_v2/autoware
OUT=$MAP/intermediate/loc_verify
mkdir -p "$OUT"
source "$AW/install/setup.bash"
export ROS_LOG_DIR=/tmp/ros_logs_eng_verify_$$
LOG=$OUT/autoware_replay13.log
PLAY=$OUT/bag_play13.log
VERIFY=$OUT/verify13.json

cleanup() {
  jobs -p | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT

echo "Launching localization_replay..."
ros2 launch eng_loading_dock_sensor_kit_launch localization_replay.launch.xml \
  map_path:="$MAP" rviz:=false >"$LOG" 2>&1 &
LAUNCH_PID=$!
sleep 30

echo "Playing bag..."
ros2 bag play "$BAG" --clock -r 1.0 >"$PLAY" 2>&1 &
PLAY_PID=$!

echo "Waiting for EKF+NDT seed (up to 300s)..."
for i in $(seq 1 300); do
  if grep -q "Localization seed complete" "$LOG"; then
    echo "Seed complete after ${i}s"
    break
  fi
  if grep -q "Timed out waiting for init inputs" "$LOG"; then
    echo "Init script timed out after ${i}s"
    break
  fi
  sleep 1
done

echo "Probing 120s with use_sim_time..."
python3 - <<'PY' | tee "$VERIFY"
import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from autoware_internal_debug_msgs.msg import Float32Stamped
from tf2_msgs.msg import TFMessage

def yaw(q):
    return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

class P(Node):
    def __init__(self):
        super().__init__('verify')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.c = {}
        self.d = []
        self.y = []
        self.g = {}
        self.scores = []
        self.create_subscription(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', self.gnss, 10)
        self.create_subscription(Odometry, '/localization/kinematic_state', self.ekf, 10)
        self.create_subscription(PointCloud2, '/localization/util/downsample/pointcloud', lambda _m: self.b('down'), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance', lambda _m: self.b('ndt_pwc'), 10)
        self.create_subscription(PoseStamped, '/localization/pose_estimator/pose', lambda _m: self.b('ndt_pose'), 10)
        self.create_subscription(PointCloud2, '/localization/pose_estimator/points_aligned', lambda _m: self.b('aligned'), qos_profile_sensor_data)
        self.create_subscription(Float32Stamped, '/localization/pose_estimator/nearest_voxel_transformation_likelihood', self.nvtl, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, qos_profile_sensor_data)

    def b(self, k):
        self.c[k] = self.c.get(k, 0) + 1

    def nvtl(self, m):
        self.b('nvtl')
        self.scores.append(float(m.data))

    def tf_cb(self, m):
        for tf in m.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'base_link':
                self.b('map_base_tf')
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'ndt_base_link':
                self.b('map_ndt_base_tf')

    def gnss(self, m):
        self.b('gnss')
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.g[t] = m.pose.pose
    def ekf(self,m):
        self.b('ekf')
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.g:
            return
        gt = min(self.g.keys(), key=lambda k: abs(k-t))
        if abs(gt-t) > 0.15:
            return
        g = self.g[gt]; e = m.pose.pose
        dx=e.position.x-g.position.x; dy=e.position.y-g.position.y; dz=e.position.z-g.position.z
        self.d.append((dx*dx+dy*dy+dz*dz)**0.5)
        gy=yaw(g.orientation); ey=yaw(e.orientation)
        self.y.append(abs((ey-gy+180)%360-180))

rclpy.init(); n=P(); end=time.time()+120
while time.time()<end:
    rclpy.spin_once(n, timeout_sec=0.1)
el=120
out={
 'counts': n.c,
 'rates_hz': {k: n.c.get(k,0)/el for k in ['down','ndt_pwc','ndt_pose','aligned','nvtl','map_base_tf','map_ndt_base_tf','ekf','gnss']},
 'ndt_score_nvtl': {
   'n': len(n.scores),
   'median': sorted(n.scores)[len(n.scores)//2] if n.scores else None,
   'p95': sorted(n.scores)[int(len(n.scores)*0.95)] if n.scores else None,
   'max': max(n.scores) if n.scores else None,
 },
 'ekf_gnss_pos_delta_m': {
   'n': len(n.d),
   'median': sorted(n.d)[len(n.d)//2] if n.d else None,
   'p95': sorted(n.d)[int(len(n.d)*0.95)] if n.d else None,
   'max': max(n.d) if n.d else None,
 },
 'ekf_gnss_yaw_delta_deg': {
   'n': len(n.y),
   'median': sorted(n.y)[len(n.y)//2] if n.y else None,
   'p95': sorted(n.y)[int(len(n.y)*0.95)] if n.y else None,
   'max': max(n.y) if n.y else None,
 },
}
print(json.dumps(out, indent=2))
n.destroy_node(); rclpy.shutdown()
PY

echo "Done. Results: $VERIFY"
