"""Record mission state + local pose (ENU) to CSV/log for the SITL test. Run with ROS_DOMAIN_ID=42."""
import json, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

out = sys.argv[1]
rclpy.init()
n = rclpy.create_node('sitl_rec')
pf = open(out + '/pose.csv', 'w'); pf.write('t,x,y,z,state\n')
sf = open(out + '/state.log', 'w')
cur = {'state': ''}

def on_state(m):
    d = json.loads(m.data)
    if d.get('state') != cur['state'] or d.get('reason') != cur.get('reason'):
        sf.write(f"{time.time():.2f} {d.get('state')} | {d.get('reason')} | mode {d.get('mode')} armed {d.get('armed')} "
                 f"alt {d.get('alt')} pos {d.get('pos')} probs {d.get('preflight_problems')} errs {d.get('fc_errors')[-2:] if d.get('fc_errors') else []}\n")
        sf.flush()
    cur.update(d)

def on_pose(m):
    p = m.pose.position
    pf.write(f"{time.time():.2f},{p.x:.3f},{p.y:.3f},{p.z:.3f},{cur['state']}\n")

n.create_subscription(String, '/uav/mission/state', on_state, 10)
n.create_subscription(PoseStamped, '/mavros/local_position/pose', on_pose, qos_profile_sensor_data)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
