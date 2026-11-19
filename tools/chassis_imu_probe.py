import argparse,json,math,statistics,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

parser=argparse.ArgumentParser()
parser.add_argument('--duration',type=float,default=35)
parser.add_argument('--warmup',type=float,default=5)
parser.add_argument('--output',required=True)
parser.add_argument('--label',default='stationary_level')
args=parser.parse_args()
if args.warmup < 0 or args.duration <= args.warmup:
 parser.error('require duration > warmup >= 0')
rclpy.init()
n=Node('chassis_imu_observer')
start=time.monotonic()
imu=[]; odom=[]; diag=[]
def vec(v): return [v.x,v.y,v.z]
def on_imu(m):
 now=time.monotonic(); stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 imu.append({'t':now-start,'stamp':stamp,'age_ms':(time.time()-stamp)*1000,'gyro':vec(m.angular_velocity),'accel':vec(m.linear_acceleration),'frame':m.header.frame_id,'orientation_cov0':m.orientation_covariance[0]})
def on_odom(m):
 odom.append({'t':time.monotonic()-start,'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'vx':m.twist.twist.linear.x,'wz':m.twist.twist.angular.z})
def on_diag(m): diag.append({'t':time.monotonic()-start,'data':json.loads(m.data)})
n.create_subscription(Imu,'/imu/data',on_imu,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
n.create_subscription(Odometry,'/odom',on_odom,30)
n.create_subscription(String,'/chassis/diagnostics',on_diag,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
publishers_seen=set()
next_graph_check=0
next_report=5
while time.monotonic()-start<args.duration:
 rclpy.spin_once(n,timeout_sec=0.1)
 if time.monotonic()-start>=next_graph_check:
  publishers_seen.update(p.node_name for p in n.get_publishers_info_by_topic("/cmd_vel"))
  next_graph_check+=1
 if time.monotonic()-start>=next_report:
  print(json.dumps({'elapsed_s':round(time.monotonic()-start,1),'imu_samples':len(imu),'odom_samples':len(odom),'diagnostics':diag[-1]['data'] if diag else None}),flush=True)
  next_report+=10

def stats(values):
 return {'mean':statistics.mean(values),'std':statistics.stdev(values) if len(values)>1 else 0,'min':min(values),'max':max(values)} if values else None

def rate(rows,key='t'):
 ts=[x[key] for x in rows]
 gaps=[b-a for a,b in zip(ts,ts[1:])]
 return {'samples':len(ts),'hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None,'max_interval_ms':max(gaps)*1000 if gaps else None,'nonpositive_intervals':sum(g<=0 for g in gaps)}
valid=[x for x in imu if x['t']>=args.warmup]
summary={'cmd_vel_publishers_seen': sorted(publishers_seen), 'label':args.label,'duration_s':time.monotonic()-start,'warmup_s':args.warmup,'imu_receive':rate(valid),'imu_header':rate(valid,'stamp'),'odom_receive':rate([x for x in odom if x['t']>=args.warmup]),'cmd_vel_publishers':[p.node_name for p in n.get_publishers_info_by_topic('/cmd_vel')],'last_diagnostic':diag[-1]['data'] if diag else None}
if valid:
 summary.update({'gyro_rad_s':[stats([x['gyro'][i] for x in valid]) for i in range(3)],'gyro_mean_deg_s':[statistics.mean(x['gyro'][i] for x in valid)*180/math.pi for i in range(3)],'accel_m_s2':[stats([x['accel'][i] for x in valid]) for i in range(3)],'accel_norm':stats([math.sqrt(sum(v*v for v in x['accel'])) for x in valid]),'stamp_age_ms':stats([x['age_ms'] for x in valid]),'frame_ids':sorted(set(x['frame'] for x in valid)),'orientation_cov0':sorted(set(x['orientation_cov0'] for x in valid))})
 for name,window in [('first_half',valid[:len(valid)//2]),('second_half',valid[len(valid)//2:])]:
  if window: summary[name+'_gyro_mean_rad_s']=[statistics.mean(x['gyro'][i] for x in window) for i in range(3)]
report={'summary':summary,'imu_samples':imu,'odom_samples':odom,'diagnostics':diag,'motion_commands_sent':False}
Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
print('SUMMARY '+json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if len(valid)>10 else 2)
