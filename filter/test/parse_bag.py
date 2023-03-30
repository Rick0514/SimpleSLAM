import rospy
import rosbag
import tf.transformations as tr

from nav_msgs.msg import Odometry

ekf_topic = '/ekf_odom'
gt_topic = '/odom'

def odom2tum(t, msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # t x y z qx qy qz qw
    return '{} {} {} {} {} {} {} {}\n'.format(t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)

bag = rosbag.Bag('./data/arb.bag')
ekf_fn = open('./data/ekf.txt', 'w')
gt_fn = open('./data/gt.txt', 'w')

for topic, msg, t in bag.read_messages(topics=[ekf_topic, gt_topic]):
    t = rospy.Time.to_sec(t)
    if topic == ekf_topic:
        ekf_fn.write(odom2tum(t, msg))        
    elif topic == gt_topic:
        gt_fn.write(odom2tum(t, msg))         

bag.close()
ekf_fn.close()
gt_fn.close()
