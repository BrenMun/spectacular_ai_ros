from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo

def to_pose_message(camToWorld, world_frame_id):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = world_frame_id
    msg.pose.position.x = camToWorld[0, 3]
    msg.pose.position.y = camToWorld[1, 3]
    msg.pose.position.z = camToWorld[2, 3]
    R_CW = Rotation.from_matrix(camToWorld[0:3, 0:3])
    q_cw = R_CW.as_quat()
    msg.pose.orientation.x = q_cw[0]
    msg.pose.orientation.y = q_cw[1]
    msg.pose.orientation.z = q_cw[2]
    msg.pose.orientation.w = q_cw[3]
    return msg

def to_tf_message(camToWorld, ts, frame_id, world_frame_id):
    msg = TFMessage()
    msg.transforms = []
    transform = TransformStamped()
    transform.header.stamp = ts
    transform.header.frame_id = world_frame_id
    transform.child_frame_id = frame_id
    transform.transform.translation.x = camToWorld[0, 3]
    transform.transform.translation.y = camToWorld[1, 3]
    transform.transform.translation.z = camToWorld[2, 3]
    R_CW = Rotation.from_matrix(camToWorld[0:3, 0:3])
    q_cw = R_CW.as_quat()
    transform.transform.rotation.x = q_cw[0]
    transform.transform.rotation.y = q_cw[1]
    transform.transform.rotation.z = q_cw[2]
    transform.transform.rotation.w = q_cw[3]
    msg.transforms.append(transform)
    return msg