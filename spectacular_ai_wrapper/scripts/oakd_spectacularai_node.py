"""
Runs spectacularAI mapping and publishes poses and frames in ROS.
"""
import spectacularAI
import depthai as dai
import rospy
import numpy as np
import time
import os
import tf
import yaml
import rospkg
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from spectacular_ai_wrapper.message_converter import to_pose_message, to_tf_message

class OAKDPublisherNode:
    def __init__(self):
        # ROS Node Variables
        rospy.init_node("oakd_publisher_node", anonymous=True)
        rospy.loginfo("ROS Node Initialized")

        # Load YAML Configuration
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('spectacular_ai_wrapper')
        config_path = os.path.join(package_path, 'config', 'oakd_config.yaml')
        with open(config_path, 'r') as yaml_file:
            config = yaml.safe_load(yaml_file)

        # ROS Topics
        self.odometry_publisher = rospy.Publisher(config['rostopic']['odometry'], PoseStamped, queue_size=10)
        self.keyframe_publisher = rospy.Publisher(config['rostopic']['keyframe'], PoseStamped, queue_size=10)
        self.rgb_publisher = rospy.Publisher(config['rostopic']['rgb'], Image, queue_size=10)
        self.tf_publisher = rospy.Publisher(config['rostopic']['tf'], TFMessage, queue_size=10)
        self.point_publisher = rospy.Publisher(config['rostopic']['pointcloud'], PointCloud2, queue_size=10)
        self.depth_publisher = rospy.Publisher(config['rostopic']['depth'], Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher(config['rostopic']['camera_info'], CameraInfo, queue_size=10)
        self.bridge = CvBridge()
        self.rgb_frame_id = config['frame']['rgb']
        self.world_frame_id = config['frame']['world']
        self.rgb_encoding = config['encoding']['rgb']
        self.depth_encoding = config['encoding']['depth']

        # OAK-D Pipeline Variables
        self.pipeline = dai.Pipeline()
        self.device = None
        self.calibration_data = None
        self.keyframes = {}
        self.rgb_queue = None
        self.rgb_queue_name = config['rgb_queue']
        self._add_rgb_camera()
        self.config_internal_params = config['config_internal_params']
        self.config = self._create_vio_pipeline_config()
        self.vioPipeline = self._configure_vio_pipeline()
        self.image_width = config['image']['width']
        self.image_height = config['image']['height']

    ################
    # INIT METHODS #
    ################
    def _create_vio_pipeline_config(self):
        config = spectacularAI.depthai.Configuration()
        config.internalParameters = self.config_internal_params
        config.useSlam = True
        return config

    def _configure_vio_pipeline(self):
        vio_pipeline = spectacularAI.depthai.Pipeline(self.pipeline, self.config, self._mapping_callback)
        vio_pipeline.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        vio_pipeline.stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        vio_pipeline.stereo.setLeftRightCheck(True)
        vio_pipeline.stereo.setExtendedDisparity(False)
        vio_pipeline.stereo.setSubpixel(True)
        return vio_pipeline

    def _add_rgb_camera(self):
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        camRgb.setIspScale(5, 16)
        xoutRgb.setStreamName(self.rgb_queue_name)
        camRgb.video.link(xoutRgb.input)

    def start_device(self):
        self.device = dai.Device(self.pipeline)
        self.rgb_queue = self.device.getOutputQueue(name=self.rgb_queue_name, maxSize=4, blocking=False)

    ##################
    # ROS PUBLISHERS #
    ##################
    def _publish_odometry(self, camToWorld):
        msg = to_pose_message(camToWorld, self.world_frame_id)
        self.odometry_publisher.publish(msg)

    def _publish_tf(self, camToWorld, timestamp):
        tf_message = to_tf_message(camToWorld, timestamp, self.rgb_frame_id, self.world_frame_id)
        self.tf_publisher.publish(tf_message)         
    
    def _publish_depth(self, depth_image, timestamp, sequence_number):
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding=self.depth_encoding)
        depth_msg.header.stamp = timestamp
        depth_msg.header.frame_id = self.rgb_encoding
        depth_msg.header.seq = sequence_number
        self.depth_publisher.publish(depth_msg)

    def _publish_keyframe(self, camToWorld, timestamp, sequence_number):
        msg = to_pose_message(camToWorld, self.world_frame_id)
        msg.header.seq = sequence_number
        msg.header.stamp = timestamp
        self.keyframe_publisher.publish(msg)

    def _publish_point_cloud(self, keyframe, camToWorld, timestamp):
        positions = keyframe.pointCloud.getPositionData()
        pc = np.zeros((positions.shape[0], 6), dtype=np.float32)
        p_C = np.vstack((positions.T, np.ones((1, positions.shape[0])))).T
        pc[:, :3] = (camToWorld @ p_C[:, :, None])[:, :3, 0]

        msg = PointCloud2()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.world_frame_id
        if keyframe.pointCloud.hasColors():
            pc[:, 3:] = keyframe.pointCloud.getRGB24Data() * (1. / 255.)
        msg.point_step = 4 * 6
        msg.height = 1
        msg.width = pc.shape[0]
        msg.row_step = msg.point_step * pc.shape[0]
        msg.data = pc.tobytes()
        msg.is_bigendian = False
        msg.is_dense = False
        ros_dtype = PointField.FLOAT32
        itemsize = np.dtype(np.float32).itemsize
        msg.fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyzrgb')]
        self.point_publisher.publish(msg)
    
    def _publish_rgb(self, device):
        in_rgb = self.rgb_queue.tryGet()
        if in_rgb is not None:
            rgb_frame = in_rgb.getCvFrame()
            if rgb_frame is not None:
                try:
                    # Convert the OpenCV image to a ROS Image message
                    rgb_message = self.bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
                    self.rgb_publisher.publish(rgb_message)
                except Exception as e:
                    rospy.logerr("Failed to convert and publish RGB frame: %s", str(e))

    def _keyframe_publishers(self, frame_id, keyframe):
        self.keyframes[frame_id] = True
        timestamp = rospy.Time.now()
        sequence_number = int(frame_id)
        camToWorld = keyframe.frameSet.rgbFrame.cameraPose.getCameraToWorldMatrix()
        depth_frame = keyframe.frameSet.getAlignedDepthFrame(keyframe.frameSet.rgbFrame)
        depth_image = depth_frame.image.toArray()
        
        # Publishers
        self._publish_keyframe(camToWorld, timestamp, sequence_number)
        self._publish_depth(depth_image, timestamp, sequence_number)
        self._publish_point_cloud(keyframe, camToWorld, timestamp)
        
    ###########
    # GETTERS #
    ###########
    def _has_keyframe(self, frame_id):
        return frame_id in self.keyframes
    
    def get_camera_info_msg(self, device):
        self.calibration_data = device.readCalibration()
        intrinsics = np.array(self.calibration_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.image_width, self.image_height))
        camera_info_msg = CameraInfo()
        camera_info_msg.K = intrinsics.ravel().tolist()
        camera_info_msg.width = self.image_width
        camera_info_msg.height = self.image_height
        camera_info_msg.header.frame_id = self.rgb_frame_id
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.distortion_model = "none"
        camera_info_msg.D = []
        return camera_info_msg 

    #############
    # CALLBACKS #
    #############
    def _vio_callback(self, vio_output):
        camera_pose = vio_output.getCameraPose(0)
        cam_to_world = camera_pose.getCameraToWorldMatrix()
        self._publish_odometry(cam_to_world)
        self._publish_tf(cam_to_world, rospy.Time.now())

    def _mapping_callback(self, output):
        for frame_id in output.updatedKeyFrames:
            key_frame = output.map.keyFrames.get(frame_id)
            if not key_frame or not key_frame.pointCloud:
                continue
            if not self._has_keyframe(frame_id):
                self._keyframe_publishers(frame_id, key_frame)
        if output.finalMap:
            rospy.loginfo("Final map ready!")

    ########
    # MAIN #
    ########
    def run(self):
        rospy.loginfo("Starting OAK-D device")
        self.start_device()
        with self.device as device, self.vioPipeline.startSession(device) as vio_session:
            camera_info_msg = self.get_camera_info_msg(device)
            rospy.loginfo("OAK-D device started")
            while not rospy.is_shutdown():
                self._vio_callback(vio_session.waitForOutput())
                self._publish_rgb(device)
                self.camera_info_publisher.publish(camera_info_msg)

if __name__ == "__main__":
    oakd_publisher_node = OAKDPublisherNode()
    oakd_publisher_node.run()