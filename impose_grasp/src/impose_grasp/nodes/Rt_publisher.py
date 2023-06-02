import rospy
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
import numpy as np

from impose_grasp.models.cameras.realsense_D415 import D415
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.nodes.node_utils import create_detector

class RtPublisher:
    def __init__(self):
        rospy.init_node('Rt_publisher', anonymous=True)

        self._obj_detector = create_detector() # should load the paths from a json file, not hard coded
        self._camera = D415(name="realsense_D415")
        self._camera.start()

        self.pub = rospy.Publisher('Rt_topic', numpy_msg(Float32MultiArray), queue_size=10)
    
    def publish_Rt(self):
        """
        Grabs a frame, computes the Rt matrix and publishes it through ROS
        """
        frame = self._grab_frame_from_camera()
        Rt = self._get_Rt_from_frame(frame)

        if(Rt == None):
            Rt = np.zeros((2,2))
        msg = Float32MultiArray()
        msg.data = Rt.flatten().tolist()
        self.pub.publish(msg)

    def test_publish_Rt(self, rgb, dpt):
        frame = CamFrame
        frame.rgb = rgb
        frame.depth = dpt

        Rt = self._get_Rt_from_frame(frame)
        if(type(Rt) == None):
            Rt = np.zeros((2,2))
        msg = Float32MultiArray()
        msg.data = Rt.flatten().tolist()
        self.pub.publish(msg)
       

    def _grab_frame_from_camera(self) -> CamFrame:
        """ 
        Grabs a frame to detect the object.
        """
        frame = self._camera.grab_frame()
        if frame is None:
            print("No frame was grabbed check if camera is connected")
        else:
            return frame
        
    def _get_Rt_from_frame(self, frame: CamFrame) -> np.ndarray:
        """
        Computes the Rt (rotation and translation) matrix for the object in the given frame. It also returns it.
        """
        self._obj_detector.compute_pose_in_frame(frame)
        return self._obj_detector.get_Rt()