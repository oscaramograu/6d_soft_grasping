import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from rospy.numpy_msg import numpy_msg
import numpy as np

from impose_grasp.models.cameras.realsense_D415 import D415
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.networks.detector import PositionDetector

class RtPublisher:
    def __init__(self, freq: int = 10):
        rospy.init_node('obj_to_cam_Rt_publisher', anonymous=True)
        self.rate = rospy.Rate(freq)  # Hz       

        self._obj_detector = PositionDetector("cpsduck")
        try:
            self._camera = D415(name="realsense_D415")
            self._camera.start()
        except:
            print("Camera could not be found")

        self.pub = rospy.Publisher('obj_to_cam_Rt', numpy_msg(Float32MultiArray), queue_size=10)
    
    def publish_Rt(self):
        """
        Grabs a frame, computes the Rt matrix and publishes it through ROS
        """
        while not rospy.is_shutdown():
            frame = self._grab_frame_from_camera()
            Rt = self._get_Rt_from_frame(frame)

            if(Rt == None):
                Rt = np.zeros((2,2))
            msg = Float32MultiArray()
            msg.data = Rt.flatten().tolist()
            self.pub.publish(msg)

            self.rate.sleep()

    def test_publish_Rt(self, rgb, dpt):
        while not rospy.is_shutdown():
            frame = CamFrame
            frame.rgb = rgb
            frame.depth = dpt

            Rt = self._get_Rt_from_frame(frame)
            if(type(Rt) == None):
                Rt = np.zeros((2,2))
            msg = self._numpy_to_array_msg(Rt)
            self.pub.publish(msg)

            self.rate.sleep()

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
        detected, affine_matrix = self._obj_detector.detect_object(frame)
        if detected:
            return affine_matrix
        else:
            return None
    
    def _numpy_to_array_msg(self, np_array: np.ndarray) -> Float32MultiArray:
        """
        Converts a numpy array to a Float32Multiarray msg type to be sent with ros.
        """
        row, col = np_array.shape
        row_dim, col_dim = MultiArrayDimension("rows", row, row*col), MultiArrayDimension("columns", col, col)
        dim_list = [row_dim, col_dim]

        layout = MultiArrayLayout(dim_list, 0)
        data = np_array.flatten().tolist()

        array_msg = Float32MultiArray(layout, data)

        return array_msg     