import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from drims2_motion_server.motion_client import MotionClient
from drims2_msgs.srv import DiceIdentification
from moveit_msgs.msg import MoveItErrorCodes

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock


from scipy.spatial.transform import Rotation as R


class RobotCameraParams():
    def __init__(self,  robot_number=0, name = "",  intrinsics= np.zeros((3,3)),  dist_coeffs =  np.array((5,)), extrinsics = np.eye(4)):
        self.name = name
        self.robot_number = robot_number
        self.intrinsics = intrinsics
        self.dist_coeffs = dist_coeffs
        self.extrinsics = extrinsics


CAMERA_1 = RobotCameraParams(
    robot_number= 1,
    name = "Red Hand",
    intrinsics = np.array
                ([[1.54588237e+03, 0.00000000e+00, 9.46934926e+02],
                [0.00000000e+00, 1.55502898e+03, 5.44959477e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],
                dtype
                =np.float64),
#---Distortion coefficients (k1, k2, p1, p2, k3)
    dist_coeffs = np.array
                ([ 0.13503889, - 0.28098708, 0.0003601, - 0.00038133, 0.13643112],
                dtype
                =np.float64),
    extrinsics =np.array([[0.999831,-0.015292,-0.010167,-0.062168],
                          [0.015271, 0.999881,-0.002154,-0.071062],
                          [0.010198, 0.001998, 0.999946, 0.717149],
                          [0.0,0.0,0.0,1.0]],dtype=np.float64)
)
CAMERA_2 = RobotCameraParams(
    robot_number= 2,
    name = "Black Bar",
    intrinsics = np.array
                ([[1.57855692e+03, 0.00000000e+00, 9.52440456e+02],
                [0.00000000e+00, 1.58246225e+03, 5.45655012e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],
                dtype
                =np.float64),
#---Distortion coefficients (k1, k2, p1, p2, k3)
    dist_coeffs = np.array
                ([ 0.07206577, 0.08106335, 0.00300317, 0.00042163,-0.40383728],
                dtype
                =np.float64),
    extrinsics =np.array([[0.999581,-0.018623, 0.022175,-0.013014],
                          [0.019132, 0.999553,-0.022993,-0.037747],
                          [-0.021737, 0.023408, 0.999490, 0.737709],
                          [0.0,0.0,0.0,1.0]],dtype=np.float64)
)
CAMERA_3 = RobotCameraParams(
    robot_number= 3,
    name = "TIAGO",
    intrinsics = np.array
                ([[1.58947500e+03, 0.00000000e+00, 9.50230660e+02],
                [0.00000000e+00, 1.59544888e+03, 5.19907455e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],
                dtype
                =np.float64),
#---Distortion coefficients (k1, k2, p1, p2, k3)
    dist_coeffs = np.array
                ([  0.0851912,0.05857935,-0.00558687,-0.00350163,-0.36887004] ,
                dtype
                =np.float64),
    extrinsics =np.array([[0.999165,-0.031847, 0.025596,-0.006487],
                          [0.031435, 0.999373, 0.016305,-0.036230],
                          [-0.026099,-0.015486, 0.999539, 0.766089],
                          [0.0,0.0,0.0,1.0]],dtype=np.float64)
)

__all__ = [
    "rclpy",
    "Node",
    "PoseStamped",
    "MotionClient",
    "DiceIdentification",
    "MoveItErrorCodes",
    "Image",
    "cv2",
    "CvBridge",
    "np", 
    "R", 
    "TransformBroadcaster", 
    "TransformStamped", 
    "Clock",
    "CAMERA_1", 
    "CAMERA_2",
    "CAMERA_3"
]