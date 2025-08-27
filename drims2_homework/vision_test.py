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


# --- Params (tune once) ---
HSV_LO = np.array([18, 80, 80])     # yellow lower bound
HSV_HI = np.array([35, 255, 255])   # yellow upper bound

class VisionTest(Node):
    def __init__(self):
        super().__init__('vision_test_node', use_global_arguments=False) 

        self.rgb_subscribe = self.create_subscription(
            Image,
            '/color/video/image',
            self.rgb_callback,
            10
        )

        self.background_limit = None

    def rgb_callback(self, msg):
        # self.get_logger().info(f"Received RGB image with size: {msg.width}x{msg.height}")

        cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.background_limit == None:

            # Segment the background of the dice
            mask = self.segment_background(cv_image)

            # Extract the contour to identify where the dice will be
            contours, hierarchy = self.contour_background(mask)

            # cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)

            # Get bounding box of the contour in order to crop the image
            self.background_limit = self.bound_background(contours, hierarchy)

            

        # Crop the image so that only the green screen and the dice are visible
        x, y, w, h = self.background_limit
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Crop the image to the bounding box
        cv_image = cv_image[y:y+h, x:x+w]


        # Find the die
        mask = self.segment_die(cv_image)

        # Get die contours
        die_countour, die_hierarchy = self.find_die_contour(mask)
        # Select internal contours (those with a parent, i.e., hierarchy[0][i][3] != -1)
        internal_contours = [cnt for i, cnt in enumerate(die_countour) if die_hierarchy[0][i][3] != -1]

        # For each internal contour, fit an ellipse
        for cnt in internal_contours:
            if len(cnt) >= 5:  # Fit ellipse requires at least 5 points
                ellipse = cv2.fitEllipse(cnt)
                cv2.ellipse(cv_image, ellipse, (0, 0, 255), 2)

        circle_contr, circle_hier = self.find_circles(mask)

        # cv2.drawContours(cv_image, internal_contours, -1, (0,255,0), 3)
        # print(len(countour))



        # Draw contours on the image
        # cv2.drawContours(cv_image, countour, -1, (0,255,0), 3)

        # cv2.drawContours(cv_image, circle_conts, -1, (0,0,255), 3)

        cv_image = cv2.resize(cv_image, (640, 480))
        # # cv_image = mask



        cv2.imshow("Mask", cv_image)
        cv2.waitKey(1)

    def segment_background(self, image): 

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Green color bounds in HSV
        # These values can be tuned depending on lighting and camera
        green_low = np.array([35, 100, 100])
        green_high = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, green_low, green_high)


        return mask
    
    def contour_background(self, mask): 
        cnts, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        return cnts, hierarchy
    
    def bound_background(self, contours, hierarchy): 
        if len(contours) == 0:
            return None

        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Get the bounding rectangle for the largest contour
        bbox  = cv2.boundingRect(largest_contour) #x, y, w, h

        return bbox

    def segment_die(self, image): 
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LO, HSV_HI)
        # mask = cv2.dilate(mask, np.ones((5,5), np.uint8), iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

        return mask
    
    def find_die_contour(self, mask):
        # cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        
        return cnts, hierarchy
    
    def find_circles(self, mask): 

        el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        mask = cv2.morphologyEx(mask,  cv2.MORPH_DILATE, el)

        cnts, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        return cnts, hierarchy

    def dice_identification(self):
        if not self.dice_identification_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("DiceIdentification service not available")

        request = DiceIdentification.Request()

        result_future = self.dice_identification_client.call_async(request)
        rclpy.spin_until_future_complete(self, result_future)
        
        face_number = result_future.result().face_number
        pose = result_future.result().pose
        success = result_future.result().success

        return face_number, pose, success



def main():
    rclpy.init()

    # MotionClient is already a Node, so we can use its logger directly
    
    vision_test_node = VisionTest()

    rclpy.spin(vision_test_node)

    vision_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
