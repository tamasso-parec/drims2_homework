from drims2_homework.vision_lib import *


# --- Params (tune once) ---
HSV_LO = np.array([18, 80, 80])     # yellow lower bound
HSV_HI = np.array([35, 255, 255])   # yellow upper bound

class DiceIdentificationNode(Node):
    def __init__(self):
        super().__init__('dice_identification_node', use_global_arguments=False)

        self.rgb_subscribe = self.create_subscription(
            Image,
            '/color/video/image',
            self.rgb_callback,
            10
        )

        self.dice_identification_srv = self.create_service(DiceIdentification, 'dice_identification', self.dice_identification_callback)

        self.background_limit = None

        self.dice_position = None
        self.dice_angle = None
        self.face_number = None

        # --- Camera intrinsics
        self.intrinsics = np.array(
            [[1.57855692e+03, 0.00000000e+00, 9.52440456e+02],
             [0.00000000e+00, 1.58246225e+03, 5.45655012e+02],
             [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]],
            dtype=np.float64
        )

        # ---Distortion coefficients (k1, k2, p1, p2, k3)
        self.dist_coeffs = np.array([0.07206577, 0.08106335, 0.00300317, 0.00042163, -0.40383728], dtype=np.float64)

        #--- Extrinsics
        self.extrinsics = np.array([[0.998855,-0.019105,0.043869,-0.236381],
                                    [0.020615,0.999201,-0.034238,-0.214228],
                                    [-0.043180, 0.035103,0.998450,0.742136],
                                    [0.0,0.0,0.0,1.0]],dtype=np.float64)


    def rgb_callback(self, msg):
        # self.get_logger().info(f"Received RGB image with size: {msg.width}x{msg.height}")

        original_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')

        

        original_image = cv2.undistort(original_image, self.intrinsics, self.dist_coeffs)

        if self.background_limit == None:

            # Segment the background of the dice
            mask = self.segment_background(original_image)

            # Extract the contour to identify where the dice will be
            contours, hierarchy = self.contour_background(mask)

            # cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)

            # Get bounding box of the contour in order to crop the image
            self.background_limit = self.bound_background(contours, hierarchy)
            # Add padding to the background limit
            pad = 20  # pixels of padding
            x, y, w, h = self.background_limit
            height, width = original_image.shape[:2]
            x = max(x - pad, 0)
            y = max(y - pad, 0)
            w = min(w + 2 * pad, width - x)
            h = min(h + 2 * pad, height - y)
            self.background_limit = (x, y, w, h)

            

        # Crop the image so that only the green screen and the dice are visible
        xback, yback, wback, hback = self.background_limit
        cv2.rectangle(original_image, (xback, yback), (xback + wback, yback + hback), (255, 0, 0), 2)

        # Crop the image to the bounding box
        cv_image = original_image[yback:yback+hback, xback:xback+wback]


        # Find the die
        mask = self.segment_die(cv_image)

        # mask_copy = mask.copy()

        # mask_copy = cv2.resize(mask_copy, (640, 480))

        # cv2.imshow("Die Mask", mask_copy)

        # Get die contours
        die_contour, die_hierarchy = self.find_die_contour(mask)

        # Choose the OUTER die contour (largest area) to define the top polygon
        outer_idx = np.argmax([cv2.contourArea(c) for c in die_contour])
        die_outer = die_contour[outer_idx]

        # Build top-face polygon and a "safe" mask inside it
        top_poly, rotatedRect = self.minarea_top_polygon(die_outer, shrink=1.0)
        top_mask = self.polygon_mask(cv_image.shape, top_poly, erode_px=4)

        rect_size =  rotatedRect[1]
        rect_angle = rotatedRect[2]

        # Convert top_poly (4 points) to integer coordinates

        # Draw the top face polygon on the image
        cv2.polylines(cv_image, [top_poly], isClosed=True, color=(0, 255, 0), thickness=2)



        # (Optional) visualize top polygon
        # cv2.polylines(cv_image, [top_poly], True, (255, 0, 0), 2)

        # Select internal contours (those with a parent, i.e., hierarchy[0][i][3] != -1)
        internal_contours = [cnt for i, cnt in enumerate(die_contour) if die_hierarchy[0][i][3] != -1]

        dots_number = 0
        mean = np.zeros((2,))
        # For each internal contour, fit an ellipse
        for cnt in internal_contours:
            if len(cnt) < 5:
                continue 

            if not np.all(top_mask[cnt[:, 0, 1], cnt[:, 0, 0]]):
                continue
            # Fit ellipse requires at least 5 points
            ellipse = cv2.fitEllipse(cnt)

            center = np.array(ellipse[0])

            # Retranslate the center to the initial image using xback, yback
            center_translated = center + np.array([xback, yback])

            
            mean += center 

            cv2.ellipse(cv_image, ellipse, (0, 0, 255), 2)

            dots_number +=1

        mean_point = None

        if dots_number > 0:
            mean = mean / dots_number
            mean_point = tuple(np.round(mean).astype(int))
            cv2.circle(cv_image, mean_point, 8, (255, 0, 255), -1)

            # Draw an arrow indicating the direction of the dice

            # Calculate arrow length proportional to the rectangle size
            arrow_length = int(max(rect_size) * 2.0)

            # Calculate direction vector based on angle
            angle_rad = np.deg2rad(rect_angle)
            dx = int(arrow_length * np.cos(angle_rad))
            dy = int(arrow_length * np.sin(angle_rad))

            arrow_tip = (mean_point[0] + dx, mean_point[1] + dy)

            cv2.arrowedLine(cv_image, mean_point, arrow_tip, (0, 255, 255), 3, tipLength=0.2)

        else: 
            return 
        
        self.dice_position = mean_point

        self.face_number = dots_number

        self.dice_angle = rect_angle

        print("Face: ", dots_number)
        cv_image = cv2.resize(original_image, (640, 480))
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
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        mask = cv2.dilate(mask, np.ones((7,7), np.uint8), iterations=1)
        mask = cv2.erode(mask, np.ones((3,3), np.uint8), iterations=1)

        # mask = cv2.dilate(mask, np.ones((3,3), np.uint8), iterations=1)

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
    
    def minarea_top_polygon(self, die_outer_cnt, shrink=0.72):
        """
        Return a 4-pt polygon (np.int32) that approximates the TOP face,
        by shrinking the minAreaRect so lateral faces fall outside.
        """
        rect = cv2.minAreaRect(die_outer_cnt)       # (center, (w,h), angle)
        # box = cv2.boxPoints(rect).astype(np.float32)  # 4x2
        box = cv2.boxPoints(rect)  # 4x2

        # Shrink around the rectangle center
        c = box.mean(axis=0, keepdims=True)          # 1x2
        box_shrunk = (box - c) * shrink + c
        return box_shrunk.astype(np.int32), rect

    
    def polygon_mask(self, shape, poly, erode_px=4):
        """
        Create a mask for the polygon and erode to enforce a margin so
        side pips near edges are discarded.
        """
        m = np.zeros(shape[:2], dtype=np.uint8)
        cv2.fillPoly(m, [poly], 255)
        if erode_px > 0:
            m = cv2.erode(m, np.ones((erode_px, erode_px), np.uint8), iterations=1)
        return m


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
    
    def get_TF_cam_dice(self): 

        # Convert 2D pixel to normalized camera coordinates

        if self.dice_position is None:
            raise ValueError("Dice position is not available")
        
        x_pixel, y_pixel = self.dice_position
        Z_cam_dice = 0.742136 - 0.05

        X = Z_cam_dice * (x_pixel - self.intrinsics[0, 2]) / self.intrinsics[0, 0]
        Y = Z_cam_dice * (y_pixel - self.intrinsics[1, 2]) / self.intrinsics[1, 1]

        theta = np.deg2rad(self.dice_angle)
        Rz = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])

        tf_cam_dice = np.eye(4)
        tf_cam_dice[:3, :3] = Rz
        tf_cam_dice[:3, 3] = [X, Y, Z_cam_dice]

        return tf_cam_dice

    def cam_to_world(self, tf_cam_dice):

        tf_world_dice = self.extrinsics @ tf_cam_dice

        
        return tf_world_dice
    
    def dice_identification_callback(self , request, response):

        try: 
                
            pose = PoseStamped()

            tf_world_dice = self.cam_to_world(self.get_TF_cam_dice())

        except:
            
            self.get_logger().error("Failed to identify dice")
            response.success = False
            return response


        r_world_dice = R.from_matrix(tf_world_dice[:3, :3]).as_quat()  # x, y, z, w
        t_world_dice = tf_world_dice[:3, 3]

        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = t_world_dice[0]
        pose.pose.position.y = t_world_dice[1]
        pose.pose.position.z = t_world_dice[2]
        pose.pose.orientation.x = r_world_dice[0]
        pose.pose.orientation.y = r_world_dice[1]
        pose.pose.orientation.z = r_world_dice[2]
        pose.pose.orientation.w = r_world_dice[3]

        response.pose = pose

        response.face_number = self.face_number

        response.success = True

        
        return response

        



def main():
    rclpy.init()

    # MotionClient is already a Node, so we can use its logger directly
    
    dice_id_node = DiceIdentificationNode()

    rclpy.spin(dice_id_node)

    dice_id_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
