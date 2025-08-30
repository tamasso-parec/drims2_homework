import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from drims2_motion_server.motion_client import MotionClient
from drims2_msgs.srv import DiceIdentification
from moveit_msgs.msg import MoveItErrorCodes
from scipy.spatial.transform import Rotation as R
import numpy as np

HANDE_ACTION_NAME = '/gripper_action_controller/gripper_cmd'

class VisionClientNode(Node):
    def __init__(self):
        super().__init__('vision_client_node', use_global_arguments=False) 
        self.dice_identification_client = self.create_client(DiceIdentification, 
                                                             'dice_identification')

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

class Brain():
    def __init__(self):
        self.trial = 0
        self.done = False
        self.special_case = False
        self.roll_side = False

    def process_dice_action(self, pose, face_number, desired_face):

        
        self.trial += 1
        print(f"--------------------------- trial {self.trial} --------------------------")
        if self.trial == 2 and not self.done and not self.special_case:
            print("########roll side case activated##########")
            self.roll_side = True
            # self.trial = 0
        if face_number == desired_face:
            self.done = True
        print(f"face number isss {face_number} and desired {desired_face}")
        if face_number + desired_face == 7:
            self.special_case = True

            # Special processing for trial 2
            print("Special processing for trial 2")
        else:
            print("Regular processing")

        return 1

def main():

    rclpy.init()

    desired_face = 3

    brain = Brain()

    # MotionClient is already a Node, so we can use its logger directly
    motion_client_node = MotionClient(gripper_action_name=HANDE_ACTION_NAME)
    vision_client_node = VisionClientNode()
    demo_node          = rclpy.create_node(node_name="demo_node", use_global_arguments=False)

    logger = demo_node.get_logger()

    # --- 0) Move to home configuration and open gripper---
    home_joints = list(np.deg2rad([90, -120, 130, -90, -90, 0.0]))

    logger.info("Moving to home configuration...")
    result = motion_client_node.move_to_joint(home_joints)
    logger.info(f"Home reached: {result}")
    reached_goal, stalled = motion_client_node.gripper_command(position=0.0)  # Open gripper

    
    if result.val != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach home configuration: {result}")
        motion_client_node.destroy_node()
        vision_client_node.destroy_node()
        demo_node.destroy_node()
        rclpy.shutdown()
        return 0
    face_number, dice_pose, success = vision_client_node.dice_identification()

    print(f"face number is {face_number}")

    # print(f"face number isss {face_number} and desired {desired_face}")

    brain.process_dice_action(dice_pose, face_number, desired_face)
    print(f"required action is special {brain.special_case}// side wise roll {brain.roll_side} and done {brain.done}")
    # --- 1) Dice Identification
    X = False
    Y = False
    Z = False
    i = 0
    while brain.done == False:
        
        print(f"dice pose is {dice_pose}")
        if not success:
            logger.error('Dice identification failed')
            motion_client_node.destroy_node()
            vision_client_node.destroy_node()
            demo_node.destroy_node()
            rclpy.shutdown()
            return 0

        logger.info(f'Dice position {dice_pose}')
        

        if brain.special_case:

            X = True
            i += 1
            # Handle special case
            logger.info("Handling special case...")

        if brain.roll_side and not Z:
            Y = True
            Z = True
            logger.info("Handling roll side case...")
            pick_pose = PoseStamped()
            pick_pose.header.frame_id = "dice_center"
            pick_pose.header.stamp = demo_node.get_clock().now().to_msg()
            pick_pose.pose.position.x = 0.0
            pick_pose.pose.position.y = 0.0
            pick_pose.pose.position.z = 0.012
            pick_pose.pose.orientation.x = 0.0
            pick_pose.pose.orientation.y = 0.0
            pick_pose.pose.orientation.z = 0.0
            pick_pose.pose.orientation.w = 1.0

            logger.info("Moving to pick pose...")
            result = motion_client_node.move_to_pose(pick_pose, cartesian_motion=True)


            
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            
            # --- 3) Close gripper + attach object ---
            logger.info("Closing gripper...")
            reached_goal, stalled = motion_client_node.gripper_command(position=0.67)  # 0.0 = closed
            logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

            logger.info("Attaching object...")
            success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
            logger.info(f"Attach success: {success}")

            r, p, y = 0.0, 0.0, -np.pi/2
            rot = R.from_euler('xyz', [r, p, y]).as_matrix()
            q_xyzw = R.from_matrix(rot).as_quat()


            #### just pick
            place_pose = PoseStamped()
            place_pose.header.frame_id = "tip"
            place_pose.header.stamp = demo_node.get_clock().now().to_msg()
            place_pose.pose.position.x = 0.0
            place_pose.pose.position.y = 0.0
            place_pose.pose.position.z = -0.1
            place_pose.pose.orientation.x = q_xyzw[0]
            place_pose.pose.orientation.y = q_xyzw[1]
            place_pose.pose.orientation.z = q_xyzw[2]
            place_pose.pose.orientation.w = q_xyzw[3]

            logger.info("Moving to place pose...")
            result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
            logger.info(f"Place pose reached: {result}")
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            

            #### rotation around pick
            place_pose = PoseStamped()
            place_pose.header.frame_id = "tip"
            place_pose.header.stamp = demo_node.get_clock().now().to_msg()
            place_pose.pose.position.x = 0.0
            place_pose.pose.position.y = 0.0
            place_pose.pose.position.z = 0.1
            place_pose.pose.orientation.x = 0.0
            place_pose.pose.orientation.y = 0.0
            place_pose.pose.orientation.z = 0.0
            place_pose.pose.orientation.w = 1.0

            logger.info("Moving to place pose...")
            result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
            logger.info(f"Place pose reached: {result}")
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            
            logger.info("Closing gripper...")
            reached_goal, stalled = motion_client_node.gripper_command(position=0.1)  # 0.0 = closed
            logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

            logger.info("Detaching object...")
            success = motion_client_node.detach_object("dice")  # "tool0" is the default TCP frame of UR10e
            logger.info(f"Detach success: {success}")
            

        




        if (((not brain.roll_side and not brain.special_case and not brain.done) or X) or Y) or Z:
            Y = False
            
            pick_pose = PoseStamped()
            pick_pose.header.frame_id = "dice_center"
            pick_pose.header.stamp = demo_node.get_clock().now().to_msg()
            pick_pose.pose.position.x = 0.0
            pick_pose.pose.position.y = 0.0
            pick_pose.pose.position.z = 0.012
            pick_pose.pose.orientation.x = 0.0
            pick_pose.pose.orientation.y = 0.0
            pick_pose.pose.orientation.z = 0.0
            pick_pose.pose.orientation.w = 1.0

            logger.info("Moving to pick pose...")
            result = motion_client_node.move_to_pose(pick_pose, cartesian_motion=True)


            
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            
            # --- 3) Close gripper + attach object ---
            logger.info("Closing gripper...")
            reached_goal, stalled = motion_client_node.gripper_command(position=0.67)  # 0.0 = closed
            logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

            logger.info("Attaching object...")
            success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
            logger.info(f"Attach success: {success}")


            #### just pick
            place_pose = PoseStamped()
            place_pose.header.frame_id = "tip"
            place_pose.header.stamp = demo_node.get_clock().now().to_msg()
            place_pose.pose.position.x = 0.0
            place_pose.pose.position.y = 0.0
            place_pose.pose.position.z = -0.1
            place_pose.pose.orientation.x = 0.0
            place_pose.pose.orientation.y = 0.0
            place_pose.pose.orientation.z = 0.0
            place_pose.pose.orientation.w = 1.0 

            logger.info("Moving to place pose...")
            result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
            logger.info(f"Place pose reached: {result}")
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            

            #### rotation around pick
            r, p, y = -np.pi/2.7, 0.0, 0.0
            rot = R.from_euler('xyz', [r, p, y]).as_matrix()
            q_xyzw = R.from_matrix(rot).as_quat()


            #### rotation around pick
            place_pose = PoseStamped()
            place_pose.header.frame_id = "tip"
            place_pose.header.stamp = demo_node.get_clock().now().to_msg()
            place_pose.pose.position.x = 0.0
            place_pose.pose.position.y = 0.0
            place_pose.pose.position.z = -0.0
            place_pose.pose.orientation.x = q_xyzw[0]
            place_pose.pose.orientation.y = q_xyzw[1]
            place_pose.pose.orientation.z = q_xyzw[2]
            place_pose.pose.orientation.w = q_xyzw[3]

            logger.info("Moving to place pose...")
            result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
            logger.info(f"Place pose reached: {result}")
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            

            #### rotation around pick
            place_pose = PoseStamped()
            place_pose.header.frame_id = "tip"
            place_pose.header.stamp = demo_node.get_clock().now().to_msg()
            place_pose.pose.position.x = 0.0
            place_pose.pose.position.y = np.sin(r)*0.133
            place_pose.pose.position.z = np.cos(r)*0.133
            place_pose.pose.orientation.x = 0.0
            place_pose.pose.orientation.y = 0.0
            place_pose.pose.orientation.z = 0.0
            place_pose.pose.orientation.w = 1.0

            logger.info("Moving to place pose...")
            result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
            logger.info(f"Place pose reached: {result}")
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            
            logger.info("Closing gripper...")
            reached_goal, stalled = motion_client_node.gripper_command(position=0.0)  # 0.0 = closed
            logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

            logger.info("Detaching object...")
            success = motion_client_node.detach_object("dice")  # "tool0" is the default TCP frame of UR10e
            logger.info(f"Detach success: {success}")


            #### Turn back
            r, p, y = -np.pi/2.7, 0.0, 0.0
            rot = R.from_euler('xyz', [r, p, y]).as_matrix()
            q_xyzw = R.from_matrix(rot).as_quat()

            place_pose = PoseStamped()
            place_pose.header.frame_id = "tip"
            place_pose.header.stamp = demo_node.get_clock().now().to_msg()
            place_pose.pose.position.x = 0.0
            place_pose.pose.position.y = 0.0
            place_pose.pose.position.z = 0.0
            place_pose.pose.orientation.x = q_xyzw[0]
            place_pose.pose.orientation.y = q_xyzw[1]
            place_pose.pose.orientation.z = q_xyzw[2]
            place_pose.pose.orientation.w = q_xyzw[3]

            logger.info("Moving to place pose...")
            result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
            logger.info(f"Place pose reached: {result}")
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0
            

            place_pose = PoseStamped()
            place_pose.header.frame_id = "tip"
            place_pose.header.stamp = demo_node.get_clock().now().to_msg()
            place_pose.pose.position.x = 0.0
            place_pose.pose.position.y = 0.0
            place_pose.pose.position.z = -0.1
            place_pose.pose.orientation.x = 0.0
            place_pose.pose.orientation.y = 0.0
            place_pose.pose.orientation.z = 0.0
            place_pose.pose.orientation.w = 1.0

            logger.info("Moving to place pose...")
            result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
            logger.info(f"Place pose reached: {result}")
            if result.val != MoveItErrorCodes.SUCCESS:
                logger.error(f"Failed to reach home configuration: {result.val}")
                motion_client_node.destroy_node()
                vision_client_node.destroy_node()
                demo_node.destroy_node()
                rclpy.shutdown()
                return 0

            # break
        if i == 2:
            X = False
            # break 
            



        # pick_pose = PoseStamped()
        # pick_pose.header.frame_id = "dice_center"
        # pick_pose.header.stamp = demo_node.get_clock().now().to_msg()
        # pick_pose.pose.position.x = 0.0
        # pick_pose.pose.position.y = 0.0
        # pick_pose.pose.position.z = 0.0
        # pick_pose.pose.orientation.x = 1.0
        # pick_pose.pose.orientation.y = 0.0
        # pick_pose.pose.orientation.z = 0.0
        # pick_pose.pose.orientation.w = 0.0


        # logger.info("Moving to pick pose...")
        # result = motion_client_node.move_to_pose(pick_pose, cartesian_motion=True)



        # if result.val != MoveItErrorCodes.SUCCESS:
        #     logger.error(f"Failed to reach home configuration: {result}")
        #     motion_client_node.destroy_node()
        #     vision_client_node.destroy_node()
        #     demo_node.destroy_node()
        #     rclpy.shutdown()
        #     return 0

        # logger.info(f"Pick pose reached: {result}")

        # # --- 3) Close gripper + attach object ---
        # logger.info("Closing gripper...")
        # reached_goal, stalled = motion_client_node.gripper_command(position=0.0)  # 0.0 = closed
        # logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

        # logger.info("Attaching object...")
        # success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
        # logger.info(f"Attach success: {success}")

        # # --- 4) Move to another pose (place pose) ---
        # place_pose = PoseStamped()
        # place_pose.header.frame_id = "tip"
        # place_pose.header.stamp = demo_node.get_clock().now().to_msg()
        # place_pose.pose.position.x = 0.0
        # place_pose.pose.position.y = 0.0
        # place_pose.pose.position.z = -0.1
        # place_pose.pose.orientation.x = 0.0
        # place_pose.pose.orientation.y = 0.0
        # place_pose.pose.orientation.z = 0.0
        # place_pose.pose.orientation.w = 1.0

        # logger.info("Moving to place pose...")
        # result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
        # logger.info(f"Place pose reached: {result}")
        # if result != MoveItErrorCodes.SUCCESS:
        #     logger.error(f"Failed to reach home configuration: {result}")
        #     motion_client_node.destroy_node()
        #     vision_client_node.destroy_node()
        #     demo_node.destroy_node()
        #     rclpy.shutdown()
        #     return 0
        # # --- 6) Open gripper + detach object ---
        # logger.info("Opening gripper...")
        # reached_goal, stalled = motion_client_node.gripper_command(position=0.045)  # 0.045 = open, adjust depending on gripper model
        # logger.info(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")
        
        
        if not X:
            i = 0
            face_number, dice_pose, success = vision_client_node.dice_identification()
        
            A = brain.process_dice_action(dice_pose, face_number, desired_face=desired_face)
            print(f"required action is special {brain.special_case}// side wise roll {brain.roll_side} and done is {brain.done}")

            # --- 5) Homing again

            home_joints = list(np.deg2rad([90, -120, 130, -90, -90, 0.0]))

            logger.info("Moving to home configuration...")
            result = motion_client_node.move_to_joint(home_joints)
        if brain.trial==3:
            break



    

    logger.info("Detaching object...")
    success = motion_client_node.detach_object("dice")
    logger.info(f"Detach success: {success}")

    motion_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()