import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from drims2_motion_control.drims2_motion_server import MotionClient  # import your class


def main():
    rclpy.init()

    # MotionClient is already a Node, so we can use its logger directly
    node = MotionClient()

    logger = node.get_logger()

    # --- 1) Move to home configuration ---
    # Typical UR10e joint values (rad) for "home", adapt if you have a different setup
    home_joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    logger.info("Moving to home configuration...")
    result = node.move_to_joint(home_joints)
    logger.info(f"Home reached: {result}")

    # --- 2) Move to pose of the dice (pick pose) ---
    pick_pose = PoseStamped()
    pick_pose.header.frame_id = "base_link"
    pick_pose.pose.position.x = 0.4
    pick_pose.pose.position.y = 0.0
    pick_pose.pose.position.z = 0.2
    pick_pose.pose.orientation.x = 0.0
    pick_pose.pose.orientation.y = 1.0
    pick_pose.pose.orientation.z = 0.0
    pick_pose.pose.orientation.w = 0.0

    logger.info("Moving to pick pose...")
    result = node.move_to_pose(pick_pose, cartesian_motion=True)
    logger.info(f"Pick pose reached: {result}")

    # --- 3) Close gripper + attach object ---
    logger.info("Closing gripper...")
    reached_goal, stalled = node.gripper_command(position=0.0)  # 0.0 = closed
    logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

    logger.info("Attaching object...")
    success = node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
    logger.info(f"Attach success: {success}")

    # --- 4) Move to another pose (place pose) ---
    place_pose = PoseStamped()
    place_pose.header.frame_id = "base_link"
    place_pose.pose.position.x = 0.6
    place_pose.pose.position.y = -0.2
    place_pose.pose.position.z = 0.2
    place_pose.pose.orientation.x = 0.0
    place_pose.pose.orientation.y = 1.0
    place_pose.pose.orientation.z = 0.0
    place_pose.pose.orientation.w = 0.0

    logger.info("Moving to place pose...")
    result = node.move_to_pose(place_pose, cartesian_motion=True)
    logger.info(f"Place pose reached: {result}")

    # --- 5) Open gripper + detach object ---
    logger.info("Opening gripper...")
    reached_goal, stalled = node.gripper_command(position=0.8)  # 0.8 = open, adjust depending on gripper model
    logger.info(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")

    logger.info("Detaching object...")
    success = node.detach_object("dice")
    logger.info(f"Detach success: {success}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
