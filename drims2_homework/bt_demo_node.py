import rclpy
from geometry_msgs.msg import PoseStamped
from drims2_motion_server.motion_client import MotionClient

def main():
    rclpy.init()

    node = MotionClient()

    # --- 1) Move to home configuration ---
    # Typical UR10e joint values (rad) for "home", adapt if you have a different setup
    home_joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    print("Moving to home configuration...")
    result = node.move_to_joint(home_joints)
    print("Home reached:", result)

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

    print("Moving to pick pose...")
    result = node.move_to_pose(pick_pose, cartesian_motion=True)
    print("Pick pose reached:", result)

    # --- 3) Close gripper + attach object ---
    print("Closing gripper...")
    reached_goal, stalled = node.gripper_command(position=0.0)  # 0.0 = closed
    print(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

    print("Attaching object...")
    success = node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
    print("Attach success:", success)

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

    print("Moving to place pose...")
    result = node.move_to_pose(place_pose, cartesian_motion=True)
    print("Place pose reached:", result)

    # --- 5) Open gripper + detach object ---
    print("Opening gripper...")
    reached_goal, stalled = node.gripper_command(position=0.8)  # 0.8 = open, adjust depending on gripper model
    print(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")

    print("Detaching object...")
    success = node.detach_object("dice")
    print("Detach success:", success)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
