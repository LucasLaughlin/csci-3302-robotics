import intera_interface
import rospy
import copy 

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import *

g_limb = None
g_orientation = None
g_orientation2 = None
g_position_neutral = None
pi = 3.14159265359

def init():
    global g_limb, g_orientation, g_orientation2, g_position_neutral
    rospy.init_node('cairo_sawyer_ik_example')
    g_limb = intera_interface.Limb('right')

    # This quaternion will have the hand face straight down (ideal for picking tasks)
    """ g_orientation_hand_down = Quaternion()
    g_orientation_hand_down.x = 0.704238785359
    g_orientation_hand_down.y =0.709956638597
    g_orientation_hand_down.z = -0.00229009932359
    g_orientation_hand_down.w = 0.00201493272073 """
   
   # Ros quaternion info - http://wiki.ros.org/tf2/Tutorials/Quaternions
   #FIRST ROTATION
    q_orig = quaternion_from_euler(0, 0, 0) # Set origin
    q_rot1 = quaternion_from_euler(0, pi/2, 0) # Rotate 90 degrees around y axis
    q_rot2 = quaternion_from_euler(0, 0, pi/4)   # Rotate 45 degrees around z axis
    quat_tf = quaternion_multiply(q_rot2, quaternion_multiply(q_rot1, q_orig)) # multiply rotations in order
    g_orientation = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3]) # Create Quaternion object ROS can read

    #SECOND ROTATION
    q_rot3 = quaternion_from_euler(0, 0,0.3) # rotate 0.3 around z axis
    quat_tf2 = quaternion_multiply(q_rot3, quat_tf) # Multiply rotation times position quat_tf to get quat_tf2
    g_orientation2 = Quaternion(quat_tf2[0], quat_tf2[1], quat_tf2[2], quat_tf2[3]) # Create Quaternion object ROS ccan read

    # This is the default neutral position for the robot's hand (no guarantee this will move the joints to neutral though)
    g_position_neutral = Point()
    g_position_neutral.x = 0.449559195663
    g_position_neutral.y = 0.16070379419
    g_position_neutral.z = 0.212938808947

def main():
    global g_limb, g_position_neutral, g_orientation, g_orientation2
    init()

    # Move the arm to its neutral position
    g_limb.move_to_neutral()

    rospy.loginfo("Old Hand Pose:\n %s" % str(g_limb._tip_states.states[0].pose))
    rospy.loginfo("Old Joint Angles:\n %s" % str(g_limb.joint_angles()))

    # Create a new pose (Position and Orientation) to solve for
    target_pose = Pose()
    target_pose.position = copy.deepcopy(g_position_neutral)
    target_pose.orientation = copy.deepcopy(g_orientation)

    target_pose2 = Pose()
    target_pose2.position = copy.deepcopy(g_position_neutral)
    target_pose2.orientation = copy.deepcopy(g_orientation2)

    target_pose.position.x += 0.2 # Add 20cm to the x axis position of the hand
    target_pose2.position.x += 0.2
    target_pose.position.z += 0.4 # Add 20cm to the x axis position of the hand
    target_pose2.position.z += 0.4

    # Call the IK service to solve for joint angles for the desired pose
    target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
    target_joint_angles2 = g_limb.ik_request(target_pose2, "right_hand")

    # The IK Service returns false if it can't find a joint configuration
    if target_joint_angles is False:
        rospy.logerr("Couldn't solve for position %s" % str(target_pose))
        return

    # Set the robot speed (takes a value between 0 and 1)
    g_limb.set_joint_position_speed(0.3)

    # Send the robot arm to the joint angles in target_joint_angles, wait up to 2 seconds to finish
    g_limb.move_to_joint_positions(target_joint_angles, timeout=2)

    g_limb.move_to_joint_positions(target_joint_angles2, timeout=2)

    # Find the new coordinates of the hand and the angles the motors are currently at
    new_hand_pose = copy.deepcopy(g_limb._tip_states.states[0].pose)
    new_angles = g_limb.joint_angles()
    rospy.loginfo("New Hand Pose:\n %s" % str(new_hand_pose))
    rospy.loginfo("Target Joint Angles:\n %s" % str(target_joint_angles))
    rospy.loginfo("New Joint Angles:\n %s" % str(new_angles))

if __name__ == "__main__":
    main()