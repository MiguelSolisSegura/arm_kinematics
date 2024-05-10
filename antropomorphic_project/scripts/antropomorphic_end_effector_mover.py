#!/usr/bin/env python3
import rospy
from math import atan2, acos, sqrt, pi
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import PoseStamped
from rviz_marker import MarkerBasics
from move_joints import JointMover

# Joint limits in radians
theta2_min = -pi / 4
theta2_max = 3 * pi / 4

theta3_min = -3 * pi / 4
theta3_max = 3 * pi / 4

# Link lengths
r2 = 1.0
r3 = 1.0

def normalize_angle(angle):
    """Normalize angles between -pi and pi."""
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

class EndEffectorMover:
    def __init__(self):
        rospy.init_node('antropomorphic_end_effector_mover', anonymous=True)

        # Initialize joint mover and marker publisher
        self.joint_mover = JointMover()
        self.marker_publisher = MarkerBasics()

        # Subscribers
        rospy.Subscriber('/ee_pose_commands', EndEffector, self.ee_pose_callback)
        rospy.Subscriber('/end_effector_real_pose', PoseStamped, self.real_pose_callback)

        # Initialize a marker index counter
        self.marker_index = 0

    def ee_pose_callback(self, ee_msg):
        """Callback function for the /ee_pose_commands topic."""
        # Extract the end-effector position
        target_position = ee_msg.ee_xy_theta
        Px = target_position.x
        Py = target_position.y
        Pz = target_position.z

        # Inverse Kinematics calculations to determine joint angles
        # Calculate two possible theta1 values
        theta1_candidate1 = atan2(Py, Px)
        r1 = sqrt(Px**2 + Py**2)
        d = Pz
        D = sqrt(r1**2 + d**2)

        # Calculate possible theta2 values
        cos_theta2 = (r2**2 + D**2 - r3**2) / (2 * r2 * D)
        if cos_theta2 > 1 or cos_theta2 < -1:
            rospy.logwarn("Unreachable position: Unable to compute theta2.")
            return
        theta2_candidate1 = atan2(d, r1) + acos(cos_theta2)

        # Calculate possible theta3 values
        cos_theta3 = (r2**2 + r3**2 - D**2) / (2 * r2 * r3)
        if cos_theta3 > 1 or cos_theta3 < -1:
            rospy.logwarn("Unreachable position: Unable to compute theta3.")
            return
        theta3_candidate1 = acos(cos_theta3) - pi

        # Normalize the angles to the range [-pi, pi]
        theta1_candidate1 = normalize_angle(theta1_candidate1)
        theta2_candidate1 = normalize_angle(theta2_candidate1)
        theta3_candidate1 = normalize_angle(theta3_candidate1)

        # Check if the solution is feasible given the joint limits
        if not (theta2_min <= theta2_candidate1 <= theta2_max) or not (theta3_min <= theta3_candidate1 <= theta3_max):
            rospy.logwarn("Unreachable position: Solution is outside joint limits.")
            return

        # Move the robot using the provided JointMover
        self.joint_mover.move_all_joints(theta1_candidate1, theta2_candidate1, theta3_candidate1)

        # Publish the desired marker using MarkerBasics
        self.marker_publisher.publish_point(Px, Py, Pz, self.marker_index)
        self.marker_index += 1

    def real_pose_callback(self, pose_msg):
        """Callback function for the /end_effector_real_pose topic."""
        rospy.loginfo(f"Real End-Effector Pose: {pose_msg.pose}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    mover = EndEffectorMover()
    mover.run()
