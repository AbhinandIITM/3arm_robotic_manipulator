#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
class Manipulator(Node):

    def __init__(self):
        super().__init__('controller')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.current_joint_positions = [0.0, 0.0, 0.0]
        self.joint_names = ["rev_1", "rev_2", "rev_3"]
        self.joint_states_received = False
        self.create_subscription(topic='/joint_states',msg_type=JointState,callback=self.joint_state_callback,qos_profile=10)

    def joint_state_callback(self,msg):
        positions = {}
        for name, pos in zip(msg.name, msg.position):   
            positions[name] = pos

        try:
            self.current_joint_positions = [positions[name] for name in self.joint_names]
            self.joint_states_received = True
        except KeyError:
            self.get_logger().warn('Not all joint positions are available in /joint_states')

    def send_goal(self, angle1,angle2,angle3):

        while not self.joint_states_received:
            rclpy.spin_once(self)
        goal_msg = FollowJointTrajectory.Goal()

        
        joint_names = [
            "rev_1","rev_2","rev_3"
        ]

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = self.current_joint_positions
        points.append(point1)
       
        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()
        point2.positions = [angle1,angle2,angle3]
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=3, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


def main(args=None):
    
    rclpy.init()
    action_client = Manipulator()

    if len(sys.argv) < 4:
        print("Please provide an angles as command-line arguments.")
        return

    angle1 = float(sys.argv[1])
    angle2 = float(sys.argv[2])
    angle3 = float(sys.argv[3])

    action_client.send_goal(angle1,angle2,angle3)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
