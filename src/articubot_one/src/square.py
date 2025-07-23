#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import math

class WaypointFromInitialPose(Node):
    def __init__(self):
        super().__init__('waypoint_from_initial_pose')
        self.initial_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info("Đang chờ initial pose...")
        self.canh_hinhvuong = 2.0
        self._goal_handle = None
        self._last_feedback_wp = -1

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

    def create_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        return pose

    def initialpose_callback(self, msg):
        self.initial_pose = msg.pose.pose
        self.get_logger().info(f"Đã nhận initial pose tại ({self.initial_pose.position.x:.2f}, {self.initial_pose.position.y:.2f})")
        self.send_waypoints()

    def send_waypoints(self):
        if self.initial_pose is None:
            self.get_logger().warn("Chưa có initial pose.")
            return

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Không kết nối được follow_waypoints action server.")
            return

        x0 = self.initial_pose.position.x
        y0 = self.initial_pose.position.y

        waypoints = [
            self.create_pose(x0, y0, 0.0),
            self.create_pose(x0 + self.canh_hinhvuong, y0, math.pi/2),
            self.create_pose(x0 + self.canh_hinhvuong, y0 + self.canh_hinhvuong, math.pi),
            self.create_pose(x0, y0 + self.canh_hinhvuong, 3*math.pi/2),
            self.create_pose(x0, y0, 0.0),
        ]

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info(f"Gửi {len(waypoints)} điểm hình vuông từ initial pose...")

        send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal bị từ chối bởi action server")
            return
        self._goal_handle = goal_handle
        self.get_logger().info("Goal được chấp nhận, robot đang di chuyển...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_wp = feedback.current_waypoint
        # Chỉ in khi waypoint thay đổi để không spam log
        if current_wp != self._last_feedback_wp:
            self._last_feedback_wp = current_wp
            self.get_logger().info(f"Đã đến waypoint thứ {current_wp + 1}. Điểm tiếp theo là waypoint thứ {current_wp + 2 if current_wp + 1 < len(feedback_msg.goal_id.stamp.sec) else 'không còn nữa'}")

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # STATUS_ABORTED
            self.get_logger().error("Hành trình bị hủy.")
        elif status == 5:  # STATUS_REJECTED
            self.get_logger().error("Goal bị từ chối.")
        elif status == 3:  # STATUS_SUCCEEDED
            self.get_logger().info("Đã hoàn thành toàn bộ waypoint.")
        else:
            self.get_logger().info(f"Trạng thái kết thúc: {status}")

def main():
    rclpy.init()
    node = WaypointFromInitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
