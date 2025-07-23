#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
from tf_transformations import quaternion_from_euler

def generate_custom_path(length, width, step, sub_step=1.0):
    path = []
    y = 0.0
    direction = 1  # 1: sang phải, -1: sang trái

    while y < width:
        # Tạo các điểm theo chiều dài đã chia nhỏ
        num_segments = int(length // sub_step) #chia lấy phần nguyên
        for i in range(num_segments + 1):
            x = i * sub_step
            if direction == -1:
                x = length - x  # đảo chiều nếu đi từ phải sang trái
            x = max(0.0, min(x, length))  # giới hạn trong [0, length]
            path.append([x, y, 0])

        y += step
        direction *= -1  # đổi hướng

    return path

def main() -> None:
    rclpy.init()

    # Khởi tạo Navigator
    navigator = BasicNavigator()
   
    # Tuyến đường kiểm tra (inspection route)
    inspection_route = generate_custom_path(5.0,2.5,0.45)
    
    # Đặt Initial Pose mà không cần sử dụng AMCL
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Bỏ qua AMCL và đợi Nav2 hoạt động
    #navigator.waitUntilNav2Active()

    # Lặp qua các waypoint và tạo các goal point
    goal_points = []
    for pt in inspection_route:
        goal_pose = PoseStamped()  # Tạo một đối tượng goal_pose mới cho mỗi waypoint
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        
        # Chuyển các giá trị thành float để tránh lỗi kiểu dữ liệu
        goal_pose.pose.position.x = float(pt[0])
        goal_pose.pose.position.y = float(pt[1])
        
        # Chuyển đổi góc yaw thành quaternion
        yaw = float(pt[2])  # Yaw được chỉ định cho từng waypoint
        quat = quaternion_from_euler(0.0, 0.0, yaw)  # Chuyển đổi yaw thành quaternion

        # Cập nhật quaternion vào goal_pose
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        
        # Thêm goal_pose vào danh sách goal_points
        goal_points.append(goal_pose)

    # Bắt đầu theo dõi các waypoint
    follow_waypoints_task = navigator.followWaypoints(goal_points)

    # Vòng lặp theo dõi tiến độ
    i = 0
    while not navigator.isTaskComplete():  # Kiểm tra nếu task hoàn thành
        feedback = navigator.getFeedback()  # Nhận phản hồi về tiến độ
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_points))
            )
        i += 1

        # Kiểm tra kết quả của task
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            (error_code, error_msg) = navigator.getTaskError()
            print(f'Goal failed! {error_code}: {error_msg}')
        else:
            print('Goal has an invalid return status!')

    # Chờ cho đến khi task hoàn thành
    while not navigator.isTaskComplete():
        pass

if __name__ == '__main__':
    main()
