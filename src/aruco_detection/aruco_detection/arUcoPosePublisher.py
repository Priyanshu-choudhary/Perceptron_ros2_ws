import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import cv2
# For transformations in ROS 2, you typically use the 'tf_transformations' package
# which is the Python port of tf2's transformation utilities.
from aruco_detection.transformations import quaternion_from_euler
from aruco_detection.arUcoBase import ArucoCamera 

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher') # Initialize the ROS 2 node

        self.publisher_ = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback) # 0.1 seconds = 10 Hz
        self.detector = ArucoCamera("/mnt/usbdrive/jetson-home/camera/calibration_data/intrinsics_2.yml")

        self.get_logger().info('ArUco Pose Publisher Node started.')

    def timer_callback(self):
        markers = self.detector.get_markers()
        if markers:
            for marker_id, data in markers.items():
                ''' logger '''
                # self.get_logger().info(f"Marker {marker_id}:")
                # self.get_logger().info(f"  Distance: {data['distance']:.2f}m")
                # self.get_logger().info(f"  Position: {data['position']}")

                position = data['position']
                orientation_rpy = data['rvec']

                x, y, z = position
                roll, pitch, yaw = orientation_rpy

                # Convert RPY to Quaternion using tf_transformations
                # Note: quaternion_from_euler returns [x, y, z, w]
                quaternion = quaternion_from_euler(roll, pitch, yaw)

                # Fill the PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg() # ROS 2 way to get time
                pose_msg.header.frame_id = "camera_link" # IMPORTANT: Set your camera frame ID here!

                pose_msg.pose.position.x = float(x) # Ensure float type
                pose_msg.pose.position.y = float(y)
                pose_msg.pose.position.z = float(z)

                pose_msg.pose.orientation.x = float(quaternion[0])
                pose_msg.pose.orientation.y = float(quaternion[1])
                pose_msg.pose.orientation.z = float(quaternion[2])
                pose_msg.pose.orientation.w = float(quaternion[3])

                self.publisher_.publish(pose_msg)
                

def main(args=None):
    rclpy.init(args=args)
    aruco_pose_publisher_node = ArucoPosePublisher()
    rclpy.spin(aruco_pose_publisher_node) # Keeps the node alive

    aruco_pose_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()