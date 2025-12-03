import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        
        # We listen to the "filtered" IMU data which has orientation (quaternion)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # This topic comes from the Madgwick filter
            self.imu_callback,
            10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("Simple Pan/Tilt TF Broadcaster Started")

    def imu_callback(self, msg):
        t = TransformStamped()

        # Give the transform the same timestamp as the IMU data
        t.header.stamp = msg.header.stamp
        
        # 'world' is the fixed frame (the tripod/base)
        t.header.frame_id = 'world'
        
        # 'livox_frame' is the moving frame (the lidar)
        t.child_frame_id = 'livox_frame'

        # No Translation (assuming the lidar rotates around its center)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set Rotation directly from the IMU filter
        t.transform.rotation = msg.orientation

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()