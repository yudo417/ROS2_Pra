import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import math

class BoxNode(Node):
    def __init__(self):
        super().__init__('box_node')
        self.get_logger().info('BoxNode has been started')
        
        # Markerパブリッシャーの作成
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # 1秒ごとに実行されるタイマーの作成例
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Hello from BoxNode: {self.counter}')
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # 名前空間とIDの設定
        marker.ns = "basic_shapes"
        marker.id = 0
        
        # マーカーの種類をCUBE（立方体）に設定
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 位置と姿勢の設定
        marker.pose.position.x = math.sin(self.counter)
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # スケールの設定 (1x1x1メートル)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # 色の設定 (緑色, 不透明)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # マーカーの寿命（0の場合は永続）
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.publisher_.publish(marker)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = BoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ノードの明示的な破棄（ガベージコレクションでも行われるが、明示するのが良い習慣）
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
