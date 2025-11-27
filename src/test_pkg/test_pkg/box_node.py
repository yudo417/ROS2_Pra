import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray 
from std_msgs.msg import String
import math

class BoxNode(Node):
    def __init__(self):
        super().__init__('box_node')
        self.pub = self.create_publisher(String,"Topic",10)
        self.timer = self.create_timer(1.0,self.timer_callback)
        # self.sub = self.create_subscription(String,"Topic",self.sub_callback,10)
        self.i = 10
    
    def timer_callback(self):
        msg = String()
        if self.i > 0:
            msg.data = f"ハッピーワールド{self.i}"
        else:
            msg.data = f"終わり"
            self.destroy_timer(self.timer)
        self.pub.publish(msg)
        self.get_logger().info(f"パブリッシュ:{msg.data}")
        self.i -= 1

    # def sub_callback(self,msg):
    #     self.get_logger().info(f"サブスクライブ:{msg.data}")

def main(args=None):
    rclpy.init()
    node = BoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()