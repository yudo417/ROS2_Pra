import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver_node')
        # 受信設定：「Topic」という名前の放送を待ち受ける
        self.subscription = self.create_subscription(
            String,
            'Topic', # box_node.py で決めたトピック名と同じにする！
            self.listener_callback,
            10)
        self.subscription  # 保持しておく

    def listener_callback(self, msg):
        # データが届いたらここが実行される
        self.get_logger().info(f'聞こえた！: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()