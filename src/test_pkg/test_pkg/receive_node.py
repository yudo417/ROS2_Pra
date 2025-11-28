import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        # クライアント作成（接続先: 'cmd_srv'）
        self.cli = self.create_client(AddTwoInts, 'cmd_srv')
        
        # サーバーが見つかるまで待機
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サーバーを探しています...')
        
        self.req = AddTwoInts.Request()

    def send_command(self, cmd_num):
        # 命令をセット (1=前進)
        self.req.a = cmd_num
        self.req.b = 0 # 使わないけど埋める
        
        # 送信して、結果が来る未来(Future)を受け取る
        self.future = self.cli.call_async(self.req)
        
        # 結果が来るまでここで待つ
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()
    
    # ここで命令を送る！ (1 = 前進)
    print("命令「1 (前進)」を送信します...")
    response = node.send_command(1)
    
    # 結果を確認
    if response.sum == 1:
        print("結果: 成功しました！")
    else:
        print("結果: 失敗しました...")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()