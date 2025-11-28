import rclpy
from rclpy.node import Node
# 代用品を使う（これならビルド不要！）
from example_interfaces.srv import AddTwoInts 

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        # 店を開く（サービス名: 'cmd_srv'）
        self.srv = self.create_service(AddTwoInts, 'cmd_srv', self.callback)
        self.get_logger().info('命令待ち... (1=前進, 0=停止)')

    def callback(self, request, response):
        # request.a を「命令コード」として代用する
        # 本来なら request.command == "前進" と書くところ
        cmd_id = request.a
        
        if cmd_id == 1:
            self.get_logger().info("【受信】命令: 前進します！")
            response.sum = 1 # 成功フラグ (1=OK)
        elif cmd_id == 0:
            self.get_logger().info("【受信】命令: 停止します！")
            response.sum = 1 # 成功フラグ
        else:
            self.get_logger().info(f"【受信】謎の命令: {cmd_id}")
            response.sum = 0 # 失敗フラグ
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()