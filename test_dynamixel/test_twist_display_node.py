import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition

# Subscriber Class
class SubscriberNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("twist_display_node")

        # サブスクライバーの生成
        self.subscriber = self.create_subscription(SetPosition, "/cmd_vel", self.on_subscribe, 10)


    # サブスクライブ時に呼ばれる
    def on_subscribe(self, msg):
        # メッセージの生成
        # ログの出力
        self.get_logger().info(f"x vel : [{msg.linear.x}]")


def main(args=None):
    # RCLの初期化
    rclpy.init(args=args)

    # ノードの生成
    node = SubscriberNode()

    # ノード終了まで待機
    rclpy.spin(node)

    # ノードの破棄
    node.destroy_node()

    # RCLのシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()