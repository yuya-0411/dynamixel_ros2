import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition

# Publisher Class
class PublisherNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("test_node")

        # パブリッシャーの生成
        self.publisher = self.create_publisher(SetPosition, "/set_position", 10)

        # タイマーの生成
        self.timer = self.create_timer(1, self.on_tick)

    # １秒ごとに呼ばれる
    def on_tick(self):
        # メッセージの生成
        msg = SetPosition()
        msg.id = 0
        msg.position = 512

        # メッセージのパブリッシュ
        self.publisher.publish(msg)

        # ログの出力
        self.get_logger().info(f"Publish ID[{msg.id}] Position[{msg.position}]")


def main(args=None):
    # RCLの初期化
    rclpy.init(args=args)

    # ノードの生成
    node = PublisherNode()

    # ノード終了まで待機
    rclpy.spin(node)

    # ノードの破棄
    node.destroy_node()

    # RCLのシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()