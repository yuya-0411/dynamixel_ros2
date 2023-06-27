import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk_custom_interfaces.msg import SetPosition

# PubSub Class
class PubSubNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("control_node")

        # サブスクライバーの生成
        self.subscriber = self.create_subscription(Twist, "/cmd_vel", self.on_subscribe, 10)

        # パブリッシャーの生成
        self.publisher = self.create_publisher(SetPosition, "/set_position", 10)

    # サブスクライブ時に呼ばれる
    def on_subscribe(self, msg):
        # メッセージの生成
        # ログの出力
        # self.get_logger().info(f"x vel : [{msg.linear.x}]")
        
        posMsg = SetPosition()
        posMsg.id = 0
        posMsg.position = int(2048 * (1 + msg.linear.x))
        
        if posMsg.position < 1023:
            posMsg.position = 1023
        elif posMsg.position > 3060:
            posMsg.position = 3060
            
        # メッセージのパブリッシュ
        self.publisher.publish(posMsg)

        # ログの出力
        self.get_logger().info(f"Publish ID[{posMsg.id}] Position[{posMsg.position}]")


def main(args=None):
    # RCLの初期化
    rclpy.init(args=args)

    # ノードの生成
    node = PubSubNode()

    # ノード終了まで待機
    rclpy.spin(node)

    # ノードの破棄
    node.destroy_node()

    # RCLのシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
    main()