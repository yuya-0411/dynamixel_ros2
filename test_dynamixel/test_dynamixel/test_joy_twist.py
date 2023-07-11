import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dynamixel_sdk_custom_interfaces.msg import SetPosition

# PubSub Class
class PubSubNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("joy_twist_node")

        # サブスクライバーの生成
        self.subscriber = self.create_subscription(Joy, "/joy", self.on_subscribe, 10)

        # パブリッシャーの生成
        self.publisher = self.create_publisher(SetPosition, "/set_position", 10)

    # サブスクライブ時に呼ばれる
    def on_subscribe(self, msg):
        # メッセージの生成
        # ログの出力
        # self.get_logger().info(f"x vel : [{msg.linear.x}]")

        left_x  = msg.axes[0]
        left_y  = msg.axes[1]
        right_x = msg.axes[2]
        right_y = msg.axes[3]

        triangle    = msg.buttons[0]
        circle      = msg.buttons[1]
        cross       = msg.buttons[2]
        square      = msg.buttons[3]
        
        posMsg = SetPosition()
        posMsg.id = triangle * 1 + circle * 2 + cross * 3 - 1
        
        if (posMsg.id != -1):
            posMsg.position = 2048 * (1 + left_x)
            
            if posMsg.position < 1023:
                posMsg.position = 1023
            elif posMsg.position > 3060:
                posMsg.position = 3060
            
            # メッセージのパブリッシュ
            self.publisher.publish(posMsg)
            self.get_logger().info(f"Publish ID[{posMsg.id}] Position[{posMsg.position}]")

        else:
            self.get_logger().info(f"NO Triger to publish. trianble or circle or crosss is the triger.")


            
        # # メッセージのパブリッシュ
        # self.publisher.publish(posMsg)

        # ログの出力
        # self.get_logger().info(f"Publish ID[{posMsg.id}] Position[{posMsg.position}]")


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