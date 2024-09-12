import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('path', '')
        self.declare_parameter('topic', '/image')
        self.declare_parameter('format', 'color')

        self.path = self.get_parameter(
            'path').get_parameter_value().string_value
        self.topic = self.get_parameter(
            'topic').get_parameter_value().string_value
        self.format = self.get_parameter(
            'format').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, self.topic, 10)

        self.get_logger().info("Image publisher node has been started.")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    bridge = CvBridge()

    try:
        if not os.path.isfile(node.path):
            raise Exception(f"'{node.path}' is not exist")

        if node.format == "color":
            img = cv2.imread(node.path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            ros_image = bridge.cv2_to_imgmsg(img, 'rgb8')
        elif node.format == "mono":
            img = cv2.imread(node.path, cv2.IMREAD_GRAYSCALE)
            ros_image = bridge.cv2_to_imgmsg(img, 'mono8')
        else:
            raise Exception("Allowed formats are 'color' or 'mono'")
    except CvBridgeError as e:
        print(e)
        node.destroy_node()
        return
    except Exception as e:
        print(e)
        node.destroy_node()
        return

    node.publisher_.publish(ros_image)

    cv2.waitKey(0)

    rclpy.spin_once(node,timeout_sec=1.0)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
