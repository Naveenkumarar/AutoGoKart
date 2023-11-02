import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped

class Command_Node(Node):
        def __init__(self):
                super().__init__("gokart_command")
                self.get_logger().info("Node Created")
                self.subscribe = self.create_subscription(AckermannDriveStamped,"/teleop",self.getControll,10)
                self.sbw_publisher = self.create_publisher(PointStamped,"/sbw_control",10)
                
        def map(self,input):
                value = float(input +1) / 2
                return -90 + (value*180)
                
        def getControll(self,msg):
                data = msg
                msg = PointStamped();
                msg.header.frame_id = "sbw_input"
                msg.point.x = data.drive.steering_angle
                msg.point.y = 0.0
                msg.point.z = 0.0
                self.sbw_publisher.publish(msg)
        
def main(args=None):
        rclpy.init(args=args)
        node = Command_Node()
        node.get_logger().info("Object created")
        rclpy.spin(node)
        rclpy.shutdown()
        
if __name__ == "__main__":
        main() 

