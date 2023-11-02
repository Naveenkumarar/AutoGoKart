import time
import serial

import rclpy
from rclpy.node import Node

# from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

class ATJoyNode(Node):
        def __init__(self):
                super().__init__('atjoy_node')
                # self.steering_publisher_ = self.create_publisher(Float64, 'steering', 1)
                # self.throttle_publisher_ = self.create_publisher(Float64, 'throttle', 1)
                self.command_publisher_ = self.create_publisher(Joy, 'joy/at9s', 1)
                timer_period = 0.05  # seconds
                self.timer = self.create_timer(timer_period, self.timer_callback)
                
                self.ser = serial.Serial( port = '/dev/ttyACM0',baudrate=115200)  # series port of arduino
                self.ser.isOpen()  # open port receiver
                
        def isint(self,num):
                try:
                        for k in num:
                                int(k)
                        return True
                except ValueError:
                        return False
                
        def timer_callback(self):
                out = ''
                while self.ser.inWaiting() > 0:
                        out += str(self.ser.readline(),encoding='utf-8')
                        
                if out != '':
                        spl = out.split(",")
                        
                        # at beginning, arduino send some cache string data
                        if self.isint(spl):
                                # The first two are for axes
                                # mapping from [2000,1000] to [-1,1]
                                spl[0] = 1.0-2.0*(float(spl[0])-1000.0)/(2000.0-1000.0) 
                                spl[1] = 1.0-2.0*(float(spl[1])-1000.0)/(2000.0-1000.0)
                                for i in range(2,len(spl)):
                                        # For buttons: back: 2000, forward: 1000;
                                        if int(spl[i])<1500:
                                                spl[i]=1;
                                        else:
                                                spl[i]=0;
                        
                                # Publisher
                                msg  = Joy()
                                msg.header.stamp = self.get_clock().now().to_msg()
                                msg.header.frame_id = 'at9s_joy'
                                msg.axes = spl[0:2]
                                msg.buttons = spl[2:len(spl)]
                                self.command_publisher_.publish(msg)
                                
                                # printing for debug purpose
                                self.get_logger().info('throttle: "%f, steering: "%f" /n' % (msg.axes[0], msg.axes[1]))
                                self.get_logger().info('sWA: "%d, sWD: "%d" /n' % (msg.buttons[0],msg.buttons[3]))


def main(args=None):
    rclpy.init(args=args)

    atjoy_node = ATJoyNode()

    rclpy.spin(atjoy_node)

    atjoy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
