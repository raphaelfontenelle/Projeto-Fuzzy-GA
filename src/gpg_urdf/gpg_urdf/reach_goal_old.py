import math
from warnings import catch_warnings

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn
from geometry_msgs.msg import Pose2D, TwistStamped
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray

class MyNode(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)
        #self.subscription = self.create_subscription(
        #    Pose2D,
        #    'goal',
        #    self.goal_callback,
        #    10)
        self.subscription  # prevent unused variable warning
        self.subscription  # prevent unused variable warning 
        
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        #self.pub2 = self.create_publisher(Float64MultiArray, '/servo_controller/commands', 10)
        
        #self.subscription = self.create_subscription(
        #    Image,
        #    '/image',
        #    self.image_callback,
        #    10)
        
        self.goal = None
        
    def on_timer(self):
        if self.goal is None:
            return
        ps = PointStamped() 
        ps.point.x = self.goal.x
        ps.point.y = self.goal.y
        ps.header.frame_id = 'odom'
        try:
            frame_robot = PointStamped()
            frame_robot = self.tf_buffer.transform(ps, "base_link")
            ang = math.atan2(frame_robot.point.y, frame_robot.point.x)*2
            dir = math.sqrt(frame_robot.point.x**2 + frame_robot.point.y**2)*1
            msg = TwistStamped()
            msg.twist.linear.x = dir*1.5
            msg.twist.angular.z = ang*2
            if dir > 0.1:
                self.pub.publish(msg)

        except Exception as e:
            print("Ocorru um erro: " + str(e))
            return

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        img2 = cv2.inRange(img, ( 0, 200, 50),
		(40, 255, 255))
        
        m = cv2.moments(img2)
        cx = m['m10']/m['m00']
        cy = m['m01']/m['m00']
        
        ma  = Float64MultiArray()
        h = img2.shape[0]/2
        de = h-cx
        self.pos += 0.01*e
        ma.data = self.pos
        self.pub2.publish(ma)
        
        
        
        
    def goal_callback(self, msg):
        self.goal = msg

        if True:
            ps = PointStamped() 
            ps.point.x = msg.x
            ps.point.y = msg.y
            ps.header.frame_id  = 'base_link'
            try:
                frame_robot = PointStamped()
                frame_robot = self.tf_buffer.transform(ps, "odom")
                self.goal.x = frame_robot.point.x
                self.goal.y = frame_robot.point.y
            except Exception as e:
                print("Ocorru um erro: " + str(e))
                return

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                
