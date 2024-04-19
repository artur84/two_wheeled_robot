import numpy as np 
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class PublishWheelSpeeds(Node):

  def __init__(self):
    super().__init__('publish_wr_wl')

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)


    # Create wr publisher
    self.pub_wr = self.create_publisher(Float32, 'wr', 10)
    # Create wr publisher
    self.pub_wl = self.create_publisher(Float32, 'wl', 10)

    #
    self.prev_theta_r = 0.0 #previous value of theta r
    self.current_theta_r = 0.0 #current value of theta r
    self.prev_theta_l = 0.0
    self.current_theta_l = 0.0

    self.wr = Float32()
    self.wl = Float32()

    # Call on_timer function every second
    self.dt= 0.1
    self.timer = self.create_timer(self.dt, self.on_timer)

  def on_timer(self):

    # Look up for the transformation between target_frame and turtle2 frames
    # and send velocity commands for turtle2 to reach target_frame
    try:
      tf_right = self.tf_buffer.lookup_transform(
      "drivewhl_r_link",
      "base_link",
      rclpy.time.Time())
    except TransformException as ex:
      self.get_logger().info('Could not make the transformation from right_wheel to base_link transform')
      return
    try:
      tf_left = self.tf_buffer.lookup_transform(
      "drivewhl_l_link",
      "base_link",
      rclpy.time.Time())
    except TransformException as ex:
      self.get_logger().info('Could not make the transformation from left_wheel to base_link transform')
      return


    qx=tf_right.transform.rotation.x
    qy=tf_right.transform.rotation.y
    qz=tf_right.transform.rotation.z
    qw=tf_right.transform.rotation.w
    [rx,self.current_theta_r,rz]=euler_from_quaternion([qx,qy,qz,qw])

    self.wr.data=(self.current_theta_r-self.prev_theta_r)/self.dt
    self.prev_theta_r = self.current_theta_r

    qx=tf_left.transform.rotation.x
    qy=tf_left.transform.rotation.y
    qz=tf_left.transform.rotation.z
    qw=tf_left.transform.rotation.w


    [rx,self.current_theta_l,rz]=euler_from_quaternion([qx,qy,qz,qw])

    self.wl.data=(self.current_theta_l-self.prev_theta_l)/self.dt
    self.prev_theta_l = self.current_theta_l
    print("wr: ")
    print(self.wr)
    print("wl: ")
    print(self.wl)
    self.pub_wr.publish(self.wr)
    self.pub_wl.publish(self.wl)






def main(args=None): 
  rclpy.init(args=args) 
  f_p=PublishWheelSpeeds() 
  rclpy.spin(f_p) 
  f_p.destroy_node() 
  rclpy.shutdown() 

     

if __name__ == '__main__': 
    main() 