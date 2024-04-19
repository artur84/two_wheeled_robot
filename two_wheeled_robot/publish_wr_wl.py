#! /usr/bin/env python3
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node



class PublishWheelSpeeds(Node):

  def __init__(self):
    super().__init__('publish_wr_wl')
    self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10 )
    # Create wr publisher
    self.pub_wr = self.create_publisher(Float32, 'VelocityEncR', 10)
    # Create wr publisher
    self.pub_wl = self.create_publisher(Float32, 'VelocityEncL', 10)

    #Robot parameters
    self.L = 0.52 # wheel separation [m]
    self.r = 0.14 # radius of the wheels in [m]
    self.wr = Float32()
    self.wl = Float32()
    self.v = 0.0
    self.w = 0.0
    self.robot_vel = Twist()

    # Call on_timer function every second
    self.dt= 0.02
    self.timer = self.create_timer(self.dt, self.on_timer)

  def on_timer(self):
    self.pub_wr.publish(self.wr)
    self.pub_wl.publish(self.wl)

  def cmd_vel_cb(self, msg):
    self.v = msg.linear.x
    self.w = msg.angular.z
    [self.wr.data, self.wl.data] = self.get_wheel_speeds(self.v, self.w)

  def get_wheel_speeds(self, v, w):
    #Get the wheel speeds from robot velocity
    wr = (2*v + w * self.L)/(2*self.r)
    wl = (2*v - w * self.L)/(2*self.r)
    return [wr, wl]



def main(args=None): 
  rclpy.init(args=args) 
  f_p=PublishWheelSpeeds() 
  rclpy.spin(f_p) 
  f_p.destroy_node() 
  rclpy.shutdown() 

     

if __name__ == '__main__': 
    main() 