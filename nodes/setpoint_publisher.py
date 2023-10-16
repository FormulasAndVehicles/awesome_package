#!/usr/bin/env python3

import math

import rclpy
from hippo_msgs.msg import ActuatorSetpoint
from rclpy.node import Node


class MyFirstNode(Node):

    # the __init__ function gets called, when we create the object, i.e.
    # run something like
    #
    # node = MyFirstNode()
    def __init__(self):
        # we initialize the rclpy Node with a unique name
        super().__init__(node_name='my_first_node')

        # create a publisher. we need to specify the message type and the topic
        # name. The last argument specifies the queue length
        self.thrust_pub = self.create_publisher(ActuatorSetpoint,
                                                'thrust_setpoint', 1)
        self.torque_pub = self.create_publisher(ActuatorSetpoint,
                                                'torque_setpoint', 1)
        self.timer = self.create_timer(1 / 50, self.on_timer)

    def on_timer(self):
        self.publish_setpoints()

    def publish_setpoints(self):
        # create the message object
        thrust_msg = ActuatorSetpoint()
        now = self.get_clock().now()
        thrust_msg.header.stamp = now.to_msg()
        # get the time as floating point number in seconds
        t = now.nanoseconds * 1e-9

        thrust_msg.x = 0.5 * math.sin(t)
        thrust_msg.y = -0.5 * math.sin(t)
        thrust_msg.z = 0.5 * math.cos(t)

        torque_msg = ActuatorSetpoint()
        torque_msg.header.stamp = now.to_msg()

        torque_msg.x = 0.4 * math.sin(t)
        torque_msg.y = -0.4 * math.sin(t)
        torque_msg.z = 0.4 * math.cos(t)
        # publish the messages with the publishers we created during the object
        # initialization
        self.thrust_pub.publish(thrust_msg)
        self.torque_pub.publish(torque_msg)


def main():
    rclpy.init()
    node = MyFirstNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
