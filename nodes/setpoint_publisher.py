#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from hippo_msgs.msg import ActuatorControls


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
        self.my_publisher = self.create_publisher(ActuatorControls,
                                                  'thruster_controls', 1)
        self.timer = self.create_timer(1 / 50, self.on_timer)

    def on_timer(self):
        self.publish_my_msg()

    def publish_my_msg(self):
        # create the message object
        msg = ActuatorControls()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        # get the time as floating point number in seconds
        t = now.nanoseconds * 1e-9

        # the list holds 8 values for the 8 thrusters of the bluerov
        msg.control[0] = 0.2 * math.sin(t)
        msg.control[1] = -0.2 * math.sin(t)
        msg.control[2] = 0.2 * math.cos(t)
        msg.control[3] = -0.2 * math.cos(t)
        msg.control[4] = 0.4 * math.sin(t)
        msg.control[5] = -0.4 * math.sin(t)
        msg.control[6] = 0.4 * math.cos(t)
        msg.control[7] = -0.4 * math.cos(t)

        # publish the message with the publisher we created during the object
        # initialization
        self.my_publisher.publish(msg)


def main():
    rclpy.init()
    node = MyFirstNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
