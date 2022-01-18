# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import RPi.GPIO as GPIO
import time
import os

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(33, GPIO.OUT)
GPIO.output(33, GPIO.LOW)

pwm = GPIO.PWM(33, 50)
pwm.start(0)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'building_bot',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.old_cmd = ""

    def listener_callback(self, msg):
        cmd = msg.data
        # apply the command only if it was not applied!
        # (don't try to open the gate if it's already open.)
        if cmd != self.old_cmd:
            self.old_cmd = cmd
            if cmd == '1':
                # open gate
                pwm.ChangeDutyCycle(2.2)
                time.sleep(1)
                # to avoid servo noise!
                pwm.ChangeDutyCycle(0)

            elif cmd == '0':
                # close gate
                pwm.ChangeDutyCycle(10.3)
                time.sleep(1)
                # to avoid servo noise!
                pwm.ChangeDutyCycle(0)


            os.system("clear")
            self.get_logger().info('Received: ' + cmd)

        else:
            pwm.ChangeDutyCycle(0)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()