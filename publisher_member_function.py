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

import telepot
from telepot.loop import MessageLoop

import time
import os
import speech_recognition as sr
from pydub import AudioSegment

cmd = ""

# Token is deleted to avoid bad things!!!
bot = telepot.Bot('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')
r = sr.Recognizer()

def bot_response(chat_id, cmd):
    if cmd == '1':
        bot.sendMessage(chat_id, "Gate Open!")
    elif cmd == '0':
        bot.sendMessage(chat_id, "Gate Close!")
    elif cmd == 'time':
        bot.sendMessage(chat_id, time.ctime())
    else:
        bot.sendMessage(chat_id, "Unknown command!")

def handle(msg):
    global cmd
    chat_id = msg['chat']['id']

    if 'text' in msg:
        cmd = msg['text']
        bot_response(chat_id, cmd)

    elif 'voice' in msg:
        bot.download_file(msg['voice']['file_id'], '/home/ubuntu/DEIS/building_bot/voice.ogg')
        voice = AudioSegment.from_ogg('/home/ubuntu/DEIS/building_bot/voice.ogg')
        voice.export('/home/ubuntu/DEIS/building_bot/voice.wav', format='wav')
        voice = sr.AudioFile('/home/ubuntu/DEIS/building_bot/voice.wav')
        bot.sendChatAction(chat_id, 'typing')
        with voice as source:
            command = r.record(source)
            command = r.recognize_google(command, language='en-US', show_all=False)
            bot.sendMessage(chat_id, 'Heard: ' + command)
            if command == 'open':
                cmd = '1'
                bot_response(chat_id, cmd)
            elif command == 'close':
                cmd = '0'
                bot_response(chat_id, cmd)
            elif command == 'time':
                bot_response(chat_id, command)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'building_bot', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Run the bot as a thread
        MessageLoop(bot, handle).run_as_thread()
        self.old_cmd = ""

    def timer_callback(self):
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        if cmd != self.old_cmd:
            os.system("clear")
            self.get_logger().info('Publishing:' + msg.data)
            self.old_cmd = cmd

def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
