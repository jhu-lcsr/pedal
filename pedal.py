#!/usr/bin/env python

from __future__ import print_function

import time
import fcntl
from math import pi

import rospy
from sensor_msgs.msg import *

from evdev import InputDevice, list_devices, categorize, ecodes, events

class Pedal(object):
    def __init__(self):

        self.pedal_dev = None
        self.joy_msg = Joy(buttons=[0])

    def update_value(self):

        while not rospy.is_shutdown():
            event = self.pedal_dev.read_one()
            if event is None:
                break
            if event.type == ecodes.EV_KEY:
                print(categorize(event))
                if event.value in [events.KeyEvent.key_down, events.KeyEvent.key_hold]:
                    self.joy_msg.buttons[0] = 1
                else:
                    self.joy_msg.buttons[0] = 0

    def spin(self):
        devices = [InputDevice(fn) for fn in list_devices()]
        print(devices)
        for dev in devices:
            if dev.info.vendor == 0x0c45 and dev.info.product == 0x7403:
                print('Using pedal "'+dev.name+'" on '+dev.phys)
                self.pedal_dev = dev
                break

        self.pedal_dev.grab()

        self.joy_pub = rospy.Publisher('joy', Joy)
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            self.update_value()
            self.joy_msg.header.stamp = rospy.Time.now()
            self.joy_pub.publish(self.joy_msg)
            r.sleep()

        self.pedal_dev.close()
        print('done')

def main():
    rospy.init_node('pedal')

    p = Pedal()
    p.spin()

if __name__ == '__main__':
    main()
