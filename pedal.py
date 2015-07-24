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

        self.n_pedals = rospy.get_param('~n_pedals',1)
        self.pedal_devs = []
        self.joy_msg = Joy(buttons=[0]*self.n_pedals)

    def get_value(self, dev):
        latest_value = None

        while not rospy.is_shutdown():
            event = dev.read_one()
            if event is None:
                break
            if event.type == ecodes.EV_KEY:
                print(categorize(event))
                if event.value in [events.KeyEvent.key_down, events.KeyEvent.key_hold]:
                    latest_value = 1
                else:
                    latest_value = 0

        return latest_value

    def spin(self):
        devices = [InputDevice(fn) for fn in list_devices()]
        print(devices)
        for dev in devices:
            if dev.info.vendor == 0x0c45 and dev.info.product == 0x7403:
                print('Using pedal "'+dev.name+'" on '+dev.phys)
                print('Info:\n\n{}'.format(dev.info))

                try:
                   dev.grab()
                except IOError as err:
                    print('Couldn\'t grab device, trying again.')
                    continue

                self.pedal_devs.append(dev)
                if len(self.pedal_devs) == self.n_pedals:
                    break

        self.joy_pub = rospy.Publisher('joy', Joy)
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            for i,dev in enumerate(self.pedal_devs):
                latest_value = self.get_value(dev)
                if latest_value is not None:
                    self.joy_msg.buttons[i] = latest_value
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
