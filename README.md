ROS Pedal HID
=============

## Dependencies

* `python-evdev` `sudo pip install evdev`

## Installing UDEV Rules

```
sudo cp 95-pedal.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```
