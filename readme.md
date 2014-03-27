## Behringer BCF2000

ROS driver to use the Behringer B-Control Fader motorized MIDI device as a joystick.

See [the wiki](http://ros.org/wiki/) for more information.



test with e.g.
rostopic echo /bcf2000/joy

rostopic pub -1 /bcf2000/joy/input sensor_msgs/Joy 'axes: [0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,90]'

rostopic pub -1 /bcf2000/joy/input sensor_msgs/Joy 'buttons: [1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,1,1,1]'

