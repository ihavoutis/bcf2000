#!/usr/bin/env python
#
# joystick input driver for Behringer BCF2000 Motorized midi fader device
#			(...based on the korg nanokontrol/kontol.py source)
# Author: Ioannis Havoutis

import roslib; roslib.load_manifest('bcf2000')
import rospy

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import *

control_axes = {
  # knobs [1st set]
   1:  0,  2:  1,  3:  2,  4:  3,  5:  4,  6:  5,  7:  6, 8:  7,
  # knobs [2nd set]
   9:  8,  10:  9,  11:  10,  12:  11,  13:  12,  14:  13,  15:  14, 16:  15,
  # knobs [3rd set]
   17:  16,  18:  17,  19:  18,  20:  19,  21:  20,  22:  21,  23:  22, 24:  23,
  # knobs [4th set]
   25:  24,  26:  25,  27:  26,  28:  27,  29:  28,  30:  29,  31:  30, 32:  31,
  # sliders
   81:  32,  82:  33,  83:  34,  84:  35,  85:  36,  86:  37,  87:  38, 88:  39
  }

control_buttons = [
  # knob-buttons [1st set]
  33, 34, 35, 36, 37, 38, 39, 40,
  # knob-buttons [2nd set]
  41, 42, 43, 44, 45, 46, 47, 48,
  # knob-buttons [3rd set]
  49, 50, 51, 52, 53, 54, 55, 56,
  # knob-buttons [4th set]
  57, 58, 59, 60, 61, 62, 63, 64,
  # buttons under the knobs [1st line]
  65, 66, 67, 68, 69, 70, 71, 72,
  # buttons under the knobs [2nd line]
  73, 74, 75, 76, 77, 78, 79, 80,
  # buttons bottom-right
  89, 90, 91, 92
]


def main():
   pygame.midi.init()
   devices = pygame.midi.get_count()
   if devices < 1:
      print "No MIDI devices detected"
      exit(-1)
   print "Found %d MIDI devices" % devices

   if len(sys.argv) > 1:
      input_dev = int(sys.argv[1])
   else:
      input_dev = pygame.midi.get_default_input_id()
      print "Using DEFAULT input device %d" % input_dev
      if input_dev == -1:
         print "No default MIDI input device"
         exit(-1)
   print "Using input device %d" % input_dev

   if len(sys.argv) > 2:
      output_dev = int(sys.argv[2])
   else:
      output_dev = pygame.midi.get_default_output_id()
      print "Using DEFAULT output device %d" % input_dev
      if output_dev == -1:
         print "No default MIDI output device"
         exit(-1)
   print "Using output device %d" % output_dev

   controller = pygame.midi.Input(input_dev)
   controller_input = pygame.midi.Output(output_dev)

   m = Joy()
   m.axes = [ 0 ] * len(control_axes)
   m.buttons = [ 0 ] * len(control_buttons)

   
   rospy.init_node('bcf2000_driver_node')
   pub = rospy.Publisher('/bcf2000/joy', Joy, latch=True, queue_size=1)

   def callback(data):
	   if len(data.axes)>len(control_axes):
		print "Joy message malformed: axes field too long!"
		return
	   index = 0
	   for val in data.axes:
		val = 0.0 if val < 0.0 else 1.0 if val > 1.0 else val
		#print 'index: ' + str(index) + ', axis: ' + str(control_axes.keys()[index]) + '  ...'
		controller_input.write([[[176, int(sorted(control_axes.keys())[index]), int(val*127.0), 0], 0]])
		m.axes[index] = val
		index += 1
	
	   if len(data.buttons)>len(control_buttons):
		print "Joy message malformed: buttons field too long!"
		return
	   index = 0
	   for val in data.buttons:
		if val > 0: val = 1
		if val < 0: val = 0
		val = 0 if val < 0 else 1 if val > 1 else val
		controller_input.write([[[176, int(control_buttons[index]), int(val*127), 0], 0]])
		m.buttons[index] = val
		index += 1

	   pub.publish(m)

   sub = rospy.Subscriber('/bcf2000/joy/input', Joy, callback)

   p = False

   if len(sys.argv) > 3: #Set everything to 0 at startup if (ANY) 3rd argument is passed
    for i in range(1,93):
     controller_input.write([[[176, i, 0, 0], 0]])

   while not rospy.is_shutdown():
      m.header.stamp = rospy.Time.now()
      # count the number of events that are coalesced together
      c = 0
      while controller.poll():
         c += 1
         data = controller.read(1)
         #print data
         # loop through events received
         for event in data:
            control = event[0]
            timestamp = event[1]

            # look for continuous controller commands
            if (control[0] & 0xF0) == 176:
               control_id = control[1] | ((control[0] & 0x0F) << 8)

            if control_id in control_axes:
               control_val = int(control[2])
               axis = control_axes[control_id]
               m.axes[axis] = control_val/127.0 # normalize value to 1
               p = True

            if control_id in control_buttons:
               button = control_buttons.index(control_id)
               if control[2] != 0:
                 m.buttons[button] = 1
               else:
                 m.buttons[button] = 0
               p = True

      if p:
         pub.publish(m)
         p = False
   
      rospy.sleep(0.1) # 10Hz maximum input


if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException: pass
