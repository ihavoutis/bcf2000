#!/usr/bin/env python

import pygame
import pygame.midi
import sys
import time

def main():

	pygame.midi.init()
	pygame.init()
	devices = pygame.midi.get_count()

	if devices < 1:
		print("No MIDI devices detected")
		exit(-1)

	print("Found %d MIDI devices" % devices)
	print("id- interface -- name ------ input -- ouput -- opened")

	inputDev = [];
	outputDev = [];

	for ii in range(devices):
		ll = pygame.midi.get_device_info(ii)
		bool1 = bool(ll[2])
		bool2 = bool(ll[3])
		bool3 = bool(ll[4])
		print(str(ii) + " -- " + str(ll[0]) + " -- " + str(ll[1]) + " -- " + str(bool1) + " -- " + str(bool2) + " -- " + str(bool3))
		if bool1:
			inputDev.append(ii)
		else:
			outputDev.append(ii)

	output2use = False
	raw_input("\nPlease move all sliders down and press enter\n")
	for ii in outputDev:
		outputController = pygame.midi.Output(ii)
		outputController.write([[[176, 81, 127, 0], 0]])
		cc = raw_input("Did the first slider move? (y/n)\n")
		if cc == "y":
			output2use = ii
			break
	
	if not output2use:
		print("No output device found. Is the mixer pluged? is it connected? is it on?\n")
		exit(-2)

	input2use = False
	raw_input("\nI'll try to detect the input device now. Please move in a continuous maner any slider or knob, and press enter when ready\n")
	for ii in inputDev:
		inputController = pygame.midi.Input(ii)
		start_time = time.time()
		while time.time() - start_time < 1:
			if inputController.read(1):
				input2use = ii
				break

	if not input2use:
		print("No input device found. Is the mixer pluged? is it connected? is it on?\n")
		exit(-2)

	print("You should use input device ",input2use," and output device ",output2use,"\n")


if __name__ == '__main__':
	main()
