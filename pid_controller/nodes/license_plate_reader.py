#!/usr/bin/env python

import cv2
import numpy as np
from keras import models as m
from PIL import Image
from matplotlib import pyplot as plt

from time import sleep

number_file =  '/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/CNN models/NumberModel'
letter_file = '/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/CNN models/LetterModel'


class Reader:
	def __init__(self):
		self.number_model = m.load_model(number_file, compile=False)
		self.number_model._make_predict_function()
		self.letter_model = m.load_model(letter_file, compile=False)
		self.letter_model._make_predict_function()

	def license_read(self, img):
		#Resize image
		image = cv2.resize(img, (100,20))

		#Convert to RGB
		image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

		#Section image
		let1 = image[:,7:25]
		let2 = image[:,26:44]
		num1 = image[:,56:74]
		num2 = image[:,74:92]

		#Send through CNNs
		l1 = self.letter_model.predict(np.expand_dims(let1,axis=0))[0]
		l2 = self.letter_model.predict(np.expand_dims(let2,axis=0))[0]
		n1 = self.number_model.predict(np.expand_dims(num1,axis=0))[0]
		n2 = self.number_model.predict(np.expand_dims(num2,axis=0))[0]

		#Convert to characters
		l1 = chr(np.argmax(l1) + ord('A'))
		l2 = chr(np.argmax(l2) + ord('A'))
		n1 = str(np.argmax(n1).item())
		n2 = str(np.argmax(n2).item())

		#Combine and send off
		plate = l1 + l2 + n1 + n2

		return plate
		

def main():
	img = cv2.imread('/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/New_labeled/AD61.png',1)
	read = Reader()
	print(read.license_read(img))

if __name__ == '__main__':
	main()