#!/usr/bin/env python

import cv2
import numpy as np
from keras import models as m

from PIL import Image

number_file =  '/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/CNN models/NumberCNN'
#letter_file = '/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/CNN models/LetterModel'


class Reader:
	def __init__(self):
		self.number_model = m.load_model(number_file)
		#self.letter_model = m.load_model(letter_file)

	def license_read(self, img):
		#Resize image
		img = cv2.resize(img, (100,20))

		#Section image
		let1 = img[:,6:24]
		let2 = img[:,25:42]
		num1 = img[:,57:74]
		num2 = img[:,75:92]

		#Send through CNNs
		l1 = letter_model.predict(let1)
		l2 = letter_model.predict(let2)
		n1 = number_model.predict(num1)
		n2 = number_model.predict(num2)

		#Convert to characters
		l1 = chr(np.argmax(l1) + ord('A'))
		l2 = chr(np.argmax(l2) + ord('A'))
		n1 = np.argmax(n1)
		n2 = np.argmax(n2)

		#Combine and send off
		plate = l1 + l2 + n1 + n2

		return plate

def main():
	Image.open('/home/fizzer/ros_ws/src/Competition_Package/pid_controller/nodes/AA01.png')
	read = Reader()
	print('Jello')

if __name__ == '__main__':
	main()