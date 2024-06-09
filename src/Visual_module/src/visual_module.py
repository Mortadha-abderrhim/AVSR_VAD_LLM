#!/usr/bin/env python3

""" 
@Author: David Roch, Daniel Tozadore
@Date: 01.05.2022
@Description : Visual_module Class
	Take the images and transfer them in the correct format to the Video module.
"""

import rospy
from std_msgs.msg import Bool, String, UInt64, Int16MultiArray, Int16
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge,CvBridgeError

import cv2
import numpy as np

from utils.FPS import FPS



#robot_connected = True

class Visual_module:
	def __init__(self):
		self.image_format = rospy.get_param("/image/image_format", "jpeg")
		self.fps = 25
		self.rate = rospy.Rate(self.fps)
		self.robot_connected = rospy.get_param("/robot/connected")
		self.camera_topic = rospy.get_param("/robot/camera")

		if self.image_format == "landmarks":
			import sys, os
			sys.path.append(os.environ['VSR_DIR'])
			from tracker.mediapipe_preprocess import Mediapipe
			self.landmarks_tracker = Mediapipe()

		self.eval = rospy.get_param("/evaluation/eval", "False")

		self.bridge = CvBridge()
		
		self.state = ""
		rospy.Subscriber("/state_manager/state", String, self.setState)

		self.nb_frame_sent = 0
		self.nb_frame_received = 0
		rospy.Subscriber("/video_builder/nb_frame_received", UInt64, self.update_nb_frame_received)
		self.pub_transfer_done = rospy.Publisher("/visual_module/transfer_done", Bool, queue_size=1)
		
		self.pub_image = rospy.Publisher("/visual_module/raw_image", Image, queue_size=1000)
		self.pub_compressed_image = rospy.Publisher("/visual_module/raw_image/compressed", CompressedImage, queue_size=1000)
		self.pub_landmark = rospy.Publisher("/visual_module/landmarks", Int16MultiArray, queue_size=1000)
		
		if self.robot_connected:
			self.sub_image = rospy.Subscriber(self.camera_topic, Image, self.get_qt_image)
			self.img_msg = None

		self.pub_noise_lvl = rospy.Publisher('/visual_module/update_noise_lvl', Bool, queue_size=1)
		self.noise_lvl = 0
		rospy.Subscriber('/state_manager/noise_lvl', Int16, self.update_noise_lvl)
		self.pub_update_sentence = rospy.Publisher('/visual_module/update_sentence', Bool, queue_size=1)

		self.pub_recording_request = rospy.Publisher("/visual_module/recording_request", Bool, queue_size=1)

		self.msg_understood = ""
		rospy.Subscriber("/video_builder/result", String, self.save_result)

		self.sentence_to_read = ""
		rospy.Subscriber("/state_manager/sentence", String, self.update_sentence_to_read)

		self.FPS = FPS()

		rospy.Subscriber('/evaluator/video_path', String, self.start_image_eval)
		self.video = None

	def setState(self, msg):
		self.state = msg.data

	def update_nb_frame_received(self, msg):
			self.nb_frame_received = msg.data
	
	def control_nb_frame_sent(self):
		rospy.logdebug("There is %s frame sent and %s frame received.", self.nb_frame_sent , self.nb_frame_received)
		if self.nb_frame_received == self.nb_frame_sent:
			self.pub_transfer_done.publish(True)
			self.nb_frame_sent = 0
			self.nb_frame_received = 0
		else:
			self.pub_transfer_done.publish(False)
	
	def get_qt_image(self, msg):
		try:
			# self.img_msg = msg
			try:
				self.img_msg = self.bridge.imgmsg_to_cv2(msg, "bgr8")
				# ret = True
			except CvBridgeError as e:
				print(e)
			(rows, cols, channels) = self.img_msg.shape
			if cols > 60 and rows > 60:
				cv2.circle(self.img_msg, (50,50), 10, 255)
				
		except:
			print(" ERROR TAKING MSG IMAGE ")
			
	def save_result(self, msg):
		self.msg_understood = msg.data

	def update_sentence_to_read(self, msg):
		self.sentence_to_read = msg.data

	def update_noise_lvl(self, msg):
		self.noise_lvl = msg.data

	def start_image_eval(self, msg):
		self.video = cv2.VideoCapture(msg.data)
		self.pub_recording_request.publish(True)
		rospy.loginfo_once("Waiting to start evaluation")

	def get_image_eval(self):
		ret, img = self.video.read()
		if not ret:
			self.pub_recording_request.publish(False)
			self.video = None
		return ret, img

	def __call__(self):
		
		if not self.robot_connected:
			vid_capture = cv2.VideoCapture(-1)
			

			vid_capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # stop the auto-exposure
			vid_capture.set(cv2.CAP_PROP_FPS, self.fps)
			vid_capture.set(cv2.CAP_PROP_EXPOSURE, 250)
			if vid_capture.get(cv2.CAP_PROP_FPS) != self.fps:
				print(vid_capture.get(cv2.CAP_PROP_FPS) )
				self.fps = vid_capture.get(cv2.CAP_PROP_FPS)
			
		ret = False




		while not rospy.is_shutdown():

			if  self.robot_connected:
				try:
					img = self.img_msg
					if self.img_msg is not None:
						ret = True
				except CvBridgeError as e:
					ret = False
					print(e)


			else:

				if self.state == "Evaluation_recording":
					if self.video:
						ret, img = self.get_image_eval()
				else:
					ret, img = vid_capture.read()

	


			if ret:
				draw_img = img.copy()
				draw_img = np.pad(draw_img, [(0,20),(0,0),(0,0)], mode='constant')
				draw_img = cv2.resize(draw_img, (int(1.8*draw_img.shape[1]),int(1.8*draw_img.shape[0])))
				if self.state == "Idle" or self.state == "Evaluation_idle":
					self.nb_frame_sent = 0
				elif self.state == "Recording" or self.state == "Evaluation_recording":
					self.nb_frame_sent += 1
					if self.image_format == "raw_images":
						msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
						self.pub_image.publish(msg)
					elif self.image_format == "jpeg" or self.image_format == "png" or self.image_format == "landmarks":
						im_format = self.image_format

						if self.image_format == "landmarks":
							msg = Int16MultiArray()
							self.landmarks_tracker(img)
							msg.data = (np.int16(self.landmarks_tracker.findFacesMesh(img)).flatten()).tolist()
							self.pub_landmark.publish(msg)
							im_format = "jpeg"

						msg = self.bridge.cv2_to_compressed_imgmsg(img, dst_format=im_format)
						self.pub_compressed_image.publish(msg)
					else: raise NameError('The image_format ', self.image_format," is not recognised.")

					cv2.circle(draw_img, (40,draw_img.shape[0]-80), radius=30, color=(0, 0, 255), thickness=-1)
				elif self.state == "Transfering":
					self.control_nb_frame_sent()

				draw_img = self.FPS.printFps(draw_img)

				draw_img = cv2.putText(draw_img, self.msg_understood, (100, int(draw_img.shape[0]*9/10)), cv2.FONT_HERSHEY_SIMPLEX, 
                   			1, (0, 0, 255), 2, cv2.LINE_AA)
				draw_img = cv2.putText(draw_img, self.state +", noise lvl: "+ str(self.noise_lvl), (20, int(draw_img.shape[0]*1/10)), cv2.FONT_HERSHEY_SIMPLEX, 
                   			1, (0, 0, 255), 2, cv2.LINE_AA)
				draw_img = cv2.putText(draw_img, self.sentence_to_read, (0, int(draw_img.shape[0])-15), cv2.FONT_HERSHEY_SIMPLEX, 
                   			1, (255, 255, 255), 2, cv2.LINE_AA)
   
				cv2.imshow('visual_module',draw_img)
				k = cv2.waitKey(1)
				if not self.eval:
					# press i to increase the noise_lvl
					if k == 105:
						self.pub_noise_lvl.publish(True)
					# press n to go to the next sentence of the experiment
					elif k == 110:
						self.pub_update_sentence.publish(True)
					# press p to come back to the previous sentence of the experiment
					elif k == 112:
						self.pub_update_sentence.publish(False)
					# press r to record the stream
					elif k == 114:
						self.pub_recording_request.publish(True)
					# press s to stop the recording
					elif k == 115:
						self.pub_recording_request.publish(False)
					# press space to start/stop recording
					elif k == 32:
						if self.state == "Idle":
							self.pub_recording_request.publish(True)
						elif self.state == "Recording":
							self.pub_recording_request.publish(False)

				# press q to quit the stream (and stop the recording)
				if k == 113:
					cv2.destroyAllWindows()
					
					if not self.robot_connected:
						vid_capture.release()
						vid_capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # set back the auto-exposure
						vid_capture.set(cv2.CAP_PROP_FPS, 30)
					
					break
			else:
				rospy.loginfo("Waiting visual_module.")
				#break

			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('visual_module')

	visual_module = Visual_module()
	try:
		visual_module()
	except rospy.ROSInterruptException:
		pass
