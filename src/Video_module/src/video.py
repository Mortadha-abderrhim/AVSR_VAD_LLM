#!/usr/bin/env python3

""" 
@Author: David Roch, Lucas Represa, Zeyu Pang, Daniel Tozadore
@Date: 05.01.2024
@Description : Video Class
	This is the main part of the Video module.
	First make sure to receive all data to build the video properly and then process it.
"""

import rospy
from std_msgs.msg import String, Bool

from video_builder import Video_builder
from video_processing import Video_processing
from LLM import LLM

import pickle
import os
import rospkg

import shutil
import time
from Phi import Phi

class Video:
	def __init__(self):
		self.fps = 25
		self.image_format = rospy.get_param("/image/image_format", "jpeg")
		self.evaluation = rospy.get_param("/evaluation/eval", False)
		experiment = rospy.get_param("/video/experiment", False)
		self.LLM_bool = rospy.get_param("/robot/LLM", False)
		config_file = "/home/karray/Desktop/catkin_ws_PANG_M/VSR/configs/LRS3_AV_WER0.9.ini"

		# prepare where to save the video
		self.data_path = "/home/karray/Desktop/catkin_ws_PANG_M/src/Video_module"
		self.data_path += "/data"
		if experiment:
			os.makedirs(self.data_path,exist_ok=True)
			totalDir = len([name for name in os.listdir(self.data_path)]) - 1
			self.data_path = self.data_path+"/"+str(totalDir)
		
			number_of_noise_lvl = rospy.get_param("/video/number_of_noise_lvl", "4")
			for i in range(number_of_noise_lvl):
				os.makedirs(self.data_path+"/noise"+str(i))
		else:
			self.data_path += "/current"
			if not os.path.exists(self.data_path):
				os.makedirs(self.data_path)
		
		# Video builder init
		self.Video_builder = Video_builder(self.image_format)
		
		# Video processing init
		if self.evaluation:
			self.filename = "/video"
			rospy.Subscriber('/evaluator/video_path', String, self.update_filename)
			self.Video_processing = Video_processing(config_file, self.data_path + "/video.mp4")
		else:
			rospy.Subscriber("/state_manager/video_name", String, self.update_filename)
			self.filename = "/video"
			print(".... " +config_file)
			print("**** " +self.data_path + self.filename)
			self.Video_processing = Video_processing(config_file, self.data_path + self.filename + ".mp4")

		if self.image_format == "landmarks":
			self.Video_processing.args.landmarks_filename = self.data_path + self.filename + ".pkl"

		# LLM init
		self.LLM = Phi()
		self.model_id = "-1"
		rospy.Subscriber("/model/choice",String,self.update_model)
		self.state = ""
		rospy.Subscriber("/state_manager/state", String, self.setState)

		self.pub_ready = rospy.Publisher("/video_builder/ready", Bool, queue_size=1)
		self.pub_video_build = rospy.Publisher("/video_builder/video_building_done", Bool, queue_size=1)
		self.pub_res = rospy.Publisher("/video_builder/result", String, queue_size=1)
		self.pub_LLM = rospy.Publisher("/video_builder/LLM", String, queue_size=1)
		self.pub_LLM_done = rospy.Publisher("/video_builder/LLM_done", Bool, queue_size=1)
		self.video_built = False
		self.inference_done = False		

	def setState(self, msg):
		self.state = msg.data

	def update_model(self, msg):
		if msg.data != self.model_id:
			self.model_id = msg.data
			self.LLM.new_conv(msg.data)

	def update_filename(self, msg):
		if msg.data:
			self.filename = msg.data
		
	def __call__(self):
		rate = rospy.Rate(self.fps)
		
		while not rospy.is_shutdown():
			if self.state == "Idle" or self.state == "Evaluation_idle":
				self.Video_builder.current_audio = []
				self.Video_builder.current_video = []
				self.pub_ready.publish(True)
			else:
				self.pub_ready.publish(False)

			if self.state == "Video_building" and (self.Video_builder.current_video or self.Video_builder.current_audio):
				if not self.evaluation:
					if self.image_format == "landmarks":
						pickle.dump(self.Video_builder.current_landmarks, open(self.data_path + self.filename + ".pkl", 'wb'))

					"""
					Reducing the number of fps avoid floating precision error in the moviepy library which is calculating 
					the duration of the video based on the fps and the number of frames to then place 
					the frame[n] every duration/fps which sometime produce a list index out of bound.
					"""
					fps = self.fps
					built = False
					while not built:
						clip = self.Video_builder.buildVideo(fps, self.data_path)
						clip.write_videofile((self.data_path + self.filename + ".mp4").replace(" ", "_"),fps=self.fps)
						save_videos = True
						if save_videos:
							if not os.path.exists(self.data_path + '/save'):
								os.makedirs(self.data_path + '/save')
							shutil.copyfile((self.data_path + self.filename + ".mp4").replace(" ", "_"), (self.data_path + '/save' + self.filename + str(time.time()) + ".mp4").replace(" ", "_"))
						built = True
						"""except:
							if fps <= 1:
								rospy.loginfo("There was a float precision exceptions error in the building of the video.")
								break
							fps -= 1
							rospy.loginfo("There was a float precision exceptions error in the building of the video. Retrying with " + str(fps) + " fps.")
						"""
					
					self.Video_builder.current_audio = []
					self.Video_builder.current_video = []
					self.current_landmarks = []

				else: # To handle the case of built vLLM_done.mp4" in save data path
						source_path =  self.filename
						destination_path = self.data_path +  '/video.mp4'

						shutil.copy(source_path, destination_path)

						built = True

				self.pub_video_build.publish(True)
				self.video_built = True

			elif self.state == "Inference" and self.video_built:
				self.video_built = False
				res = ""
				
				try:
					res = self.Video_processing.compute()
				except AttributeError:
					print('The video procesing didnt work properly. Maybe it was not able to detect faces in the image.')
				except:
					print('The video procesing didnt work properly. Further investigation are required.')
				self.pub_res.publish(res)
				self.inference_done = True
				
			elif self.state == "LLM" and self.inference_done:
				self.inference_done = False
				corr = ""
				
				corr = self.LLM.compute(self.filename, res)

				
				self.pub_LLM.publish(corr)
				self.pub_LLM_done.publish(True)

			rate.sleep()


if __name__ == '__main__':
	rospy.init_node('Video')

	video = Video()
	try:
		video()
	except rospy.ROSInterruptException:
		pass
