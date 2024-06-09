#!/usr/bin/env python3

""" 
@Author: David Roch, Lucas Represa, Zeyu Pang, Daniel Tozadore
@Date: 05.01.2024
@Description : State_Manager Class
	Make sure that the process is going well by changing the states when the control are passed.
"""

import rospy
from std_msgs.msg import Bool, String, Int16
import time

import rospkg

class State_Manager:
	def __init__(self):
		self.time_ref = 0

		self.state = ""
		self.pub = rospy.Publisher('/state_manager/state', String, queue_size=1)
		rospy.Subscriber('/state_manager/state', String, self.setState)

		self.fps = rospy.get_param("/image/fps", "30")
		self.rate = rospy.Rate(self.fps)

		self.experiment = rospy.get_param("/video/experiment", "False")
		self.sentence_file = rospy.get_param("/video/sentence_file")
		self.number_of_noise_lvl = rospy.get_param("/video/number_of_noise_lvl", "4")
		self.use_LLM = rospy.get_param("/robot/LLM", "False")
		state_manager_path = rospkg.RosPack().get_path('state_manager')

		self.eval = rospy.get_param("/evaluation/eval", "False")

		self.current_sentence = 0
		rospy.Subscriber('/visual_module/update_sentence', Bool, self.update_sentence)

		self.pub_sentence_to_read = rospy.Publisher("/state_manager/sentence", String, queue_size=1)
		self.sentences = []
		if self.experiment:
			file = open(state_manager_path +"/sentences/"+ self.sentence_file, 'r')
			self.sentences = file.readlines()
			for i in range(len(self.sentences)):
				self.sentences[i] = self.sentences[i].rstrip("\n")
			self.pub_sentence_to_read.publish(self.sentences[self.current_sentence])

			experiment_procedure = open(state_manager_path +"/sentences/experiment_procedure.txt", 'r')
			print(experiment_procedure.read())

		self.noise_lvl = 0
		rospy.Subscriber('/visual_module/update_noise_lvl', Bool, self.setNoise)
		self.pub_noise_lvl = rospy.Publisher('/state_manager/noise_lvl', Int16, queue_size=1)

		self.pub_video_name = rospy.Publisher("/state_manager/video_name", String, queue_size=1)

		self.record = False
		rospy.Subscriber("/visual_module/recording_request", Bool, self.cam_recording_request)
		rospy.Subscriber("/audio_module/recording_request", Bool, self.mic_recording_request)

		self.transfering = False
		self.image_transfer = False
		rospy.Subscriber("/visual_module/transfer_done", Bool, self.is_visual_module_transfer_done)
		self.audio_transfer = False
		rospy.Subscriber("/audio_module/transfer_done", Bool, self.is_audio_module_transfer_done)

		self.video_building = False
		rospy.Subscriber("/video_builder/video_building_done", Bool, self.is_video_building_done)

		self.inference = False
		rospy.Subscriber("/video_builder/result", String, self.is_inference_done)

		self.LLM = False
		rospy.Subscriber("/video_builder/record_done", Bool, self.is_LLM_done)

		self.vb_ready = False
		rospy.Subscriber("/video_builder/ready", Bool, self.is_VB_ready)

		self.visual_module_ready_to_record = False
		self.audio_module_ready_to_record = False

	def setState(self, msg):
		self.state = msg.data

	def is_VB_ready(self, msg):
		self.vb_ready = msg.data

	def cam_recording_request(self, msg):
		self.visual_module_ready_to_record = msg.data

	def mic_recording_request(self, msg):
		self.audio_module_ready_to_record = msg.data

	def recording_request(self, msg):
		if (self.state == "Idle" or self.state == "Evaluation_idle") and not self.vb_ready:
			rospy.loginfo("The video_builder node is not launched or not ready.")
		elif msg.data:
			if self.state == "Idle":
				if self.experiment:
					video_name = "/noise" + str(self.noise_lvl)+"/" + self.sentences[self.current_sentence]
					self.pub_video_name.publish(video_name)
				self.record = True
				rospy.loginfo("Recording")
			elif self.state == "Evaluation_idle":
				self.record = True
				rospy.loginfo("Evaluation_recording")
		elif not msg.data:
			if self.state == "Recording" or self.state == "Evaluation_recording":
				self.time_ref = time.time()
				self.record = False
				self.transfering = True
				rospy.loginfo("Transfering")


	def is_visual_module_transfer_done(self, msg):
		if self.state == "Transfering" and msg.data:
			self.image_transfer = True
	
	def is_audio_module_transfer_done(self, msg):
		if self.state == "Transfering" and msg.data:
			self.audio_transfer = True

	def is_transfer_done(self):
		rospy.logdebug("The image transfer is %sly done and the audio transfer is %sly done", self.image_transfer , self.audio_transfer)
		if self.image_transfer and self.audio_transfer:
			rospy.logdebug("transfer done")
			self.transfering = False
			self.video_building = True
			self.image_transfer = False
			self.audio_transfer = False
			rospy.loginfo("Video_building")

	def is_video_building_done(self, msg):
		if self.state == "Video_building" and msg.data:
			rospy.logdebug("It tooks %s seconds from the end of the recording to the begginning of inference.", time.time()-self.time_ref)
			self.video_building = False
			if not self.experiment:
				self.inference = True
				rospy.loginfo("Inference")
			else:
				rospy.loginfo("Idle")
				self.current_sentence = (self.current_sentence + 1)%len(self.sentences)
				self.pub_sentence_to_read.publish(self.sentences[self.current_sentence])
	
	def is_inference_done(self, msg):
		if self.state == "Inference" and msg.data:
			if not self.experiment:
				if self.use_LLM:
					rospy.loginfo("LLM")
					self.LLM = True
				elif self.eval:
					self.state = "Evaluation_idle"
				else:
					self.state = "Idle"
			else:
				rospy.loginfo("Idle")
				self.current_sentence = (self.current_sentence + 1)%len(self.sentences)
				self.pub_sentence_to_read.publish(self.sentences[self.current_sentence])
			self.inference = False
			  # Transition to LLM state after inference is done

	def is_LLM_done(self, msg):
		if self.state == "LLM":
			self.LLM = False
			if self.eval:
				self.state = "Evaluation_idle"
			else:
				self.state = "Idle"

	def setNoise(self, msg):
		if self.state == "Idle":
			self.noise_lvl = (self.noise_lvl + 1)%self.number_of_noise_lvl
			self.pub_noise_lvl.publish(self.noise_lvl)
		
	def update_sentence(self, msg):
		if self.state == "Idle":
			if msg.data:
				self.current_sentence = self.current_sentence + 1
			else:
				self.current_sentence = self.current_sentence - 1
			self.current_sentence %= len(self.sentences)
			self.pub_sentence_to_read.publish(self.sentences[self.current_sentence])

	def __call__(self):
		while not rospy.is_shutdown():
			if self.record:
				msg = Bool()
				msg.data = False
				if self.eval:
					self.pub.publish("Evaluation_recording")
					if not self.audio_module_ready_to_record and not self.visual_module_ready_to_record:
						self.recording_request(msg)
				else:
					self.pub.publish("Recording")
					if not self.visual_module_ready_to_record:
						self.recording_request(msg)
			elif self.transfering:
				self.is_transfer_done()
				self.pub.publish("Transfering")
			elif self.video_building:
				self.pub.publish("Video_building")
			elif self.inference:
				self.pub.publish("Inference")
			elif self.LLM:
				self.pub.publish("LLM")
			else:
				msg = Bool()
				msg.data = True
				if self.eval:
					self.pub.publish("Evaluation_idle")
					if self.audio_module_ready_to_record and self.visual_module_ready_to_record:
						self.recording_request(msg)
				else:
					self.pub.publish("Idle")
					if self.visual_module_ready_to_record:
						self.recording_request(msg)

				if self.experiment:
					self.pub_sentence_to_read.publish(self.sentences[self.current_sentence])

			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('State_Manager')

	state_manager = State_Manager()
	try:
		state_manager()
	except rospy.ROSInterruptException:
		pass
