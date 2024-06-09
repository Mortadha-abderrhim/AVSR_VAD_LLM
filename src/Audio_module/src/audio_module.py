#!/usr/bin/env python3

""" 
@Author: David Roch, Daniel Tozadore
@Date: 01.05.2022
@Description : Audio_module Class
	Take the sounds and transfer them in the correct format to the Video module.
"""

import rospy
from std_msgs.msg import Int16MultiArray, String, UInt64, Bool
import torch
import pyaudio
from moviepy.editor import VideoFileClip
from moviepy.editor import AudioFileClip
import numpy as np
from utils.audio_format import float_to_byte
from audio_common_msgs.msg import AudioData
class Audio_module:
	def __init__(self):
		self.record = False
		self.robot_record = rospy.get_param("/audio/robot","False")
		self.channels = rospy.get_param("/audio/channels", "2")
		self.sample_rate = rospy.get_param("/audio/sample_rate", "44100")
		self.sample_format = rospy.get_param("/audio/sample_format", "pyaudio.paInt16")
		self.chunk = rospy.get_param("/audio/chunk", "1024")
		self.eval = rospy.get_param("/evaluation/eval", "False")
		if self.robot_record == "True":
			rospy.Subscriber('/qt_respeaker_app/channel0', AudioData,self.get_audio_from_robot)
		self.fps = rospy.get_param("/image/fps", "30")
		
		self.state = ""
		rospy.Subscriber("/state_manager/state", String, self.setState)

		self.nb_chunk_sent = 0
		self.nb_chunk_received = 0
		rospy.Subscriber("/video_builder/nb_chunk_received", UInt64, self.update_nb_chunk_received)
		self.pub_transfer_done = rospy.Publisher("/audio_module/transfer_done", Bool, queue_size=1)

		self.pub = rospy.Publisher("/audio_module/audio", Int16MultiArray, queue_size=20000)
		rospy.Subscriber('/evaluator/video_path', String, self.start_sound_eval)
		self.video = None

		self.pub_recording_request = rospy.Publisher("/audio_module/recording_request", Bool, queue_size=1)
		self.pub_recording_request_viz = rospy.Publisher("/visual_module/recording_request", Bool, queue_size=1)
		self.model, _ = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                              model='silero_vad',
                              force_reload=True,
                              onnx=False)
		self.muted_windows = 0
	def get_audio_from_robot(self,msg):
		self.audio_from_robot = msg.data


	def audio_to_tensor(self,audio_frame: bytes):
		buffer = np.frombuffer(audio_frame, dtype=np.int16).astype(np.float32) / 32767.0
		return torch.from_numpy(buffer)
	
	def setState(self, msg):
		self.state = msg.data

	def update_nb_chunk_received(self, msg):
		self.nb_chunk_received = msg.data

	def control_nb_chunk_sent(self):
		rospy.logdebug("There is %s chunk sent and %s chunk received.", self.nb_chunk_sent , self.nb_chunk_received)
		if self.nb_chunk_received == self.nb_chunk_sent:
			self.pub_transfer_done.publish(True)
			self.nb_chunk_sent = 0
			self.nb_chunk_received = 0
		else:
			self.pub_transfer_done.publish(False)

	def start_sound_eval(self, msg):

		print("msg.data: ", msg.data)
		audio = VideoFileClip(msg.data).audio.iter_chunks(chunksize = self.chunk, fps = self.sample_rate)
		
		video = []
		while True:
			n = next(audio, None)
			if n is None:
				break
			for sublist in n:
				# Read the first audio channel of the video
				video.append(sublist[0])
		self.video = list(float_to_byte(video))
		self.pub_recording_request.publish(True)
		rospy.loginfo_once("Waiting to start evaluation")

	def get_sound_eval(self):
		first = None
		if len(self.video)>= self.chunk:
			first = self.video[0:self.chunk]
			del self.video[0:self.chunk]
		else:
			self.video = None
			self.pub_recording_request.publish(False)
			
		return first


	def __call__(self):
		rate = rospy.Rate(self.sample_rate)
		if self.robot_record == "False":
			p = pyaudio.PyAudio()  # Create an interface to PortAudio

			dict = {"pyaudio.paInt8" : pyaudio.paInt8, "pyaudio.paInt16": pyaudio.paInt16, "pyaudio.paInt32": pyaudio.paInt32}
			
			stream = p.open(format=dict[self.sample_format],
					channels=self.channels,
					rate=self.sample_rate,
					frames_per_buffer=self.chunk,
					input=True)


		data = None
		

		self.spoken= False
		while not rospy.is_shutdown():
			if self.state == "Evaluation_recording":
				data = None
				if self.video is not None:
					data = self.get_sound_eval()
			else:
				if self.robot_record == "False":
					data = stream.read(self.chunk)
				else:
					data = self.audio_from_robot

			if not(data is None):
				if self.state == "Idle" or self.state == "Evaluation_idle":
					self.nb_chunk_sent = 0
				elif self.state == "Recording" or self.state == "Evaluation_recording":
					self.nb_chunk_sent += 1
					msg = Int16MultiArray()
					msg.data = list(data)
					self.pub.publish(msg)
					if self.model(self.audio_to_tensor(data), self.sample_rate).item() < 0.3:
						self.muted_windows += 1
					else:
						self.muted_windows = 0
						self.spoken = True
					print(self.muted_windows)
					if self.muted_windows > 85:
						self.pub_recording_request_viz.publish(False)
						self.pub_recording_request.publish(False)
						self.muted_windows = 0
				elif self.state == "Transfering":
					self.control_nb_chunk_sent()
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('Audio_module')

	audio_module = Audio_module()
	try:
		audio_module()
	except rospy.ROSInterruptException:
		pass
