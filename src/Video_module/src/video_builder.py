#!/usr/bin/env python3

""" 
@Author: David Roch, Daniel Tozadore
@Date: 01.05.2022
@Description : Video_builder Class
	Receive the data to then build the video.
"""

import rospy
from std_msgs.msg import Int16MultiArray, String, UInt64
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge

import cv2

import pyaudio
import wave

from moviepy.editor import ImageSequenceClip
from moviepy.audio.io.AudioFileClip import AudioFileClip

import numpy as np

class Video_builder:
	def __init__(self, image_format):
		self.channels = rospy.get_param("/audio/channels", "1")
		self.sample_rate = rospy.get_param("/audio/sample_rate", "44100")
		self.sample_format = rospy.get_param("/audio/sample_format", "pyaudio.paInt16")
		self.chunk = rospy.get_param("/audio/chunk", "1024")
		self.image_format = image_format

		self.bridge = CvBridge()

		self.pub_nb_frame_received = rospy.Publisher("/video_builder/nb_frame_received", UInt64, queue_size=1)
		self.pub_nb_chunk_received = rospy.Publisher("/video_builder/nb_chunk_received", UInt64, queue_size=1)

		self.current_audio = []
		self.current_video = []
		self.current_landmarks = []
		

		rospy.Subscriber("/audio_module/audio", Int16MultiArray, self.recordAudio)

		rospy.Subscriber("/visual_module/raw_image", Image, self.recordImage)
		rospy.Subscriber("/visual_module/raw_image/compressed", CompressedImage, self.recordImage)
		rospy.Subscriber("/visual_module/landmarks", Int16MultiArray, self.recordLandmarks)


	def recordAudio(self, msg):
		self.current_audio.append(msg.data)
		self.pub_nb_chunk_received.publish(len(self.current_audio))

	def recordImage(self, msg):
		if self.image_format == "raw_images":
			img = self.bridge.imgmsg_to_cv2(msg)
			img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		elif self.image_format == "jpeg" or self.image_format == "png" or self.image_format == "landmarks":
			img = self.bridge.compressed_imgmsg_to_cv2(msg)
			img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		else: raise NameError('The image_format ', self.image_format," is not recognised.")

		self.current_video.append(img)
		self.pub_nb_frame_received.publish(len(self.current_video))

	def recordLandmarks(self, msg):
		self.current_landmarks.append(np.array(msg.data).reshape(-1, 2))
		print(len(self.current_landmarks))

	def buildVideo(self, fps, data_path):
		video_clip = None

		audio = []
		
		for a in self.current_audio:
			audio.append(bytes(a))

		dict = {"pyaudio.paInt8" : pyaudio.paInt8, "pyaudio.paInt16": pyaudio.paInt16, "pyaudio.paInt32": pyaudio.paInt32}
		# Save the recorded data as a WAV file
		wf = wave.open(data_path + "/audio.wav", 'wb')
		wf.setnchannels(self.channels)
		wf.setsampwidth(pyaudio.PyAudio().get_sample_size(dict[self.sample_format]))
		wf.setframerate(self.sample_rate)
		wf.writeframes(b''.join(audio))
		wf.close()

		if self.current_video:
			image_clip = ImageSequenceClip(self.current_video, fps=fps)
			if audio:
				audio_clip = AudioFileClip(data_path + "/audio.wav", fps=self.sample_rate)
				audio_clip.nchannels = self.channels
				video_clip = image_clip.set_audio(audio_clip)
			else: video_clip = image_clip
			
		elif audio:
				audio_clip = AudioFileClip(data_path + "/audio.wav", fps=self.sample_rate)
				audio_clip.nchannels = self.channels
				video_clip = audio_clip
		
		return video_clip
