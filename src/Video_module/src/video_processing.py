#!/usr/bin/env python3

""" 
@Author: David Roch, Zeyu Pang, Daniel Tozadore
@Date: 05.01.2024
@Description : Video_processing Class
	Init and launch the Video_processing algorithm.
"""

import sys
import os
sys.path.append(os.environ['VSR_DIR'])
os.environ["WANDB_DISABLED"] = "true"
os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"]="0" # limiting to one GPU

from VSR import one_step_inference, load_args
from pipelines.pipeline import InferencePipeline
from std_msgs.msg import String
import torch
import rospy

import time

class Video_processing:
	def __init__(self, config_file, video_path, landmarks_path=None):
		"""
		@ Description:
			Prepare the args of the video processing algorithm.
		@ Input:
			video_path: Path to the video to process.
			landmarks_path: Path to the landmarks of the video.
		"""
		
		self.args = load_args()
		path = ""
		print(config_file)
		self.args.config_filename = path + config_file
		self.args.data_filename = video_path
		self.args.landmarks_filename = landmarks_path
		self.args.detector = rospy.get_param("/evaluation/preprocessing", 'mediapipe')

		device = "cpu"
		# -- pick device for inference.
		"""if torch.cuda.is_available():
			device = torch.device(f"cuda:0")"""
		print(self.args.config_filename)
		self.lipreader = InferencePipeline(
			self.args.config_filename,
			device=device,
			detector=self.args.detector,
			face_track=True)
		
	def compute(self):
		"""
		@ Description:
			Do the video processing.
		"""
		print("Start video processing.")
		print("Video path:", self.args.data_filename)
		
		start_t = time.time()
		pub_AVSR_start=rospy.Publisher("/video_builder/AVSR_start", String, queue_size=1)
		msg_AVSR_start=String()
		msg_AVSR_start.data=str(start_t)
		pub_AVSR_start.publish(msg_AVSR_start)
		
		sentence = ""
		sentence = one_step_inference(
            self.lipreader,
            self.args.data_filename,
            self.args.landmarks_filename,
            self.args.dst_filename,
        )

		end_t=time.time()
		pub_AVSR_end=rospy.Publisher("/video_builder/AVSR_end", String, queue_size=1)
		msg_AVSR_end=String()
		msg_AVSR_end.data=str(end_t)
		pub_AVSR_end.publish(msg_AVSR_end)

		print("The video processing took:", end_t-start_t)
		return sentence
