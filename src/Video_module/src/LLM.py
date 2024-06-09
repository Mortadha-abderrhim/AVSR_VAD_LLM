#!/usr/bin/env python3

""" 
@Author: Lucas Represa, Pang Zeyu, Daniel Tozadore
@Date: 10.06.2023
@Description : LLM Class
	Init and launch the LLM algorithm.
"""

import os

import fastchat.serve.one_LLM_inference as inference
from fastchat.conversation import get_conv_template
import rospy
from std_msgs.msg import String
import json
import time
import string
import unicodedata

class LLM:
	def __init__(self, LLM_bool=False):
		"""
		@ Description:
			Prepare the args of the video processing algorithm.
		@ Input:
			video_path: Path to the video to process.
			landmarks_path: Path to the landmarks of the video.
		"""
		# Chat
		self.LLM_bool = LLM_bool
		# Initialize the chatbot conv object
		self.conv = None
		self.chat_model = rospy.get_param("/robot/chat_model", '/home/chiliadmin/Documents/catkin_ws/FastChat/weights/fastchat-t5-3b-v1.0')
		self.task = rospy.get_param("/robot/task", "chat")
		self.context_path = rospy.get_param("/robot/context", None)
		self.context = os.path.isfile(self.context_path)
		self.chat_device = 'cuda' 		 # ['cuda', 'cpu'] the memory is to be carefully managed with cuda
		self.chat_gpus = None     
		self.chat_num_gpus = 1
		self.chat_max_gpu_memory = 0.1   
		self.chat_load_8bit = False 	 # No possible with t5 for now
		self.chat_cpu_offloading = False # To be activated with 8-bit loading
		self.temperature = 0.7 
		self.max_new_tokens = 512
		self.debug = False

		# LLM
		if self.LLM_bool:
			print('DEVICE USED FOR CHAT: ', self.chat_device)
			print('')
			if self.context:
				print('CONTEXT USED FOR CHAT: ', self.context_path)
				print('')
			else:
				print('NO CONTEXT USED FOR CHAT')
	
	def get_context(self, conv, filename):
		""" Function to get the context of the conversation """
		type = filename.split('/')[-1].split('-')[1]
		num  = filename.split('/')[-1].split('-')[2].replace('.mp4', '')
		
		with open(self.context_path, 'r') as file:
			lesson_dict = json.load(file)
           
		if type == 'anglais':
			lessons = lesson_dict['anglais']
		elif type == 'us':
			lessons = lesson_dict['us']
		else:
			print("Didn't found the type of the lesson, maybe the filename is erronated. Type:", type)
			lessons = lesson_dict['anglais'] # default

		if num == '1':
			lesson_question = lessons['1']
		elif num == '2':
			lesson_question = lessons['2'] 
		else:
			print("Didn't found the type of the lesson, maybe the filename is erronated. Type:", num)
			lesson_question = lessons['1'] # default
		
		conv.append_message(conv.roles[1], lesson_question["lesson"] + "\n" + lesson_question["question"])
		
		return conv
	
	def conv_init(self, filename, inp):
		""" Initialize the conversation with the user."""
		if self.task == 'chat':
			conv = get_conv_template('QTChat')
			inp = inp +"?" # Influence the model to answer
			conv.append_message(conv.roles[1], None) 
		elif self.task == 'summarize':
			conv = get_conv_template('SummaryGenerator')
			# is context a file path ?
			if self.context:
				conv = self.get_context(conv, filename)
			else: 
				conv.append_message(conv.roles[1], None)
			inp = "summarize: " + inp + "." # Influence the model to translate from English to French
		elif self.task == 'translate':
			conv = get_conv_template('FRtranslator')
			if self.context:
				self.get_context(conv, filename)
			else:
				conv.append_message(conv.roles[1], None)
			inp = "translate English to French: " + inp + "." # Influence the model to translate from English to French
		else:
			raise ValueError('Task not recognized. Please choose between chat, summarize or translate.')
			
		conv.append_message(conv.roles[0], inp)
		
		
		return conv

	def remove_punctuation_accents(self, text):
		# Remove a non exhaustive list of punctuation
		text = text.replace('-', ' ')
		punctuation = string.punctuation.replace("'", "")  # Remove the single quote character from the punctuation string
		text = text.translate(str.maketrans('', '', punctuation))
		# Remove accents
		text = ''.join(
			c for c in unicodedata.normalize('NFD', text)
			if unicodedata.category(c) != 'Mn'
		)

		return text.upper()


	def compute(self, filename, inp):
		"""
		@ Description:
			Do the LLM processing.
		"""
		print(f"Start {self.task} LLM.")

		t_start = time.time()

		inp = inp.lower() # fastchat performs better with lower case

		self.conv = self.conv_init(filename, inp)

		output = inference.chat_memory(
			model_path=self.chat_model,
			device=self.chat_device,
			num_gpus=self.chat_num_gpus,
			max_gpu_memory=self.chat_max_gpu_memory,
			load_8bit=self.chat_load_8bit,
			cpu_offloading=self.chat_cpu_offloading,
			conv=self.conv,
			temperature=self.temperature,
			max_new_tokens=self.max_new_tokens,
			debug=self.debug,
			)

		# NOTE: strip is important to align with the training data.
		self.conv.messages[-1][-1] = output.strip()
		# Normalize the ouput format
		output = self.remove_punctuation_accents(self.conv.messages[-1][-1])

		t_end= time.time()
		pub_LLM_last=rospy.Publisher("/video_builder/LLM_last",String)
		msg_LLM_last=String()
		msg_LLM_last.data=str(t_end)
		pub_LLM_last.publish(msg_LLM_last)

		print("The LLM took :", t_end - t_start)

		return output
	
