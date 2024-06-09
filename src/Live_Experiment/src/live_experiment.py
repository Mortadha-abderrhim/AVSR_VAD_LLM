#!/usr/bin/env python3

""" 
@Author: Zeyu Pang, Daniel Tozadore
@Date: 05.01.2024
@Description : Live experiment Class
	Facilitate interaction with users during experiments, and save critical data in the live-stream mode
"""


from typing import Any
import rospy
from std_msgs.msg import String,Bool
import os
import csv
import sys
import random
from TalkText import TalkText
import tkinter as tk
from tkinter import simpledialog, messagebox
import pandas as pd
from robot import *

# Config colors of printing 
GREEN = "\033[0;32m"
BLUE="\033[0;94m"
RESET="\033[0m"


class Live_experiment:
    def __init__(self):
        
        # Basic preparation
        self.fps = rospy.get_param("/image/fps", "30")
        self.init=True
        self.round=0
        self.username=''
        self.folder_path='/home/karray/Desktop/catkin_ws_PANG_M/src/Live_Experiment/data'      # to be re-defined after moving !!!

        # Initial conditions
        self.state = ""
        rospy.Subscriber("/state_manager/state", String, self.setState)
        self.Idle_init=True
        self.Record_init=True
        self.Inference_init=True
        self.LLM_init=True
        self.LLM_done=False
        rospy.Subscriber("/video_builder/LLM_done", Bool, self.is_LLM_done)

        # Get 4 time points
        self.AVSR_start_time=0
        self.AVSR_end_time=0
        self.LLM_first_time=0
        self.LLM_last_time=0
        rospy.Subscriber("/video_builder/AVSR_start", String, self.get_AVSR_start_time)
        rospy.Subscriber("/video_builder/AVSR_end", String, self.get_AVSR_end_time)
        rospy.Subscriber("/video_builder/LLM_first", String, self.get_LLM_first_time)    
        rospy.Subscriber("/video_builder/LLM_last", String, self.get_LLM_last_time)   

        # Get 2 results
        self.AVSR_result=''
        self.LLM_result=''
        rospy.Subscriber("/video_builder/result", String, self.get_AVSR_result)
        rospy.Subscriber("/video_builder/LLM", String, self.get_LLM_result)
        rospy.Subscriber("/video_builder/LLM_words", String, self.print_LLM_words)
        
        # Publisher to state_manager
        self.pub_record_done = rospy.Publisher("/video_builder/record_done", Bool, queue_size=1)
        self.talkSpeech =  rospy.ServiceProxy('/qt_robot/behavior/talkText', TalkText)
        rospy.wait_for_service('/qt_robot/behavior/talkText')
        self.model_pub = rospy.Publisher('/model/choice', String, queue_size=10)

        self.model_list = [0,1,2]
        self.get_model()
    # Other tool functions    
    def setState(self, msg):
        self.state = msg.data
    
    def is_LLM_done(self,msg):
        self.LLM_done=msg.data
    
    def get_AVSR_start_time(self,msg):
        self.AVSR_start_time=float(msg.data)

    def get_AVSR_end_time(self,msg):
        self.AVSR_end_time=float(msg.data)

    def get_LLM_first_time(self,msg):
        self.LLM_first_time=float(msg.data)
    
    def get_LLM_last_time(self,msg):
        self.LLM_last_time=float(msg.data)

    def get_AVSR_result(self,msg):
        self.AVSR_result=msg.data
        sys.stdout.write(BLUE)
        print('{}:'.format(self.username),end=' ')
        print(msg.data)
        sys.stdout.write(RESET)
    
    def get_LLM_result(self,msg):
        self.LLM_result=msg.data

    def print_LLM_words(self,msg):
        sys.stdout.write(GREEN)
        print(msg.data, end=" ", flush=True)
        sys.stdout.write(RESET)
    
    def get_model(self):
        self.model_id = random.choice(self.model_list)
        return str(self.model_list.remove(self.model_id))
    

    def conduct_survey(self,name):
        # Crea2te the main window
        print("started")
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        # Collect responses using dialogs
        comments = [name]
        comments.append(simpledialog.askinteger("Input", "It was easy to interact with the robot(1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        comments.append(simpledialog.askinteger("Input", "I would like to use the robot to learn about new concepts(1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        comments.append(simpledialog.askinteger("Input", "I would recommend interacting with the robot to others (especially children)(1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        comments.append(simpledialog.askinteger("Input", "The robot perfectly understood what I was saying ( in terms of transcription)(1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        comments.append(simpledialog.askinteger("Input", "The robot stopped recording at the perfect moment.(1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        comments.append(simpledialog.askstring("Input", "What did you like most about the interaction with the robot?", parent=root))
        comments.append(simpledialog.askstring("Input", "What did you like least about the interaction with the robot?", parent=root))
        comments.append(simpledialog.askstring("Input", "What are the features you think should be added to the robot? ", parent=root))

        # Ask a multiple-choice question

        # Save responses to a file
        with open("/home/karray/Desktop/catkin_ws_PANG_M/src/Live_Experiment/data/survey_results.txt", "a") as file:
            file.write(f"{comments}\n")

        # Thank you message
        root.destroy()

    def conduct_survey_topic(self, name, model):
        # Create the main window
        root = tk.Tk()
        root.withdraw()  # Hide the main window
        satisfaction = [name,model]
        # Ask a multiple-choice question
        satisfaction.append(simpledialog.askinteger("Input", "The answers provided by the robot are relevant to my questions (1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        satisfaction.append(simpledialog.askinteger("Input", "The answers provided by the robot are coherent (1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        satisfaction.append(simpledialog.askinteger("Input", "The answers provided by the robot seem accurate (1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        satisfaction.append(simpledialog.askinteger("Input", "The discussion with the robot was engaging (1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        satisfaction.append(simpledialog.askinteger("Input", "The response time was acceptable and did not break the flow of the conversation.(1 Completely Disagree to 5 Compltely agree)", parent=root, minvalue=1, maxvalue=5))
        # Save responses to a file
        
        df = pd.read_csv("/home/karray/Desktop/catkin_ws_PANG_M/src/Live_Experiment/data/survey.csv")
        df.loc[df.shape[0]+1] = satisfaction
        df.to_csv("/home/karray/Desktop/catkin_ws_PANG_M/src/Live_Experiment/data/survey.csv",index=False)
        # Thank you message
        if self.round != 9:
            messagebox.showinfo("Please Wait for the robot's signal to start the new conversation", "The robot is getting ready for next round! Wait for the robot's signal",parent = root)
        root.destroy()


    # The main call function
    def __call__(self):
        rate = rospy.Rate(self.fps)
        
        while not rospy.is_shutdown():
            self.model_pub.publish(str(self.model_id))
                 # Initialise experiment 
            if self.init==True:
                print('\n')
                print('Hello, my dear friend! Welcome to our experiment!')
                self.talkSpeech('Hello, my dear friend! Welcome to our experiment! Please enter your name')
                self.username=input('Please enter your name (could use a nickname if you want):')
                print('Hi! Dear {}~'.format(self.username))
                self.talkSpeech('Hi! Dear {}'.format(self.username))
                self.init=False

            # Interaction during different states
            if self.state == "Idle":
                if self.Idle_init==True:
                    if self.round==0:
                        print("\r")
                        print('This is Round 0 for testing, please click on the camera-live-window, and wait fot the signal from the robot to start chatting!')
                        self.talkSpeech('This is Round 0 for testing, please click on the camera-live-window, and wait fot the signal from the robot to start chatting!')
                    elif self.round==1:
                        print('\n')
                        print('Now the experiment officially starts, we will do 3 conversations on three different topics. You choose the topic. For each topic, we would have 3 turns.')
                        self.talkSpeech("Now the experiment officially starts, we will do 3 conversations on three different topics. You choose the topic. For each topic, we would have 3 rounds.")
                        print('You should ask a question about a concept or a topic that you choose. Press SPACE to start chatting!')
                        self.talkSpeech('You should ask a question about a concept or a topic that you choose! Press SPACE to start chatting!')
                    elif self.round == 4 or self.round == 7:
                        self.get_model()
                        self.model_pub.publish(str(self.model_id))
                        print("here1")
                        print("That was an interesting conversation, I hope you had as much fun as I did! Let us fill the following survey.")
                        self.talkSpeech("That was an interesting conversation, I hope you had as much fun as I did! Let us fill the following survey.")
                        self.conduct_survey_topic(self.username, str(self.model_id))
                        print('\n')
                        print('You should ask a question about a concept or a topic that you choose!')
                        self.talkSpeech('You should ask a question about a concept or a topic that you choose! Wait for the signal to start!')
                    else:
                        print('\n')
                        print('I am ready for another Round, press SPACE to start again!')
                        self.talkSpeech('press SPACE to further discuss this point! You can discuss my answer and ask for further clarifications!')
                    self.Idle_init=False  
            
            elif self.state == "Recording":
                if self.Record_init==True:
                    print('\r')
                    print('Round {} starts!'.format(self.round))
                    print('I am listening...')
                    self.Record_init=False

            elif self.state == "Inference":
                if self.Inference_init== True:
                    print('I am understanding...')
                    self.talkSpeech('I am understanding...')
                    print('\r')
                    self.Inference_init=False
            
            elif self.state == "LLM":
                if self.LLM_init==True:
                    print('\r')
                    print('\r')
                    self.LLM_init=False
                
                if self.LLM_done==True:

                    if self.round!=0:
                        # Save the experiment data in a .csv file

                        ## difine the file path
                        file_name=self.username+'.csv'
                        file_path=os.path.join(self.folder_path,file_name)

                        ## calculate 3 needed time
                        AVSR_time=round(self.AVSR_end_time-self.AVSR_start_time,3)
                        thinking_time=round(self.LLM_first_time-self.AVSR_end_time,3)
                        talking_time=round(self.LLM_last_time-self.LLM_first_time,3)

                        ## create the data of current round
                        contents=[self.round, AVSR_time, thinking_time, talking_time, self.AVSR_result, self.LLM_result,self.model_id]
                    
                        ## write file
                        with open(file_path,'a') as file:
                            csv_writer=csv.writer(file)
                        
                            ### write head labels
                            if self.round==1:
                                labels=['Round','AVSR_time','Thinking_time','Talking_time','AVSR_result','LLM_response',"LLM_id"]
                                csv_writer.writerow(labels)
                        
                            ### write data of current round
                            csv_writer.writerow(contents)
                    

                    # End the experiment after 10 rounds
                    if self.round==9:
                        self.talkSpeech("That was an interesting conversation, I hope you had as much fun as I did! Let us fill the following survey.")
                        self.conduct_survey_topic(self.username, str(self.model_id))
                        self.talkSpeech('Experiment is over! Let us evaluate it!')
                        self.conduct_survey(self.username)
                        print('\n')
                        print('Experiment is over!')
                        print('Thank you so much for your cooperation and patience! Have a nice day~')
                        self.talkSpeech('Experiment is over! Thank you so much for your cooperation and patience! Have a nice day!')
                        rospy.signal_shutdown('Experiment finished.')


                    # Update initial conidtions
                    self.Idle_init=True
                    self.Record_init=True
                    self.Inference_init=True
                    self.LLM_init=True
                    self.LLM_done=False
                    self.round+=1

                    # Update state to state_manager
                    self.pub_record_done.publish(True)
                    
                    
            rate.sleep()




# The main running
if __name__=='__main__':

    rospy.init_node('live_experiment')
    live_experiment=Live_experiment()       # __init__

    try:
        live_experiment()                   # __call__
    except rospy.ROSInterruptException:
        pass



