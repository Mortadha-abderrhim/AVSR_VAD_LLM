#!/usr/bin/env python3

"""
@Author: David Roch, Lucas Represa, Daniel Tozadore
@Date: 10.06.2023
@Description : Evaluator Class
	Select the video to test, measure and save the results.
"""

import rospy
from std_msgs.msg import String
import os
import rospkg
import pandas as pd
from os.path import exists
import time
import sys
import unicodedata

sys.path.append(os.environ['VSR_DIR'])
from pipelines.metrics.measures import get_wer, get_cer


class DataAnalyzer:
    def __init__(self, res_file, LLM):
        self.res_file = res_file
        self.LLM = LLM
        # open the file
        
    def start_file(self):
        is_file = exists(self.res_file)

        self.init_columns = ["name", "noise", "preprocessing", "recording", "transfering", "video_building",
                                "Inference"]
        if self.LLM:
            self.init_columns.extend(
                ["LLM", "wer", "nb_word", "cer", "nb_character", "groundtruth", "prediction",
                    "wer_LLM", "nb_word_LLM", "cer_LLM", "nb_character_LLM", "task"])
        else:
            self.init_columns.extend(["wer", "nb_word", "cer", "nb_character", "groundtruth", "prediction"])
        if is_file:
            self.data_frame = pd.read_excel(self.res_file + '.xlsx')
        else:
            self.data_frame = pd.DataFrame(columns=self.init_columns)

    def write_data_row(self, data_row):
        # convert to pandas dataframe
        self.data_row = pd.DataFrame([data_row], columns=self.data_frame.columns)
        # save at each new row to avoid losing data if the program crash
        self.save_excel_file()

    def save_excel_file(self):
        self.data_frame = pd.concat((self.data_frame, self.data_row), ignore_index=True, axis=0)
        self.data_frame.to_excel(self.res_file + '.xlsx', index=False)

class Evaluator:
    def __init__(self):
        self.fps = rospy.get_param("/image/fps", "30")
        self.rate = rospy.Rate(self.fps * 4)  # Need to be faster for time measure accuracy

        self.video_folder = rospy.get_param("/evaluation/video_names", "")
        self.preprocessing_used = rospy.get_param("/evaluation/preprocessing", "mediapipe")
        self.save_folder = rospy.get_param("/evaluation/save_folder", "res_test.xlsx")
        self.LLM = rospy.get_param("/robot/LLM", "False")
        with_noise = rospy.get_param("/evaluation/with_noise", "False")
        video_path = rospkg.RosPack().get_path('visual_module')
        video_path += rospy.get_param("/evaluation/video_path", "/video_saved2/")

        evaluator_path = rospkg.RosPack().get_path('evaluator')

        # Get a list of all video paths
        self.eval_video = False
        self.videos = []
        self.video_names = []
        self.noise_types = []  # Store noise types
        self.noise_levels = []  # Store noise levels
        self.participants_names = []
        self.init_row = []

        file = open(evaluator_path + "/video_folder_names/" + self.video_folder, 'r')
        self.video_folder_names = file.readlines()

        # Get a list of all noise types and levels
        if with_noise:
            noise_types = ["noise_gauss", "train-en", "train-fr", "classroom", "video_noise"]
            noise_levels = ["1", "2"]
        else:  # If no noise, add empty string to noise types and levels
            noise_types = ["0"]
            noise_levels = [""]

        for i in range(len(self.video_folder_names)):
            for noise_type in noise_types:
                for noise_level in noise_levels:
                    if with_noise:
                        path = video_path + self.video_folder_names[i].rstrip("\n") + "/noise/" + noise_type + "/" + noise_level + "/"
                    else:
                        if self.video_folder.startswith("QT"): # change of path organization for QT dataset due to lazy coder...
                            path = video_path + self.video_folder_names[i].rstrip("\n") + "/"
                        else:
                            path = video_path + self.video_folder_names[i].rstrip("\n") + "/noise" + noise_type + "/"
                    for dir in os.listdir(path):
                        if dir == "Ambient_noise.mp4":
                            continue
                        self.videos.append(path + dir)
                        self.video_names.append(dir.replace(".mp4", '').replace("_", " ").upper())
                        self.noise_types.append(noise_type)  # Append noise type
                        if with_noise:
                            self.noise_levels.append(noise_type + "-" + noise_level)  # Append noise level
                        else:
                            self.noise_levels.append(noise_type)

                        self.participants_names.append(self.video_folder_names[i].rstrip("\n"))

        self.pub_video_path = rospy.Publisher('/evaluator/video_path', String, queue_size=1, latch=True)

        self.state = "Evaluation_idle"
        rospy.Subscriber("/state_manager/state", String, self.setState)

        self.state_changed = False

        self.dict = {"name": 0, "noise": 1, "time": 2}
        self.res_file = evaluator_path + "/results/" + self.save_folder

        # Prepare the results file
        self.data_analyzer = DataAnalyzer(self.res_file, self.LLM)
        self.data_analyzer.start_file()

        self.csv_data = []
        self.csv_data.append(self.participants_names.pop(0))
        self.csv_data.append(self.noise_levels.pop(0))
        self.csv_data.append(self.preprocessing_used)

        self.prediction = ""
        rospy.Subscriber("/video_builder/result", String, self.save_prediction)

        if self.LLM:
            self.task = ""
            rospy.Subscriber("/video_builder/LLM", String, self.save_LLM_task)

        self.time = None

    def save_prediction(self, msg):
        self.prediction = ""
        self.prediction = msg.data
        print("Prediction: ", msg.data)

    def save_LLM_task(self, msg):
        self.task = ""
        self.task = msg.data
        print("Task: ", self.task)

    def update_evaluation_results(self):
        if self.state_changed:
            self.state_changed = False
            if self.state == "Evaluation_idle":
                if self.time:
                    self.csv_data.append(time.time() - self.time)
                self.time = None
                groundtruth = self.video_names.pop(0)
                wer = get_wer(self.prediction, groundtruth)
                self.csv_data.append(wer)
                nb_word = len(groundtruth.split())
                self.csv_data.append(nb_word)
                cer = get_cer(self.prediction, groundtruth)
                self.csv_data.append(cer)
                nb_char = len(groundtruth)
                self.csv_data.append(nb_char)
                self.csv_data.append(groundtruth)
                self.csv_data.append(self.prediction)
                if self.LLM:
                    wer = get_wer(self.task, groundtruth)
                    self.csv_data.append(wer)
                    nb_word = len(groundtruth.split())
                    self.csv_data.append(nb_word)
                    cer = get_cer(self.task, groundtruth)
                    self.csv_data.append(cer)
                    nb_char = len(groundtruth)
                    self.csv_data.append(nb_char)
                    self.csv_data.append(self.task)

                # save the data
                self.data_analyzer.write_data_row(self.csv_data)

                self.csv_data = []
                if self.participants_names:
                    self.csv_data.append(self.participants_names.pop(0))
                    self.csv_data.append(self.noise_levels.pop(0))
                    self.csv_data.append(self.preprocessing_used)
            else:
                if self.time:
                    self.csv_data.append(time.time() - self.time)
                self.time = time.time()
            self.state_changed = False

    def setState(self, msg):
        if self.state != msg.data:
            self.state = msg.data
            print("State changed to: ", self.state)
            self.state_changed = True

    def __call__(self):
        while not rospy.is_shutdown():
            if self.state == "Evaluation_idle" and not self.eval_video:
                self.eval_video = True
                if len(self.videos):
                    rospy.loginfo("Evaluating video: " + self.videos[0])
                    self.pub_video_path.publish(self.videos.pop(0))
                else:
                    rospy.loginfo("Evaluation done.")
            elif self.state != "Evaluation_idle":
                self.eval_video = False

            self.update_evaluation_results()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('Evaluator')

    evaluator = Evaluator()
    try:
        evaluator()
    except rospy.ROSInterruptException:
        pass
