""" 
@Author: David Roch
@Date: 10.03.2022
@Description : This class write the fps on an average of nb_frames on the images
"""

import time
import cv2

class FPS():    
    def __init__(self, nb_frames = 10):
        self.tQueu = []
        self.tQueu.append(time.time())

        self.nb_frames = nb_frames

        self.start_time = time.time()


    def printFps(self, img):
        draw_img = img.copy()
        self.tQueu.append(time.time())
        if len(self.tQueu) > self.nb_frames :
            fps = self.nb_frames/(self.tQueu[-1]-self.tQueu.pop(0))
            cv2.putText(draw_img, f'FPS: {int(fps)}', (int(draw_img.shape[1]/2),int(draw_img.shape[0]/10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        return draw_img

    def printTime(self, img):
        draw_img = img.copy()
        stop_time = time.time()
        t = (stop_time - self.start_time)*1000 # ms
        cv2.putText(draw_img, f'{int(t)} ms', (img.shape[1] - 150,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        return draw_img