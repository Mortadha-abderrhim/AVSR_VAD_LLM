#!/usr/bin/env python3

from autobahn.twisted.component import Component, run
from twisted.internet.defer import inlineCallbacks
from autobahn.twisted.util import sleep
import os
import rospkg
import sys
import rospy
import smach
import pandas as pd
from std_msgs.msg import String
from TalkText import TalkText,TalkTextResponse
import time
class Robot:
    
    #@inlineCallbacks
    def __init__(self, session):
        
        self.connected = True
        self.session = session
        self.alive = True
        self.counter_excuted = 0
        self.counter_to_be_excuted = 0
         # initialize ROS node
        rospy.init_node('AlphaMini_Node', anonymous=True)
        # self.pubMsg = rospy.Publisher('/irecheck/button_name', String, queue_size=10)
        
        s = rospy.Service('/qt_robot/behavior/talkText', TalkText, lambda x: self.robotCommand(x))
        # initialize publishers/subscribers
        # rospy.Subscriber([topic_name],[topic_type],[callback_function_name])
        # rospy.Subscriber('dynamicomsg', String, self.dynamicoCallback)
        # rospy.Publisher([topic_name],[topic_type],[max_queue_size])

    # test for making the robot speak
    def talk(self, text, block = False, lang='en'):
        
        # check whether it is this call that makes it blocking
        if block:
            yield self.session.call("rie.dialogue.say", text=text)
        else:
            self.session.call("rie.dialogue.say_animated", text=text)



    def on_keyWords(self, frame):
        if ("certainty" in frame["data"]["body"] and
            frame["data"]["body"]["certainty"] > 0.45):
            
            # self.session.call("rie.dialogue.say", text= "Yes")
            self.talk(text= "Yes")

    def callback(self,result):
        self.counter_excuted = self.counter_excuted +1 
        print("excuted :",self.counter_excuted)

    # Print the data received
    def robotCommand(self,data):
        self.counter_to_be_excuted = self.counter_to_be_excuted + 1
        # checking the received data
        mine = self.counter_to_be_excuted
        # finishes the thread if receives an "end" as message
        if(data == 'end'):
            self.alive=False
        else:
            # talks the message its received
            # self.session.call("rie.dialogue.say", text=data.data)
            while self.counter_excuted != self.counter_to_be_excuted-1:
                time.sleep(0.05)
            # self.session.call("rie.dialogue.say_animatedHave", text=str(data.data), lang='de')
            self.session.call("rie.dialogue.say_animated", text=str(data.message)).addCallback(self.callback)
            # self.session.call("rie.dialogue.say", text=str(data.data))
        return TalkTextResponse(True)

            

    #@inlineCallbacks
    def off(self):
        self.session.leave()


@inlineCallbacks
def main(session, details):
    
    try: 
        myrobot = Robot(session)
    except Exception as e:
        print(e)
    
    # myrobot.talk("Hello, I am testing the class comunication here.", block=False)
    

    while(myrobot.alive):
        # the goal of this loop is to keep the node alive and receiving messages in the topic "robotCommand" still it is killed
        # pass
        yield sleep(0.01) 


    myrobot.off()



    
    return

    # The code below does not necessarily work (of course when I remove the "return" in the line above). Any ideas why?
    txt = ""
    while (txt != "bye"):
    
        yield session.call("rie.dialogue.say",
        text="What to say?")
    
        txt = input("Type here: ")
    
        yield session.call("rie.dialogue.say",
        text=txt)
    


    
    
    
    
    
    session.leave() # Sluit de verbinding met de robot

# Create wamp connection




wamp = Component(
transports=[{
    "url": "ws://wamp.robotsindeklas.nl",
    "serializers": ["msgpack"]
    }],
    realm="rie.6661600e29fca0a53366de98",
)
    
wamp.on_join(main)


if __name__ == "__main__":
    run([wamp])
    # print("CTL+C to exit!")
    # rospy.spin()
