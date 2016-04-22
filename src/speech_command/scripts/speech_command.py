#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr

#from os import path
#audio_file = path.join(path.dirname(path.realpath(__file__)), "pick_up_a_take2.wav")

#recognizer = sr.Recognizer()
#with sr.AudioFile(audio_file) as source:
#    audio = recognizer.record(source)

recognizer = sr.Recognizer()
with sr.Microphone() as source:
   print "Give the robot a letter block to pick up!"
   audio = recognizer.listen(source)

try:
    # for testing purposes, we're just using the default API key
    # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
    # instead of `r.recognize_google(audio)`
    print("Google Speech Recognition thinks you said " + recognizer.recognize_google(audio))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))

def send_command():
    pub = rospy.Publisher('speech_command', String, queue_size=10)
    rospy.init_node('speech_command', anonymous=True)
    block_letters = "A"
    #block_letters = "CAB"
    rospy.loginfo(block_letters)
    pub.publish(block_letters)

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass
