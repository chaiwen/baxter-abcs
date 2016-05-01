#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr

rospy.init_node('speech_command')
pub = rospy.Publisher('speech_command', String, queue_size=10)

#uncomment the following to use speech from audio file:

#from os import path
#audio_file = path.join(path.dirname(path.realpath(__file__)), "pick_up_a_take2.wav")

#recognizer = sr.Recognizer()
#with sr.AudioFile(audio_file) as source:
#    audio = recognizer.record(source)

recognizer = sr.Recognizer()
with sr.Microphone() as source:
   print "\nGive the robot a letter block to pick up!"
   audio = recognizer.listen(source)

audio_text = ""

try:
    # we are using the default API key. to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
    audio_text = recognizer.recognize_google(audio)
    print("\nThe command you gave was: \n" + audio_text + "\n")
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))

block_letters = ""

words = audio_text.split()
if "spell" in words or "Spell" in words:
    block_letters = words[-1]
else:
    for word in words[1:]:
        if len(word) == 1:
            block_letters = word

block_letters = block_letters.upper()

def send_command():
    #block_letters = "A"
    rospy.loginfo(block_letters)
    pub.publish(block_letters)

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass
