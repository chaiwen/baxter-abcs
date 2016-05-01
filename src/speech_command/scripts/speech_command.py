#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr

# init node and publisher

rospy.init_node('speech_command')
pub = rospy.Publisher('speech_command', String, queue_size=10)

def listen_for_command():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
       print "\nGive the robot a letter block to pick up!"
       audio = recognizer.listen(source)

    audio_text = ""

    try:
        # we are using the default API key. to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        audio_text = recognizer.recognize_google(audio)
    except sr.UnknownValueError:
        pass
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

    return audio_text

def parse_command(command_text):
    block_letters = ""

    words = command_text.split()
    if "spell" in words or "Spell" in words:
        block_letters = words[-1]
    else:
        for word in words[1:]:
            if len(word) == 1:
                block_letters = word

    block_letters = block_letters.upper()

    return block_letters

def send_command(block_letters):
    rospy.loginfo(block_letters + "\n")
    pub.publish(block_letters)

if __name__ == '__main__': 
    continue_listening = True
    while (continue_listening):
        try:
            command_text = listen_for_command()
            if command_text.upper() == "NO" or command_text.upper() == "STOP":
                continue_listening = False
                break
            if command_text != "":
                print("\nThe command you gave was: \n" + command_text + "\n")
                block_letters = parse_command(command_text)
                send_command(block_letters)
        except rospy.ROSInterruptException:
            pass

