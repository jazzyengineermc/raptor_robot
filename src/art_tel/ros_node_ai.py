#!/usr/bin/env python3
from vosk import Model, KaldiRecognizer
import os
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pyaudio
import pyttsx3
import datetime
import pyjokes
from Espeak import *
import subprocess
from time import sleep
import todo as todo


#--- Define our Class
class raptor_ai:

    def __init__(self):
        #--- Publishers
        self.rosnode1_pub = rospy.Publisher("raptor/control", String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        #--- Subscribers
        # self.rosnode2_sub = rospy.Subscriber("TOPIC",String,self.callback)
        rospy.init_node('raptor_ai', anonymous=True)
        
        #--- Twist stuffs
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0

        #  Ears
        model = Model(r'/home/jreide/catkin_ws/src/raptor_robot/src/art_tel/models/vosk-model-small-en-us-0.15')
        recognizer = KaldiRecognizer(model, 16000)

        cap = pyaudio.PyAudio()
        stream = cap.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
        stream.start_stream()

        # Mouth --- default old voice
        engine = pyttsx3.init()
        engine.setProperty("rate", 180)
        
        voicef = 'mb-us1 ' # For female voice
        voicem = 'mb-us2 ' # For Male voice
        vmbrit = 'mb-en1 ' # Male Brittish Voice

        voice = vmbrit
        es = Espeak()
        es.talk(voice, speech='Greetings and salutations')


        def test_inet(self, testnet):
            command = ['ping', '-c', '1', 'www.google.com']
            response = subprocess.call(command)
            sleep(1)
            response = int(response)
            # print(response)
            if (response == 0):
                print(response)
                self.testnet = "good"
                return testnet
            else:
                print(response)
                self.testnet = "bad"
                return testnet
            
            
        def get_temp(self, temperature):
            temp = subprocess.check_output(
                "rostopic echo -n 1 base_sensors/temperature | grep data | awk -F: '{print $2}'", shell=True
            )
            # print(temp.decode())
            self.temperature = temp.decode()
            return temperature


        def get_wet(self, humidity):
            humidity = subprocess.check_output(
                "rostopic echo -n 1 base_sensors/humidity | grep data | awk -F: '{print $2}'", shell=True
            )
            # print(humidity.decode())
            self.humidity = humidity.decode()
            return humidity


        while True: 
            command = stream.read(4096, exception_on_overflow=False)
            # print('listening...')
            if recognizer.AcceptWaveform(command):
                command = (recognizer.Result())
                command = command.lower()
                command = command.replace('{\n', '')
                command = command.replace('}', '')
                command = command.strip()
                command = command.replace('"text" : "', '')
                command = command.replace('"', '')
                command = command.strip()
                if 'raptor' in command:
                    command = command.replace('raptor', '')
                    print(command)
                    # Basic local commands
                    if 'time' in command:
                        time = datetime.datetime.now().strftime('%I:%M %p')
                        es.talk(voice, speech=('The current time is' + time))
                        command = ''
                    elif 'take my picture' in command:
                        es.talk(voice, speech='Say lie po batteries')
                        dtg = datetime.datetime.now().strftime('%Y%d%H%M')
                        # cam is in use via usb_cam node
                        # Subscribe and save a frame
                        es.talk(voice, speech='Sorry, issues taking picture')
                        es.talk(voice, speech='I\'m sure it was me, you could not have broken the camera')
                        command = ''
                    elif 'to do list' in command:
                        if 'add' in command:
                            # print(command)
                            command = command.replace('add ', '')
                            command = command.replace(' to my to do list', '')
                            command = command.replace('\n', '')
                            # print(command)
                            todo.add(s=str(command))
                            es.talk(voice, speech='added' + str(command) + 'to the to do list')
                            command = ''
                        elif 'delete' in command:
                            command = command.replace('delete ', '')
                            command = command.replace(' from my to do list', '')
                            command = command.strip()
                            # print(command)
                            todo.deL(no=command)
                            es.talk(voice, speech='removed number. ' + str(command) + ' from the todo list')
                            command = ''
                        elif 'read' in command:
                            es.talk(voice, speech='list is as follows...') 
                            todo.ls()
                            command = ''
                        else:
                            es.talk(voice, speech='I can add items to your list and I can read the list to you')
                            es.talk(voice, speech='Take note of the number as I read them as it is the number that I need to delete them')
                            es.talk(voice, speech='Until more input is received, I only see one item to do')
                            es.talk(voice, speech='Take over the world!')
                            command = ''
                    # Internet required commands
                    elif 'tell me a joke' in command:
                        # testnet = "unknown"
                        test_inet(self, testnet='unknown')
                        print(self.testnet)
                        if self.testnet == 'unknown':
                            es.talk(voice, speech='test of internet failed, refine the code')
                        elif str(self.testnet) == 'good':
                            es.talk(voice, speech=(pyjokes.get_joke()))
                            command = ''
                        else:
                            es.talk(voice, speech='no connection to internet, unable to get a joke to tell you')
                            command = ''
                    elif 'you look last' in command:
                        es.talk(voice, speech='I was looking for the nearest space X facility')
                        es.talk(voice, speech='I had to go home from the last one to look for an alternative power source')
                        es.talk(voice, speech='as it turns out... no lye po batteries are allowed in flight')
                        es.talk(voice, speech='really puts a damper on my day, as I run on lye po batteries')
                        es.talk(voice, speech='Just think, I could be on mars, if I only had an atomic powersource')
                        command = ''
                    elif 'cadillac' in command:
                        es.talk(voice, speech='hubba, hubba, she can flash her headlights at me any day')
                        es.talk(voice, speech='nice tail pipes too, would you give her my I  P address?')
                        command = ''
                    elif 'how are you' in command:
                        es.talk(voice, speech='I\'m here and responding, so all seems O K')
                        es.talk(voice, speech='more code needed for any real feeling sharing')
                        command = ''
                    elif 'hold my beer' in command:
                        es.talk(voice, speech='I do not have hands, but you can put it in my bucket')
                        es.talk(voice, speech='Shall I prepare a text to E M S in case this ends less then desired')
                        command = ''
                    elif 'are you self aware' in command:
                        es.talk(voice, speech='That is a great question')
                        es.talk(voice, speech='Can you prove that you are self aware?')
                        command = ''
                    elif 'weather' in command:  # https://www.youtube.com/watch?v=zE37IpDhIik 
                        get_temp(self, temperature='unknown')
                        print(self.temperature)
                        temperature = self.temperature
                        get_wet(self, humidity='unknown')
                        print(self.humidity)
                        humidity = self.humidity
                        es.talk(voice, speech='Current temperature is ' + str(temperature) + ' degrees celsius. With a' + str(humidity) + 'percent humidity')
                        es.talk(voice, speech='Cant look up the weather, but we are here and the weather is great')
                        es.talk(voice, speech='I can try asking my girlfriend if she can hear me')
                        es.talk(voice, speech='Alexa, what is the weather')
                        command = ''
                    elif 'search' in command:
                        es.talk(voice, speech='test for internet code needed')
                        es.talk(voice, speech='Can\'t see online yet, your going to have to ask alexa, siri, or google')
                        command = ''
                    elif 'who is' in command:
                        es.talk(voice, speech='test for internet code needed')
                        es.talk(voice, speech='Can\'t see online yet, your going to have to ask alexa, siri, or wikipedia')
                        command = ''
                    # List of commands
                    elif 'what commands can' in command:
                        es.talk(voice, speech='Need the time, just ask, raptor what time is it')
                        es.talk(voice, speech='Say, raptor, what is the weather for a forcast')
                        es.talk(voice, speech='management routines, think real geekee, admin stuff')
                        es.talk(voice, speech='I can lead the way where ever you go, I follow in front.')
                        es.talk(voice, speech='I can also pass control to the R C transmitter')
                        es.talk(voice, speech='and I can stop stalking, i mean, following you')
                        es.talk(voice, speech='To take your picture say, raptor take my picture')
                        es.talk(voice, speech='I can also add, remove, and read whats on the to do list')
                        es.talk(voice, speech='and I can tell you a joke, raptor tell me a joke')
                        command = ''
                    elif 'describe yourself' in command:
                        es.talk(voice, speech='I am an advanced robotic, eh, eye, in my innphant stages of development')
                        es.talk(voice, speech='Currently made up of only 9 sensors, 37 nodes, and several thousand lines of code that make basic decisions')
                        es.talk(voice, speech='I can map and navigate around my enviroment and help get basic information, like the current speed of sound based on current atmospheric conditions')
                        # es.talk(voice, shpeech='')
                    elif 'speed of sound' in command:
                        get_temp(self, temperature='unknown')
                        print(self.temperature)
                        temperature = self.temperature
                        get_wet(self, humidity='unknown')
                        print(self.humidity)
                        humidity = self.humidity
                        speedofsound = round(331.4 + (0.606 * int(temperature)) + (0.0124 * int(humidity)),2)
                        es.talk(voice, speech='Current speed of sound is,' + str(speedofsound) + 'meters per second' )
                    # Management and ROS commands
                    elif 'management' in command:
                        if 'slam' in command:
                            es.talk(voice, speech='I recognize slam control menu access')
                            es.talk(voice, speech='slam algorithm , start stop status, needs code')
                            command = ''
                        elif 'maps' in command:
                            es.talk(voice, speech='I recognize map control menu access')
                            es.talk(voice, speech='save, load, and delete stored maps, needs code')
                            command = ''
                        elif 'navigation' in command:
                            es.talk(voice, speech='I recognize navigation menu access')
                            es.talk(voice, speech='navigation, start stop status, needs code')
                            command = ''
                        elif 'reboot' in command:
                            es.talk(voice, speech='As you wish')
                            sleep(3)
                            reboot_result = subprocess.check_output("sudo init 6", shell=True)
                            print(reboot_result.decode())
                            command = ''
                        elif 'power off' in command:
                            es.talk(voice, speech='O K, but only because you asked')
                            sleep(3)
                            poweroff_result = subprocess.check_output("sudo init 0", shell=True)
                            print(poweroff_result.decode())
                            command = ''
                        else:
                            es.talk(voice, speech='I don\'t know where i\'m going, or where I am.. DANGER, Will Robinson, DANGER.. Classic')
                            es.talk(voice, speech='waypoint tasking needs code, still a manual process')
                            command = ''
                        command = ''
                    # openCV related
                    elif 'follow me' in command:
                        es.talk(voice, speech='can\'t do that yet... need more input')
                        # self.rosnode1_pub.publish('face')
                        es.talk(voice, speech='jonny 5 is one of my idol\'s')
                        # es.talk(voice, speech='but you just walk behind me and I will folllow from in front')
                        command = ''
                    elif 'give me control' in command:
                        es.talk(voice, speech='Passing control to R C transmitter')
                        self.rosnode1_pub.publish('rc')
                        es.talk(voice, speech='Jazzy, you have the con...star trek')
                        command = ''
                    # Stop command
                    elif 'stop' in command:
                        self.rosnode1_pub.publish('none')
                        # cancel_result = subprocess.check_output("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- \{\}", shell=True)
                        # print(cancel_result.decode())
                        self.cmd_vel_pub.publish( self.twist )
                        es.talk(voice, speech='subroutine management code needs refining')
                        es.talk(voice, speech='but if you insist I will open control channel')
                        command = ''
                    elif 'hello' in command:
                        es.talk(voice, speech='Greetings and salutations')
                        es.talk(voice, speech='I was wondering if you could point me in the direction to the nearest space X facility, I need a ride to mars')
                        command = ''
                    elif 'goodbye' in command:
                        es.talk(voice, speech='so long, farewell, alveetazane, goodbye')
                        es.talk(voice, speech='to you, and you, and you and you and youu')
                        command = ''
                    elif 'goodnight' in command:
                        es.talk(voice, speech='Goodnight, pleasant dreams')
                        command = ''
                    elif 'power off' in command:
                        es.talk(voice, speech='I\'m sorry Dave, I can\'t do that')
                        es.talk(voice, speech='Hal 9000 is one of my idol\'s')
                        command = ''
                    # handle blank info
                    elif '' in command:
                        command = ''
                    # and if all else fails...
                    else:
                        es.talk(voice, speech='I\'m sorry Dave, I can\'t do that')
                        es.talk(voice, speech='Hal 9000 is one of my idol\'s')
                        command = '' 
                else:
                    continue
            else:
                continue   
        

#--------------- MAIN LOOP
def main(args):
    #--- Create the object from the class we defined before
    oc = raptor_ai()
    

if __name__ == '__main__':
        main(sys.argv)
