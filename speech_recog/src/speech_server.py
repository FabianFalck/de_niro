#!/usr/bin/env python
"""
Server code for the Speech Recognition service.
This service is responsible for recognising speech.

Request:
Response: string


Modified, but based on: # Source: https://github.com/Uberi/speech_recognition/blob/master/examples/microphone_recognition.py

pointer to grammar and keyword definition example:
https://github.com/Uberi/speech_recognition/blob/master/examples/special_recognizer_features.py

Author: Fabian Falck
Date: 05/18
"""

import speech_recognition as sr
import rospy
from speech_recog.srv import SpeechRecognition, SpeechRecognitionResponse
from configs import create_configs

import sys, os

root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_server import ROSServiceServer

sr.energy_threshold = 3000


class SpeechServer(ROSServiceServer):

    def __init__(self, _service_name, _service_type):
        super(SpeechServer, self).__init__(_service_name, _service_type)

        rospy.loginfo("Speech recognition service established")
        self.configs = create_configs()['configs_python_lib']

    def list_comma_separated(self, words):
        """
        Function to split string into comma separated words.
        :param words: list of string words
        :return: string of comma separated words
        """
        enumeration = ""
        for i, word in enumerate(words):

            if i == len(words) - 2:
                enumeration += word + " and "

            elif i == len(words) - 1:
                enumeration += word

            else:
                enumeration += word + ", "

        return enumeration

    def get_audio_input(self, recognizer):
        """
        Listens to a source to get an audio wave.
        :param recognizer:
        :return:
        """
        with sr.Microphone() as source:
            audio = recognizer.listen(source)
        return audio

    def output_message(self, output_string):
        """
        Prints the voice out
        :param output_string: string for DE NIRO to say
        :return:
        """
        # TODO change to nicos speech out
        # Must terminate right after the output voice so that the input can run
        rospy.loginfo(output_string)

    def audio_to_string(self, audio, configs, recognizer):
        """
        Transcribes the audio wave into text using an API specified in configs
        :param audio: Audio wave
        :param configs: API configurations to do speech-to-text
        :param recognizer:
        :return: String of recognised text
        """

        if configs['speech_recognition_API'] == 'sphinx':
            # keyword_sensitivities = [(keyword, configs['sphinx_keyword_sensitivity']) for keyword in configs['list_of_warehouse_objects']]
            # Before what worked: return recognizer.recognize_sphinx(audio)

            return recognizer.recognize_sphinx(audio, grammar= root_path + "/src/speech_recog/src/fetch.gram")

        elif configs['speech_recognition_API'] == 'google':
            return recognizer.recognize_google_cloud(audio, credentials_json=configs['GOOGLE_CLOUD_SPEECH_CREDENTIALS'])

    def calibrate_recognizer(self, recognizer):
        """
        Calibrates the API to the current background noise level.
        :param recognizer:
        :return:
        """
        with sr.Microphone() as source:
            recognizer.adjust_for_ambient_noise(source)  # Listen for 1 second to calibrate the energy threshold for ambient noise levels

    def update_recognition_state(self, recognized_string, list_of_warehouse_objects):
        """
        Extracts warehouse objects from recognised string.
        :param recognized_string: The string recognised by the API
        :param list_of_warehouse_objects: Objects in DE NIRO knowledge base as defined in configs
        :return: A list of recognised objects as strings
        """
        recognized_objects = []

        if "enumerate" in recognized_string.lower():
            recognized_objects = ["enumerate"]  # Indicates that the user wants an enumeration of the objects

        else:
            # Check which of the warehouse objects is in the string
            for warehouse_object in list_of_warehouse_objects:
                if warehouse_object in recognized_string:
                    recognized_objects.append(warehouse_object)

        return recognized_objects

    def response_message(self, recognized_objects, configs):
        """
        Gives a response to the user depending on the recognised objects.

        Can respond in 4 ways:
            1) enumerate again request
            2) 0 objects understood
            3) exactly 1 object understood
            4) more than one object understood

        :param recognized_objects: A list of recognised objects as strings
        :param configs: API configurations
        :return:
        """

        if "enumerate" in recognized_objects:
            self.output_message('I can fetch you one of the following things: ' +
                                self.list_comma_separated(configs['list_of_warehouse_objects']))

        elif len(recognized_objects) == 0:
            self.output_message("Sorry, I didn't recognise any fetchable object. Please try again.")

            self.output_message('I can fetch you one of the following things: ' +
                                self.list_comma_separated(configs['list_of_warehouse_objects']))

            self.count_misattempts += 1

        elif len(recognized_objects) == 1:
            # Potentially add confirmation from that the user actually wanted this object
            self.output_message("Ok, I am going to fetch you the following object: " + recognized_objects[0])
            rospy.loginfo(recognized_objects[0])

        elif len(recognized_objects) > 1:
            self.output_message("Don't be greedy, you can select only one object. Please try again.")
            self.count_misattempts += 1

    def callback(self, request=None):
        """
        Main function of the speech recognition service.
        :param request: std_msgs/Empty
        :return: String of object to pick up
        """

        recognizer = sr.Recognizer()
        self.calibrate_recognizer(recognizer)

        speech_API = self.configs['speech_recognition_API']

        self.count_misattempts = 0

        self.output_message("I am FEZZIK, and I can speak. Yes, I do. Sometimes. If I am charged.")
        self.output_message('I can fetch you one of the following things: ' +
                            self.list_comma_separated(self.configs['list_of_warehouse_objects']))

        while self.count_misattempts < self.configs['max_misattempts']:
            self.output_message("Please tell me the object you want to recognize!")
            # Get the audio from the microphone
            audio = self.get_audio_input(recognizer)

            # Convert the audio into a string using a speech recognition API
            recognized_string = ""  # Empty string due to try-except block required, since otherwise declared only inside

            try:
                recognized_string = self.audio_to_string(audio, self.configs, recognizer)

            except sr.UnknownValueError:
                self.output_message("Sorry, I didn't get your request. Please try again.")

            except sr.RequestError as e:
                rospy.logerr(e)
                self.output_message("Sorry, I cannot reach the API " + speech_API)

            self.output_message("Sentence understood: " + recognized_string)

            recognized_objects = self.update_recognition_state(recognized_string, self.configs['list_of_warehouse_objects'])

            self.response_message(recognized_objects, self.configs)

            # Recognized object found -> return from Service to Client
            if len(recognized_objects) == 1:
                return recognized_objects[0]

        # After configs['max_misattempts']:
        self.output_message("Thug life. I gotta charge my batteries, dude. I'm off.")
        return ""


if __name__ == "__main__":
    rospy.init_node("speech_recognition")
    speech_server = SpeechServer("speech_recognition_service", SpeechRecognition)
    rospy.spin()
