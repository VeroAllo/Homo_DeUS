from gtts import gTTS
import os 

import rospy
from hdTTS import hdTTS

class AudioTalk:
    def __init__(self, tts_type:str):
        self.tts_type = tts_type
        if self.tts_type == 'hdTTS':
            self.__tts = hdTTS()
        else:
            self.__sound_file:str = "response.mp3"

    def __tts_prepare(self, text, lang):
        if self.tts_type == 'hdTTS':
            self.__tts.set_goal(text=text, lang=lang)
        else:
            tts = gTTS(text=text, lang=lang)
            tts.save(self.__sound_file)

    def __talk(self):
        if self.tts_type == 'hdTTS':
            self.__tts.talk_blocking()
        else:
            os.system("mpg321 {self.__sound_file}".format(self=self))

    def __tts_talk(self, text, lang):
        self.__tts_prepare(text, lang)
        self.__talk()

    def talk(self, initial_message):
        self.__tts_talk(initial_message, 'en_US')

