from gtts import gTTS
import os 

import rospy
from HD_audio.hdTTS import hdTTS

class AudioTalk:
    def __init__(self, tts_type:str):
        print("AudioTalk::__init__")
        self.tts_type = tts_type
        if self.tts_type == 'hdTTS':
            self.__tts = hdTTS()
        else:
            self.__sound_file:str = "/home/tiblond/Homo_DeUS/homodeus_ws/src/HD_audio/utils/response.mp3"
            self.__talk()

    def __tts_prepare(self, text, lang):
        print('prepare')
        if self.tts_type == 'hdTTS':
            self.__tts.set_goal(text=text, lang=lang)
        else:
            tts = gTTS(text=text, lang=lang)
            print("Enregistrement")
            tts.save(self.__sound_file)

    def __talk(self):
        if self.tts_type == 'hdTTS':
            self.__tts.talk_blocking()
        else:
            os.system("mpg321 /home/tiblond/Homo_DeUS/homodeus_ws/src/HD_audio/utils/response.mp3".format(self=self))

    def __tts_talk(self, text, lang):
        self.__tts_prepare(text, lang)
        self.__talk()

    def talk(self, initial_message):
        self.__tts_talk(initial_message, 'fr-CA')

