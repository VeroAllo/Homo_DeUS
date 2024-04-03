from gtts import gTTS
import os 


class AudioTalk:
    def __init__(self) -> None:
        pass 

    def talk(self, initial_message):
        tts = gTTS(text=initial_message, lang='en')
        tts.save("response.mp3")
        os.system("mpg321 response.mp3")