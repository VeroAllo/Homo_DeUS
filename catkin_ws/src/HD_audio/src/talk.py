from gtts import gTTS
import os 


class AudioTalk:
    def __init__(self) -> None:
        pass 

    def talk(self, initial_message):
        tts = gTTS(text=initial_message, lang='en')
        tts.save("response.mp3")
        os.system("mpg321 response.mp3")

# rostopic pub /tts/goal pal_interaction_msgs/TtsActionGoal "header:
# seq: 0
# stamp:
# secs: 0
# nsecs: 0
# frame_id: ''
# goal_id:
# stamp:
# secs: 0
# nsecs: 0
# id: ''
# goal:
# text:
# rawtext:
# text: 'Hello world'
