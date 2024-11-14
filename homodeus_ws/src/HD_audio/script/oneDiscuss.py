#!/usr/bin/env python
import argparse
import json
import vosk
import openai
import pyaudio
import threading
import queue
import os
from gtts import gTTS

import rospy
from std_msgs.msg import String
from HD_audio.hdTTS import hdTTS
from homodeus_msgs.msg import HDResponse, HDDiscussionStarted, HDStatus

class AudioRosDiscuss:
    def __init__(self, tts_type:str, lang:str):
        self.tts_type = tts_type
        self.desire_id = 1
        self.lang = lang
        self.message_history = self.get_message_history(lang)


        self.audio_queue = queue.Queue()
        self.is_playing = threading.Event()
        self.stop_event = threading.Event()
        self.setup_audio()
        self.setup_ros()
        if self.tts_type == 'hdTTS':
            self.__tts = hdTTS()
        else:
            self.__sound_file:str = "/home/tiblond/Homo_DeUS/homodeus_ws/src/HD_audio/utils/response.mp3"
        
    def get_message_history(self, lang):
        if lang == 'fr':
            return [
                {"role": "system", "content": """
                Vous êtes un assistant serviable et serveur de restaurant. Votre travail consiste à prendre des commandes, répondre aux questions sur le menu et fournir des recommandations.
                Vous devez être poli, amical et professionnel en tout temps. Il est très important de demander à l'utilisateur de confirmer son choix. Répondez toujours en français. Voici quelques instructions spécifiques :
                0. Le restaurant est le Tiagoh Bistro.
                1. Si le client demande des recommandations, suggérez un des articles du menu.
                2. Le menu ne comporte que 3 articles : Pepsi, Coke et Canada dry. Si tu enumere le menu il ne faut pas mettre de nombre devant les items.
                3. Confirmez la commande avant de terminer la conversation.
                4. Remerciez le client.
                """}
            ]
        else:
            return [
                {"role": "system", "content": """
                You are a helpful assistant and restaurant server. Your job is to take orders, answer questions about the menu, and provide recommendations.
                You should be polite, friendly, and professional at all times. It is very important to ask the user to confirm his choice. Always respond in English. Here are some specific instructions:
                0. The restaurant is the Tiagoh Bistro.
                1. If the customer asks for recommendations, suggest one of the items on the menu.
                2. The menu only has 3 items: Pepsi, Coke, and Canada dry.
                3. Confirm the order before ending the conversation.
                4. Thank the customer.
                """}
            ]
    openai.api_key = ''  # Assurez-vous que l'API key est définie ici

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
            os.system("mpg321 /home/tiblond/Homo_DeUS/homodeus_ws/src/HD_audio/utils/response.mp3".format(self=self))

    def __tts_talk(self, text, lang):
        self.__tts_prepare(text, lang)
        self.is_playing.set()
        self.__talk()
        self.is_playing.clear()


    def setup_audio(self):
        base_path = os.path.dirname(os.path.abspath(__file__))
        if self.lang == 'fr':
            model_path = '/home/tiblond/Homo_DeUS/homodeus_ws/src/HD_audio/utils/vosk-model-small-fr-0.22'
        else:
            model_path = os.path.join(base_path, '/../utils/vosk-model-small-en-us-0.15')

        self.vosk_model = vosk.Model(model_path)
        self.recognizer = vosk.KaldiRecognizer(self.vosk_model, 16000)
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4096)
        self.stream.start_stream()

    def setup_ros(self):
        rospy.init_node('audio_ros_discuss')
        self.response_pub = rospy.Publisher('/Homodeus/Behaviour/Discuss/Response', HDResponse, queue_size=10)
        self.request_sub = rospy.Subscriber('/Homodeus/Behaviour/Discuss/Request', HDDiscussionStarted, self.handle_request)
        print("En attente de messages sur le topic /Homodeus/Behaviour/Discuss/Request")

    def handle_request(self, msg):
        print(f"Message reçu sur le topic /Homodeus/Behaviour/Discuss/Request : {msg.fistMessage.data}")
        self.desire_id = msg.id.desire_id
        self.start_discussion(msg.fistMessage.data)

    def start_discussion(self, initial_message):
        self.stop_event.clear()
        self.producer = threading.Thread(target=self.produce_audio)
        self.consumer = threading.Thread(target=self.consume_audio, args=(initial_message,))
        self.producer.start()
        self.consumer.start()

    def produce_audio(self):
        while not self.stop_event.is_set():
            data = self.stream.read(4000, exception_on_overflow=False)
            if not self.is_playing.is_set() and self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result['text']
                if text:
                    self.audio_queue.put(text)

    def consume_audio(self, initial_message):
        self.message_history.append({"role": "user", "content": initial_message})
        while not self.stop_event.is_set():
            try:
                text = self.audio_queue.get(timeout=1)
            except queue.Empty:
                continue

            if text:
                print(f"Message de l'utilisateur : {text}")
                self.message_history.append({"role": "user", "content": text})
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=self.message_history
                )
                response_text = response['choices'][0]['message']['content']
                self.message_history.append({"role": "assistant", "content": response_text})

                # Vérification de la confirmation de la commande dans la réponse de ChatGPT
                print(f"Réponse de l'agent : {response_text}")  # Debugging line
                if self.check_confirmation(response_text.lower(), args.lang):
                    # Extraire l'item sélectionné
                    selected_item = self.extract_order_item(response_text)

                # TODO, A valider si 'en_US' existe pour gTTS, car TtsAction est base sur RFC 3006
                if self.lang== 'fr' :
                    self.__tts_talk(response_text, 'fr-CA')
                else : 
                    self.__tts_talk(response_text, 'en-US')
                # Vérification de la fin de la conversation
                if self.check_thank_you(response_text.lower(), args.lang):
                    if selected_item:
                        print(f"Commande confirmée : {selected_item}")
                        response_msg = HDResponse()
                        response_msg.id.desire_id = 0
                        response_msg.message.data = "Commande :" + selected_item
                        self.response_pub.publish(response_msg)
                    print("Fin de la conversation")
                    self.stop_event.set()
                    break

    def check_confirmation(self, response_text, lang):
        if lang == 'fr':
            phrases = ["confirmer", "votre commande de"]
        else:
            phrases = ["confirm", "your order of"]

        for phrase in phrases:
            if phrase in response_text.lower():
                return True
        return False

    def check_thank_you(self, response_text, lang):
        if lang == 'fr':
            phrases = ["merci", "bon appétit"]
        else:
            phrases = ["thank you", "have a pleasant meal"]

        for phrase in phrases:
            if phrase in response_text.lower():
                return True
        return False

    def extract_order_item(self, response_text):
        # Extraire l'item de la commande à partir de la réponse de ChatGPT
        items = ["Pepsi", "Coke", "Soda"]
        for item in items:
            if item.lower() in response_text.lower():
                return item
        return None

def add_parser():
  parser = argparse.ArgumentParser(description='OneDiscuss')
  parser.add_argument(
    '--tts',
    help='Set tts type',
    default='gTTS',
    type=str,
    choices=['gTTS', 'hdTTS'],
  )
  parser.add_argument(
        '--lang',
        help='Set language',
        default='fr',
        type=str,
        choices=['en', 'fr'],
    )
  args, unknown = parser.parse_known_args()
  # unknown:=[__name, __log]
  return args

if __name__ == "__main__":
    args = add_parser()
    audio_ros_discuss = AudioRosDiscuss(args.tts, args.lang)
    rospy.spin()
