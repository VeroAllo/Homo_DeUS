import json
import vosk
import rospy
from std_msgs.msg import String
import openai
import pyaudio
import threading
import queue
import os
from gtts import gTTS
from homodeus_msgs.msg import HDResponse, HDDiscussionStarted, HDStatus

class AudioRosDiscuss:
    def __init__(self):
        self.audio_queue = queue.Queue()
        self.is_playing = threading.Event()
        self.stop_event = threading.Event()
        self.setup_audio()
        self.setup_ros()
        

        
        self.message_history = [
            {"role": "system", "content": """
            You are a helpful assistant and restaurant server. Your job is to take orders, answer questions about the menu, and provide recommendations. 
            You should be polite, friendly, and professional at all times. It is very important to ask the user to confirm his choice. Always respond in english. Here are some specific instructions:
            0. the restaurant is the Tiagoh Bistro.
            2. If the customer asks for recommendations, suggest one of the item in the menu.
            3. The menu only have 3 items Dr. Pepper, Coke, and Sprite
            4. Confirm the order before ending the conversation.
            5. Thank the customer and wish them a pleasant meal.
            """}
        ]
        openai.api_key = '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'  # Assurez-vous que l'API key est définie ici

    def setup_audio(self):
        self.vosk_model = vosk.Model('vosk-model-small-en-us-0.15')  # Spécifiez le chemin correct ici
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
        print(f"Message reçu sur le topic /Homodeus/Behaviour/Discuss/Request : {msg.fistMessage}")
        self.start_discussion(msg.fistMessage)

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
                self.message_history.append({"role": "user", "content": text})
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=self.message_history
                )
                response_text = response['choices'][0]['message']['content']
                self.message_history.append({"role": "assistant", "content": response_text})

                # Vérification de la confirmation de la commande dans la réponse de ChatGPT
                print(f"Réponse de l'agent : {response_text}")  # Debugging line
                if "confirm" in response_text.lower() or "your order of" in response_text.lower():
                    # Extraire l'item sélectionné
                    selected_item = self.extract_order_item(response_text)


                tts = gTTS(text=response_text, lang='en')
                tts.save("response.mp3")
                self.is_playing.set()
                os.system("mpg321 response.mp3")
                self.is_playing.clear()

                # Vérification de la fin de la conversation
                if "thank you" in response_text.lower() or "have a pleasant meal" in response_text.lower():
                    if selected_item:
                        print(f"Commande confirmée : {selected_item}")
                        response_msg = HDResponse()
                        response_msg.id.desire_id = 0
                        response_msg.result = selected_item
                        self.response_pub.publish(response_msg)
                    print("Fin de la conversation")
                    self.stop_event.set()
                    break

    def extract_order_item(self, response_text):
        # Extraire l'item de la commande à partir de la réponse de ChatGPT
        items = ["Dr. Pepper", "Coke", "Sprite"]
        for item in items:
            if item.lower() in response_text.lower():
                return item
        return None

if __name__ == "__main__":
    audio_ros_discuss = AudioRosDiscuss()
    rospy.spin()