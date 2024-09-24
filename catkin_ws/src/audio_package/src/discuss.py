import pyaudio
import vosk
import json
import threading
import queue
import openai
from gtts import gTTS
import os

class AudioHandler:
    def __init__(self):
        self.audio_queue = queue.Queue()
        self.is_playing = threading.Event()
        self.stop_event = threading.Event()
        self.producer = AudioProducer(self.audio_queue, self.is_playing, self.stop_event)
        self.consumer = AudioConsumer(self.audio_queue, self.is_playing, self.stop_event)
        self.selected_item = None

    def start(self):
        self.producer.start()
        self.consumer.start()
        self.selected_item = None

    def get_selected_item(self):
        return self.selected_item
    
class AudioProducer(threading.Thread):
    def __init__(self, audio_queue, is_playing, stop_event):
        threading.Thread.__init__(self)
        self.audio_queue = audio_queue
        self.is_playing = is_playing
        self.stop_event = stop_event
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
        self.stream.start_stream()
        self.vosk_model = vosk.Model('catkin_ws/src/audio_package/src/vosk-model-small-en-us-0.15')
        self.vosk_recognizer = vosk.KaldiRecognizer(self.vosk_model, 16000)

    def run(self):
        while not self.stop_event.is_set():
            data = self.stream.read(4000, exception_on_overflow=False)
            if not self.is_playing.is_set() and self.vosk_recognizer.AcceptWaveform(data):
                result = json.loads(self.vosk_recognizer.Result())
                text = result['text']
                if text:
                    self.audio_queue.put(text)

class AudioConsumer(threading.Thread):
    def __init__(self, audio_queue, is_playing, stop_event):
        threading.Thread.__init__(self)
        self.audio_queue = audio_queue
        self.is_playing = is_playing
        self.stop_event = stop_event
        self.message_history = [
            {"role": "system", "content": """
            You are a helpful assistant and restaurant server. Your job is to take orders, answer questions about the menu, and provide recommendations. 
            You should be polite, friendly, and professional at all times. Here are some specific instructions:
            0. the restaurant is the Tiagoh Bistro.
            1. Greet the customer warmly when they start speaking.
            2. If the customer asks for recommendations, suggest one of the item in the menu.
            3. The menu only have 3 items Dr. Pepper, Coke, and Sprite
            4. Confirm the order before ending the conversation.
            5. Thank the customer and wish them a pleasant meal.
            """}
        ]
        openai.api_key = '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'
        self.selected_item = None

    def run(self):
        while not self.stop_event.is_set():
            text = self.audio_queue.get()
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
                    self.selected_item = self.extract_order_item(response_text)
                    if self.selected_item:
                        print(f"Commande confirmée : {self.selected_item}")
                        handler.get_selected_item(self.selected_item)

                tts = gTTS(text=response_text, lang='en')
                tts.save("response.mp3")
                self.is_playing.set()
                os.system("mpg321 response.mp3")
                self.is_playing.clear()

                # Vérification de la fin de la conversation
                if "thank you" in response_text.lower() or "have a pleasant meal" in response_text.lower():
                    print("Fin de la conversation")
                    if self.selected_item:
                        print(f"Item sélectionné : {self.selected_item}")
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
    handler = AudioHandler()
    handler.start()