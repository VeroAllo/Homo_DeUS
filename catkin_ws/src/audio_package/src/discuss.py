import pyaudio
import vosk
import json
import threading
import queue
import numpy as np
import asyncio
import openai

from gtts import gTTS
import os

class AudioHandler:
    def __init__(self):
        self.audio_queue = queue.Queue()
        self.producer = AudioProducer(self.audio_queue)
        self.consumer = AudioConsumer(self.audio_queue)

    def start(self):
        
        self.consumer.start()
        self.producer.start()
        
class AudioProducer(threading.Thread):
    def __init__(self, audio_queue):
        threading.Thread.__init__(self)
        openai.api_key = 'your-api-key'
        self.agent = openai.ChatCompletion.create(model="gpt-3.5-turbo")
        self.audio_queue = audio_queue
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
        self.stream.start_stream()

    def run(self):
        while True:
            data = self.stream.read(4000)
            if len(data) > 0:
                self.audio_queue.put(data)

class AudioConsumer(threading.Thread):
    def __init__(self, audio_queue):
        threading.Thread.__init__(self)
        self.audio_queue = audio_queue
        self.initial_message_sent = threading.Event() 
        self.vosk_model = vosk.Model('/home/tiblond/Documents/Homo_DeUS/catkin_ws/src/audio_package/src/vosk-model-small-en-us-0.15')
        self.vosk_recognizer = vosk.KaldiRecognizer(self.vosk_model, 16000)

        # Chargez votre modèle RASA
        self.agent = Agent.load('/home/tiblond/Documents/Homo_DeUS/catkin_ws/src/audio_package/src/rasa_serveur/models/20240402-144236-all-caption.tar.gz')
    
    def clear_queue(self, q):
        try:
            while True:
                q.get_nowait()  # Non bloquant
        except queue.Empty:
            pass

    async def process_text(self, text):
        if text.strip():  # Vérifie si le texte n'est pas vide
            response = self.agent.complete(prompt=text, max_tokens=100)
            print(response)  # Ajoutez cette ligne pour déboguer les réponses
            if 'choices' in response:  # Vérifie si le message est de type 'text'
                print(response['choices'][0]['text'])
                # Convertir le texte en parole
                tts = gTTS(text=response['choices'][0]['text'], lang='en')
                tts.save("response.mp3")
                os.system("mpg321 response.mp3")
                # Vider la queue
                self.clear_queue(self.audio_queue)

    async def send_initial_message(self, initial_message):
        tts = gTTS(text=initial_message, lang='en')
        tts.save("response.mp3")
        os.system("mpg321 response.mp3")

        # Vider la queue
        self.clear_queue(self.audio_queue)

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        print(f"Rasa agent: {self.agent}")  # Debug print statement

        # Envoyer le message initial
        initial_message = "Welcome to Billy Bob Buger"  
        loop.run_until_complete(self.send_initial_message(initial_message))

        while True:
            data = self.audio_queue.get()
            if self.vosk_recognizer.AcceptWaveform(data):
                result = json.loads(self.vosk_recognizer.Result())
                loop.run_until_complete(self.process_text(result['text'])) 
    


if __name__ == "__main__":
    handler = AudioHandler()
    
    handler.start()