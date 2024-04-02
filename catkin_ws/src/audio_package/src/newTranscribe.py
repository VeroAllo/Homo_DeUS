import pyaudio
import vosk
import json
import threading
import queue

import asyncio
from rasa.core.agent import Agent

from gtts import gTTS
import os

from rasa.core.channels.channel import UserMessage
from rasa.shared.core.events import Event
from rasa.core.processor import MessageProcessor

class AudioProducer(threading.Thread):
    def __init__(self, audio_queue):
        threading.Thread.__init__(self)
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
        self.vosk_model = vosk.Model('/home/tiblond/Documents/Homo_DeUS/catkin_ws/src/audio_package/src/vosk-model-small-en-us-0.15')
        self.vosk_recognizer = vosk.KaldiRecognizer(self.vosk_model, 16000)

        # Chargez votre modèle RASA
        self.agent = Agent.load('/home/tiblond/Documents/Homo_DeUS/catkin_ws/src/audio_package/src/rasa_serveur/models/20240327-181016-senior-area.tar.gz')
    
    def clear_queue(self, q):
        try:
            while True:
                q.get_nowait()  # Non bloquant
        except queue.Empty:
            pass

    async def process_text(self, text):
        if text.strip():  # Vérifie si le texte n'est pas vide
            responses = await self.agent.handle_text(text)
            for response in responses:
                print(response)  # Ajoutez cette ligne pour déboguer les réponses
                if 'text' in response:  # Vérifie si le message est de type 'text'
                    print(response['text'])

                    # Convertir le texte en parole
                    tts = gTTS(text=response['text'], lang='en')
                    tts.save("response.mp3")
                    os.system("mpg321 response.mp3")

                    # Vider la queue
                    self.clear_queue(self.audio_queue)

    

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        print(f"Rasa agent: {self.agent}")  # Debug print statement
        while True:
            data = self.audio_queue.get()
            if self.vosk_recognizer.AcceptWaveform(data):
                result = json.loads(self.vosk_recognizer.Result())
                loop.run_until_complete(self.process_text(result['text'])) 
    

audio_queue = queue.Queue()
producer = AudioProducer(audio_queue)
consumer = AudioConsumer(audio_queue)
producer.start()
consumer.start()