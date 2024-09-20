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
        self.producer = AudioProducer(self.audio_queue)
        self.consumer = AudioConsumer(self.audio_queue)

    def start(self):
        self.producer.start()
        self.consumer.start()

class AudioProducer(threading.Thread):
    def __init__(self, audio_queue):
        threading.Thread.__init__(self)
        self.audio_queue = audio_queue
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
        self.stream.start_stream()
        self.vosk_model = vosk.Model('catkin_ws/src/audio_package/src/vosk-model-small-en-us-0.15')
        self.vosk_recognizer = vosk.KaldiRecognizer(self.vosk_model, 16000)

    def run(self):
        while True:
            data = self.stream.read(4000, exception_on_overflow=False)
            if self.vosk_recognizer.AcceptWaveform(data):
                result = json.loads(self.vosk_recognizer.Result())
                text = result['text']
                if text:
                    self.audio_queue.put(text)

class AudioConsumer(threading.Thread):
    def __init__(self, audio_queue):
        threading.Thread.__init__(self)
        self.audio_queue = audio_queue
        openai.api_key = ''

    def run(self):
        while True:
            text = self.audio_queue.get()
            if text:
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": "You are a helpful assistant and restaurant server."},
                        {"role": "user", "content": text}
                    ]
                )
                response_text = response['choices'][0]['message']['content']
                tts = gTTS(text=response_text, lang='en')
                tts.save("response.mp3")
                os.system("mpg321 response.mp3")

if __name__ == "__main__":
    handler = AudioHandler()
    handler.start()