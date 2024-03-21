import threading
import queue
from gtts import gTTS
from playsound import playsound
import requests
import json
import numpy as np
import torch
from datetime import datetime, timedelta
import os
import speech_recognition as sr
import whisper
from time import sleep
import base64


# Initialisation
q = queue.Queue()
source = sr.Microphone()
recorder = sr.Recognizer()
audio_model = whisper.load_model("tiny.en")

record_timeout = 10
phrase_timeout = 10
transcription = [] 


def record_callback(recognizer, audio_data):
    audio_bytes = audio_data.get_wav_data()
    audio_str = base64.b64encode(audio_bytes).decode()  # Convertir bytes en str
    q.put(audio_str)

# Thread 1 : Enregistrement et transcription de l'audio
def transcribe():
    phrase_time = None
    with source:
        recorder.adjust_for_ambient_noise(source)
    recorder.listen_in_background(source, record_callback, phrase_time_limit=record_timeout)
    print("Model loaded.\n")
    while True:
        try:
            now = datetime.utcnow()
            if not q.empty():
                phrase_complete = False
                if phrase_time and now - phrase_time > timedelta(seconds=phrase_timeout):
                    phrase_complete = True
                phrase_time = now
                # Convertir toutes les chaînes en bytes avant de les joindre
                audio_data = b''.join(item.encode('utf-8') if isinstance(item, str) else item for item in q.queue)
                q.queue.clear()
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
                result = audio_model.transcribe(audio_np, fp16=torch.cuda.is_available())
                text = result['text'].strip()
                if phrase_complete:
                    transcription.append(text)
                else:
                    transcription[-1] = text
                os.system('cls' if os.name=='nt' else 'clear')
                for line in transcription:
                    if isinstance(line, bytes):
                        continue
                    q.put(line)
                    
                print('', end='', flush=True)
                sleep(0.25)
        except KeyboardInterrupt:
            break

# Thread 2 : Interaction avec le chatbot RASA et conversion de la réponse en parole
def interact_with_rasa(q):
    while True:
        text = q.get()
        headers = {"Content-Type": "application/json"}
        response = requests.post("http://localhost:5005/webhooks/rest/webhook", headers=headers, data=json.dumps(text))
        response_data = json.loads(response.text)
        if "text" in response_data:  # Vérifier que la clé 'text' existe
            response_text = response_data["text"]
        else:
            response_text = ""
        tts = gTTS(text=response_text, lang='en')
        tts.save("response.mp3")
        playsound("response.mp3")

# Démarrage des threads
t1 = threading.Thread(target=transcribe)
t2 = threading.Thread(target=interact_with_rasa, args=(q,))
t1.start()
t2.start()