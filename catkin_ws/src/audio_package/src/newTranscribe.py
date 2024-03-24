import pyaudio
import vosk
import json
import threading
import queue

# Créez une file d'attente pour stocker les données audio
audio_queue = queue.Queue()

# Créez une fonction pour le thread du producteur qui capture le flux audio
def audio_producer(audio_queue):
    # Configurez pyaudio pour capturer le flux audio
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
    stream.start_stream()

    while True:
        # Capturez les données audio et ajoutez-les à la file d'attente
        data = stream.read(4000)
        if len(data) > 0:
            audio_queue.put(data)

# Créez une fonction pour le thread du consommateur qui effectue la transcription
def audio_consumer(audio_queue):
    # Configurez vosk pour la transcription
    vosk_model = vosk.Model("/home/tiblond/Documents/Homo_DeUS/catkin_ws/src/audio_package/src/vosk-model-small-en-us-0.15")
    vosk_recognizer = vosk.KaldiRecognizer(vosk_model, 16000)

    while True:
        # Obtenez les données audio de la file d'attente et effectuez la transcription
        data = audio_queue.get()
        if vosk_recognizer.AcceptWaveform(data):
            result = json.loads(vosk_recognizer.Result())
            print(result)

# Créez et démarrez les threads du producteur et du consommateur
producer_thread = threading.Thread(target=audio_producer, args=(audio_queue,))
consumer_thread = threading.Thread(target=audio_consumer, args=(audio_queue,))
producer_thread.start()
consumer_thread.start()