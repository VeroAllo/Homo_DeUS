# HD Audio System

## Description
HD Audio System est un système ROS (Robot Operating System) permettant la gestion de la synthèse vocale et des interactions vocales. Le système propose deux modes de synthèse vocale (TTS - Text-to-Speech) : gTTS (Google Text-to-Speech) et hdTTS (un service TTS personnalisé via ROS).

## Architecture
Le système est composé de 4 fichiers principaux :

### 1. hdTTS.py
Implémente une classe de synthèse vocale personnalisée utilisant ROS Action Server :
- Utilise le namespace `/tts`
- Gère les requêtes TTS de manière synchrone ou asynchrone
- Supporte différentes langues via les codes RFC 3006

### 2. talk.py
Classe intermédiaire qui abstrait l'utilisation des différents systèmes TTS :
- Supporte gTTS et hdTTS
- Gère la création et la lecture des fichiers audio
- Implémente une interface commune pour les deux systèmes TTS

### 3. talkInterface.py
Interface ROS pour la gestion des requêtes de synthèse vocale :
- Souscrit au topic `/Homodeus/Behaviour/Talk/Request`
- Publie sur les topics :
  - `/Homodeus/Behaviour/Talk/Response`
  - `/Homodeus/Behaviour/Talk/Status`
- Gère la file d'attente des messages et leur exécution

### 4. oneDiscuss.py
Implémente un système de dialogue interactif :
- Utilise OpenAI GPT pour le traitement du langage naturel
- Intègre la reconnaissance vocale via Vosk
- Gère des conversations en français et en anglais
- Maintient un contexte de conversation pour un restaurant virtuel

## Prérequis
- ROS (Robot Operating System)
- Python 3.x
- Bibliothèques Python :
  - rospy
  - openai
  - vosk
  - pyaudio
  - gTTS
  - actionlib

## Installation
1. Cloner le dépôt dans votre workspace ROS
2. Installer les dépendances :
```bash
pip install gtts vosk pyaudio openai
catkin_make
```

## Configuration
1. Créer un fichier secret.txt contenant votre clé API OpenAI
2. S'assurer que les modèles Vosk sont présents dans le dossier utils/:
- vosk-model-small-fr-0.22 pour le français
- vosk-model-small-en-us-0.15 pour l'anglais

## Utilisation
### Lancer le système TTS basique
```bash
python talkInterface.py --tts [gTTS|hdTTS]
catkin_make
```
### Lancer le système de dialogue
```bash
python oneDiscuss.py --tts [gTTS|hdTTS] --lang [fr|en]
catkin_make
```
## Topic ROS
### Publiction
- /Homodeus/Behaviour/Talk/Response : Réponses du système TTS
- /Homodeus/Behaviour/Talk/Status : État du système
- /Homodeus/Behaviour/Discuss/Response : Réponses du système de dialogue
### Souscriptions
- /Homodeus/Behaviour/Talk/Request : Requêtes de synthèse vocale
- /Homodeus/Behaviour/Discuss/Request : Requêtes de dialogue

## Notes
- Le système de dialogue est configuré pour simuler un serveur de restaurant
- Les conversations sont limitées à la commande de boissons
- Le système supporte uniquement une commande à la fois