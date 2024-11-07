# Module audio
Ce module implémente un agent conversationnel (RASA) basé sur une banque de connaissance (VOSK).


## Noeud talk


## Noeud discuss


<!-- ## Structure du dépôt (Transformations)
!Mettre a jour code selon commit sur branche audio_dev
HD_audio
  launch 
    hd_audio.launch [<arg name="tts"        default="hdTTS"/>,
      <arg name="lang"        default="fr"/>,
      in <node name="talk_interface" ...>, add args="--tts $(arg tts) --lang $(arg lang)"]
  python/HD_audio
    __init__.py [vide]
    hdTTS.py
    talk.py [from hdTTS import hdTTS -> from HD_audio.hdTTS import hdTTS]
  scripts
    discuss.py [ligne 45 -> new path, ligne 75 -> open("secret.txt","r").read()]
    oneDiscuss [from hdTTS import hdTTS -> from HD_audio.hdTTS import hdTTS,
      ligne 70 -> new path, ligne 45 -> open("secret.txt","r").read()]
    talkInterface [from talk import AudioTalk -> from HD_audio.talk import AudioTalk]
  CMakeLists.txt [catkin_install_python, only files in scritps/]
  README.md
  package.xml
  setup.py [package_dir={'':'src'} -> package_dir={'':'python'}]
  secret.txt [ou copier-coller directement dans code sans push]
utils 
  speech_recognition (provient branche audio_dev) 
     vosk-model-small-en-us-0.15
     vosk-model-small-fr-0.22
-->
