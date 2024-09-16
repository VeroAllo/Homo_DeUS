Pour utiliser chacun des modules, il veuillez vous référer au README de chacun des modules.

## installation sur le robot
Après avoir installer les modules dans son workspace ROS et confirmer le fonctionnement individuel des modules. Il est possible de tester sur le robot.

1. Avoir un terminal par node d'ouvert (chaque terminal doit pouvoir faire une commande spécifique)
    - Pour chaque terminal faire les commandes suivantes:

` export ROS_MASTER_URI=http://10.68.0.1:113111` 

`export ROS_IP=10.<ip_ordi>`

`source ../devel/setup.bash`

2. À partir du root fichier homodeus où tous les modules sont placés
    1. lancer filter node avec   
    `rosrun hbba_state filter_node.py`
    2. lancer la state machine node    
    `rosrun hbba_state hbba_state_node`
    3. lancer le talk node et le navigation node (l'ordre n'est pas important entre ces deux nodes)  
    `rosrun NavigationSelector main_navSelector.py`   
    `rosrun HD_audio talkInterface.py` ou `python3 ./audio_package/src/talkInterface.py `
3. À partir du dossier root du package vision (homodeus/vision)
    1. lancer le node de détection d'objet  
    `python pseudo_detection/scripts/object_detection_package/detect.py`
