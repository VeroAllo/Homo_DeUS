#
Chaque dossier représente un paquet (*package*), vous trouverez un README par paquet.

## Paquets
- [hbba_lite](hbba_lite) contient une fourchette du [hbba_lite](https://github.com/introlab/hbba_lite) développé par introlab.
- [hbba_state](hbba_state) contient les differents états qui créent les désirs HBBA qui motivent les comportements
- [hd_audio](hd_audio) contient les comportements parler (*talk*) et discuter (*discuss*)
- [homodeus_common](homodeus_common) contient les éléments commons à tout Homo DeUS
- [homodeus_hbba_lite](homodeus_hbba_lite) contient les désirs et stratégies propres au projet Homo DeUS
- [homodeus_hbba_lite](homodeus_hbba_lite) contient le comportement prendre (*take*) et déposer (*drop*)
- [navigation](navigation) contient la perception pose et le comportement aller à (*goto*)
- [pseudo_detection](pseudo_detection) contient les perceptions Détecter des personnes (*dectect_person*) et Détecter des objets spécifiques (*detect_product*)
  
## Installation sur le robot
Après avoir installer les modules dans son workspace ROS et confirmer le fonctionnement individuel des modules. Il est possible de tester sur le robot.

0. Quelques vérifications avant de lancer les modules
    1. Établir la connexion avec le robot via un câble Ethernet
       - ping 10.68.0.1 à partir de la machine hôte
       - ping <ip_machine_hote> à partir du robot (ssh pal@10.68.0.1)
    1. Vérifier l'état du robot via l'onglet 'Diagnostics' du WebCommander
       - `http://control:8080` dans votre navigateur préféré
    1. Arrêter le mouvement de tête si besoin
       - stop < head_motion > dans l'onglet 'Startup' du WebCommander
    1. Ajuster la naviguation si besoin
       - Changer la [carte d'occupation](https://github.com/VeroAllo/Homo_DeUS/tree/main/catkin_ws/src/homodeus_common/maps#readme)
       - Localiser le robot à l'aide de Rviz
       - Nettoyer la carte `rosservice call /move_base/clear_costmaps "{}"`

2. Avoir un terminal par node d'ouvert (chaque terminal doit pouvoir faire une commande spécifique)
    - Pour chaque terminal faire les commandes suivantes:

    ` export ROS_MASTER_URI=http://10.68.0.1:113111` 

    `export ROS_IP=10.68.0.<ip_ordi>`

    `source ../devel/setup.bash`

2. À partir du root fichier homodeus où tous les modules sont placés
    1. lancer filter node avec   
    `rosrun hbba_lite_main filter_node.py`
    2. lancer la state machine node    
    `rosrun hbba_lite_main hbba_lite_main_node`
    3. lancer le talk node et le navigation node (l'ordre n'est pas important entre ces deux nodes)  
    `rosrun NavigationSelector main_navSelector.py`   
    `rosrun HD_audio talkInterface.py`
3. À partir du dossier root du package vision (homodeus/vision)
    1. lancer le node de détection d'objet  
    `python pseudo_detection/scripts/object_detection_package/detect.py`
