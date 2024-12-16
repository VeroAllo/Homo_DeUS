#
Chaque dossier représente un paquet (*package*), vous trouverez un README par paquet.

## Paquets
- [hbba_lite](hbba_lite) contient une fourchette du [hbba_lite](https://github.com/introlab/hbba_lite) développé par introlab.
- [hbba_state](hbba_state) contient les differents états qui créent les désirs HBBA qui motivent les comportements
- [hd_audio](HD_audio) contient les comportements parler (*talk*) et discuter (*discuss*)
- [homodeus_common](homodeus_common) contient les éléments commons à tout Homo DeUS
- [homodeus_hbba_lite](homodeus_hbba_lite) contient les désirs et stratégies propres au projet Homo DeUS
- [homodeus_prehension](homodeus_prehension) contient le comportement prendre (*take*) et déposer (*drop*)
- [navigation](Navigation) contient la perception pose et le comportement aller à (*goto*)
- [pseudo_detection](pseudo_detection) contient les perceptions Détecter des personnes (*dectect_person*) et Détecter des objets spécifiques (*detect_product*)
- [scenarios](Scénarios) contient les roslaunch pour lancer une motivation ou le scénario robot serveur dans son entier
  
## Communication avec le robot
Quelques vérifications avant de lancer les modules
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

## Déploiement de l'architecture décisionnelle Homo DeUS 
Après avoir installer les modules dans son *workspace* ROS et confirmer le fonctionnement individuel des modules. Il est possible de tester sur le robot. Vous pouvez vous référer au [README à la racine, deuxième point](https://github.com/VeroAllo/Homo_DeUS/tree/main#utilisation)

Mais voici le déroulement manuel du déploiement sur le robot pour la motivation « Accueillir Client »
1. Avoir un terminal par noeud d'ouvert (chaque terminal doit pouvoir faire une commande spécifique)
    - Pour chaque terminal faire les commandes suivantes:
    ```bash
    export ROS_MASTER_URI=http://10.68.0.1:113111
    export ROS_IP=10.68.0.<ip_ordi>
    source ~/tiago_public_ws/devel/setup.bash  
    ```

1. À partir du root fichier homodeus où tous les modules sont placés
    1. lancer le noeud `filter_node`
    `rosrun hbba_state filter_node.py`
    1. lancer le noeud `state_machine_node`
    `rosrun hbba_state hbba_state_node`
    1. lancer le noeud `talk` et le noeud `navigation` (l'ordre n'est pas important entre ces deux noeuds)  
    `rosrun NavigationSelector main_navSelector.py`   
    `rosrun HD_audio talkInterface.py`
1. À partir du dossier root du package vision (homodeus/pseudo_detection)
    1. lancer les noeuds de détection de personnes
   ```bash
   cd ~/tiago_public_ws/src/homodeus/pseudo_detection/scripts/object_detection_package
   python -m detect --weights ./yolov7-tiny.pt --conf-thres 0.4
   ```
