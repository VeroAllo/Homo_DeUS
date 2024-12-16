
# Préhension

## Description
Le module de préhension gère le déplacement du bras du robot.

## Architecture
Le système est divisé en deux parties :  
  - Détection d'objets  
  - Déplacement du bras du robot  

### Détection d'objets
Cette partie a pour objectif de trouver les informations (dimensions et positions) des objets dans le champ de vision du robot.
- Fichiers importants : `object_detector.cpp` et `segment_table.cpp`
- Inspiré du tutoriel "Cylinder Detector" par Jordi Pages ([Lien vers le tutoriel](https://wiki.ros.org/Robots/TIAGo/Tutorials/CylinderDetector))

### Déplacement du bras du robot
Cette partie sert à déplacer le bras du robot à un point donné.
- Fichiers importants : `homodeus_arm_interface_node.cpp` et `ArmInterface.cpp`
- Utilise MoveIt pour planifier les mouvements.

## Utilisation
### Lancer le module au complet
```bash
roslaunch homodeus_prehension prehension_with_detection.launch
```
### Lancer les nœuds individuellement
#### Monde
```bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper world:=3it_P2_Closer
```
#### Détection d'objets
```bash
roslaunch homodeus_prehension object_segmentation.launch
```
#### Déplacement du bras
```bash
rosrun homodeus_prehension arm_interface_node
```
### Autres nœuds utiles
#### Pseudo détection 
Pour sélectionner l'objet à prendre :
```bash
rosrun pseudo_detection pseudo_detection
```
#### Contrôleur des articulations
Pour contrôler un à un les joints du robot :
```bash
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller 
```

## Topics ROS
### Publications
- `/Homodeus/Behaviour/Take/Response` : Réponses du module de prise d'objet.
- `/Homodeus/Behaviour/Take/Status` : État du module de prise d'objet.
- `/Homodeus/Behaviour/Drop/Response` : Réponses du module de dépôt d'objet.
- `/Homodeus/Behaviour/Drop/Status` : État du module de dépôt d'objet.

### Souscriptions
- `/Homodeus/Behaviour/Take/Request` : Requêtes de prise d'objet.
- `/Homodeus/Behaviour/Drop/Request` : Requêtes de dépôt d'objet.

## Notes importantes
### MoveIt sur le TIAGo ne fonctionne pas directement
Il y a un problème avec MoveIt lorsqu'on veut utiliser cette librairie sur le robot.  
L'environnement de développement n'a pas exactement la même version que celle utilisée sur le robot, ce qui empêche tous les tests sur le robot.  
Pour corriger cela, il faut installer les bonnes versions dans un workspace et s'assurer de bien les utiliser.

Il faut s'assurer d'avoir le fichier `moveit.rosinstall` dans le workspace.  
Ce fichier contient les bonnes versions de MoveIt à installer.

Après l'installation, il faut récupérer les derniers commits contenant les versions exactes souhaitées.

### Procédure
#### Installer un nouvel espace de travail avec les bonnes versions de MoveIt
```bash
source /opt/ros/noetic/setup.bash
wstool init src
wstool merge -t src moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)
```
#### Récupérer exactement les bons commits pour avoir les bonnes versions
**MoveIt**  
```bash
cd ~/ws_moveit/src/moveit
git checkout b9a1e8e648b674e71e23c5f230467e98d70f24fa
```
**MoveIt_msgs**  
```bash
cd ~/ws_moveit/src/moveit_msgs
git checkout 3667e9dabc3a9b3da669743f968fe38c265b1953
```
