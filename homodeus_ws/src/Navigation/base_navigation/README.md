# Sous-module de navigation

## Description
Ici, nous retrouvons la perception « pose du robot » et le comportement « tourner » (turn_around) de la base mobile du robot TIAGo.

## Architecture
Le système est composé de 3 fichiers principaux :

### `perception_pose.py`
Implémente (et appel ceci-ci) une classe de perception du robot personnalisée utilisant la pose initiale, l'odométrie et le filtre de particules AMCL
* Souscrit sur les canaux (*topics*)
  * /amcl_pose
  * /initialpose
  * /mobile_base_controller/odom
* Publie sur le canal (*topic*)
  * /Homodeus/Perception/RobotPose
* Ce script lance un noeud ROS et s'arrête par un Ctrl + C

### `BaseRotate.py`
Implémente une classe de rotation de la base du robot asservie par un PID
* Souscrit sur les canaux (*topics*)
  * /Homodeus/Behaviour/TurnAround/Request
  * /Homodeus/Behaviour/TurnAround/Cancel
  * /Homodeus/Perception/RobotPose
* Publie sur les canaux (*topics*)
  * /Homodeus/Behaviour/TurnAround/Response
  * nav_vel
* Appel la classe PID, interne au fichier

### `main_baseRotate.py`
Instancie une objet de la classe `BaseRotate.py`
* Un noeud ROS de `perception_pose.py` doit être lancé avant ce script
* Ce script lance un noeud ROS et s'arrête par un Ctrl + C

## Prérequis
* ROS (Robot Operating System)
  * L'environnement de simulation du TIAGo
  * Ou le vrai TIAGo
* Messages ROS
  * geometry_msgs
  * homodeus_msgs
  * nav_msgs
  * std_msgs
* Python 3.x
* Bibliothèques Python
  * rospy
  * math
  * numpy

## Installation
1. Installer l'environnement de simulation du TIAGo pour avoir accès presque tous les prérequis, excepté homodeus_msgs
2. Cloner le dépôt dans votre *workspace* ROS
3. Construire la nouvelle partie du *workspace*
```bash
catkin build $(expr `nproc` / 2) --no-deps base_navigation
```

## Configuration
1. S'assurer que le noeud `perception_pose` est lancé avant le noeud `main_baseRotate`

## Utilisation[^1]
### Lancer le robot virtuel (Si vous êtes simulation)
```bash
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true
```

### Lancer la perception Pose du robot
```bash
rosrun base_navigation perception_pose.py
```

### Lancer le comportement de rotation de la base
```bash
rosrun base_navigation baseRotateInterface.py
```

### Ou vous pouvez lancer le *launch* ROS au lieu des 3 cmds ci-dessus
```bash
roslaunch base_navigation base_navigation.launch public_sim:=true world:=3it_cafe
```

## *Topics* ROS
### Publications
* /amcl_pose : Pose provenant filtre de particules (amcl)
* /initialpose : Pose initiale du robot
* /Homodeus/Behaviour/TurnAround/Request : Requête de tourner
* /Homodeus/Behaviour/TurnAround/Cancel : Arrêter de tourner
* /Homodeus/Perception/RobotPose : Combin. odom + amcl de la pose du robot
* /mobile_base_controller/odom : Pose provenant de l'odométrie (odom)
### Souscriptions
* /Homodeus/Perception/RobotPose : Pose provenant de l'odométrie (odom)
* /Homodeus/Behaviour/TurnAround/Response : Réponse de tourner
* nav_vel : Commande vitesse avec la priorité de celle donnée à navigation (move_base)

[^1]: Vous devez avoir *sourcer* votre *workspace*, celui de TIAGo et de ROS avant de lancer les commandes
