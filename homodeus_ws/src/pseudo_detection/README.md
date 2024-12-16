# Object Detection System

## Description
Ce système combine plusieurs méthodes de détection d'objets et de perception basées sur ROS (Robot Operating System) et PyTorch. Il permet de détecter des objets dans des images RGB et des données de profondeur à l'aide de YOLOv7 et de publier les résultats via des topics ROS.

## Architecture
Le système est composé de quatre modules principaux :

### 1. pseudo_detection.cpp
Implémente une détection d'objet interactive basée sur OpenCV :
- Permet de dessiner un rectangle sur l'image pour spécifier la zone de détection.
- Calcule les coordonnées 3D à partir des images de profondeur et les publie sur le topic `/Homodeus/Behaviour/Take/Request`.
- Utilise des transformations géométriques pour déterminer la position de l'objet dans le cadre de la carte globale.

### 2. pseudo_productDetection.py
Fournit une simulation pour détecter des produits spécifiques :
- Publie des objets préconfigurés (par exemple, "pomme", "orange") sur le topic `/Homodeus/Perception/Detect/Product`.
- Génère des messages ROS contenant les informations de position et de dimensions des objets détectés.

### 3. detect.py
Utilise YOLOv7 pour détecter des objets dans des flux d'images RGB et profondeur :
- Prend en charge l'inférence en temps réel sur GPU.
- Publie les résultats sur le topic `/Homodeus/Perception/Detect`.
- Implémente une pré- et post-traitement des images pour ajuster les boîtes de détection et les annotations.

### 4. augmentation.py
Effectue l'augmentation des données pour améliorer la robustesse du modèle :
- Génère des images annotées en ajoutant des rotations, des translations, et en changeant les arrière-plans.
- Calcule des étiquettes normalisées pour les nouvelles images.
- Produit un jeu de données augmenté dans les répertoires `./new_dataset/images` et `./new_dataset/labels`.

## Prérequis
- ROS (Robot Operating System)
- Python 3.x
- Bibliothèques Python :
  - rospy
  - torch
  - opencv-python
  - numpy
  - cv_bridge
  - Pillow
- YOLOv7 (modèle `yolov7-tiny.pt`)

## Installation
1. 1. Cloner le dépôt dans votre workspace ROS
2. Installer les dépendances Python :
   ```bash
   pip install torch torchvision numpy opencv-python Pillow
   ```
3. Construire l'espace de travail ROS :
   ```bash
   catkin_make
   ```

## Configuration
1. Télécharger le modèle YOLOv7 (`yolov7-tiny.pt`) ou tout autre modèle entrainé avec YOLOv7 et le placer dans le répertoire du projet.
2. Vérifier les topics ROS associés à la caméra RGB et à la profondeur :
   - `/xtion/rgb/image_raw`
   - `/xtion/depth_registered/image_raw`

## Utilisation
### Détection interactive
Lancer le module `pseudo_detection.cpp` :
```bash
rosrun <votre_package> pseudo_detection
```
### Simulation de détection de produits
Lancer le module `pseudo_productDetection.py` :
```bash
rosrun <votre_package> pseudo_productDetection.py
```
### Détection avec YOLOv7
Configurer les arguments et lancer le module `detect.py` :
```bash
python detect.py --weights yolov7-tiny.pt --device 0
```
### Augmentation des données
Générer des données augmentées avec le module `augmentation.py` :
```bash
python augmentation.py --number 200 --class_name 0 --create_annotations --add_rotation
```

## Topics ROS
### Publications
- `/Homodeus/Behaviour/Take/Request` : Résultats de la détection interactive.
- `/Homodeus/Perception/Detect/Product` : Objets détectés par simulation.
- `/Homodeus/Perception/Detect` : Objets détectés via YOLOv7.

### Souscriptions
- `/xtion/rgb/image_raw` : Images RGB.
- `/xtion/depth_registered/image_raw` : Images de profondeur.

## Notes
- Le système peut être personnalisé pour différents ensembles de données et applications.
- Vérifiez que tous les capteurs sont correctement configurés avant de lancer les modules.
