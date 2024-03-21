Prendre les scripts Python exceptés les "*main*.py" dans les dossiers Modules/Navigation/NavSelector & Modules/Navigation/base_navigation et le fichier perception_pose.py dans le dossier Modules/Deplacement/approche_client/perception_pose.py

Pour utiliser main_baseRotate.py
 1. Lancer une simulation ayant le tiago_navigation.launch
 2. Lancer la perception « robot_pose » (rosrun base_navigation perception_pose.py)
 3. Lancer le comportement « base_rotate » (rosrun base_navigation perception_pose.py)
 4. Lancer un but dans le topic a cet effet 
    rostopic pub /homodeus/comportement/base_rotate/goal homodeus_msgs/Float32Stamped [tab]