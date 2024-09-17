## Cartographie 2D gmapping
Vous trouverez les cartes 2D (occupancy grid) des mondes du même nom.<br>
Pour le moment, vous devez copier tous les sous-dossier de ce dossier dans le répertoire $HOME/.pal/tiago_maps/configurations/. <br>
* Celles finissent par « _physic » signifie que ces cartes là sont une carte d'un lieu réel.

### Cartographier avec le vrai robot (Séquence)
1. Connexion au TIAGo <br>
  export ROS_MASTER_URI=http://10.68.0.1:11311 <br>
  export ROS_IP=10.68.0.X
2. Reduire vitesse robot <br>
  Combinaison manette (centre gauche + 4 boutons droite)
3. Sauvegarder carte en cours ...
  rosservice call /pal_map_manager/save_map "directory: 'NAME_CURRENT_MAP'" <br>
  ou copier-coller le dossier ../config dans ../configurations <br>
    $HOME/.pal/tiago_maps/config <br>
    $HOME/.pal/tiago_maps/configurations <br>
4. Basculer en cartographie <br>
  rosservice call /pal_navigation_sm "input: 'MAP'" <br>
5. Nettoyer carte (NE SUPPRIMER PAS LA CARTE D'OCCUPATION) <br>
  rosservice call /move_base/clear_costmaps "{}" <br>
6. Naviguer dans le monde (closure-loop) <br>
  Manette <br>
7. Sauvegarder nouvelle carte <br>
    rosservice call /pal_map_manager/save_map "directory: 'NEW_MAP'" <br>
  machine connectee <br>
    rosrun map_server map_saver <br>
8. Basculer en localisation <br>
  rosservice call /pal_navigation_sm "input: 'LOC'" <br>
9. Retablir carte originale <br>
  rosservice call /pal_map_manager/change_map "input: 'NAME_CURRENT_MAP'" <br>
