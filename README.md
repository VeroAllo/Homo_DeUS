# Projet Homo DeUS
Preuve de conception de l'architecture HBBA[^1] sur un robot TIAGo

## Auteurs
|  Prénom et Nom   |    Alias   |
|------------------|----------|
| Alexandre Bernier | tiblond |
| - | PhilV3 |
| - | jeromegagne |
| - | TKeita07 |
| - | spaghettipainalail |
| - | VeroAllo |
| - | JuThe72 |

## Description

Homo DeUS est un projet étudiant réalisé par des étudiants en génie informatique et en génie rootique à l'Université de Sherbrooke. Ce projet se veut une reprise du précédent projet du même nom, [HomoDeUS](https://github.com/AlexCampanozzi/HomoDeUS) où les bases de l'architecture HBBA partent de [HBBA Lite](https://github.com/introlab/hbba_lite).

## Licence

Puisque ce projet part de celui d'HBBA Lite, la même licence sera utilisée soit GPL-3.0.
<!-- - Source code files: [GPLv3](LICENSE_SOURCE_CODE) -->
<!-- - Source code files: [BSD License](LICENSE_SOURCE_CODE) -->
<!-- - Source code files: [Apache](LICENSE_SOURCE_CODE) -->

## Structure du dépôt
- [Documentation/PrecodureMotivations](Documentation/PrecodureMotivations) contient les procédures pour lancer les motivations ou le scénario complet à partir de consoles.
- [homodeus_ws/src](homodeus_ws/src) contient l'architecture HBBA d'Homo DeUS y compris ces perceptions et comportements
- [CMakeLists.txt](CMakeLists.txt)
- [terminal.sh](terminal.sh) est fichier bash permettant de lancer la motivation « Accueillir client » avec automatise de recupérer l'adresse IP hôte, export les variables et lancer les noeuds dans les consoles

## Installation

Suivre les instructions pour l'installation de [ROS TIAGo 20.04](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)
1. ROS installation
2. ROS packages installation
3. Source-based installation

Packages ajoutés au *workspace* que vous venez de créer 
1. Créer un dosier homodeus dans le dossier src du *workspace*
2. Télécharger le contenu de [homodeus_ws/src](homodeus_ws/src) dans le dossier homodeus
3. Compléter l'installation en parcours les README.md des sous-dossiers 
  - [Dépendances additionnelles pour le comportement HD_Audio](https://github.com/VeroAllo/Homo_DeUS/tree/main/homodeus_ws/src/HD_audio#installation)
  - [Dépendances propres à hbba lite](https://github.com/VeroAllo/Homo_DeUS/tree/main/homodeus_ws/src/hbba_lite#dependencies)
  - [Copier les cartes (*maps*) dans un répertoire spécifique](https://github.com/VeroAllo/Homo_DeUS/tree/main/homodeus_ws/src/homodeus_common/maps#cartographie-2d-gmapping)
  - [Copier les mondes (*worlds*) dans un répertoire spécifique](https://github.com/VeroAllo/Homo_DeUS/tree/main/homodeus_ws/src/homodeus_common/worlds#mondes-gazebo)
  - [Correctif à apporter pour le comportement préhension](https://github.com/VeroAllo/Homo_DeUS/tree/main/homodeus_ws/src/homodeus_prehension#moveit-sur-le-tiago-ne-fonctionne-pas-directement)
4. Construire (*build*) le dossier homodeus
```bash
cd ~/tiago_public_ws/src/homodeus
catkin build $(expr `nproc` / 2) --this
```

## Utilisation
- Pour utiliser un module spécifique, référez-vous à leur README
- Pour utiliser une motivation ou le scénario du robot serveur (simulation ou bien avec le vrai robot) référez-vous à
  -  [Documentation/PrecodureMotivations](Documentation/PrecodureMotivations)
  -  [homodeus_ws/src/Scenarios/](homodeus_ws/src/Scenarios/)

## Remerciement
- [Professeur François Ferland](https://www.usherbrooke.ca/recherche/fr/specialistes/details/francois.ferland)
- Alex Campanozzi, étudiant de la première itération de HomoDeUS
- [IntRoLab 3IT](https://introlab.3it.usherbrooke.ca/index.php/Main_Page)

[^1]: https://introlab.3it.usherbrooke.ca/index.php/HBBA
