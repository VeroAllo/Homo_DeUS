## Comportement « approche_client »
Ce comportement se veut un déverminage du couplage entre le module de navigation et l'émission de messages provenant du module de vision. Ce comportement n'est pas à proprement parler d'un comportement (behavior) au sens HBBA.


Pour pouvoir l'utiliser, il faut préalablement construire (build) le module Navigation/NavSelector. Ensuite, il faut construire ce dossier. Ne reste que plus qu'à lancer le point launch.


*** Reste à faire : Brancher la vraie détection de faces (Actuellement, il est possible de donner la boite englobante (bounding box) de la personne ou de l'objet avec le script « pseudo_facededection »)