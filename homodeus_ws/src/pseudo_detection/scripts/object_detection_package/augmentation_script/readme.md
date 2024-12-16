Pour utiliser le script, prendre des photos de l'objet désiré, d'assez près pour bien voir, 
mais d'assez loin pour que lors d'une rotation, l'objet n'excède pas le frame initial.

Les backgrounds désirés sont dans le fichier background, idéalement ne pas utiliser d'images haute résolution

Les objets annotés sont dans les fichiers annotated, séparé en image/labels

L'annotation peut être faite avec Roboflow pour les images de base au moins 20 recommandé par classe
Télécharger le dataset et utiliser un outil pour retirer les backgrounds. 
Remplacer l'image dans le dossier par l'image sans background, doit porter le même nom que son label. 
https://www.pixelcut.ai/t/background-remover garde la qualité de l'image et retire l'arrière plan

Une option est ajoutée pour annoter selon la transparence des images, mais moins précis que faire manuellement

Lancer le script.
Le script permet d'ajouter de la diversité dans les images en mettant une image en png sur un arrière plan varié
Cependant l'ensemble de données devrait contenir des images normales et beaucoup de variété dans les angles des objets

Très possible que les paths soient à changer
l'option de rotation est commentée