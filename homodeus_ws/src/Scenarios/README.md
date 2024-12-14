# Scenarios
Vous trouverez ici les lancements (*launch*) ROS permettant de lancer les motivations et le scénario Robot serveur.

Informations complémentaires
* Seul le lancement de la motivation « Accueillir Client » est implémentée. Les lancements des motivations « Prendre la commande » et « Chcercher la commande » restent à implémenter.
* Le lancement du scénario devrait appeler les lancements des motivations, ce qui n'est pas le cas. Ce lancement est autonome ou presque.
* Deux éléments doivent être lancés outre par les fichiers de lancement soient les perceptions « détection personne » et « détection objet » du au fait que ces scripts Python ne peuvent être appelés par un noeud ROS.
