{% set yaml_data = load('tutorial_01_falling_ball.yml') %}

On commence par définir les conventions de rotation :

{{show(yaml_data, ['rotations convention'])}}

Puis l'on donne des [constantes
environnementales](##constantes-environnementales) :

{{show(yaml_data, 'environmental constants')}}

Aucun modèle d'environnement (houle, vent...) n'est nécessaire pour cette
simulation :

{{show(yaml_data, 'environment models')}}

On définit la position du repère "body" par rapport au maillage :

{{show(yaml_data, 'bodies/0/position of body frame relative to mesh')}}

Les conditions initiales sont décrites comme suit :

{{show(yaml_data, 'bodies/0/initial position of body frame relative to NED'])}}
{{show(yaml_data, 'bodies/0/initial velocity of body frame relative to NED'])}}

Les données dynamiques comprennent la masse, la matrice d'inertie, les inerties ajoutées
et la position du centre d'inertie :

{{show(yaml_data, 'bodies/0/dynamics')}}

Seule la gravité agit sur le solide :

{{show(yaml_data, 'bodies/0/external forces')}}

En définitive, on obtient le fichier suivant :

{{show(yaml_data)}}

### Lancement de la simulation

La simulation peut s'exécuter comme suit :

{{sim('tutorial_01_falling_ball.yml', dt=0.01, tend=1, o='out.csv')}}

% On peut aussi la lancer silencieusement (c'est-à-dire sans afficher la ligne de commande).
% Pour cela, on passe la structure de données plutôt que le nom du fichier YAML :

{{sim(yaml_data, dt=0.01, tend=1)}}

On peut également changer l'instant initial (étant entendu que les conditions
initiales définies dans le fichier YAML s'appliquent à cet instant initial,
quel qu'il soit, et non pas à t = 0) :

{{sim('tutorial_01_falling_ball.yml', dt=0.01, tend=1, tstart=2, o='out.csv')}}

On peut choisir le solveur :

{{sim('tutorial_01_falling_ball.yml', dt=0.01, tend=1, s='rkck', o='out.csv')}}


La liste de toutes les options est disponible en exécutant :

{{sim('tutorial_01_falling_ball.yml', h=None)}}

### Résultats

Voici un tracé de l'élévation au cours du temps :

{% set data = csv('out.csv') %}
{% set plot = prepare_plot_data(data, x = 't', y = 'z(ball)', name='Résultat') %}

{% set g = cartesian_graph([plot], x='t (s)', y='Elévation (m)') %}

{{layout(g, title='Elévation au cours du temps')}}


# Pour les capa plots


