# Multicouleur / changement de filament automatique — état et voie pour les setups équipés

> English version: [MULTICOLOR_STATUS.en.md](MULTICOLOR_STATUS.en.md)

## Où en est le projet aujourd'hui

Sur le setup de référence, les changements de couleur se font **en manuel**
(parquer la lane, reculer aux boutons physiques, activer la lane suivante —
voir le manuel des commandes). La raison est matérielle, pas firmware : notre
installation n'a pour l'instant **ni cutter automatique** ni **ré-embobineur
passif** (type Filamentalist). Sans ces deux briques, un changement
entièrement automatique doit gérer une pointe de filament fondue (champignon,
tirages à chaud) et du filament reculé qui pendouille — ce qui rendait nos
procédures de déchargement beaucoup plus complexes et fragiles qu'elles ne
devraient l'être.

**Si votre setup a un cutter au toolhead et des ré-embobineurs passifs, la
voie automatique est réaliste avec ce que ce repo fournit** — nous l'avons
prototypée de bout en bout et elle fonctionnait ; voici ce qu'on en a appris.

## L'architecture prototypée (et pourquoi elle n'est pas livrée en officiel)

Deux capteurs et **un seul nombre mesuré** suffisent :

- un **capteur de présence à l'entrée de l'extrudeur** (souvent intégré) ;
- un **détecteur de filament après le splitter** (chez nous : BTT SFS 2.0
  sur un Pico RP2040 flashé en MCU Klipper USB — présence + mouvement) ;
- la **distance mesurée entre ce détecteur et la sortie du splitter**.
  Cette distance est TOUTE la calibration.

### Déchargement (3 phases séquencées)

1. **Retract synchronisé** — l'extrudeur recule pendant que la lane du
   buffer tourne en `back` à sa vitesse *réellement mesurée* (mesurez-la :
   la nôtre tournait ~1,9× la valeur spec). C'est le point clé : un
   extrudeur qui tire seul contre tout le chemin patine ; avec le buffer qui
   suit activement, la pointe dégage le capteur d'entrée de façon fiable.
2. **Poursuite jusqu'à ce que le détecteur post-splitter ne voie plus le
   filament** — la pointe a quitté le tronc commun.
3. **Queue calibrée** — recul d'une longueur fixe = *distance mesurée
   détecteur→sortie du splitter + marge* (mesuré 200 mm chez nous, réglé
   220 mm), qui parque la pointe juste en dehors du splitter. La lane est
   libre pour la couleur suivante.

### Chargement

Poussée **pulsée** (petits jogs bornés ~100 ms, tick côté hôte) jusqu'à ce
que le capteur d'entrée extrudeur voie le filament, puis main à l'extrudeur.
Pulsée, parce qu'une poussée continue dépasse : le flambage aux interstices
non guidés du chemin est LE mode de défaillance de la *poussée* (différent du
patinage qu'on combat en *traction*).

### Les leçons qui survivent au démontage

- La sortie **mouvement** du détecteur se lit même quand sa logique runout
  est désactivée — c'est votre discriminateur glisse-vs-avance pendant tout
  mouvement assisté.
- Tout mouvement assisté doit être **borné** (deadline) et observable ; le
  jeu de commandes livré n'offre volontairement que des mouvements bornés
  (`LLL_JOG`, `LLL_RUN`).
- Une lane **parked** ne pousse jamais toute seule — parquer d'abord est le
  workflow sûr de manipulation, automatique ou manuel.

### Pourquoi on l'a retirée

Notre installation accumulait des **problèmes de patinage/friction** sur le
chemin filament (un support de capteur mal orienté ajoute une vraie traînée ;
les épisodes de patinage répétés mâchent le filament, qui ensuite accroche
partout — cercle vicieux). Et **sans cutter**, la mise en forme de la pointe
transformait chaque déchargement en procédure longue, chaude et fragile.
Plutôt que publier quelque chose qui ne marchait que les bons jours, on a
ramené la machine au workflow manuel simple et validé, et gardé la séquence
automatique hors du périmètre officiel.

Avec un **cutter** (pointe nette, zéro mise en forme) et des
**ré-embobineurs passifs** (le filament reculé se rembobine tout seul), les
phases 1–3 perdent presque tous leurs modes de défaillance. Les primitives
livrées sont les briques : rôles par lane, `LLL_RUN`/`LLL_JOG` bornés,
événements télémétrie avec `event_gcode`, identité `LLL_FILAMENT`, sonde
`LLL_DETECT_LOADED`.

Si vous le construisez sur votre setup équipé : mesurez votre distance
détecteur→splitter, gardez chaque mouvement borné, et surveillez le signal de
mouvement — c'est toute la méthode.
