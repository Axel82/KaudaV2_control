# KaudaV2 Control

Application de contrôle pour bras robotique KaudaV2 à 5 axes avec interface graphique PyQt5 et visualisation 3D en temps réel.

## Caractéristiques

### Interface Utilisateur
- **Interface à onglets verticaux** avec 4 sections :
  - **Connexion** : Gestion des ports série et connexion Arduino
  - **Cartésien** : Contrôle en coordonnées XYZ avec cinématique inverse
  - **Articulaire** : Contrôle direct des 5 articulations (J1-J5)
  - **Outils** : Contrôle du gripper
- **Thème sombre** avec contraste élevé pour une meilleure lisibilité
- **Console intégrée** pour les logs et retours série
- **Visualisation 3D** en temps réel du bras robotique

### Fonctionnalités de Contrôle
- **Cinématique inverse** : Conversion automatique des positions XYZ en angles articulaires
- **Contrôle articulaire direct** : Sliders pour chaque articulation (J1-J5)
- **Boutons Jog** : Incrémentation/décrémentation fine de chaque axe
- **Commandes prédéfinies** :
  - `GOTO` : Déplacement vers une position articulaire
  - `GOHOME` : Retour à la position d'origine (0,0,0,0,0)
  - `Teach Origin` : Définir la position actuelle comme origine
- **Contrôle du gripper** : Ouverture/fermeture via toggle switch

### Visualisation 3D
- **Modèle procédural** du bras robotique (pas de fichier STL requis)
- **Couleurs distinctes** pour chaque segment (base grise, bras orange, gripper bleu)
- **Gripper réaliste** avec deux mâchoires mobiles qui s'ouvrent/ferment
- **Animation fluide** entre les poses
- **Transformations cinématiques correctes** pour chaque segment
- **Repère global** à l'origine (axes X/Y/Z : Rouge/Vert/Bleu)
- **Repères locaux** affichés pour chaque articulation (J1-J4)
- **Marqueurs de joints** pour visualiser les points d'articulation
- **Grille de référence** avec couleur subtile pour l'orientation spatiale
- **Fond sombre** pour meilleur contraste visuel

### Communication Série
- **Scan automatique** des ports COM disponibles
- **Thread de lecture** dédié pour les réponses Arduino
- **Logs en temps réel** dans la console
- **Baudrate** : 115200 bps (configurable)

## Prérequis

- **Système d'exploitation** : Windows (testé sur Windows 10/11)
- **Python** : 3.10 ou supérieur
- **Environnement virtuel** : Recommandé

## Installation

### 1. Créer un environnement virtuel (recommandé)

```powershell
# Créer l'environnement
python -m venv .venv

# Activer l'environnement
.\.venv\Scripts\Activate.ps1
```

### 2. Installer les dépendances

```powershell
# Mettre pip à jour
python -m pip install --upgrade pip

# Installer les dépendances
pip install -r requirements.txt
```

### Dépendances principales
- `pyserial` : Communication série avec Arduino
- `numpy` : Calculs mathématiques et matriciels
- `PyQt5` : Interface graphique
- `pyqtgraph` : Visualisation 3D
- `PyOpenGL` : Rendu OpenGL pour la 3D

## Utilisation

### Lancer l'application

```powershell
# Avec l'environnement virtuel activé
python main.py

# Ou directement
.\.venv\Scripts\python.exe main.py
```

### Workflow typique

1. **Connexion**
   - Sélectionner le port COM de l'Arduino
   - Cliquer sur "Connect"
   - Vérifier la connexion dans la console

2. **Contrôle Cartésien**
   - Ajuster les sliders X, Y, Z pour la position désirée
   - Utiliser les boutons Jog pour des ajustements fins
   - Cliquer sur "Envoyer Position" pour transmettre à l'Arduino

3. **Contrôle Articulaire**
   - Ajuster directement les angles des articulations J1-J5
   - Cliquer sur "GOTO" pour envoyer la commande
   - Utiliser "GOHOME" pour retourner à l'origine

4. **Visualisation**
   - Observer le modèle 3D se mettre à jour en temps réel
   - Les repères locaux montrent l'orientation de chaque articulation

## Configuration

### Paramètres modifiables (dans `main.py`)

```python
# Longueurs des segments (mm)
L1 = 120.0   # épaule → coude
L2 = 100.0   # coude → poignet
L3 = 80.0    # poignet → outil
LBASE = 20.0 # hauteur de la base

# Limites des sliders
X_MIN, X_MAX = -250, 250
Y_MIN, Y_MAX = -250, 250
Z_MIN, Z_MAX = 0, 350
GRIP_MIN, GRIP_MAX = -90, 90
TOOL_MIN, TOOL_MAX = -180, 180

# Limites articulaires (degrés)
J1_MIN, J1_MAX = -165, 165   # base
J2_MIN, J2_MAX = -100, 120   # épaule
J3_MIN, J3_MAX = -60, 150    # coude
J4_MIN, J4_MAX = -175, 175   # poignet
J5_MIN, J5_MAX = -30, 130    # rotation gripper

# Baudrate série
BAUDRATE = 115200
```

## Protocole de Communication

### Format des commandes envoyées à l'Arduino

- **Position articulaire** : `A,<angle1>,<angle2>,<angle3>,<angle4>,<angle5>\n`
- **GOTO** : `GOTO;A1=<angle1>;A2=<angle2>;A3=<angle3>;A4=<angle4>;A5=<angle5>\n`
- **GOHOME** : `GOHOME\n`
- **Gripper ouvert** : `GRIPPER_OPEN\n`
- **Gripper fermé** : `GRIPPER_CLOSE\n`

## Développement

### Structure du projet

```
KaudaV2_Control_workspace/
├── main.py                 # Application principale
├── config.py               # Configuration (longueurs, limites, baudrate)
├── ui_components.py        # Composants UI réutilisables
├── serial_comm.py          # Communication série
├── theme.py                # Thème sombre de l'interface
├── visualization_3d.py     # Utilitaires de visualisation 3D
├── kinematics_5dof.py      # Cinématique directe et inverse
├── version.py              # Gestion de version
├── requirements.txt        # Dépendances Python
├── pyproject.toml          # Configuration du projet
├── kauda_icon.png          # Icône de l'application
└── README.md               # Ce fichier
```

### Outils de développement

```powershell
# Lancer les tests
python -m pytest

# Formater le code
python -m black main.py

# Vérifier le style
python -m flake8 main.py

# Trier les imports
python -m isort main.py
```

## Cinématique Inverse

L'application utilise une cinématique inverse 5-DOF basée sur :
- **Angle de base (θ1)** : Calculé par `atan2(y, x)`
- **Angles épaule/coude (θ2, θ3)** : Loi des cosinus dans le plan vertical
- **Angle poignet (θ4)** : Maintien de l'orientation de l'outil
- **Angle gripper (θ5)** : Contrôle indépendant

## Gripper 3D

Le gripper est modélisé avec deux mâchoires mobiles qui reflètent l'état du toggle switch :

### Caractéristiques
- **Deux mâchoires parallélépipédiques** : 20mm × 3mm × 8mm chacune
- **Attachement** : Extrémité du segment wrist (L3 = 80mm)
- **Animation** : Synchronisée avec le toggle "Pince" dans l'onglet "Outils"

### États
- **Fermé** : Espacement de 4mm entre les mâchoires (±2mm)
- **Ouvert** : Espacement de 15mm entre les mâchoires (±7.5mm)

### Commandes
- Toggle ON → `GRIPPER_OPEN\n` → Mâchoires s'écartent
- Toggle OFF → `GRIPPER_CLOSE\n` → Mâchoires se rapprochent

## Système de Repères 3D

L'application affiche plusieurs repères pour faciliter la compréhension de l'orientation spatiale :

### Repère Global (World Frame)

Affiché à l'origine de la grille avec le label "World" :

| Axe | Couleur | Direction | Longueur |
|-----|---------|-----------|----------|
| **X** | 🔴 Rouge | Horizontal (droite) | 40mm |
| **Y** | 🟢 Vert | Horizontal (avant) | 40mm |
| **Z** | 🔵 Bleu | Vertical (haut) | 40mm |

**Convention** : RGB = XYZ (standard en robotique et infographie 3D)

### Repères Locaux

Chaque articulation (J1 à J4) possède son propre repère local :
- **Longueur des axes** : 20mm
- **Couleurs** : Rouge (X), Vert (Y), Bleu (Z)
- **Labels** : "J1", "J2", "J3", "J4"
- **Utilité** : Visualiser l'orientation de chaque articulation

### Palette de Couleurs des Segments

| Segment | Couleur | Code RGB |
|---------|---------|----------|
| Base | Gris foncé | (0.3, 0.3, 0.35) |
| Shoulder | Orange vif | (0.85, 0.45, 0.15) |
| Forearm | Orange foncé | (0.75, 0.40, 0.12) |
| Wrist | Gris moyen | (0.4, 0.4, 0.45) |
| Gripper | Bleu | (0.2, 0.5, 0.8) |

## Dépannage

### L'application ne se lance pas
- Vérifier que toutes les dépendances sont installées : `pip install -r requirements.txt`
- Vérifier la version de Python : `python --version` (doit être ≥ 3.10)

### Pas de ports COM détectés
- Vérifier que l'Arduino est connecté
- Installer les drivers USB appropriés
- Cliquer sur "Refresh" pour rescanner les ports

### L'icône n'apparaît pas dans la barre des tâches
- Relancer l'application
- Vérifier que `kauda_icon.png` est présent dans le répertoire

### Erreurs de communication série
- Vérifier le baudrate (doit correspondre à l'Arduino : 115200)
- Vérifier que le port COM est le bon
- Fermer les autres applications utilisant le port série

## Changelog

### Version 1.0.1 (2025-12-06)

**Améliorations de la visualisation 3D**
- ✨ Ajout d'un gripper réaliste avec deux mâchoires mobiles
- ✨ Animation d'ouverture/fermeture synchronisée avec le toggle UI
- 🐛 Correction des transformations cinématiques pour tous les segments
- 🐛 Correction de l'attachement du gripper à l'extrémité de L3 (wrist)

**Améliorations de l'interface**
- ♻️ Refactorisation : création de la fonction `create_joint_slider()` dans `ui_components.py`
- 🎨 Simplification du code des sliders articulaires (réduction de ~15 lignes)

**Validation et documentation**
- ✅ Validation complète des équations de cinématique inverse
- ✅ Correction de la séquence des axes dans `kinematics_5dof.py` (z/y/y/y/x)
- 📝 Ajout de documentation détaillée sur le modèle cinématique

### Version 1.0.0 (2025-12-02)

**Version initiale**
- Interface graphique PyQt5 avec onglets verticaux
- Contrôle cartésien et articulaire
- Visualisation 3D procédurale
- Communication série avec Arduino
- Cinématique inverse 5-DOF

## Auteur

Axel Habeillon

## Licence

Ce projet est un utilitaire de contrôle pour le bras robotique KaudaV2.
