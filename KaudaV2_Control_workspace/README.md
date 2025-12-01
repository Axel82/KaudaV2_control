# KaudaV2_Control

Utilitaires de contrôle pour KaudaV2 (esquisse de projet).

Prérequis
- Windows PowerShell
- Python 3.10+ (virtualenv recommandé)

Installation (PowerShell)

```powershell
# Créer un environnement virtuel (si nécessaire)
python -m venv .venv

# Activer l'environnement
.\.venv\Scripts\Activate.ps1

# Mettre pip à jour
python -m pip install --upgrade pip

# Installer dépendances
pip install -r requirements.txt
```

Commandes utiles

```powershell
# Afficher la version du CLI
.\.venv\Scripts\python.exe main.py --version

# Lancer une action d'exemple (remplacer COM3 par le port série)
.\.venv\Scripts\python.exe main.py run --port COM3 --count 2

# Lancer les tests
.\.venv\Scripts\python.exe -m pytest
```

Notes
- `requirements.txt` liste les dépendances runtime et quelques outils de développement (pytest, flake8, black, isort).
- Le fichier `pyproject.toml` contient des métadonnées minimales et des réglages pour `black`/`isort`.
- Prochaine étape recommandée : créer le package `src/kaudav2_control` et ajouter des tests et la configuration VS Code.
