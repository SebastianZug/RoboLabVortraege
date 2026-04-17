# ChatCompletion – Minimalprojekt mit uv

Zeigt den minimalen Aufbau eines Python-Projekts für Mistral-Aufrufe mit dem [uv](https://docs.astral.sh/uv/)-Paketmanager.

## Dateien

| Datei               | Zweck                                                                 |
| :------------------ | :-------------------------------------------------------------------- |
| `pyproject.toml`    | Projekt-Metadaten und Abhängigkeitsliste.                             |
| `.python-version`   | Legt die genutzte Python-Version fest (`uv` installiert sie ggf.).    |
| `.env.example`      | Vorlage – nach `.env` kopieren und eigenen Key eintragen.             |
| `.gitignore`        | `.venv/` und `.env` bleiben aus dem Repo draußen.                     |
| `chat.py`           | Das eigentliche Skript – ein Mistral-Aufruf, eine Ausgabe.            |
| `uv.lock`           | Wird von `uv` erzeugt – exakte Versionen für reproduzierbare Builds.  |

## Einmalig einrichten

```bash
# 1. uv installieren (falls noch nicht vorhanden)
#    Linux/macOS:
curl -LsSf https://astral.sh/uv/install.sh | sh
#    Windows PowerShell:
#    powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

# 2. In dieses Verzeichnis wechseln und Abhängigkeiten ziehen
cd codebeispiele/ChatCompletion
uv sync

# 3. API-Key hinterlegen
cp .env.example .env
# .env öffnen und MISTRAL_API_KEY=<dein-key> eintragen
```

## Ausführen

```bash
uv run chat.py
```

`uv run` aktiviert das Virtualenv automatisch – kein manuelles `activate` nötig, plattformunabhängig.

## Eine Abhängigkeit hinzufügen

```bash
uv add pydantic
```

`uv` aktualisiert `pyproject.toml` und `uv.lock` in einem Schritt.
