# EmbeddingsExplorer – Text-Embeddings eines lokalen Ollama-Servers erkunden

Notebook-Projekt zum Vortrag `90_Ollama`. Schickt kurze Texte an den lokalen
Ollama-Server, holt die zugehörigen Vektoren zurück und untersucht sie auf
drei Ebenen:

1. Kosinus-Ähnlichkeit paarweise – als Tabelle und als Heatmap.
2. Dimensionsreduktion per PCA auf 2D – zum optischen Vergleich der Cluster.
3. Kleine semantische Suche – Freitextanfrage findet den ähnlichsten Satz
   aus dem Korpus.

## Voraussetzung

Auf dem Rechner muss ein Ollama-Server laufen und ein Embedding-Modell
installiert sein:

```bash
# Ollama installieren (Linux)
curl -fsSL https://ollama.com/install.sh | sh

# Embedding-Modell laden (~1,2 GB, multilingual)
ollama pull bge-m3

# Server läuft danach automatisch auf http://localhost:11434
```

Kurzer Funktionstest ohne Python:

```bash
curl http://localhost:11434/v1/embeddings \
  -H "Content-Type: application/json" \
  -d '{"model": "bge-m3", "input": "Hallo Welt"}' | head -c 200
```

## Einrichtung

```bash
cd codebeispiele/EmbeddingsExplorer
uv sync
```

## Notebook starten

```bash
uv run jupyter lab explore_embeddings.ipynb
```

Alternativ öffnet VS Code die Datei direkt — als Kernel den `.venv` aus diesem
Verzeichnis auswählen.

## Struktur

| Datei                      | Zweck                                                    |
| :------------------------- | :------------------------------------------------------- |
| `pyproject.toml`           | Abhängigkeiten (openai, numpy, pandas, sklearn, …).      |
| `uv.lock`                  | Wird von `uv` erzeugt – exakte Versionen.                |
| `explore_embeddings.ipynb` | Das eigentliche Notebook, fünf aufeinander aufbauende Schritte. |

## Warum lokale Embeddings?

Embeddings über die Cloud-API anzufordern, bedeutet, jeden Satz in Klartext an
den Anbieter zu schicken. Für schützenswerte Texte (Prüfungsarbeiten,
Patientendokumentation, unveröffentlichte Manuskripte) ist das keine Option.
Ein lokal laufendes Modell wie `bge-m3` ist klein genug, um auf jedem halbwegs
aktuellen Laptop zu funktionieren, und liefert multilingual – also auch auf
deutschen Texten – bedeutungsbasierte Vektoren.
