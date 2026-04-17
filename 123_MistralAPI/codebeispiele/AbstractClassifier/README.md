# AbstractClassifier – Abstracts strukturiert klassifizieren

Zeigt, wie aus einem CSV mit Abstract-Texten pro Zeile ein strukturierter
Datensatz mit _Forschungsgebiet_, _Methode_ und _verwendeten Programmiersprachen_
gemacht wird. Kombiniert:

+ `mistralai` – Chat-Completion mit `response_format: json_object`,
+ `pydantic` – Schema-Validierung der Mistral-Antwort,
+ `pandas` – CSV-Einlesen und tabellarische Weiterverarbeitung.

## Dateien

| Datei               | Zweck                                                              |
| :------------------ | :----------------------------------------------------------------- |
| `pyproject.toml`    | Abhängigkeiten (mistralai, pydantic, pandas, python-dotenv).       |
| `.env.example`      | Vorlage – nach `.env` kopieren und Key eintragen.                  |
| `.gitignore`        | `.env`, `.venv/`, erzeugte Ausgabedatei `classified.parquet`.      |
| `abstracts.csv`     | Fünf Beispiel-Abstracts aus dem Umfeld TU Bergakademie Freiberg.   |
| `classify.py`       | Extraktionspipeline – CSV → Mistral → Pydantic → Parquet.          |

## Ausführen

```bash
cd codebeispiele/AbstractClassifier
uv sync
cp .env.example .env   # und eigenen Key in .env eintragen
uv run classify.py
```

## Was der Code tut

1. `pd.read_csv("abstracts.csv")` – fünf Abstracts einlesen.
2. Für jeden Abstract wird `classify()` aufgerufen.
3. Der System-Prompt zwingt Mistral zu einer **JSON-Antwort** mit genau drei Feldern.
4. `AbstractInfo.model_validate_json(...)` parst die Antwort und validiert die
   Typen – Fehler (z. B. fehlendes Feld, falscher Typ) werden hier früh sichtbar.
5. Die extrahierten Felder werden als neue Spalten an den DataFrame angehängt.
6. Ausgabe als lesbare Tabelle auf der Konsole und als `classified.parquet` für
   Weiterverarbeitung (z. B. in Jupyter).

## Skalierung auf 200+ Abstracts

Die Pipeline funktioniert bis zu einem gewissen Umfang so wie sie ist. Bei
großen Korpora lohnt:

+ __Rate Limits__ beachten – `time.sleep` zwischen Calls oder `tenacity`-Retry
  mit exponential backoff.
+ __Batch-API__ – Mistral bietet eine asynchrone Batch-Verarbeitung (`/batch`),
  die für nicht-interaktive Jobs ca. 50 % günstiger ist.
+ __Caching__ – identische Prompts zwischenspeichern (z. B. `diskcache`),
  spart Geld bei wiederholten Durchläufen während der Prompt-Entwicklung.
