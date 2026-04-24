<!--
author:   Sebastian Zug

email:    sebastian.zug@informatik.tu-freiberg.de

version:  0.1.0

language: de

narrator: Deutsch Female

icon:     https://media.aubi-plus.com/institution/thumbnail/3f3de48-technische-universitaet-bergakademie-freiberg-logo.jpg

import:   https://raw.githubusercontent.com/LiaScript/CodeRunner/master/README.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/refs/heads/main/90_Ollama/presentation.md#1)

# TUBAF Bits&Bytes

Lokale Sprachmodelle betreiben – Ollama vom Laptop bis zum DGX
----------------------------------------------------------------

Donnerstag, 23.04.2026, 17 Uhr, RoboLab der TU Bergakademie Freiberg

---------------------

Prof. Dr. Sebastian Zug (Fakultät 1)

> _Vorige Woche haben wir gezeigt, wie man Mistral-Modelle über die Cloud-API anspricht. Diese Woche stellen wir die Frage: Was, wenn die Daten das Gebäude nicht verlassen dürfen – oder wenn wir schlicht kein Budget für API-Tokens haben? Mit **Ollama** lässt sich eine ganze Zoo-Sammlung offener Modelle (Mistral, Llama, Qwen, Gemma, DeepSeek) mit drei Befehlen lokal betreiben. Wir schauen auf den Laptop-Betrieb, vergleichen die Qualität ehrlich mit der Cloud und werfen zum Abschluss einen Blick auf die DGX-Infrastruktur der TUBAF, sobald ein 7B-Modell nicht mehr reicht._
>
> _Die Beispiele laufen gegen einen lokal installierten Ollama-Server — der LiaScript-Coderunner kann sie nicht ausführen. Zum Mitmachen bitte Ollama vorab installieren (siehe Abschnitt 2)._


## 1. Motivation

> Was hindert Sie bislang daran, die Mistral-Cloud-API für Ihre Daten zu nutzen?

> **Einschub:** Bis Juli können wir als TUBAF angehörige die Angebote der Academic Cloud nutzen. Dies schließt sehr viele unterschiedliche Modelle ein. 

https://docs.hpc.gwdg.de/services/ai-services/chat-ai/models/index.html

__Warum lokale Modelle statt Cloud-API?__

+ __Datenschutz__ – Prüfungsarbeiten, Patientendaten, Industrie-NDA-Texte: dürfen den Rechner nicht verlassen.
+ __Keine laufenden Kosten__ – Tokens werden nicht gezählt, nur Strom. Nach dem ersten Experimentieren gleicht das aus.
+ __Offline-fähig__ – funktioniert im Zug, im Labor ohne Netz, bei Anbieter-Ausfall.
+ __Volle Kontrolle__ – Modellversion bleibt fix, kein „over-night"-Update, das die Pipeline kaputt-macht.
+ __Experimentierfreude__ – mehrere Modelle parallel installiert, freies Umschalten, keine Abrechnung im Hintergrund.

__Die ehrliche Kehrseite__

+ Lokale Modelle sind __deutlich kleiner__ als die Cloud-Flaggschiffe. Auf einem Laptop läuft ein 7B- oder 14B-Modell; `mistral-large` hat 123B, GPT-4-Klasse noch mehr.
+ Qualität ist entsprechend schwächer, besonders bei Reasoning und langen Kontexten.
+ Erste Antwort-Token kommt langsamer — Cold Start, kein Lastausgleich im Rechenzentrum.

> __Die Leitfrage für heute:__ Wo reicht ein lokales 7B-Modell, und wo führt kein Weg an der Cloud oder an Hochschul-Infrastruktur (DGX) vorbei?

## 2. Ollama – Installation und erste Schritte

[Ollama](https://ollama.com) ist ein schlanker Wrapper um [llama.cpp](https://github.com/ggerganov/llama.cpp): Er kümmert sich um Download, Modell-Management und stellt eine HTTP-API bereit, die __OpenAI-kompatibel__ ist.

__Installation__

```bash
# Linux
curl -fsSL https://ollama.com/install.sh | sh

# macOS
brew install ollama

# Windows
# Installer von https://ollama.com/download
```

Nach der Installation läuft ein Hintergrunddienst auf `http://localhost:11434`.

__Ein Modell laden und im Terminal nutzen__

```bash
ollama pull mistral-small   # lädt ~14 GB herunter
ollama run mistral-small    # interaktive REPL
```

Beim ersten `run` startet eine Chat-Session im Terminal. `/bye` beendet sie. `ollama list` zeigt, was lokal installiert ist, `ollama rm <modell>` räumt auf.

> Die Modelle landen unter `~/.ollama/models/`. 14 GB pro Modell — eine externe SSD ist bei ausgiebigem Experimentieren keine schlechte Idee.

__Modellbibliothek__

Ollama's öffentliche Registry [ollama.com/library](https://ollama.com/library) enthält hunderte Varianten. Ein paar relevante:

| Modell               | Größe (GB)   | Stärke                                       |
| :------------------- | :----------- | :------------------------------------------- |
| `mistral-small`      | ~14          | Allrounder, deutschfreundlich                |
| `llama3.1:8b`        | ~5           | solide Baseline, sehr verbreitet             |
| `qwen2.5:14b`        | ~9           | starkes Reasoning für seine Größe            |
| `qwen2.5-coder:7b`   | ~4.5         | Code-Spezialist, Alternative zu Codestral    |
| `gemma3:12b`         | ~8           | Google, mehrsprachig, multimodal             |
| `deepseek-r1:8b`     | ~5           | Open-Reasoning-Modell                        |
| `bge-m3`             | ~1.2         | Embeddings (multilingual) – Grundlage für RAG |

> __Faustregel VRAM/RAM:__ Modellgröße in GB ≈ benötigter VRAM für flüssige Inferenz. Ein 7B-Modell (4–5 GB quantisiert) läuft auf einer 8-GB-Grafikkarte flott; ohne dedizierte GPU fällt Ollama auf CPU+RAM zurück – funktioniert, ist aber spürbar langsamer.

## 3. Die HTTP-API – derselbe Code wie bei Mistral

Der eigentliche Gewinn von Ollama steckt in der __OpenAI-kompatiblen REST-API__: exakt dieselbe Payload-Struktur wie bei Mistral, nur andere URL und kein API-Key nötig.

__Nacktes HTTP – wie im Mistral-Vortrag__

```bash
curl http://localhost:11434/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "mistral-small",
    "messages": [
      {"role": "user", "content": "Nenne drei Fakten zur TU Bergakademie Freiberg."}
    ]
  }'
```

__Der gleiche Aufruf in Python__

Wir können __buchstäblich denselben SDK-Code__ aus dem Mistral-Vortrag verwenden — entweder mit dem `openai`-SDK oder mit `mistralai`, beide unterstützen eine freie `base_url`:

```python
from openai import OpenAI

client = OpenAI(
    base_url="http://localhost:11434/v1",
    api_key="ollama",          # Dummy – Ollama prüft ihn nicht, Feld wird aber erwartet
)

resp = client.chat.completions.create(
    model="mistral-small",
    messages=[
        {"role": "user",
         "content": "Nenne drei Fakten zur TU Bergakademie Freiberg."}
    ],
)

print(resp.choices[0].message.content)
```

> Dieser Code-Block lässt sich **nicht** über den LiaScript-Coderunner ausführen – der läuft in einer Sandbox, die `localhost:11434` auf dem Rechner des Zuschauers nicht erreicht. Wir führen das live im Terminal vor. Das komplette Projekt liegt unter [codebeispiele/ChatCompletionLocal/](codebeispiele/ChatCompletionLocal/).

__Was sich im Vergleich zu Mistral ändert__

| Aspekt              | Mistral Cloud                  | Ollama lokal                          |
| :------------------ | :----------------------------- | :------------------------------------ |
| Endpoint            | `api.mistral.ai/v1`            | `localhost:11434/v1`                  |
| API-Key             | Pflicht (Bearer)               | Dummy-String, wird ignoriert          |
| Modellnamen         | `mistral-small-latest`         | `mistral-small` (ohne `-latest`)      |
| Abrechnung          | Tokens                         | Strom                                 |
| Rate Limits         | Ja                             | Nur was die GPU schafft               |
| Strukturierte JSON  | `response_format: json_object` | identisch unterstützt                 |
| Streaming           | unterstützt                    | unterstützt                           |
| Function Calling    | unterstützt                    | modellabhängig                        |

__Der Knackpunkt für die Lehre__ — weil die Protokolle identisch sind, können Studierende __eine Code-Basis__ pflegen und je nach Kontext zwischen lokal und Cloud umschalten. Typisches Muster: Entwicklung und Prompt-Tuning lokal (kostenfrei, iterativ), Produktiv-Run auf Cloud (Qualität).

## 4. Ein grafisches Frontend – Open-WebUI

Wer lieber klickt als tippt, installiert sich [Open-WebUI](https://openwebui.com) als lokale ChatGPT-ähnliche Oberfläche vor den Ollama-Server:

```bash
docker run -d -p 3000:8080 \
  --add-host=host.docker.internal:host-gateway \
  -v open-webui:/app/backend/data \
  --name open-webui --restart always \
  ghcr.io/open-webui/open-webui:main
```

Nach dem Start unter `http://localhost:3000` erreichbar. Open-WebUI erkennt den laufenden Ollama-Server automatisch, zeigt alle installierten Modelle, verwaltet Chat-Verläufe, unterstützt Bild-Upload für multimodale Modelle und bringt ein simples RAG-Modul mit (Dokumente hochladen → befragen).

> Für Lehrveranstaltungen oder kleine Arbeitsgruppen ist das __die__ Kombination: Ein Rechner mit GPU betreibt Ollama + Open-WebUI, ein Dutzend Nutzer:innen greifen über den Browser zu.

## 5. Strukturierte Ausgaben – das kennen wir schon

Wie im Mistral-Vortrag können wir JSON erzwingen. Ollama akzeptiert dafür genau das gleiche `response_format`-Argument:

```python
from openai import OpenAI

client = OpenAI(base_url="http://localhost:11434/v1", api_key="ollama")

text = "Die Vorlesung findet am 23.04.2026 um 17:00 Uhr im Raum MIB-1108 statt."

resp = client.chat.completions.create(
    model="mistral-small",
    temperature=0.0,
    response_format={"type": "json_object"},
    messages=[
        {"role": "system", "content":
            "Extrahiere Datum, Uhrzeit und Raum. Antworte ausschließlich als JSON "
            "mit den Schlüsseln 'date' (ISO-8601), 'time' (HH:MM), 'room'."},
        {"role": "user", "content": text},
    ],
)

import json
print(json.loads(resp.choices[0].message.content))
```

> Damit läuft __der AbstractClassifier aus dem Mistral-Vortrag__ – derselbe Pydantic-Validator, dieselbe pandas-Schleife – 1:1 lokal weiter. Nur Ausführungsgeschwindigkeit und Ausgabequalität unterscheiden sich.

## 6. Embeddings – von Text zu Vektoren

Bisher haben wir Text __hinein__ und Text __heraus__ geschickt. Für eine Vektor-Datenbank brauchen wir etwas anderes: eine Funktion, die einem Satz einen __Zahlenvektor__ zuordnet, so dass __ähnliche Sätze ähnliche Vektoren__ bekommen. Diese Funktion heißt _Embedding_.

### Was ist ein Vektor in diesem Kontext?

Ein Vektor ist schlicht eine Liste von Zahlen – Koordinaten in einem Raum. In der Schule kennen wir 2D oder 3D. Hier arbeiten wir mit __1024 Dimensionen__ (bei `bge-m3`) – für die Anschauung zu viel, für den Computer kein Problem.

__Spielzeug-Beispiel in 2D__ – wir ordnen Tiere nach zwei Eigenschaften ein:

| Tier    | Größe (x) | Zahm (y) |
| :------ | :-------: | :------: |
| Maus    | 0.1       | 0.2      |
| Katze   | 0.3       | 0.9      |
| Hund    | 0.5       | 0.95     |
| Wolf    | 0.6       | 0.1      |
| Elefant | 1.0       | 0.4      |

Katze und Hund landen __nah beieinander__, Katze und Wolf __weit auseinander__ – obwohl Katze und Wolf biologisch verwandter sind. Die Metrik folgt dem Raum, den wir aufgespannt haben.

__Genau das macht ein Embedding-Modell__ – nur dass es die Achsen nicht von uns vorgegeben bekommt, sondern sie aus Milliarden Textbeispielen selbst lernt. Achse 17 könnte „Wissenschaftsbezug" kodieren, Achse 256 „geografischer Ort in Ostdeutschland", Achse 512 „Fachterminus aus dem Bergbau" – niemand weiß es genau, und das ist auch egal, solange __Bedeutungsähnlichkeit = geometrische Nähe__ gilt.

### Ein Näherungsmaß: Kosinus-Ähnlichkeit

Zwei Vektoren sind sich _ähnlich_, wenn sie in __dieselbe Richtung__ zeigen. Die Länge interessiert uns nicht — ein doppelt so langer Satz darf nicht automatisch „unähnlicher" werden. Deshalb nehmen wir den **Kosinus des Winkels** zwischen den Vektoren:

$$
\text{cos\_sim}(\vec a, \vec b) \;=\; \frac{\vec a \cdot \vec b}{\|\vec a\| \; \|\vec b\|}
$$

Werte zwischen -1 und 1:

+ __1.0__ – identische Richtung (≈ gleiche Bedeutung)
+ __0.0__ – orthogonal (kein Bezug)
+ __-1.0__ – entgegengesetzt (bei Text in der Praxis selten)

Bei typischen Text-Embeddings liegen verwandte Sätze oft bei __0.6–0.9__, unverwandte bei __0.2–0.4__.

### Live-Rechnung: TUBAF vs. Uni Leipzig

Wir holen uns drei Sätze und schauen, welche Paare das Modell für ähnlich hält:

```python
from openai import OpenAI
import numpy as np

client = OpenAI(base_url="http://localhost:11434/v1", api_key="ollama")

texte = [
    "Die TU Bergakademie Freiberg ist eine Universität mit Schwerpunkt auf "
    "Ressourcen, Geowissenschaften und Materialforschung.",

    "Die Universität Leipzig ist mit über 30 000 Studierenden eine der "
    "größten Universitäten in Sachsen.",

    "Ein Rezept für klassischen sächsischen Apfelkuchen mit Streuseln.",
]

resp = client.embeddings.create(model="bge-m3", input=texte)
vektoren = np.array([e.embedding for e in resp.data])

print("Form:", vektoren.shape)          # (3, 1024)
print("Erste 5 Werte TUBAF-Vektor:", vektoren[0][:5])
```

Typische Ausgabe:

```text
Form: (3, 1024)
Erste 5 Werte TUBAF-Vektor: [ 0.0213 -0.0447  0.0618  0.0291 -0.0154 ]
```

__Drei Zahlen pro Embedding sieht man nicht — aber 1024 Dimensionen gehorchen denselben Regeln wie 2D.__ Wir rechnen paarweise den Kosinus:

```python
def kosinus(a, b):
    return float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))

paare = [
    ("TUBAF ↔ Uni Leipzig",  0, 1),
    ("TUBAF ↔ Apfelkuchen",  0, 2),
    ("Leipzig ↔ Apfelkuchen", 1, 2),
]

for label, i, j in paare:
    print(f"{label:26s}  cos = {kosinus(vektoren[i], vektoren[j]):.3f}")
```

Erwartetes Muster (genaue Zahlen kommen live aus Ollama):

| Paar                      | cos (Größenordnung) | Interpretation                              |
| :------------------------ | :------------------ | :------------------------------------------ |
| TUBAF ↔ Uni Leipzig       | ~0.75               | beide „deutsche Universität in Sachsen"     |
| TUBAF ↔ Apfelkuchen       | ~0.35               | nur das Wörtchen „sächsisch" gemeinsam      |
| Leipzig ↔ Apfelkuchen     | ~0.40               | ähnlich gering                              |

> __Das ist die Magie hinter semantischer Suche:__ Wir haben kein Wort „Universität" im Apfelkuchen-Text, und trotzdem erkennt das Modell, dass TUBAF und Uni Leipzig __inhaltlich zusammengehören__. Eine klassische Stichwort-Suche würde hier versagen.

### Skalierung: Vektor-Datenbanken

Drei Texte im Array sind schön für die Demo. Bei 10 000 Dokumenten will niemand bei jeder Anfrage 10 000 Skalarprodukte ausrechnen. Dafür gibt es __Vektor-Datenbanken__:

+ [`chromadb`](https://www.trychroma.com/) – einfachste Variante, läuft in-process, auf Dateien.
+ [`qdrant`](https://qdrant.tech/) – Docker-Container, HTTP-API, produktionsreif.
+ [`faiss`](https://github.com/facebookresearch/faiss) – Bibliothek von Meta, sehr schnell, kein Server.

Alle nehmen Vektoren an und liefern zu einer Query die `k` ähnlichsten zurück – intern über _Approximate Nearest Neighbor_-Verfahren, die aus Millionen Dokumenten in Millisekunden die Treffer fischen.

Kombiniert mit einem lokalen LLM entsteht daraus ein __vollständig lokales Retrieval-Augmented-Generation-Setup__: Skripte, Promotionen, Archive lassen sich semantisch durchsuchen, ohne dass ein Byte den eigenen Rechner verlässt.

> __Das ist ein eigenes Vortragsthema.__ Wir streifen es hier nur so weit, dass klar ist, was unter der Haube passiert.

## 7. Wo der Laptop aufhört – die DGX an der TUBAF

Lokale 7B-Modelle reichen für vieles – bei langen Kontexten, Mehrsprachigkeit, Code-Verständnis in großen Projekten oder echtem Reasoning merkt man den Qualitätssprung zu einem 70B- oder 123B-Modell aber deutlich. Dafür brauchen wir ernsthafte Hardware.

__Was braucht ein großes Modell wirklich?__

| Modellgröße    | VRAM (4-bit quantisiert)  | Beispiel-Hardware                     |
| :------------- | :------------------------ | :------------------------------------ |
| 7B             | ~5 GB                     | RTX 3060, Apple M1/M2                 |
| 14B            | ~9 GB                     | RTX 4080, Apple M3 Pro                |
| 70B            | ~45 GB                    | 2× RTX 3090, 1× A100 40 GB            |
| 123B           | ~80 GB                    | 1× A100 80 GB, 1× H100                |
| 405B           | ~240 GB                   | 4× H100 / DGX-Knoten                  |

__Die TUBAF-Rechenzentrum als Option__

Die Universität betreibt GPU-Knoten, auf denen sich Ollama oder vLLM für große Modelle betreiben lassen. Typischer Ablauf:

1. Zugang über das Rechenzentrum beantragen (Nutzungsantrag, Kurz-Projektbeschreibung).
2. Ollama auf dem Cluster-Knoten starten und den Port per SSH-Tunnel auf den eigenen Rechner weiterleiten:

   ```bash
   ssh -L 11434:localhost:11434 user@dgx.tu-freiberg.de
   ```

3. Das lokale Python-Skript unverändert weiterverwenden – `base_url="http://localhost:11434/v1"` zeigt jetzt über den Tunnel auf die DGX.

> Details zu Zugang, Quoten und geeigneten Lastprofilen bespreche ich gern im Nachgang persönlich – der konkrete Workflow hängt an der URZ-Policy und wechselt schneller als ein Foliensatz aktuell bleibt.

## 8. Grenzen und Fallstricke

__Quantisierung ist ein Kompromiss__

+ Ollama liefert standardmäßig 4-bit-quantisierte Modelle – 4× kleiner, spürbar weniger präzise.
+ Für hochwertige Inferenz ggf. auf 8-bit- oder unquantisierte Varianten umsteigen (Tag `:q8_0`, `:fp16`) – dann gelten die VRAM-Angaben oben nicht mehr.

__Kontextfenster ist kurz voreingestellt__

+ Ollama nutzt per Default nur 2048 Tokens Kontext, auch wenn das Modell 128 k könnte.
+ Anheben per Modelfile (`PARAMETER num_ctx 32768`) oder beim Aufruf – Speicher steigt entsprechend.

__Halluzinationen bleiben__

+ Alle Aussagen aus dem Mistral-Vortrag zu Halluzinationen gelten __verstärkt__ für kleinere lokale Modelle.
+ Fachliche Details immer validieren – ein 7B-Modell erfindet Fakten noch bereitwilliger als ein 123B-Cloud-Modell.

__Lizenzen beachten__

+ Nicht jedes Modell in Ollama's Registry ist kommerziell frei nutzbar.
+ Llama 3 hat eine Custom License, Mistral-Small steht unter Apache 2.0 — vor produktivem Einsatz prüfen.

__Datenschutz ≠ Sicherheit__

+ „Läuft lokal" heißt nicht automatisch „DSGVO-konform". Logging, Backups, Multi-User-Zugriffe gehören weiter betrachtet.

## Zusammenfassung

+ Ollama installiert offene LLMs mit drei Befehlen und stellt sie über eine OpenAI-kompatible HTTP-API bereit.
+ Der __Python-Code aus dem Mistral-Vortrag läuft unverändert__ gegen Ollama – nur `base_url` und Modellname ändern sich.
+ Open-WebUI liefert ein grafisches Frontend für Teams, die nicht programmieren möchten.
+ Lokal läuft (je nach Hardware) ein 7B- bis 14B-Modell gut. Für mehr: DGX an der TUBAF.
+ Datenschutz und Kostenfreiheit sind stark, Qualität und Geschwindigkeit schwächer als Cloud-Flaggschiffe. __Die richtige Antwort ist fast immer: beides, und zwar wissen, wann was.__

__Material & Code__

+ [Ollama-Dokumentation](https://docs.ollama.com)
+ [Ollama-Modellbibliothek](https://ollama.com/library)
+ [Open-WebUI](https://openwebui.com)
+ [llama.cpp](https://github.com/ggerganov/llama.cpp) – der Motor unter Ollama's Haube
+ Die Beispielskripte dieses Vortrags liegen unter `90_Ollama/codebeispiele/` im Repository.
