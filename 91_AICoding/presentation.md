<!--
author:   Sebastian Zug

email:    sebastian.zug@informatik.tu-freiberg.de

version:  0.1.1

language: de

narrator: Deutsch Female

icon:     https://media.aubi-plus.com/institution/thumbnail/3f3de48-technische-universitaet-bergakademie-freiberg-logo.jpg

import:   https://raw.githubusercontent.com/LiaScript/CodeRunner/master/README.md

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/refs/heads/main/91_AICoding/presentation.md#1)

# TUBAF Bits&Bytes

KI-gestützte Softwareentwicklung – von VS Code bis GitHub
----------------------------------------------------------------

Donnerstag, 30.04.2026, 17 Uhr, RoboLab der TU Bergakademie Freiberg

---------------------

Prof. Dr. Sebastian Zug (Fakultät 1)

> _In den vorangegangenen Vorträgen wurden Sprachmodelle als API-Dienst betrachtet, der aus eigenem Code heraus angesprochen wird — in der Cloud (Mistral) wie auch lokal (Ollama). Der heutige Vortrag wechselt die Perspektive: Das Modell tritt als **Assistent im Entwicklungsprozess** selbst auf und beteiligt sich unmittelbar am Schreiben, Refaktorieren und Prüfen von Code. Anhand eines kleinen Python-Projekts wird der Bogen von der initialen Implementierung in VS Code bis zur Code-Review im Pull-Request auf GitHub gespannt. Dabei werden die unterschiedlichen Interaktionsmuster der gängigen Werkzeuge (GitHub Copilot, Claude Code, Cursor) eingeordnet und ihre jeweiligen Stärken und Grenzen herausgearbeitet._
>
> _Zur aktiven Teilnahme empfohlen: VS Code, ein GitHub-Account sowie ein aktivierter Copilot-Zugang (für Studierende über GitHub Education kostenfrei verfügbar). Ohne Copilot-Zugang ist der Vortrag passiv verfolgbar — die Live-Demonstration erfolgt am Rechner des Vortragenden._

## 1. Motivation

> Wer von Ihnen hat in den letzten 30 Tagen mindestens einmal eine KI gefragt, wie ein bestimmter Code-Block funktioniert oder geschrieben werden sollte?

In den letzten zwei Jahren hat sich die Art, wie Software entsteht, leise verschoben. Eine [GitHub-Studie 2024](https://github.blog/2024-09-05-research-quantifying-github-copilots-impact-in-the-enterprise-with-accenture/) berichtet von ~55 % schnellerem Aufgabenabschluss bei Routine-Code mit Copilot. Eine [METR-Studie 2025](https://metr.org/blog/2025-07-10-early-2025-ai-experienced-os-dev-study/) zeigt das Gegenteil: Erfahrene Open-Source-Entwickler:innen wurden mit KI-Assistenz **langsamer**, schätzten sich aber als **schneller** ein. Beides ist wahr — der Unterschied liegt in **Aufgabe**, **Codebasis** und **Erfahrung**.

> __Die Leitfrage für heute:__ Wann macht uns ein KI-Coding-Assistent schneller, wann führt er uns in die Irre — und welcher Workflow passt zu welcher Phase eines Projekts?

__Was wir heute nicht machen__

+ Einen Lobgesang auf „die KI schreibt jetzt allen Code". Tut sie nicht, und auch nicht morgen.
+ Eine vollständige Tool-Vergleichstabelle. Die wäre nächste Woche veraltet.

__Was wir heute machen__

+ Eine kleine **Taxonomie**: Was unterscheidet Autocomplete, Chat, Agent und Review?
+ Ein **gemeinsames Mini-Projekt** in VS Code: ein CLI-Tool zum Auswerten von CSV-Daten — von der ersten Funktion bis zum Test.
+ Den **Sprung in die Cloud**: Dasselbe Projekt auf GitHub bringen und dort die Copilot-Funktionen für Issues, PR-Beschreibungen und Code-Review nutzen.
+ Eine **ehrliche Bilanz**: Wo der Workflow trägt, wo er bricht, und was das für die Lehre bedeutet.

## 2. Eine kleine Taxonomie

Die Begriffe „KI-Coding-Tool" und „Copilot" werden synonym benutzt — sind es aber nicht. Vier Interaktionsmuster lassen sich klar unterscheiden:

| Muster              | Was passiert                                                | Beispiele                                          |
| :------------------ | :---------------------------------------------------------- | :------------------------------------------------- |
| **Autocomplete**    | Das Modell schlägt während des Tippens den nächsten Block vor; Tab akzeptiert. | GitHub Copilot (Inline), Codeium, Tabnine          |
| **Chat im Editor**  | Eine Sidebar/Panel, in dem man Fragen stellt oder Refactors anfordert; Modell sieht ausgewählten Code. | Copilot Chat, Cursor Chat, Continue.dev            |
| **Agent**           | Modell plant und **führt mehrere Schritte selbstständig aus** (Dateien lesen, schreiben, Befehle ausführen). | Claude Code, Cursor Agent, Copilot Coding Agent, Codex |
| **Review/CI**       | Modell sieht Diffs in einem Pull-Request und kommentiert oder schlägt Änderungen vor. | Copilot PR Review, CodeRabbit, Claude on GitHub    |

> Diese vier Muster lassen sich **kombinieren**, decken aber unterschiedliche Phasen ab: Autocomplete im Tippfluss, Chat beim Verstehen, Agent beim Bau ganzer Features, Review beim Absichern. Wer nur eines kennt, übersieht den Rest.

__Wo läuft das Modell eigentlich?__

| Ort              | Beispiele                                  | Datenschutz                            |
| :--------------- | :----------------------------------------- | :------------------------------------- |
| Cloud (Anbieter) | Copilot (Azure), Claude (Anthropic), Cursor (eigene Backends) | Code wird übertragen, Anbieter-Policy beachten |
| Lokal            | Continue.dev + Ollama, Tabnine on-premise  | Code bleibt auf dem Rechner            |
| Hybrid           | Copilot Enterprise (Indexierung im Tenant) | Anbieter-spezifisch                    |

> Für **Industrieprojekte mit NDA** oder **Prüfungsleistungen** ist die Frage nach dem Modellort nicht akademisch. Wir hatten dazu im [Ollama-Vortrag](../90_Ollama/presentation.md) das nötige Werkzeug — Continue.dev + Ollama liefert eine vollständig lokale Copilot-Alternative.

## 3. Setup für die Demo

Wir nutzen heute live folgendes Setup. Wer mitlaufen will:

```bash
# VS Code installieren
# https://code.visualstudio.com

# Extensions
# - GitHub Copilot
# - GitHub Copilot Chat
# - Python
# - Pylance

# uv für das Python-Projekt (wie in den Vorträgen 89/90)
# https://docs.astral.sh/uv/
curl -LsSf https://astral.sh/uv/install.sh | sh
```

> Wir nutzen [uv](https://docs.astral.sh/uv/) konsistent mit den vorigen Vorträgen — `uv sync` legt `.venv/` an und installiert die Abhängigkeiten aus `pyproject.toml`, `uv run <script>` führt direkt darin aus. Keine OS-spezifischen `activate`-Tänze.

__Copilot-Zugang__

+ Studierende: kostenlos über [GitHub Education](https://education.github.com/pack)
+ Beschäftigte: aktuell ~10 USD/Monat, oder kostenfrei mit Copilot Free (limitierte Anfragen)
+ Open-Source-Maintainer: kostenlos auf Antrag

__Tastenkürzel, die heute oft fallen__

| Aktion              | Shortcut (VS Code, Linux/Win)            |
| :------------------ | :--------------------------------------- |
| Vorschlag annehmen  | `Tab`                                    |
| Vorschlag verwerfen | `Esc`                                    |
| Nächster Vorschlag  | `Alt + ]`                                |
| Inline-Chat         | `Ctrl + I`                               |
| Chat-Sidebar        | `Ctrl + Alt + I`                         |

## 4. Das Beispielprojekt — `csv-insight`

Wir bauen ein kleines Kommandozeilen-Tool, das eine CSV-Datei einliest und für jede numerische Spalte Mittelwert, Median, Min und Max ausgibt. Klein genug für 30 Minuten, groß genug, um alle vier Interaktionsmuster zu zeigen.

```text
csv-insight/
├── README.md
├── .python-version    # festgelegte Python-Version für uv
├── pyproject.toml     # Abhängigkeiten + Projekt-Metadaten
├── uv.lock            # exakte Versionen (von uv erzeugt)
├── src/
│   └── csv_insight/
│       ├── __init__.py
│       ├── cli.py
│       └── stats.py
└── tests/
    └── test_stats.py
```

```bash
uv init csv-insight
cd csv-insight
uv add pandas
uv add --dev pytest
```

__Beispiel-Datensatz__

Wir arbeiten mit einer kleinen Wetter-Tabelle ([project/sample.csv](project/sample.csv)) — zwei Wochen Tagesmesswerte aus Freiberg-Mitte:

```text
datum,messstation,temperatur_c,luftfeuchte_pct,niederschlag_mm,windgeschwindigkeit_kmh
2026-04-01,Freiberg-Mitte,8.4,72,0.0,11.2
2026-04-02,Freiberg-Mitte,9.1,68,0.0,9.7
2026-04-03,Freiberg-Mitte,7.8,81,2.4,14.5
2026-04-04,Freiberg-Mitte,6.2,88,5.1,18.3
...
```

Vier numerische Spalten (`temperatur_c`, `luftfeuchte_pct`, `niederschlag_mm`, `windgeschwindigkeit_kmh`), zwei nicht-numerische (`datum`, `messstation`) — gerade genug, damit die Filter-Logik in `column_summary` etwas zu tun hat.

__Anforderungen__

1. CLI: `csv-insight daten.csv` druckt eine Zusammenfassung über alle numerischen Spalten.
2. Optional: `--column NAME` schränkt auf eine Spalte ein.
3. Tests für die Statistik-Funktionen.

> Bewusst gewählt: Die Aufgabe ist **klar umrissen** und **gut getestet**. Genau dort sind KI-Assistenten am stärksten — und das ist eine Aussage, keine Einschränkung. Die METR-Studie sieht den Bremseffekt vor allem in **großen, schlecht dokumentierten** Codebasen.

## 5. Phase 1 — Autocomplete im Tippfluss

Wir starten mit `stats.py` und schreiben **nur die Funktionssignatur und einen Docstring**:

```python
import pandas as pd

def column_summary(df: pd.DataFrame, column: str) -> dict:
    """Berechne Mittelwert, Median, Minimum und Maximum einer
    numerischen Spalte. Gibt ein Dict mit den Schlüsseln
    'mean', 'median', 'min', 'max' zurück.
    """
```

Sobald wir nach dem Docstring eine Leerzeile setzen, schlägt Copilot den Funktionsrumpf vor. Tab akzeptiert.

__Was hier gut funktioniert__

+ **Boilerplate verschwindet** — Code, den wir schon hundertmal geschrieben haben.
+ Der Docstring fungiert als **Spezifikation**: Je präziser wir ihn formulieren, desto näher liegt der Vorschlag am Ziel.
+ Funktionsnamen mit klarer Semantik (`column_summary`, `is_numeric`) liefern bessere Vorschläge als generische (`do_thing`).

__Was schiefgeht__

+ Bei zwei sehr ähnlichen Funktionen direkt untereinander **kopiert Copilot Code falsch** — typischer Fehler: Indizes, Vorzeichen, off-by-one.
+ Wenn die letzten 200 Zeilen Tests waren, schlägt Copilot **immer Tests vor**, auch wenn wir gerade Produktivcode schreiben. Kontext ist alles.

> __Faustregel:__ Bei Autocomplete **nie blind annehmen**. Tab ist günstig, Esc ist günstiger als ein Bug.

## 6. Phase 2 — Inline-Chat für Refactors

`stats.py` funktioniert, aber `column_summary` wirft eine wenig hilfreiche Fehlermeldung, wenn die Spalte nicht existiert. Wir markieren die Funktion, drücken `Ctrl+I` und tippen:

```text
Wirf einen aussagekräftigen ValueError, wenn die Spalte
nicht im DataFrame existiert oder nicht numerisch ist.
Liste die verfügbaren numerischen Spalten in der Fehlermeldung auf.
```

Copilot schlägt einen Diff vor. Wir können **Hunk für Hunk akzeptieren oder verwerfen**.

> **Das ist der Punkt, an dem Inline-Chat dem Autocomplete deutlich überlegen ist:** Wir bekommen einen klar umrissenen Patch und können ihn sehen, bevor er im Code landet. Bei Refactors **immer** über Inline-Chat oder Sidebar arbeiten, nicht über Autocomplete.

__Typische Refactor-Anweisungen, die zuverlässig funktionieren__

+ „Extrahiere diese Schleife in eine eigene Funktion und benenne sie sinnvoll."
+ „Konvertiere diese Funktion in eine asynchrone Variante."
+ „Schreibe Type-Hints für alle Parameter und Rückgabewerte."
+ „Ersetze die manuellen `assert`-Aufrufe durch `pytest.raises`-Blöcke."

## 7. Phase 3 — Tests vom Chat aus generieren

Wir öffnen `tests/test_stats.py` (leer) und stellen in der **Chat-Sidebar** (`Ctrl+Alt+I`) folgende Frage, mit der `stats.py`-Datei als Kontext (`#stats.py` per @-Mention oder Drag-and-Drop):

```text
Schreibe pytest-Tests für column_summary in #stats.py.
Decke ab:
- Glücksfall mit ganzzahligen und Float-Werten
- Spalte existiert nicht
- Spalte ist nicht numerisch (Strings)
- Spalte enthält NaN-Werte
- Leerer DataFrame
Nutze parametrize, wo es Sinn ergibt.
```

Innerhalb von Sekunden steht ein vollständiger Test-Block da. Wir lassen `pytest` laufen und schauen uns an:

+ Welche Tests **bestanden auf Anhieb**?
+ Welche Tests sind **inhaltlich daneben** (z.B. weil das Modell den Edge-Case anders interpretiert hat als wir)?
+ Welche Tests decken **mehr ab als nötig** (Over-Engineering)?

> **Das ist der entscheidende didaktische Moment:** Generierten Test-Code als **Vorschlag** behandeln, nicht als fertige Lösung. Das aktive Lesen-und-Beurteilen ist die eigentliche Lernarbeit.

## 8. Phase 4 — Der Sprung zum Agent

Bisher hat das Modell **Vorschläge gemacht**, wir haben akzeptiert oder verworfen. Mit einem **Agent** geben wir mehrere Schritte aus der Hand. In VS Code seit Mitte 2025 verfügbar als **Copilot Agent Mode** (im Chat-Panel auf „Agent" umschalten); alternativ **Claude Code** im Terminal oder **Cursor Agent**.

Anweisung an den Agent:

```text
Bau das Projekt zu einem installierbaren Python-Paket aus:
- Ergänze pyproject.toml um einen Console-Entry-Point
  'csv-insight' auf csv_insight.cli:main
- Schreibe cli.py mit argparse: positionales 'file', optionales '--column'
- README mit Installationsanleitung (uv sync / uv run csv-insight ...)
- Verifiziere am Ende mit `uv run csv-insight tests/data/sample.csv`
  und mit `uv run pytest`, dass alles grün ist
```

Der Agent **liest Dateien, legt sie an, ruft `pip` auf, prüft die Ausgabe, korrigiert sich selbst**. Wir sehen jeden Schritt im Log und können abbrechen.

__Was hier kippt__

+ Der Agent ist **enorm produktiv für Boilerplate** — `pyproject.toml`, `argparse`, README in zwei Minuten.
+ Bei **unscharfen Anweisungen** baut er Dinge, die wir nie gewollt hätten („füg auch noch Logging und Konfiguration hinzu, weil das gehört doch dazu").
+ Bei **größeren Codebasen** ohne klares Architekturbild fängt er an zu raten — das ist die METR-Studien-Falle.

> __Faustregel Agent-Modus:__ Aufgabe **eng umreißen** und **Verifikationskriterium** mitgeben („…und prüfe mit `pytest`, dass alles grün ist"). Ohne Kriterium meldet der Agent „fertig", auch wenn er es nicht ist.

## 9. Phase 5 — Auf GitHub veröffentlichen

Wir legen ein Repository an und pushen den Code. Ab jetzt verschiebt sich der Wirkungsort vom Editor in den Browser.

```bash
gh repo create csv-insight --public --source=. --push
```

> Die GitHub-CLI (`gh`) ist hier praktisch — auch sie versteht KI-Aufrufe (`gh copilot suggest "rename branch"`, `gh copilot explain "git rebase -i"`). Aber das ist heute nur Beifang.

## 10. Copilot auf GitHub — was die Cloud anders macht

Copilot in der IDE sieht **die geöffneten Dateien**. Copilot auf GitHub sieht **das ganze Repository, alle Issues, alle PRs, die ganze Historie**. Das verändert, was sinnvoll ist.

### 10a. Issue → Branch → Implementation (Coding Agent)

Wir öffnen ein Issue:

```text
Titel: --json Flag für maschinenlesbare Ausgabe
Beschreibung:
Aktuell druckt csv-insight eine Tabelle. Für die Verkettung
mit anderen Tools brauchen wir ein --json Flag, das die
Statistik als JSON auf stdout ausgibt.
```

Im Issue: **„Assign to Copilot"**. Innerhalb weniger Minuten:

1. Copilot legt einen Branch an,
2. liest die relevanten Dateien (`cli.py`, `stats.py`),
3. implementiert das Flag,
4. öffnet einen **Draft-PR** mit einer Zusammenfassung der Änderungen,
5. wartet auf unser Review.

> **Der Reviewer ist jetzt der Mensch.** Das ist eine Inversion des klassischen Workflows — und sie funktioniert nur, wenn die Issue-Beschreibung **so präzise ist wie eine gute Spezifikation**. Schlampige Issues führen zu schlampigen PRs, egal ob Mensch oder Modell sie umsetzt.

### 10b. PR-Beschreibung automatisch generieren

Beim Anlegen eines PRs gibt es den Button **„Generate description with Copilot"**. Das Modell liest den Diff, nicht das Issue, und schreibt:

+ eine Zusammenfassung der Änderungen,
+ eine Liste betroffener Dateien,
+ eine vorgeschlagene Test-Strategie.

> Das spart Tipparbeit, **ersetzt aber nicht das _Warum_**. Die generierte Beschreibung sagt korrekt, **was** sich ändert. **Warum** wir es ändern, müssen wir selbst dazuschreiben — der Diff allein erklärt das nicht.

### 10c. Copilot Code Review

In den Repo-Einstellungen: **Code review by Copilot** aktivieren. Bei jedem PR kommentiert Copilot Zeilen mit:

+ Stil-Hinweisen (Naming, Docstrings),
+ möglichen Bugs (Off-by-one, fehlende Null-Checks),
+ Test-Lücken.

__Realitätsabgleich__

+ Copilot Review **findet nicht alle Bugs**, aber es **fängt 60–70 % der Stil-Issues**, die sonst der Mensch reviewen müsste. Das ist der Hebel.
+ Es **erkennt keine Architektur-Probleme** und keine subtilen Race Conditions. Dafür brauchen wir weiter Menschen.
+ Werkzeuge wie [CodeRabbit](https://www.coderabbit.ai/) oder Claude on GitHub spielen in derselben Liga, mit unterschiedlichen Schwerpunkten.

### 10d. Issues und Diskussionen mit Copilot

Im Issue-Editor ein `/`-Slash-Befehl wie `/explain`, `/summarize`, oder im Chat:

```text
@copilot Welche Issues in diesem Repository sind noch offen
und betreffen die CLI?
```

Copilot durchsucht das Repository, fasst zusammen, schlägt Prioritäten vor.

> **Vorsicht im Lehrkontext:** Wenn Studierende Hausarbeiten in einem Repo bearbeiten, sieht Copilot Repo-weit alles — inklusive privater Notizen, alter PRs, Diskussionen. Das ist kein Geheimnis, aber muss bewusst sein.

## 11. Was sich an unserer Arbeit verändert

__Was wir weniger machen__

+ Boilerplate tippen.
+ API-Dokumentation aufrufen, weil wir die Signatur einer Funktion vergessen haben.
+ Test-Skelette von Hand anlegen.
+ Commit-Messages aus dem Stand schreiben.

__Was wir mehr machen — und mehr **müssen**__

+ **Lesen.** Vorschläge bewerten ist anstrengender als sie selbst zu schreiben — und genau hier liegt die Gefahr, in den Autopilot zu rutschen.
+ **Spezifikationen schreiben.** Ein guter Issue-Text ist heute mehr wert als gestern.
+ **Tests ernst nehmen.** Tests sind die Sicherung, dass das, was die KI baut, auch tut, was es soll. Ein PR ohne Tests ist ein PR ohne Bremse.
+ **Architektur denken.** Was die KI nicht aus der Codebasis ablesen kann, müssen wir vorgeben.

> __Das ist der Punkt, an dem die Lehre nachzieht:__ Wer „Programmieren" als „Zeichen tippen" begreift, wird ersetzt. Wer es als „Probleme präzise formulieren und Lösungen verifizieren" begreift, wird stärker. Beides hat es vor der KI auch schon gegeben — die KI macht nur den Unterschied größer.

## 12. Grenzen und Fallstricke

__Lizenzfragen__

+ Generierter Code kann **wörtliche Übernahmen aus Trainingsdaten** enthalten. GitHub bietet einen „Public Code Filter" an — bei strengen Lizenz-Anforderungen aktivieren.

__Datenschutz__

+ Copilot/Cursor übertragen Code an die Anbieter. **Nie über privaten Repos arbeiten, deren Inhalte das nicht dürfen** — hier gilt wieder: Continue.dev + Ollama als Fallback.

__Halluzinierte APIs__

+ Modelle erfinden Funktionsnamen, die es nie gab. Vor allem bei kleineren Bibliotheken oder neuen Versionen. **Imports immer kontrollieren.**

__Skill-Erosion__

+ Wer nur noch Tab drückt, **lernt nicht mehr**. Im Studium ein ernstes Thema. Empfehlung: kritische Lernphasen **explizit ohne Copilot** absolvieren.

__Sicherheit__

+ KI-generierter Code enthält **statistisch mehr Sicherheitslücken** als menschlicher (Stanford 2023, Snyk 2024). Code-Review und SAST-Tools werden wichtiger, nicht unwichtiger.

__Der Bremseffekt__

+ Die [METR-Studie](https://metr.org/blog/2025-07-10-early-2025-ai-experienced-os-dev-study/) hat es klar gezeigt: Bei **erfahrenen Entwicklern** in **vertrauten, großen Codebasen** macht KI-Assistenz langsamer — und die Betroffenen merken es nicht. Wer gut ist und im Flow, sollte den Copilot **gezielt ausschalten** können.

## Zusammenfassung

+ KI-Coding-Werkzeuge zerfallen in vier Muster: **Autocomplete, Chat, Agent, Review** — jedes mit eigener Stärke und eigener Falle.
+ In der **IDE** liegt der Hebel im engen Tippfluss; auf **GitHub** liegt er in der Repository-weiten Sicht (Issues, PRs, Reviews).
+ Der Agent-Modus ist enorm produktiv, **wenn die Aufgabe eng umrissen und das Verifikationskriterium gegeben** ist.
+ Was sich verändert: weniger Tippen, mehr **Lesen, Spezifizieren, Verifizieren, Architektur denken**.
+ Realität: KI ist nicht generell schneller. Bei Routine-Code ja, in komplexen Codebasen messbar **langsamer**. Die Wahl, **wann der Copilot an** ist, gehört zum Handwerk.

__Material & Code__

+ [GitHub Copilot Dokumentation](https://docs.github.com/copilot)
+ [Claude Code](https://docs.claude.com/claude-code)
+ [Cursor](https://cursor.com)
+ [Continue.dev](https://continue.dev) — Open-Source-Copilot-Alternative
+ [METR-Studie 2025: Measuring the impact of early-2025 AI on experienced open-source developer productivity](https://metr.org/blog/2025-07-10-early-2025-ai-experienced-os-dev-study/)
+ [GitHub-Studie 2024: Copilot impact at Accenture](https://github.blog/2024-09-05-research-quantifying-github-copilots-impact-in-the-enterprise-with-accenture/)
+ Das Beispielprojekt `csv-insight` liegt unter `91_AICoding/project/` im Repository.
