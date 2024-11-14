<!--
author:   Sebastian Zug

email:    sebastian.zug@informatik.tu-freiberg.de

version:  0.0.2

language: de

comment:  "I don't git it" Einführung in die Grundlagen der Versionsverwaltung 

import: https://github.com/liascript/CodeRunner
        https://raw.githubusercontent.com/liaTemplates/ExplainGit/master/README.md
        https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md

narrator: DE

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/main/07_Git_II/presentation.md#1)

# TUBAF Bits&Bytes

I don't git it! - Einführung in die Versionsverwaltung mit Git (Teil II)
----------------------------------------------------------------

Donnerstag, 11.04.2024, 17 Uhr, RoboLab der TU Bergakademie Freiberg

---------------------


Prof. Dr. Sebastian Zug

> _Versionsmanagement und git sind für Sie ein "Buch mit sieben Siegeln"? Der Vortrag motiviert die Konzepte und erläutert deren Anwendung in konkreten Situationen der wissenschaftlichen Arbeit. Darüber hinaus wird anhand von Demos die Einbettung der Versionsverwaltung in das Projektmanagement praktisch vorgestellt._

## Kurze Wiederholung

                        {{0-1}}
********************************************************************************


```ascii
+----------------------------+---------------------------+----------------------------+---------------------------+ 
|        Repository          |       Repository          |        Repository          |       Repository          | 
|        +-------+           |       +-------+           |        +-------+           |       +-------+           | 
|        |   A   |           |       |   A   |           |        |  A''  |           |       |  A''  |           | 
|        +-------+           |       +-------+           |        +-------+           |       +-------+           | 
|         /     \            |                           |              ^             |         X                 | 
|    read/       \read       |                           |               \write       |   write/                  | 
|       /         \          |                           |                \           |       /                   | 
|      v           v         |                           |                 \          |      /                    | 
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     | 
| |   A   |    |   A   |     | |   A'  |   |  A''  |     | |   A'  |    |  A''  |     | |   A'  |   |  A''  |     | 
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     | 
|   Harry        Sally       |   Harry       Sally       |   Harry        Sally       |   Harry       Sally       | 
|                            |                           |                            |                           | 
| Erzeugen der lokalen Kopie | Barbeitung                |Sally schreibt ihre Version |Harries Schreibversuch wird| 
|                            |                           |                            |blockiert                  |
+----------------------------+---------------------------+----------------------------+---------------------------+ 
|        Repository          |       Repository          |        Repository          |       Repository          |
|        +-------+           |       +-------+           |        +-------+           |       +-------+           |
|        |  A''  |           |       |  A''  |           |        |   A*  |           |       |   A*  |           |
|        +-------+           |       +-------+           |        +-------+           |       +-------+           |
|         /                  |                           |         ^                  |              \            |
|    read/                   |                           |   write/                   |               \read       |
|       /                    |                           |       /                    |                \          |
|      v                     |                           |      /                     |                 v         |
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     |
| | A',A''|    |  A''  |     | |   A*  |   |  A''  |     | |   A*  |    |  A''  |     | |   A*  |   |   A*  |     |
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     |
|   Harry        Sally       |   Harry       Sally       |   Harry        Sally       |   Harry       Sally       |
|                            |                           |                            |                           |
| Mergen der Kopien          | merge(A',A'')=A*          |Harry schreibt seine Version|Sally übermittelt A''      |
+----------------------------+---------------------------+----------------------------+---------------------------+
```

> Die Herausforderung liegt somit im Mischen von Dokumenten!

********************************************************************************


                        {{1-2}}
********************************************************************************


Zustandsmodell einer Datei in Git
================================

```text @plantUML.png
@startuml
hide empty description
[*] --> Untracked : Erzeugen einer Datei
Untracked --> Staged : Hinzufügen zum Repository <color:Red> ""git add""
Unmodified --> Modified : Editierung der Datei
Modified --> Staged : Markiert als neue Version  <color:Red>  ""git add""
Staged --> Unmodified : Bestätigt als neue Version  <color:Red>  ""git commit""
Unmodified --> Untracked : Löschen aus dem Repository   <color:Red>  ""git remove""
@enduml
```

********************************************************************************


                        {{2-3}}
********************************************************************************

Grundlegende Anwendung (lokal!)
====================================

> **Merke:** Anders als bei svn können Sie mit git eine völlig autonome Versionierung auf Ihrem Rechner realisieren. Ein Server ist dazu *zunächst* nicht nötig.

Aus dem Zustandmodell einer Datei ergeben sich drei Ebenen auf der wir eine Datei in Git Perspektivisch einordnen können - Arbeitsverzeichis, Stage und Repository.

> **Achtung:** Die folgende Darstellung dient hauptsächlich der didaktischen Hinführung zum Thema!

```ascii
                     lokal                           remote
  ---------------------------------------------  --------------
  Arbeitskopie     "Staging"        Lokales          Remote
                                   Repository      Repository
                       |               |                          
                       |               |            Dazu kommen   
                       |               |            wir gleich !  
       +-+- - - - - - -|- - - - - - - -|                          
       | | Änderungen  |               |                          
       | |             |               |                          
       +-+             |               |                          
        |   git add    |               |                          
        |------------->|  git commit   |                          
        |              |-------------->|                          
       +-+             |               |                          
       | | weitere     |               |                          
       | | Änderungen  |               |                          
       +-+             |               |                          
        |   git add    |               |                          
        +------------->|  git commit   |                          
                       |-------------->|                          
                       |               |
                       |               |                     
```

Phase 1: Anlegen des Projektes und initialer Commit

```
mkdir GitBasicExample
cd GitBasicExample
git init
touch README.md
// Hier kommen jetzt einige Anpassungen in README.md dazu
git add README.md
git commit -m "Add first commit!"
git status
```

Phase 2: Arbeit im Projekt

```
// Veränderungen im Programmcode
git add Program.cs
git commit -m "Change output of project!"
git log --oneline
```

Bis hier her haben wir lediglich eine Erfassung unserer Aktivitäten umgesetzt. Wir können anhand der Log-Files einen Überblick darüber behalten, wann welche Änderungen in welcher Datei realisiert wurden.

********************************************************************************

## "Kommando zurück"

Nun wird es aber interessanter! Lassen Sie uns jetzt aber zwischen den Varianten navigieren.

**Variante 1 - Reset (Zurücksetzen)**

`git reset` löscht die Historie bis zu einem Commit. Adressieren können wir die Commits relativ zum `HEAD` `git reset HEAD~1` oder mit der jeweiligen ID `git reset <commit-id>`.

Wichtig sind dabei die Parameter des Aufrufes:

| Attribut            |         | Vorgang                                                       |
| ------------------- | ------- | ------------------------------------------------------------- |
| `git reset --soft`  |         | uncommit changes, changes are left staged (index).            |
| `git reset --mixed` | default | uncommit + unstage changes, changes are left in working tree. |
| `git reset --hard`  |         | uncommit + unstage + delete changes, nothing left.            |

``` text @ExplainGit.eval
git commit -m V1
git commit -m V2
git commit -m V3
```

**Variante 2 - Revert**

Häufig möchte man nicht die gesamte Historie zurückgehen und alle Änderungen verwerfen, sondern vielmehr eine Anpassung zurücknehmen und deren Auswirkung korrigieren. Eine Anpassung würde aber bedeuten, dass alle nachgeordneten Commits angepasst werden müssten.

``` text @ExplainGit.eval
git commit -m V1
git commit -m V2
git commit -m V3
```

**Variante 3 - Rebase**

... ist eine weitere Möglichkeit, um Commits zu verändern. Dabei wird die Historie umgeschrieben.

### Ich sehe was, was Du nicht siehst ...

Häufig bettet ein Projekt Dateien ein, die Git nicht automatisch hinzufügen oder schon gar nicht als „nicht versioniert“ anzeigen soll. Beispiele dafür sind automatisch generierte Dateien, wie Log-Dateien oder die Binaries, die von Ihrem Build-System erzeugt wurden. In solchen Fällen können Sie die Datei .gitignore erstellen, die eine Liste mit Vergleichsmustern enthält. Hier ist eine .gitignore Beispieldatei:

```console     gitignoreExample
# ignore all .a files
*.a

# but do track lib.a, even though you're ignoring .a files above
!lib.a

# only ignore the TODO file in the current directory, not subdir/TODO
/TODO

# ignore all files in any directory named build
build/

# ignore doc/notes.txt, but not doc/server/arch.txt
doc/*.txt

# ignore all .pdf files in the doc/ directory and any of its subdirectories
doc/**/*.pdf
```

Unter [gitIgnoreBeispiele](https://github.com/github/gitignore) gibt es eine ganze Sammlung von Konfigurationen für bestimmte Projekttypen.

###  Verteiltes Versionsmanagement

_Einfaches Editieren_: Sie klonen das gesamte Repository, dass sich auf dem "Server-Rechner" befindet. Damit haben Sie eine vollständige Kopie aller Versionen in Ihrem Working Directory. Wenn wir annehmen, dass keine branches im Repository bestehen, dann können Sie direkt auf der Ihrer Arbeitskopie arbeiten und diese verändern. Danach generieren Sie einen Snappshot des Arbeitsstandes _Staging_. Ihre Version ist als relevant markiert und kann im lokalen Repository als neuer Eintrag abgelegt werden. Vielleicht wollen sie Ihren Algorithmus noch weiterentwickeln und speichern zu einem späteren Zeitpunk eine weitere Version. All diese Vorgänge betreffen aber zunächst nur Ihre Kopie, ein anderer Mitstreiter in diesem Repository kann darauf erst zurückgreifen, wenn Sie das Ganze an den Server übermittelt haben.

```ascii
                     lokal                           remote
  ---------------------------------------------  --------------
  Arbeitskopie     "Staging"        Lokales          Remote
                                   Repository      Repository
                       |               |                 |
                       |               |    git clone    |
                       |               |<----------------|
       +-+- - - - - - -|- - - - - - - -|                 |
       | | Änderungen  |               |                 |
       | |             |               |                 |
       +-+             |               |                 |
        |   git add    |               |                 |
        |------------->|  git commit   |                 |
        |              |-------------->|                 |
       +-+             |               |                 |
       | | weitere     |               |                 |
       | | Änderungen  |               |                 |
       +-+             |               |                 |
        |   git add    |               |                 |
        +------------->|  git commit   |                 |
                       |-------------->|   git push      |
                       |               |---------------->|
                       |               |                 |                                         .
```


_Kooperatives Arbeiten:_ Nehmen wir nun an, dass Ihr Kollege in dieser Zeit selbst das Remote Repository fortgeschrieben hat. In diesem Fall bekommen Sie bei Ihrem `push` eine Fehlermeldung, die sie auf die neuere Version hinweist. Nun "ziehen" Sie sich den aktuellen
Stand in Ihr Repository und kombinieren die Änderungen. Sofern keine Konflikte
entstehen, wird daraus ein neuer Commit generiert, den Sie dann mit Ihren Anpassungen an das Remote-Repository senden.

```ascii
                     lokal                           remote
  ---------------------------------------------  --------------
  Arbeitskopie     "Staging"        Lokales          Remote
                                   Repository      Repository
                       |               |                 |
                       |               |    git clone    |
                       |               |<----------------|
       +-+- - - - - - -|- - - - - - - -|                 |
       | | Änderungen  |               |                 |
       | |             |               |                 |
       +-+             |               |                 |
        |   git add    |               |                 |
        |------------->|  git commit   |                 |
        |              |-------------->|                 |
       +-+             |               |                 |
       | | weitere     |               |                 |
       | | Änderungen  |               |                 |   git push
       +-+             |               |                 |<-------------
        |   git add    |               |                 |
        +------------->|  git commit   |                 |
                       |-------------->|   git push      |
                       |               |---------------X |
                       |               |   git fetch     |
                       |               |<--------------- |     git fetch
                       |               |   git merge     |   + git merge
                       |               |<--------------- |   = git pull
                       |               |   git push      |
                       |               |---------------->|                                          .
```


Versuchen wir das ganze noch mal etwas plastischer nachzuvollziehen. Rechts oben sehen Sie unser Remote-Repository auf dem Server. Im mittleren Bereich den Status unseres lokalen Repositories.


``` text @ExplainGit.eval
create origin
```

## Arbeiten mit Branches

Die Organisation von Versionen in unterschiedlichen Branches ist ein zentrales
Element der Arbeit mit git. Branches sind Verzweigungen des Codes, die bestimmte
Entwicklungsziele kapseln.

Der größte Nachteil bei der Arbeit mit nur einem Branch liegt darin, dass bei einem
defekten Master(-Branch) die Arbeit sämtlicher Beteiligter unterbrochen wird. Branches
schaffen einen eignen (temporären) Raum für die Entwicklung neuer Features, ohne
die Stabilität des Gesamtsystems zu gefährden. Gleichzeitig haben die Entwickler den gesamten Verlauf eines Projekts in strukturierter Art zur Hand.

<!--
style="width: 100%; max-width: 560px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii

        vSoSe2019                                                   vSoSe2020
main     O-----------------------------------------  ....  ---------O
          \                                                        ^
           \               Offizielle Versionen                   /
SoSe2020    \              O-->O                 O          ---->O
             \            ^     \               /
              v          /       v             /
SoSe2020dev    O->O---->O---->O->O---->O-->O->O      ....
               Vorlesung      Vorlesung
               00             01
```

Ein Branch in Git ist einfach ein Zeiger auf einen Commit zeigt. Der zentrale Branch wird zumeist als `master`/`main` bezeichnet.

### Generieren und Navigation über Branches
<!--
@@ Hinweis für die Realisierung
    git branch feature
    git checkout feature
    git checkout 0e8bf9e
    git branch newFeature
-->

Wie navigieren wir nun konkret über den verschiedenen Entwicklungszweigen.

``` text @ExplainGit.eval
git commit -m V1
git commit -m V2
git commit -m V3
```


### Mergoperationen über Branches
<!--
@@ Hinweis für die Realisierung
    git checkout master
    git branch hotfix
    git checkout hotfix
    git commit -m "Solve bug"
    git checkout master
    git merge hotfix
    You have performed a fast-forward merge.
    git branch -d hotfix
    git checkout newFeature
    git commit -m FeatureV3
    git checkout master
    git merge newFeature
    Beim fast-forward merge gibt es keine nachfolgende Version. Der master wird
    einfach auf den anderen branch verschoben.
    Im zweiten Fall läuft ein 3 Wege Merge ab
-->
Nehmen wir folgendes Scenario an. Sie arbeiten an einem Issue, dafür haben Sie
einen separaten Branch (newFeature) erzeugt und haben bereits einige Commits
realisiert. Beim Kaffeetrinken denken Sie über den Code von letzter Woche nach und Ihnen fällt ein Bug ein, den Sie noch nicht behoben haben. Jetzt aber schnell!

Legen Sie dafür einen neuen Branch an, commiten Sie eine Version und mergen
Sie diese mit dem Master. Kehren Sie dann in den Feature-Branch zurück und
beenden Sie die Arbeit. Mergen Sie auch diesen Branch mit dem Master.
Worin unterscheiden sich beide Vorgänge?

``` text @ExplainGit.eval
git branch newFeature
git checkout newFeature
git commit -m FeatureV1
git commit -m FeatureV2
```

Mergen ist eine nicht-destruktive Operation. Die bestehenden Branches werden auf keine Weise verändert. Das Ganze "bläht" aber den Entwicklungsbaum auf.

## Arbeit mit GitHub

![](https://media.giphy.com/media/487L0pNZKONFN01oHO/giphy.gif)

Github integriert einen git-Dienst und stellt darauf aufbauend weitere Werkzeuge bereit.

| Bezeichnung         | Bedeutung                                                             |
| ------------------- | --------------------------------------------------------------------- |
| `Issue`             | Aufgabenbeschreibungen mit zugehörigen Diskussionen                   |
| `Pull-Request` (PR) | Anfrage zum Merge einer eignen Lösung mit der existierenden Codebasis |
| `Review`            | Vorgänge zur Evaluation von Pull-Requests                             |
| `Action`            | Möglichkeit der automatisierten Realsierung von Abläufen              |
| `Project`           | Verwaltungsebene von Aufgabenbeschreibungen                           |

![SWEonGithub](./img/12_VersionsverwaltungII/ScreenshotSoftwareentwicklungVorlesung.png)<!-- width="100%" -->

> Softwareprojekte sind soziale Ereignisse - entsprechend integriert Github auch Aspekte zur Vernetzung (`Discussion`), der Anerkennung (`Stars`) und Follow Mechanismen.

> Neben Github existieren weitere Plattformen für das Hosten von Projekten. Eine völlig eigenständige Nutzung ermöglicht die GitLab Community Edition (CE) von GitLab die als Open-Source-Software unter der MIT-Lizenz entwickelt wird.

Lassen Sie uns die Verwendung anhand vdes Open Source Projektes Tensorflow erörtern [https://github.com/tensorflow/tensorflow](https://github.com/tensorflow/tensorflow)

### Issues

Issues dienen dem Sammeln von Benutzer-Feedback, dem Melden von Software-Bugs und der Strukturierung von Aufgaben, die Sie erledigen möchten. Dabei sind Issues umittelbar mit Versionen verknüpft, diese können dem Issue zugeordnet werden.

Sie können eine Pull-Anfrage mit einer Ausgabe verknüpfen, um zu zeigen, dass eine Korrektur in Arbeit ist und um die Ausgabe automatisch zu schließen, wenn jemand die Pull-Anfrage zusammenführt.

Um über die neuesten Kommentare in einer Ausgabe auf dem Laufenden zu bleiben, können Sie eine Ausgabe beobachten, um Benachrichtigungen über die neuesten Kommentare zu erhalten.

https://guides.github.com/features/issues/

> Im Tensorflow Projekt sind aktuell etwa 2000 Issues offen, über 35000 wurden bereits bearbeitet. Eine Menge Koordinationsarbeit ... wie halten wir da Ordnung?

```
If you open a GitHub Issue, here is our policy:

    It must be a bug/performance issue or a feature request or a build issue or a documentation issue (for small doc fixes please send a PR instead).
    Make sure the Issue Template is filled out.
    The issue should be related to the repo it is created in.

Here's why we have this policy: We want to focus on the work that benefits the whole community, e.g., fixing bugs and adding features. Individual support should be sought on Stack Overflow or other non-GitHub channels. It helps us to address bugs and feature requests in a timely manner.
```

1. Als potentieller Mitstreiter - Durchsuchen geht vor Schreiben eines neuen Issues
2. Strukturieren Sie die Meldung von Problemen (ermutigen Sie die Meldenden, spezifisch zu sein)
3. Nutzen Sie Label aber generieren Sie keinen bunten Strauß 
4. Referenzieren Sie die entsprechenden Personen im Issue 
5. Vergessen Sie Issues nicht zu schließen

Das zugehörige Template findet sich unter [https://github.com/tensorflow/tensorflow/blob/master/ISSUE_TEMPLATE.md](https://github.com/tensorflow/tensorflow/blob/master/ISSUE_TEMPLATE.md)

### Pull requests und Reviews

Natürlich wollen wir nicht, dass "jeder" Änderungen ungesehen in unseren Code einspeist. Entsprechend kapseln wir ja auch unseren Master-Branch. Den Zugriff regeln sowohl die Rechte der einzelnen Mitstreiter als auch die
Pull-Request und Review Konfiguration.

| Status des Beitragenden | Einreichen des Codes                                                                                                                                                                                                                              |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Collaborator            | Auschecken des aktuellen Repositories und Arbeit. Wenn die Arbeit in einem eigenen Branch erfolgt, wird diese mit einem Pull-Request angezeigt und gemerged.                                                                                      |
| non Collaborator        | Keine Möglichkeit seine Änderungen unmittelbar ins Repository zu schreiben. Der Entwickler erzeugtn eine eigene Remote Kopie (_Fork_) des Repositories und dortige Realsierung der Änderungen.  Danach werden diese als Pull-Request eingereicht. |

Wird ein Pull Request akzeptiert, so spricht man von einem _Merge_, wird er geschlossen, so spricht man von einem _Close_. Vor dem Merge sollte eine gründliche Prüfung sichergestellt werden, diese kann in Teilen automatisch erfolgen oder durch Reviews [Doku](https://github.com/features/code-review/)

> Achtung: Forks sind Kopien bestehender Repositories. Sie ermöglichen die Arbeit in einem eigenen Repo. 

## Und jetzt mal praktisch

> Los geht es ...
