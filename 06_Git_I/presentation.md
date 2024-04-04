<!--
author:   Sebastian Zug

email:    sebastian.zug@informatik.tu-freiberg.de

version:  0.0.4

language: en

logo:     https://github.com/SebastianZug/RoboLabVortraege/blob/main/02_LatexUndWord/images/Logo.jpg?raw=true

comment:  "Word vs. Latex - Ein kritischer Vergleich" systematisiert die konzeptionellen Unterschiede und führt in einem Live-Tutorial in die Grundlagen von Latex ein.

import: https://github.com/liascript/CodeRunner
        https://raw.githubusercontent.com/liaTemplates/ExplainGit/master/README.md
        https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md

narrator: DE

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/main/06_Git_I/presentation.md#1)

# TUBAF Bits&Bytes

I don't git it! - Einführung in die Versionsverwaltung mit Git
----------------------------------------------------------------

Donnerstag, 04.04.2024, 17 Uhr, RoboLab der TU Bergakademie Freiberg

---------------------


Prof. Dr. Sebastian Zug

> _Versionsmanagement und git sind für Sie ein "Buch mit sieben Siegeln"? Der Vortrag motiviert die Konzepte und erläutert deren Anwendung in konkreten Situationen der wissenschaftlichen Arbeit. Darüber hinaus wird anhand von Demos die Einbettung der Versionsverwaltung in das Projektmanagement praktisch vorgestellt._

## Warum überhaupt?

                                   {{0-1}}
******************************************************************************

> Frage: Was war das umfangreichste Dokument, an dem Sie bisher gearbeitet haben? Bei vielen sicher eine Hausarbeit oder Seminararbeit. Wie haben Sie Ihren Fortschritt organisiert?

******************************************************************************

                                   {{1-3}}
******************************************************************************

1. Im schlimmsten Fall haben Sie sich gar keine Gedanken gemacht und immer wieder ins gleiche Dokument geschrieben, das in einem Ordner liegt, der alle zugehörigen Dateien umfasst.
2. Eine Spur besser ist die Idee wöchentlich neue Kopien des Ordners anzulegen und diese in etwa so zu benennen:

   ```
   console
   ▶ ls
   myProject
   myProject_test
   myProject_newTest
   myProject_Moms_corrections
   ...
   ```

3. Wenn Sie "einen Plan hatte", haben Sie täglich eine Kopie aller Dateien in einem Ordner angelegt und diese systematisch benannt.

   ```console
   ▶ ls
   myProject_01042021
   myProject_02042021
   myProject_03042021
   ...
   ```

******************************************************************************


                          {{2-3}}
******************************************************************************

> Überlegen Sie sich kurz, wie Sie vorgehen müssen, um Antworten auf die folgenden Fragen zu finden:
>
> * "Wann wurde der letzte Stand der Datei x.y gelöscht?"
> * "In welcher Version habe ich die Anpassung der Überschriften vorgenommen?"
> * "Wie kann ich dies trotz anderer zwischenzeitlicher Änderungen rückgängig machen?"
> * "Warum habe ich davon keine Kopie gemacht?"
> * "..."

In jedem Fall viel manuelle Arbeit ...

******************************************************************************

                          {{3-4}}
******************************************************************************

Und nun übertragen wir den Ansatz auf eine Projekt mit vielen Mitstreitern. Die Herausforderungen potenzieren sich.

1. _Die Erstellung der Tageskopie müsste synchron erfolgen._
2. _Ich muss in die Ordner schauen, um zu sehen welche Anpassungen vorgenommen wurden._
3. _Ich weiß nicht welche die aktuelle Version einer Datei ist._
4. _Es existieren plötzlich mehrere Varianten einer Datei mit Änderungen an unterschiedlichen Codezeilen._
5. _Ich kann den Code nicht kompilieren, weil einzelne Dateien fehlen._
6. _Ich kann eine ältere Version der Software nicht finden - "Gestern hat es noch funktioniert"._
7. _Meine Änderungen wurden von einem Mitstreiter einfach überschrieben._

******************************************************************************

### Zielstellung




### Lösungsansätze

> _Eine Versionsverwaltung ist ein System, das zur Erfassung von Änderungen an Dokumenten oder Dateien verwendet wird. Alle Versionen werden in einem Archiv mit Zeitstempel und Benutzerkennung gesichert und können später wiederhergestellt werden. Versionsverwaltungssysteme werden typischerweise in der Softwareentwicklung eingesetzt, um Quelltexte zu verwalten._

https://de.wikipedia.org/w/index.php?title=Versionsverwaltung&action=history

![VersionsmanagementWikipedia](./images/VersionenVonVersionsverwaltung.png "Versionen des Artikels Versionsverwaltung auf der Webseite Wikipedia")

                                {{1-2}}
******************************************************************************

Hauptaufgaben:

+ Protokollierungen der Änderungen: Es kann jederzeit nachvollzogen werden, wer wann was geändert hat.
+ Wiederherstellung von alten Ständen einzelner Dateien: Somit können versehentliche Änderungen jederzeit wieder rückgängig gemacht werden.
+ Archivierung der einzelnen Stände eines Projektes: Dadurch ist es jederzeit möglich, auf alle Versionen zuzugreifen.
+ Koordinierung des gemeinsamen Zugriffs von mehreren Entwicklern auf die Dateien.
+ Gleichzeitige Entwicklung mehrerer Entwicklungszweige (engl. Branch) eines Projektes, was nicht mit der Abspaltung eines anderen Projekts (engl. Fork) verwechselt werden darf.

******************************************************************************

## Problemstellung

Das Beispiel entstammt dem Buch _Version Control with Subversion_ [^Subversion]

Zwei Nutzer (Harry und Sally) arbeiten am gleichen Dokument (A), das auf einem
zentralen Server liegt:

+ Beide führen verschiedene Änderungen an ihren lokalen Versionendes Dokuments durch.
+ Die lokalen Versionen werden nacheinander in das Repository geschrieben.
+ Sally überschreibt dadurch eventuell Änderungen von Harry.

> Die zeitliche Abfolge der Schreibzugriffe bestimmt welche Variante des Dokuments A überlebt -> __Konflikt__!

```ascii
+----------------------------+---------------------------+----------------------------+---------------------------+
|        Repository          |       Repository          |        Repository          |       Repository          |
|        +-------+           |       +-------+           |        +-------+           |       +-------+           |
|        |   A   |           |       |   A   |           |        |   A'  |           |       |  A''  |           |
|        +-------+           |       +-------+           |        +-------+           |       +-------+           |
|         /     \            |                           |         ^                  |              ^            |
|    read/       \read       |                           |   write/                   |               \write      |
|       /         \          |                           |       /                    |                \          |
|      v           v         |                           |      /                     |                 \         |
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     |
| |   A   |    |   A   |     | |   A'  |   |  A''  |     | |   A'  |    |  A''  |     | |   A'  |   |  A''  |     |
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     |
|   Harry        Sally       |   Harry       Sally       |   Harry        Sally       |   Harry       Sally       |
|                            |                           |                            |                           |
| Erzeugen der lokalen Kopie | Bearbeitung               |Harry schreibt seine Version| Sally übermittelt A''     |
+----------------------------+---------------------------+----------------------------+---------------------------+
```

### Lösung: Koordinierter Zugriff

                                {{0-2}}
******************************************************************************

**Lösung I - Exklusives Bearbeiten (Sequenzialisierung)**

> Bei der pessimistische Versionsverwaltung (*Lock Modify Unlock*) werden einzelne Dateien vor einer Änderung durch den Benutzer gesperrt und nach Abschluss der Änderung wieder freigegeben werden.

```ascii
+----------------------------+---------------------------+----------------------------+---------------------------+
|        Repository          |       Repository          |        Repository          |       Repository          |
|        ╔═══════╗           |       ╔═══════╗           |        +-------+           |       +-------+           |
|        ║   A   ║  locked   |       ║   A   ║  locked   |        |   A'  |           |       |  A''  |           |
|        ╚═══════╝           |       ╚═══════╝           |        +-------+           |       +-------+           |
|         ^/                 |             X             |         ^^                 |             ^\            |
|    lock//read              |              \lock        |   write//unlock            |         lock \\read       |
|       //                   |               \           |       //                   |               \\          |
|      /v                    |                \          |      //                    |                \v         |
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     |
| |   A   |    |       |     | |   A'  |   |  A''  |     | |   A'  |    |  A''  |     | |   A'  |   |   A'  |     |
| +-------+    +-------+     | +-------+   +-------+     | +-------+    +-------+     | +-------+   +-------+     |
|   Harry        Sally       |   Harry       Sally       |   Harry        Sally       |   Harry       Sally       |
|                            |                           |                            |                           |
| Harry "locks", kopiert und | Sallys lock request wird  | Harry übermittelt seine    | Sally blockiert und liest |
| beginnt die Bearbeitung    | blockiert                 | Version und löst den Lock  | die neue Version          |
+----------------------------+---------------------------+----------------------------+---------------------------+
```

> __Frage:__ Welche Aspekte sehen Sie an dieser Lösung kritisch?

******************************************************************************

                          {{1-2}}
*************************************************************

1. Administrative Probleme ... Gesperrte Dokumente werden vergessen zu entsperren.
2. Unnötige Sequentialisierung der Arbeit ... Wenn zwei Nutzer ein Dokument an verschiedenen Stellen ändern möchten, könnten sie dies auch gleichzeitig tun.
3. Keine Abbildung von übergreifenden Abhängigkeiten ... Zwei Nutzer arbeiten getrennt auf den Dokumenten A und B. Was passiert, wenn A von B abhängig ist? A und B passen nicht mehr zusammen. Die Nutzer müssen dieses Problem diskutieren.

*************************************************************


                          {{2-3}}
******************************************************************************

__Lösung II - Kollaboratives Arbeiten mit Mischen (Mergen)__

> Optimistische Versionsverwaltungen (*Copy Modify Merge*) versuchen die die Schwächen der pessimistischen Versionsverwaltung zu beheben, in dem sie gleichzeitige Änderungen durch mehrere Benutzer an einer Datei zu lassen und anschließend diese Änderungen automatisch oder manuell zusammen führen (Merge).

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
******************************************************************************


                                {{3-4}}
******************************************************************************

Welche Konsequenzen ergeben sich daraus?

+ Unser Dokument muss überhaupt kombinierbar sein! Auf ein binäres Format ließe sich das Konzept nicht anwenden!
+ Das Dokument liegt in zeitgleich in n-Versionen vor, die ggf. überlappende Änderungen umfassen.
+ Das zentrale Repository kennt die Version von Harry nur indirekt. Man kann zwar indirekt aus A'' und A* auf A' schließen, man verliert aber zum Beispiel die Information wann Harry seine Änderungen eingebaut hat.

> Die Herausforderung liegt somit im Mischen von Dokumenten!

******************************************************************************

[^Subversion]: Brian W. Fitzpatrick, Ben Collins-Sussman, C. Michael Pilato, Version Control with Subversion, 2nd Edition, O'Reilly Media


### Lösung: Mischen von Dokumenten

                             {{0-1}}
******************************************************************************

**Schritt 1: Identifikation von Unterschieden**

Zunächst einmal müssen wir feststellen an welchen Stellen es überhaupt Unterschiede
gibt. Welche Differenzen sehen Sie zwischen den beiden Dokumenten:

```markdown                      DokumentV1.md
TU
Bergakademie
Freiberg
Softwareentwicklung
Online Course
Sommersemester 2020
Lorem ipsum dolor sit amet, CONSETETUR sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.
```

```markdown                      DokumentV2.md



TU
Bergakademie
Freiberg
Softwareentwicklung
Sommersemester 2019
Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.
```

Offenbar wurden sowohl Lehrzeichen, als auch neue Zeilen eingeführt. In anderen Zeilen wurden Inhalte angepasst.

Nutzen wir das Tool `diff` um diese Änderungen automatisiert festzustellen.

```console
▶diff DokumentV1.md DokumentV2.md
0a1,3
>
>
>
5,7c8,9
< Online Course
< Sommersemester 2020
< Lorem ipsum dolor sit amet, CONSETETUR sadipscing elitr, ...
---
> Sommersemester 2019
> Lorem ipsum dolor sit amet, consetetur sadipscing elitr, ...
```

> **Merke**: Sehr lange Zeilen erschweren die Suche nach wirklichen Änderungen!

> Dahinter steht das _Longest Common Subsequence_ Problem, auf die Herleitung verzichten wir an dieser Stelle :-)


******************************************************************************

                             {{0-1}}
******************************************************************************

**Schritt 2: Mischen**

In der Praxis wird zwischen zwei Szenarien unterschieden:

1. Mischen unabhängiger Dokumente (2-Wege-Mischen) - Ziel ist die Erzeugung eines neuen Dokumentes, dass die gemeinsamen Komponenten und individuelle Teilmengen vereint.

2. Mischen von Dokumenten mit gemeinsamen Ursprung (3-Wege-Mischen) - Ziel ist die Integration möglichst aller Änderungen der neuen Dokumente in eine weiterentwickelte Version des Ursprungsdokumentes

> Ein Paar von Änderung aus D1 bzw. D2 gegenüber einen Ausgangsdokument D0 kann unverträglich sein, wenn die Abbildung beider Änderungen in einem gemeinsamen Dokument nicht möglich ist. In diesem Fall spricht man von einem Konflikt.

Bei einem Konflikt muss eine der beiden ̈Änderungen weggelassen werden. Die Entscheidung darüber kann anhand von zwei Vorgehensweisen realisiert werden:

1. Nicht-interaktives Mischen: Es wird zunächst ein Mischergebnis erzeugt, das beide Änderungen umfasst. Über eine entsprechende Semantik werden die notwendigerweise duplizierten Stellen hervorgehoben. Ein Vorteil dieser Vorgehensweise ist, dass ein beliebiges weitergehendes Editieren zur Konfliktauflösung möglich ist.
2. Interaktives Mischen: Ein Entwickler wird unmittelbar in den Mischprozess eingebunden und um "Schritt-für-Schritt" Entscheidungen gebeten. Denkbare Entscheidungen dabei sind:

    + Übernahme der Änderung gemäß D1 oder D2,

    + Übernahme keiner Änderung,

    + Übernahme von modifizierten Änderung

******************************************************************************

### Lösung: Dateisystem

> Bislang haben wir lediglich einzelne Dateien betrachtet. Logischerweise muss ein übergreifender Ansatz auch Ordnerstrukturen integrieren.

![ProblemKollaborativesArbeiten](./images/Versionsverlauf.png)

Damit werden sowohl die Ordnerstruktur als auch die Dokumente als Struktur, wie auch deren Inhalte, erfasst.

> Wichtig für die Nachvollziehbarkeit der Entwicklung ist somit die Kontinuität der Erfassung!

Wenn sich der Ordner- oder Dateiname ändert wollen wir trotzdem noch die gesamte History der Entwicklung innerhalb eines Dokuments kennen. Folglich muss ein Link zwischen altem und neuem Namen gesetzt werden.

## Formen der Versionsverwaltung

**Lokale Versionsverwaltung**
Bei der lokalen Versionsverwaltung wird oft nur eine einzige Datei versioniert, diese Variante wurde mit Werkzeugen wie SCCS und RCS umgesetzt. Sie findet auch heute noch Verwendung in Büroanwendungen, die Versionen eines Dokumentes in der Datei des Dokuments selbst speichern (Word).

**Zentrale Versionsverwaltung**
Diese Art ist als Client-Server-System aufgebaut, sodass der Zugriff auf ein Repository auch über Netzwerk erfolgen kann. Durch eine Rechteverwaltung wird dafür gesorgt, dass nur berechtigte Personen neue Versionen in das Archiv legen können. Die Versionsgeschichte ist hierbei nur im Repository vorhanden.

Dieses Konzept wurde vom Open-Source-Projekt Concurrent Versions System (CVS) populär gemacht, mit Subversion (SVN) neu implementiert und von vielen kommerziellen Anbietern verwendet.

<!--
style="width: 100%; max-width: 560px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                                +-----------------+
                                | V 21.09         |
                              +-----------------+ |
                              | V 21.10         | |
              Zentrales     +-----------------+ | |
              Repository    | V 21.11         | | |
                            |                 | |-+
                            |                 | |
                            |                 |-+
                            |                 |
                            +-----------------+
                                    |
          +-------------------------+--------------------------+
          |                         |                          |
  +-----------------+       +-----------------+       +-----------------+
  | V 21.11         |       | V 21.11         |       | V 21.11         |
  | ABCD            |       | GEFH            |       | IKLM            |
  |                 |       |                 |       |                 |
  +-----------------+       +-----------------+       +-----------------+
    User 1                    User 2                    User 3
    Lokale Kopien
```


**Verteilte Versionsverwaltung**
Die verteilte Versionsverwaltung (DVCS, distributed VCS) verwendet kein zentrales Repository mehr. Jeder, der an dem verwalteten Projekt arbeitet, hat sein eigenes Repository und kann dieses mit jedem beliebigen anderen Repository abgleichen. Die Versionsgeschichte ist dadurch genauso verteilt. Änderungen können lokal verfolgt werden, ohne eine Verbindung zu einem Server aufbauen zu müssen.

<!--
style="width: 100%; max-width: 560px; display: block; margin-left: auto; margin-right: auto;"
-->
```ascii
                                +-----------------+
                                | V 21.09         |
                              +-----------------+ |
                              | V 21.10         | |
              Zentrales     +-----------------+ | |
              Repository    | V 21.11         | | |
                            |                 | |-+
                            |                 | |
                            |                 |-+
                            |                 |
                            +-----------------+
                                    |
          +-------------------------+--------------------------+
          |                         |                          |
    +-----------------+      +-----------------+         +-----------------+
    | V 21.09         |      | V 21.09         |         | V 21.09         |
  +-----------------+ |    +-----------------+ |       +-----------------+ |
  | V 21.10         | |    | V 21.10         | |       | V 21.10         | |
+-----------------+ | |  +-----------------+ | |     +-----------------+ | |
| V 21.11         | | |  | V 21.11         | | |     | V 21.11         | | |
|                 | |-+  |                 | |-+   +-----------------+ | |-+
|                 | |    |                 | |     | V 21.12         | | |
|                 |-+    |                 |-+     |                 | |-+
|                 |      |                 |       |                 | |
+-----------------+      +-----------------+       |                 |-+
         |                        |                +-----------------+
         |                        |                          |
         |                        |                          |
+-----------------+       +-----------------+         +-----------------+
| V 21.11         |       | V 21.09         |         | V 21.12         |
| ABCD            |       | GEFH            |         | IKLM            |
|                 |       |                 |         |                 |
+-----------------+       +-----------------+         +-----------------+
  User 1                    User 2                      User 3

```

| Zentrale Versionsverwaltung                                       | Verteilte Versionsverwaltung                                |
| ----------------------------------------------------------------- | ----------------------------------------------------------- |
| Historie liegt nur auf dem Server                                 | gesamte Historie ist den lokalen Repositiories bekannt              |
| Zentrales Repository als Verbindungselement                       | n gleichberechtigte Repositories                            |
| Konflikte bei Manipulation eines Dokumentes durch mehrere Autoren | Existenz paralleler  Versionen eines Dokumentes abgesichert |
| Sequenz von Versionen                                             | gerichteter azyklischer Graph                               |

## Git

                     {{0-1}}
******************************************************

**Geschichte und Einsatz**

Die Entwicklungsgeschichte von git ist mit der des Linux Kernels verbunden:

| Jahr | Methode der Versionsverwaltungen                               |
| ---- | -------------------------------------------------------------- |
| 1991 | Änderungen am Linux Kernel via patches und archive files       |
| 2002 | Linux Kernel mit dem Tool BitKeeper verwaltet                  |
| 2005 | Bruch zwischen der vertreibenden Firma und der Linux Community |
| 2024 | Die aktuelle Version ist 2.44.1                                |

2005 wurde einen Anforderungsliste für eine Neuentwicklung definiert. Dabei wurde hervorgehoben, dass sie insbesondere sehr große Projekte (Zahl der Entwickler, Features und Codezeilen, Dateien) unterstützen können muss. Daraus entstand `Git` als freie Software zur verteilten Versionsverwaltung von Dateien.

> Git dominiert entweder als einzelne Installation oder aber eingebettet in verschiedene Entwicklungsplattform die Softwareentwicklung!

******************************************************

                     {{1-2}}
******************************************************

**Sprachwirrwar**

> 'Git' vs 'GitLab' vs 'GitHub' ... was ist der Unterschied


******************************************************

                     {{2-3}}
******************************************************

**Wie bekommen sie Git auf Ihren Windows-Rechner?**

1. Variante 1: ... in der Konsole
 
2. Variante 2: ... als separate (GUI-Anwendung)[https://git-scm.com/downloads/guis]

3. Variante 3: ... eingebettet in die "Entwicklungsumgebung"

!?[Tutorial](https://www.youtube.com/watch?v=2j7fD92g-gE)

******************************************************


### Zustandsmodell einer Datei in Git

Dateien können unterschiedliche Zustände haben, mit denen sie in Git-Repositories markiert sind.

                       {{0-1}}
********************************************************************************

```text @plantUML.png
@startuml
hide empty description
[*] --> Untracked : Erzeugen einer Datei
Untracked --> Staged : Hinzufügen zum Repository
Unmodified --> Modified : Editierung der Datei
Modified --> Staged : Markiert als neue Version
Staged --> Unmodified : Bestätigt als neue Version
Unmodified --> Untracked : Löschen aus dem Repository
@enduml
```

********************************************************************************

                        {{1-2}}
********************************************************************************

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

### Grundlegende Anwendung (lokal!)

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

Wie sieht aber die dahinterliegende Datenstruktur aus?

``` text @ExplainGit.eval
git commit -m V1
git commit -m V2
git commit -m V3
```

### "Kommando zurück"

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

Ein sehr mächtiges Werkzeug ist der interaktive Modus von `git rebase`. Damit kann die
Geschichte neugeschrieben werden, wie es die git Dokumentation beschreibt. Im Grund können Sie damit Ihre Versionsgeschichte "aufräumen". Einzelne Commits umbenennen, löschen oder fusionieren. Dafür besteht ein eigenes Interface, dass Sie mit dem folgenden Befehl aufrufen können:

```console
▶git rebase -i HEAD~5

pick d2a06e4 Update main.yml
pick 78839b0 Reconfigures checkout
pick f70cfc7 Replaces wildcard by specific filename
pick 05b76f3 New pandoc command line
pick c56a779 Corrects md filename

# Rebase a3b07d4..c56a779 onto a3b07d4 (5 commands)
#
# Commands:
# p, pick = use commit
# r, reword = use commit, but edit the commit message
# e, edit = use commit, but stop for amending
# s, squash = use commit, but meld into previous commit
# f, fixup = like "squash", but discard this commit's log message
# x, exec = run command (the rest of the line) using shell
# d, drop = remove commit
#
# These lines can be re-ordered; they are executed from top to bottom.
#
# If you remove a line here THAT COMMIT WILL BE LOST.
#
# However, if you remove everything, the rebase will be aborted.
#
# Note that empty commits are commented out
```

Als Anwendungsfall habe ich mir meine Aktivitäten im Kontext einiger Experimente
mit den GitHub Actions, die im nächsten Abschnitt kurz eingeführt werden, ausgesucht.
Schauen wir zunächst auf den ursprünglichen Stand. Alle Experimente drehten sich darum, eine Datei anzupassen und dann auf dem Server die Korrektheit zu testen.

> Wir werden `git rebase` im Zusammenhang mit der Arbeit in  Branches wieder aufgreifen.

### Was kann schief gehen?

1. **Ups, die Datei sollte im Commit nicht dabei sein (Unstage)**

```
... ein neues Feature wird in `sourceA.cs` implementiert
... ein Bug in Codedatei `sourceB.cs` korrigiert
... wir wollen schnell sein und fügen alle Änderungen als staged ein
git add *
... Aber halt! Die beiden Dinge gehören doch nicht zusammen!
git reset
```

2. **Ups, eine Datei in der Version vergessen! (Unvollständiger Commit)**

```
... eine neue Codedatei source.cs anlegen
... zugehörige Anpassungen in der README.md erklären
git add README.md
git commit -m "Hinzugefügt neues Feature"
... Leider wurde die zugehörige Code Datei vergessen!
git add source.cs
git commit --amend --no-edit
... Hinzufügen der Datei ohne die Log Nachricht anzupassen
```

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