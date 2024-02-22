<!--
author:   Sebastian Zug

email:    sebastian.zug@informatik.tu-freiberg.de

version:  0.0.1

language: en

narrator: DE

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/main/02_LatexUndWord/presentation.md#1)

# TUABF Bits&Bytes

Word vs. Latex - Ein kritischer Vergleich
----------------------------------------------------------------

Donnerstag, 22.02.2024, 17 Uhr, RoboLab der TU Bergakademie Freiberg

---------------------


Prof. Dr. Sebastian Zug

> _Im zweiten Bits & Bytes Vortrag adressiert Sebastian Zug einen wichtigen Aspekt der wissenschaftlichen Arbeit, die Werkzeuge für das Schreiben von Publikationen. Dabei gehen die Meinungen zur idealen Umgebung stark auseinander - die (vermeintlichen) Vor- und Nachteile der Standardtextverarbeitungstools (Open Office Writer, Microsoft Word) und des Textsatzsystems Latex werden kontrovers diskutiert. Der Vortrag strukturiert die Eigenschaften und gibt Empfehlungen für bestimmte Einsatzzwecke. Der Fokus liegt dabei auf Latex-Einsteigern._

## Motivation

```
Das Entenhausener Finger-Problem

Autoren Micky Maus & Donald Duck

Entenhausen Universität

In diesem Paper Paper wird die Frage beantwortet, warum alle Bewohner von 
Entenhausen nur vier Finger an jeder Hand haben.

1. Einleitung und Forschungsfrage

In den 30er Jahre wurden die Figuren von 
+ Mickey Maus [1]
+ Donald Duck  [2]
+ Goofy [3] usw. 
von Walt Disney und Ub Iwerks geschaffen. Eine Besonderheit der Figuren ist, 
dass alle nur vier Finger an jeder Hand haben. Die Forschungsfrage, die in 
diesem Paper beantwortet werden soll, ist, warum das so ist.

https://vectorportal.com/de/vector/micky-maus-vektorgrafiken.ai/2595
"Beweisfoto" von Micky Maus

2. Erklärungsansatz

In den frühen Tagen der Animation war es üblich, Figuren mit weniger Fingern zu 
zeichnen, da es schneller ging und die Bewegungen flüssiger wirken ließ. Diese 
Stilisierung hat sich über die Jahre hinweg fortgesetzt obwohl Comics heute digital 
erstellt werden.

3. Referenzen

[1] https://de.wikipedia.org/wiki/Micky_Maus
[2] https://de.wikipedia.org/wiki/Donald_Duck
[3] https://de.wikipedia.org/wiki/Goofy
```

> Unser Text überlagert Inhalte mit Struktur und Formatierung!

### Office Lösung

"Live Demo"

![](./images/Screenshot_Word.jpg)

> Was macht das hier gezeigte Libre Office anders als unser "reiner" Text?

### Und wie sah das früher aus?

Die nachfolgende Virtualisierung zeigt die erste Version von Microsoft Word aus dem Jahr 1983.

<iframe style="width: 100%; max-width: 900px; height: 70vh" src="https://www.pcjs.org/software/pcx86/app/microsoft/word/1.15/"></iframe>

Erkenntnisse: 

+ Viele Formatierung und Strukturierungsmöglichkeiten kennen Sie aus Ihrer täglichen Arbeit
+ Ganze Standardwerkzeuge fehlen einfach (z.B. Tabellen, Bilder, mathematische Notationen, ...)

+ Die Maus fehlt und die Bedienung über Tastatur ist sehr gewöhnungsbedürftig
+ Die Formatierungen und Strukturierungsmöglichkeiten sind nicht immer offensichtlich (fehlendes [WYSIWYG](https://de.wikipedia.org/wiki/WYSIWYG))


## Latex Lösung

> Lassen Sie uns die Formatierung doch gleich in den Text einbauen! 

```latex
\documentclass{article}
\begin{document}
Hello, World!
\end{document}
```

> __Latex kombiniert Struktur und Formatierung explizit in einem Dokument! Aus dem WYSIWYG wird ein _What You See Is What You Mean_ [WYSIWYM](https://de.wikipedia.org/wiki/WYSIWYM).__

### Historie

+ Ausgangspunkt ist das Textsatzsystem von [Donald Knuth](https://de.wikipedia.org/wiki/Donald_E._Knuth) aus dem Jahr 1977,
+ darauf aufbauend entwickelte [Leslie Lamport](https://de.wikipedia.org/wiki/Leslie_Lamport) Anfang der 1980er Jahre LaTeX, eine Sammlung von TeX-Makros, die die Benutzung für den durchschnittlichen Anwender gegenüber TeX vereinfachten und erweiterten
+ LaTeX wurde schnell bei Wissenschaftlern, Forschern und Akademikern beliebt, insbesondere in mathematischen und technischen Disziplinen   

!?[](https://www.youtube.com/watch?v=9eLjt5Lrocw&t=172s)

### Ja,  aber ....

Brauche ich denn im Angesicht von interaktiven Office Textverarbeitungstools überhaupt noch LaTeX?

![](./images/Screenshot_Word.jpg)<!-- width="60%" -->

> Welche Probleme haben Sie bei der Nutzung einer WYSIWYG Textverarbeitung?

## Voraussetzungen

> Wie kann ich mit Latex arbeiten?

Variante 1: Webbasierte Tools

+ [Overleaf](https://www.overleaf.com/)
+ [LatexBase](https://latexbase.com/d/9a0d6d87-3f0b-4f80-b832-a8d41e16d411)
+ ...

Variante 2: Lokale Installation

+ ein Latex-Textsatzsystem (z.B. [MiKTeX](https://miktex.org/))
+ eine Entwicklungsumgebung die Sie bei der Arbeit mit den Dokumenten unterstützt (z.B. [Texmaker Studio](https://www.texstudio.org/))

!?["Erläuterungen zum Installationsprozess mit MikTeX"](https://www.youtube.com/watch?v=aTOfbfJvUig)

## Latex im Einsatz

Lassen Sie unser Paperbeispiel in Latex umsetzen.

```latex
\documentclass{article}
\begin{document}
Hello, World!
\end{document}
```

Das Ausgangsdokument finden Sie unter [diesem Link](XXXX), das finale pdf [hier](XXXX).

## Fazit

{{0-1}}
![https://commons.wikimedia.org/wiki/File:LaTeX-Word-Graph.svg](https://upload.wikimedia.org/wikipedia/commons/9/9f/LaTeX-Word-Graph.svg " LaTeX vs Office X; Mühe und Zeitbeanspruchung aufgetragen gegen Komplexität und Größe des Dokuments (Autor: Matthias M. https://commons.wikimedia.org/wiki/File:LaTeX-Word-Graph.svg)")

{{1-2}}
| Feature               | Open Office Writer / Microsoft Word    | LaTeX                                                                              |
| --------------------- | -------------------------------------- | ---------------------------------------------------------------------------------- |
| Preis                 | Open-Source / Kostenpflichtig          | Kostenlos, Open-Source                                                             |
| Benutzeroberfläche    | WYSIWYG (What You See Is What You Get) | Markup-Sprache (Code-basiert)                                                      |
| Formatierung          | Einfach über Menüs und Werkzeugleisten | Durch Befehle und Pakete in LaTeX                                                  |
| Tabellen              | Ja                                     | Ja                                                                                 |
| Bildverwaltung        | Ja                                     | Ja                                                                                 |
| Bibliographie         | beschränkt                             | Ja                                                                                 |
| Mathematische Formeln | Ja ( mit Latex-Notation :-))           | Sehr stark ausgeprägt und erweitert                                                |
| Typografie            | "Man kann alles falsch machen"         | Hohe Kontrolle über Typografie                                                     |
| Versionskontrolle     | Begrenzte Unterstützung                | Gut unterstützt mit Versionierungstools wie Git                                    |
| Kompatibilität        | Sehr gut, weit verbreitet              | Erfordert spezielle Software für Anzeige und Bearbeitung (z.B. TeX-Distributionen) |