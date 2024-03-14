<!--
author:   Sebastian Zug

email:    sebastian.zug@informatik.tu-freiberg.de

version:  0.0.3

language: en

logo:     https://github.com/SebastianZug/RoboLabVortraege/blob/main/02_LatexUndWord/images/Logo.jpg?raw=true

import:   https://raw.githubusercontent.com/LiaTemplates/DigiSim/master/README.md
          https://raw.githubusercontent.com/liascript-templates/plantUML/master/README.md

comment:  "Word vs. Latex - Ein kritischer Vergleich" systematisiert die konzeptionellen Unterschiede und führt in einem Live-Tutorial in die Grundlagen von Latex ein.

narrator: DE

mark: <span style="background-color: @0;
                                  display: flex;
                                  width: calc(100% + 32px);
                                  margin: -16px;
                                  padding: 6px 16px 6px 16px;
                                  ">@1</span>
red:  @mark(#FF888888,@0)
blue: @mark(lightblue,@0)
gray: @mark(gray,@0)

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/main/04_CPU_Aufbau/presentation.md#1)

# TUBAF Bits&Bytes

Wie funktioniert eigentlich eine CPU
----------------------------------------------------------------

Donnerstag, 14.03.2024, 17

Prof. Dr. Sebastian Zug

> _Der Vortrag erläutert die Funktionsweise der Befehlsabarbeitung in einem Prozessor. Prof. Dr. Zug zeigt, dass keine "Magie" in diesem Kern des Rechners steckt, sondern dieser sich grundlegend aus einfachen Komponenten zusammensetzt. Dazu führt er kurz die Basisgatter, die Realisierung von Grundschaltungen und die Umsetzung eines Modellrechners ein._

## Motivation

> Was ist eine CPU?

_Ein Computer-Prozessor ist ein (meist stark verkleinertes und meist frei) programmierbares Rechenwerk, also eine elektronische Schaltung, die gemäß übergebenen Befehlen Aktionen ausführt, wie andere elektronische Schaltungen und Mechanismen zu steuern. Es handelt sich dabei um eine hochkomplexe Form integrierter Schaltkreise (ICs) ..._

_Als Rechenwerk (auch Ausführungseinheit, englisch execution unit)[1][2] oder Operationswerk bezeichnet man in der Mikroelektronik und technischen Informatik ein Schaltwerk zur Ausführung der Maschinenbefehle eines Computerprogramms._

> Ganz schön sperrig?

## Komponenten

                      {{0-1}}
****************************************************************

![Meda42](./images/Circut.png)<!-- width="60%" -->

Wir betrachten den Schalter mit seinen 2 Zuständen als Input und die Glühlampe als Output. Das Übergangsverhalten wird ignoriert.

| Input                | Output               |
| -------------------- | -------------------- |
| Schalter geschlossen | Lampe leuchtet       |
| Schalter offen       | Lampe leuchtet nicht |

****************************************************************
                      {{1-2}}
****************************************************************
Für zwei Schalter (Inputs) lassen sich darauf aufbauend zwei grundlegende Schaltungsmuster entwerfen:

| Reihenschaltung | Parallelschaltung |
| --------------- | ----------------- |
| ![Meda42](./images/CircutSerial.png) <!-- width="80%" -->            | ![Meda42](./images/CircutParallel.png) <!-- width="80%" -->                    |
| Die Lampe leuchtet, wenn der erste und der zweite Schalter geschlossen werden               | Lampe leuchtet, wenn der erste oder der zweite Schalter geschlossen. wird.            |

****************************************************************

                      {{2-3}}
****************************************************************

![Schalter](./images/CircutComplex.png) <!-- width="80%" -->

> Es gibt verschiedene Lösungen, um die Lampe mit drei geschlossenen Schaltern zum Leuchten zu bringen. Wie viele? Wie viele Kombinationen von Schalterbelegungen sind möglich?

Dazu beschreiben wir die Wirkung des elektrischen Stromes

+ Stromfluss / kein (oder ein sehr geringer) Stromfluss
+ Spannung / keine (oder eine sehr geringe) Spannung

... aus Sicht der Logik anhand von Zuständen

+ an / aus
+ wahr / falsch
+ 1 / 0
+ 0 / 1

****************************************************************

### Schaltnetze/Schaltfunktionen

                       {{0-1}}
********************************************************

Idee
----------------------------

> Funktionen $f:\{0,1\}^n \rightarrow \{0,1\}^m$ mit $n, m \geq 1$ werden auch als Schaltfunktionen bezeichnet

Beispiel

<!-- data-type="none" style="table-layout: fixed; max-width:350px;"-->
| $x_1$ | $x_0$ | $y$ |
| ----- | ----- | --- |
| 0     | 0     | 0   |
| 0     | 1     | 1   |
| 1     | 0     | 1   |
| 1     | 1     | 1   |

![Meda42](./images/CircutParallel.png)

> __Frage:__ Welche Logische Verknüpfung beschreibt die Schaltfunktion $f(x_1, x_0)$?

> Die Schaltfunktion $f$ ist durch die Tabelle eindeutig definiert. Sie beschreibt die Wirkung eines Schaltnetzes.

********************************************************



                       {{1-2}}
********************************************************

Multiplexer / Demultiplexer
----------------------------

Eine Mulitplexerschaltung bildet analoge oder digitale Eingangssignale auf einen Kommunikationskanal ab, der Demultiplexer übernimmt die Abbildung auf n Ausgangsleitungen.

<!--
style="width: 80%; min-width: 420px; max-width: 720px;"
-->
```ascii

    Multiplexer             Demultiplexer
        +                        +
------> |\                      /|------> "$y_0$"
        | \                    / |
------> |  +        "$x$"     +  |------> "$y_1$"
        |  | ................>|  |
------> |  +                  +  |------> "$y_2$"
        | /                    \ |
------> |/|                    |\|------> "$y_3$"
        +||                    ||+
         ||                    ||
       "$a_0$ $a_1$"         "$a_0$ $a_1$"                                       
                                                     .
```

**Beispiel: 2 Bit Adresse -> 4 Ausgänge**

<!-- data-type="none" -->
| $a_0$ | $a_1$ | $x$ | $y_0$ | $y_1$ | $y_2$ | $y_3$ |
|-------|-------|-----|-------|-------|-------|-------|
| 0     | 0     | 0   |       |       |       |       |
| 0     | 0     | 1   | 1     |       |       |       |
| 0     | 1     | 0   |       |       |       |       |
| 0     | 1     | 1   |       | 1     |       |       |
| 1     | 0     | 0   |       |       |       |       |
| 1     | 0     | 1   |       |       | 1     |       |
| 1     | 1     | 0   |       |       |       |       |
| 1     | 1     | 1   |       |       |       | 1     |

$y_0 = x \cdot \overline{a_1} \cdot \overline{a_0}$

$y_1 = x \cdot \overline{a_1} \cdot a_0$

$y_2 = x \cdot a_1 \cdot \overline{a_0}$

$y_3 = x \cdot a_1 \cdot a_0$

```json @DigiSim.evalJson
{"devices":{"x":{"label":"x","type":"Button","propagation":0,"position":{"x":-50,"y":0}},"a0":{"label":"a0","type":"Button","propagation":0,"position":{"x":-45,"y":185}},"a1":{"label":"a1","type":"Button","propagation":0,"position":{"x":-50,"y":100}},"y0":{"label":"y0","type":"Lamp","propagation":0,"position":{"x":495,"y":15}},"y1":{"label":"y1","type":"Lamp","propagation":0,"position":{"x":495,"y":75}},"y2":{"label":"y2","type":"Lamp","propagation":0,"position":{"x":495,"y":130}},"y3":{"label":"y3","type":"Lamp","propagation":0,"position":{"x":500,"y":185}},"not1":{"label":"~a0","type":"Not","propagation":0,"bits":1,"position":{"x":85,"y":225}},"not2":{"label":"~a1","type":"Not","propagation":0,"bits":1,"position":{"x":45,"y":110}},"and1":{"label":"x and ~a1","type":"And","propagation":0,"bits":1,"position":{"x":150,"y":55}},"and2":{"label":"x and a1","type":"And","propagation":0,"bits":1,"position":{"x":160,"y":155}},"and3":{"label":"x and ~a1 and ~a0","type":"And","propagation":0,"bits":1,"position":{"x":345,"y":10}},"and4":{"label":"x and ~a1 and a0","type":"And","propagation":0,"bits":1,"position":{"x":345,"y":70}},"and5":{"label":"x and a1 and ~a0","type":"And","propagation":0,"bits":1,"position":{"x":345,"y":125}},"and6":{"label":"x and a1 and a0","type":"And","propagation":0,"bits":1,"position":{"x":350,"y":180}}},"connectors":[{"from":{"id":"not2","port":"out"},"to":{"id":"and1","port":"in2"}},{"from":{"id":"x","port":"out"},"to":{"id":"and1","port":"in1"}},{"from":{"id":"x","port":"out"},"to":{"id":"and2","port":"in1"},"vertices":[{"x":85,"y":75},{"x":110,"y":115}]},{"from":{"id":"and1","port":"out"},"to":{"id":"and3","port":"in1"}},{"from":{"id":"and1","port":"out"},"to":{"id":"and4","port":"in1"}},{"from":{"id":"and2","port":"out"},"to":{"id":"and5","port":"in1"},"vertices":[{"x":285,"y":135}]},{"from":{"id":"and2","port":"out"},"to":{"id":"and6","port":"in1"}},{"from":{"id":"not1","port":"out"},"to":{"id":"and3","port":"in2"},"vertices":[{"x":290,"y":225},{"x":290,"y":155}]},{"from":{"id":"not1","port":"out"},"to":{"id":"and5","port":"in2"},"vertices":[{"x":285,"y":200}]},{"from":{"id":"and3","port":"out"},"to":{"id":"y0","port":"in"}},{"from":{"id":"and4","port":"out"},"to":{"id":"y1","port":"in"}},{"from":{"id":"and5","port":"out"},"to":{"id":"y2","port":"in"}},{"from":{"id":"and6","port":"out"},"to":{"id":"y3","port":"in"}},{"from":{"id":"a1","port":"out"},"to":{"id":"not2","port":"in"}},{"from":{"id":"a1","port":"out"},"to":{"id":"and2","port":"in2"},"vertices":[{"x":35,"y":145},{"x":65,"y":185}]},{"from":{"id":"a0","port":"out"},"to":{"id":"not1","port":"in"}},{"from":{"id":"a0","port":"out"},"to":{"id":"and4","port":"in2"},"vertices":[{"x":305,"y":200}]},{"from":{"id":"a0","port":"out"},"to":{"id":"and6","port":"in2"}}],"subcircuits":{}}
```

********************************************************

### Arithmetische Einheit 

                       {{0-1}}
********************************************************

> Mit binären Zahlen kann wie mit dezimalen Zahlen addiert werden.

<!-- data-type="none" style="table-layout: fixed; max-width:1000px;"-->
| Bit       | $3$        | $2$        | $1$        | $0$           |                |
| --------- | ---------- | ---------- | ---------- | ------------- | -------------- |
|           | $1$        | $0$        | $0$        | $1$           | A $(9)_{10}$   |
| +         | $0$        | $0$        | $1$        | $1$           | B $(3)_{10}$   |
| @red($0$) | @blue($0$) | @blue($0$) | @blue($1$) | @blue( $1$  ) | @blue($Carry$) |
|           | $1$        | $1$        | $0$        | $0$           | R $(12)_{10}$  |

Die Addition zweier positiver n-stelliger Binärzahlen $a$ und $b$ kann stellenweise von rechts nach links durchgeführt werden. In jeder Stelle $i$ kann ein Übertrag $c_i = 1$ auftreten („Carry“).

********************************************************

                       {{1-2}}
********************************************************

<!-- data-type="none" -->
| $A$ | $B$ | $S$ | $C$ |
| --- | --- | --- | --- |
| 0   | 0   | 0   | 0   |
| 0   | 1   | 1   | 0   |
| 1   | 0   | 1   | 0   |
| 1   | 1   | 0   | 1   |

```json @DigiSim.evalJson
{"devices":{"a":{"label":"a","type":"Button","propagation":0,"position":{"x":15,"y":0}},"b":{"label":"b","type":"Button","propagation":0,"position":{"x":15,"y":50}},"s":{"label":"s","type":"Lamp","propagation":1,"position":{"x":315,"y":45}},"cout":{"label":"cout","type":"Lamp","propagation":1,"position":{"x":315,"y":0}},"xor":{"label":"a xor b","type":"Xor","propagation":1,"bits":1,"position":{"x":160,"y":50}},"and":{"label":"a and b","type":"And","propagation":1,"bits":1,"position":{"x":155,"y":-5}}},"connectors":[{"from":{"id":"a","port":"out"},"to":{"id":"and","port":"in1"}},{"from":{"id":"b","port":"out"},"to":{"id":"and","port":"in2"}},{"from":{"id":"and","port":"out"},"to":{"id":"cout","port":"in"}},{"from":{"id":"a","port":"out"},"to":{"id":"xor","port":"in1"}},{"from":{"id":"b","port":"out"},"to":{"id":"xor","port":"in2"}},{"from":{"id":"xor","port":"out"},"to":{"id":"s","port":"in"}}],"subcircuits":{}}
```

********************************************************

### Speicher 

                       {{0-1}}
********************************************************

Kombinatorische Logik wird durch Schaltnetze repräsentiert, die durch zyklenfreie Graphen dargestellt werden können, sprich es gibt keine Eigenschaften:

1. Zustandslos: Ausgabe ist nur von der Eingabe abhängig;
2. One-Way: keine Rückkopplungen im Schaltnetz;
3. 0-Verzögerung: keine Berücksichtigung der Gatterlaufzeit.

> __Frage:__ Können wir die bisherigen Konzepte und Techniken der logischen Schaltungen einsetzen, um wesentliche Elemente eines Rechners zu beschreiben?


********************************************************

                       {{1-2}}
********************************************************

> Nein, es fehlt ein Speicher.

Darstellung eines D-Flip-Flops (1 Bit Speicher)

<!-- data-type="none" -->
| $E(t)$ | $D(t)$ | $D(t+1)$ |
| ------ | ------ | -------- |
| 0      | 0      | $D(t)$   |
| 0      | 1      | $D(t)$   |
| 1      | 0      | 0        |
| 1      | 1      | 1        |

![Schaltung gebaut](./images/dlatch_plan.png "Beispielhafte Realisierung eines 1-Bit-Speichers")<!-- style="width: 40%; max-width: 800px;" -->

********************************************************

                       {{2-3}}
********************************************************

> **Merke** Register bilden das Zusammenspiel mehrerer Flip-Flops.

Vier Bit Register

<!--
style="width: 80%; min-width: 420px; max-width: 720px;"
-->
```ascii
        +----------------+         
  ------| D_0        Q_0 |------
  ------| D_1        Q_1 |------
  ------| D_2        Q_2 |------
  ------| D_3        Q_3 |------
        |                |
  ------|">"             |
  ------| enable write   |
  ------| enable read    |
  ------| clear          |
        +----------------+                                                     .
```

********************************************************

## Und nun alles zusammen ...

Wir wollen einen programmierbaren Rechner bauen, der in der Lage ist, eine Sequenz von Berechnungen auszuführen ... dazu fügen wir die gerade besprochenen Komponenten zusammen.

| Stufe | Funktionalität                                                       | Wunschzettel                                                     |
|-------|----------------------------------------------------------------------|------------------------------------------------------------------|
| 0     | Addition/Subtraktion von einzelnen Werten $\color{green} \mathbf{✓}$ |                                                                  |
| 1     |                                                                      | Flexibles Handling mehrerer Operationen $\color{red} \mathbf{?}$ |


### 1 - Operationsauswahl

Logische Funktionen: NOT, AND, NOR, XOR

Arithmetische Funktionen: ADD, SUB, (MUL), (DIV)

Sonstige: SHIFT LEFT (arithmetisch, logisch), SHIFT RIGHT (arithmetisch, logisch)

```text @plantUML.png
@startditaa
             Daten                 Daten
               |                     |
               |                     |
+--------------|---------------+     |
| +------+     |               |     |     Funktionsauswahl
| |      |     |               :     |        F_0 F_1 F_2
| |      V     V               V     V         |   |   | Zielregister
| | +----+-----+-----+    +----+-----+-----+   |   |   |  auswahl
| | |cBFB Register A |    |cBFB Register B |   |   |   |     Z
| | +---+------------+    +---------+------+   |   |   |     |
| |     |                           |          |   |   |     |
| |     |        +------------------+          |   |   |     |
| |     |        |                  |          |   |   |     |
| |     +--------+-----------+      |          |   |   |     |
| |     |        |           |      |          |   |   |     |
| |     V        V           V      V          |   |   |     |
| | +----------------+    +----------------+   |   |   |     |
| | |c808            |<-  |c808            |<--+   |   |     |
| | | Demultiplexer  |<-  | Demultiplexer  |<------+   |     |
| | |                |<-  |                |<----------+     |
| | ++-+-+-+-+-+-+-+-+    ++-+-+-+-+-+-+-+-+                 |
| |  | | | | | | | |       | | | | | | | |                   |
| |  | | | | | | | |       V V V V V V V V                   |
| |  | | | | | | | +----------------------------------+      |
| |  | | | | | | +-----------------------------+      |      |
| |  | | | | | +------------------------+      |      |      |
| |  | | | | +-------------------+      |      |      |      |
| |  | | | +--------------+      |      |      |      |      |
| |  | | +---------+      |      |      |      |      |      |
| |  | +----+      |      |      |      |      |      |      |
| |  |      |      |      |      |      |      |      |      |
| |  |  |   |  |   |  |   |  |   |  |   |  |   |  |   |  |   |
| |  V  V   V  V   V  V   V  V   V  V   V  V   V  V   V  V   |
| | +----+ +----+ +----+ +----+ +----+ +----+ +----+ +----+  |
| | | 000| | 001| | 010| | 011| | 100| | 101| | 110| | 111|  |
| | |cFF4| |cFF4| |cFF4| |cFF4| |cFF4| |cFF4| |cFF4| |cFF4|  |
| | | OR | |AND | |EXOR| |ADD | |SUB | |MUL | |DIV | | SL |  |
| | +-+--+ +-+--+ +-+--+ +-+--+ +-+--+ +-+--+ +-+--+ +-+--+  |
| |   :      :      :      :      :      :      :      :     |
| |   +------+------+------+---+--+------+------+------+     |
| |                            |                             |
| |                            V                             |
| |                 +----------+----------+                  |
| |                 |cCCB DeMuxer/Selektor|<-----------------+
| |                 +---+-+---------------+
| |                     : :                     |      |
| +---------------------+ |                   --+---+--+
|                         |                         | Status
+-------------------------+                         v S
@endditaa
```
<!-- data-type="none" -->
| F2  | F1  | F0  | Z   | Bedeutung |
| --- | --- | --- | --- | --------- |
| 0   | 1   | 1   | 1   | `ADD_B`   |
| ... |     |     |     |           |
| 0   | 1   | 0   | 1   | `EOR_A`   |

Beispielanwendungen

<!-- data-type="none" -->
|              | Inkrementieren   | Togglen          |
|--------------|------------------|------------------|
| Vorbereitung | A <- 1, B <-0    | A <- 1           |
| "Programm"   | `ADD_B` $(0111)$ | `EOR_A` $(0101)$ |
|              | `ADD_B` $(0111)$ | `EOR_A` $(0101)$ |
|              | `ADD_B` $(0111)$ | `EOR_A` $(0101)$ |

### 2 - Sequenz von Berechnungen

Zwischenstand

| Stufe | Funktionalität                                                                                             | Wunschzettel                                                                                                                                                                                         |
|-------|------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0     | Addition/Subtraktion von einzelnen Werten $\color{green} \mathbf{✓}$                                       |                                                                                                                                                                                                      |
| 1     | Arithmetische Einheit mit mehren Funktionen und wählbarem Ergebnisregister      $\color{green} \mathbf{✓}$ |                                                                                                                                                                                                      |
| 2     |                                                                                                            | Sequenz von Berechnungsfolgen  $\color{red} \mathbf{?}$ $$\begin{aligned} & Reg\_A \leftarrow 3 \\ & Reg\_B \leftarrow 2 \\ & ADD\_B \\ & Reg\_A \leftarrow -3 \\ & MUL\_B  \\ & ... \end{aligned}$$ |

Für diesen Schritt fassen wir das obige Schaltbild unserer hypothetischen ALU mit 8 Funktionen in einem abstrakteren Schaubild zusammen.

```text @plantUML.png
@startditaa
           +-----+-----+
           |  A  |  B  |
    Daten  +-----+-----+
<--------->+cFF4       |
           |           |
 Kontrolle |    ALU    |
---------->+           |
           |           |
   Status  |           |
<----------+           |
           +-----------+
@endditaa
```


| Bezeichnung | Bedeutung                               |
|-------------|-----------------------------------------|
| A, B        | Datenregister                           |
| Daten       | Zugriff auf die Datenregister           |
| Kontrolle   | Steuerleitungen $F_0$ bis $F_2$ und $Z$ |
| Status      | Carry oder Fehlerflags                  |

Wir erweitern diese ALU-Komponenten nun um zwei weitere Module - einen Speicher und ein Steuerwerk.

Der Speicher umfasst unsere Programmbestandteile `AND_B` usw. in jeweils einem 4 Bit breiten Register.


```text @plantUML.png
@startditaa
               +------------------+
               |c88F              |
               |     Speicher     |
               |                  |
               +-----------+------+
                           |
                           |
                           |               +-----+-----+
                           |               |  A  |  B  |
                           |        Daten  +-----+-----+
                           |     --------->+cFF4       |
                           |    Kontrolle  |           |
                           +-------------->+    ALU    |
                                   Status  |           |
                                  <--------+           |
                                           +-----------+
@endditaa
```

Wie allerdings setzen wir den Fortschritt im Programm um? Nach welcher Methodik werden die nachfolgenden Befehle aufgerufen?

Eine weitere Komponente, das Steuerwerk übernimmt diese Aufgabe. Anstatt nun eine Folge von Kontrollflags vorzugeben, erzeugen wir intern eine Folge von Adressen, die auf Speicherbereiche verweisen, in denen die Konfigurationen der ALU hinterlegt sind.

```text @plantUML.png
@startditaa
               +------------------+
               |c88F              |
               |     Speicher     |
               |                  |
               +-----------+------+
                       ^   |
                       |   |
+----------+  Adresse  |   |               +-----+-----+
|cF88      +-----------+   |               |  A  |  B  |
|          |               |        Daten  +-----+-----+
|          |               |     --------->+cFF4       |
|  Adress- |               |    Kontrolle  |           |
|berechnung|               +-------------->+    ALU    |
|          |                       Status  |           |
|          |                      <--------+           |
+----------+                               +-----------+
@endditaa
```

Allerdings bleibt bei dieser Konfiguration unser Status auf der Strecke! Im Grund müssen wir die Information dazu aber Operationsspezifisch auswerten. Es genügt also nicht allein eine Adressberechung zu realsieren, vielmehr bedarf es einer generellen Steuerungskomponente, die die Ausführung von Befehlen initiiert und überwacht.

```text @plantUML.png
@startditaa
               +------------------+
               |c88F              |
               |     Speicher     |
               |                  |
               +---------+-+------+
                       ^ |
                       | |
+----------+  Adresse  | |                 +-----+-----+
|cF88      +-----------+ |                 |  A  |  B  |
|          |             |          Daten  +-----+-----+
|          |   Befehl    |        -------->+cFF4       |
|  Steuer- |<------------+                 |           |
|   werk   |                     Kontrolle |    ALU    |
|          +------------------------------>+           |
|          |                               |           |
|          |                       Status  |           |
|          +<------------------------------+           |
+----------+                               +-----------+
@endditaa
```

Das Steuerwerk ist nun dafür verantwortlich:

+ Adressen zu berechnen
+ Befehle zu interpretieren
+ die ALU über entsprechende Flags zu konfigurieren
+ die Statusinformationen entsprechend dem aktuellen Befehl auszuwerten

Wir lösen uns von dem Zugriff auf die Kontrollbits und etablieren abstrakte Befehle.

> **Merke:** Das Steuerwerk entkoppelt die Konfiguration einzelner Komponenten und die Befehlsdarstellung.

### 3 - Handhabung der Daten

Zwischenstand

| Stufe | Funktionalität                                                                                             | Wunschzettel                                                                                                                                                                                                                                      |
|-------|------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0     | Addition/Subtraktion von einzelnen Werten $\color{green} \mathbf{✓}$                                       |                                                                                                                                                                                                                                                   |
| 1     | Arithmetische Einheit mit mehren Funktionen und wählbarem Ergebnisregister      $\color{green} \mathbf{✓}$ |                                                                                                                                                                                                                                                   |
| 2     | Sequenz von Berechnungsfolgen    $\color{green} \mathbf{✓}$                                                |                                                                                                                                                                                                                                                   |
| 3     |                                                                                                            | Freie Definition von Operanden  $\color{red} \mathbf{?}$     $$\begin{aligned} & \color{red}{Reg\_A \leftarrow 3}  \\ & \color{red}{Reg\_B \leftarrow 2} \\ & ADD\_B \\ & \color{red}{Reg\_A \leftarrow -3} \\ & MUL\_B  \\ & ... \end{aligned}$$ |

Wo kommen aber die Daten her? Bislang haben wir uns damit begnügt anzunehmen, dass diese auf "magische" Art und Weise in unseren Registern stehen.

```text @plantUML.png
@startditaa
               +------------------+
               |c88F              |
               |     Speicher     |
               |                  |
               +---------+-+------+
                       ^ | ^
                       | | |
+----------+  Adresse  | | |               +-----+-----+
|cF88      +-----------+ | |               |  A  |  B  |
|          |             | |        Daten  +-----+-----+
|          |   Befehl    | +-------------->+cFF4       |
|  Steuer- |<------------+                 |           |
|   werk   |                     Kontrolle |    ALU    |
|          +------------------------------>+           |
|          |                               |           |
|          |                       Status  |           |
|          |<------------------------------+           |
+----------+                               +-----------+
@endditaa
```

Im Speicher stehen nun nicht nur Befehle für die Ausführung unserer ALU-Funktionen, sondern auch die Daten für unsere Berechnungen. Auf diese verweisen wir mit separaten Befehlen.

<!-- data-type="none" -->
| Befehle       | Codierung        | Bedeutung                                           |
|---------------|------------------|-----------------------------------------------------|
| `OR_A`        | $0000$ bisher(!) | Logisches Oder                                      |
|               | ....             |                                                     |
| `LDA Adresse` | $10000$          | Laden der Daten von der Adresse X in das Register A |
| `LDB Adresse` | $10001$          | Laden der Daten von der Adresse X in das Register B |
| `STA Adresse` | $10010$          | Speichern der Daten aus Register A an der Adresse X |
| `STB Adresse` | $10011$          | Speichern der Daten aus Register B an der Adresse X |

Damit nimmt der Aufwand im Steuerwerk nochmals signifikant zu! Neben dem Adressbus besteht nun ein Datenbus als weiterer Kommunikationspfad.

| Schritt | Vorgang                                                                               |
|---------|---------------------------------------------------------------------------------------|
| 0       | Lesen eines Befehls aus dem Speicher                                                  |
| 1       | Erkennen, dass es sich um ein `LDA` handelt - Eine Berechnung ist nicht erforderlich. |
| 2       | Verschieben des Adresszählers auf die nächste Speicherstelle                          |
| 3       | Aktivieren eines schreibenden Zugriffs auf das Register A der ALU                     |
| 4       | Umleiten der Inhalte aus dem Speicher an die ALU (anstatt an das Steuerwerk)          |

> **Achtung:** Offenbar haben wir jetzt 2 Kategorien von Befehlen! `ADD_A` oder `OR_B` werden in einem Zyklus ausgeführt. Die `LDA` oder `STB` Befehle brauchen ein Nachladen der zusätzlichen Parameter (hier Daten).

### 4 - Ein- und Ausgaben

Zwischenstand

| Stufe | Funktionalität                                                                                             | Wunschzettel                                                   |
|-------|------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------|
| 0     | Addition/Subtraktion von einzelnen Werten $\color{green} \mathbf{✓}$                                       |                                                                |
| 1     | Arithmetische Einheit mit mehren Funktionen und wählbarem Ergebnisregister      $\color{green} \mathbf{✓}$ |                                                                |
| 2     | Sequenz von Berechnungsfolgen    $\color{green} \mathbf{✓}$                                                |                                                                |
|       | Darstellung von Programmen als Sequenzen abstrakter Befehle  $\color{green} \mathbf{✓}$                    |                                                                |
| 3     | Flexibler Zugriff auf Daten und Programme im Speicher     $\color{green} \mathbf{✓}$                       |                                                                |
|       |                                                                                                            | Ein- und Ausgabe von Daten wäre schön $\color{red} \mathbf{?}$ |


Das Steuerwerk koordiniert neben der ALU die Ein- und Ausgabeschnittstelle.

```text @plantUML.png
@startditaa
               +------------------+
               |c88F              |
               |     Speicher     |
               |                  |
               +---------+-+------+
                       ^ | ^
                       | | |
+----------+  Adresse  | | |               +-----+-----+
|cF88      +-----------+ | |               |  A  |  B  |
|          |             | |        Daten  +-----+-----+
|          |   Befehl    | +-------------->+cFF4       |
|  Steuer- |<------------+                 |           |
|   werk   |                     Kontrolle |    ALU    |
|          +------------------------------>+           |
|          |                               |           |
|          |                       Status  |           |
|          |<------------------------------+           |
|          |                               |           |
|          | Kontrolle                     |           |
|          +-------------+         Daten   |           |
|          |             | +-------------->|           |
|          |  Status     | |               |           |
|          |<----------+ | |               |           |
+----------+           | | |               +-----------+
                       | | |
                       | V V
               +-------+-----------+
               |c8F8               |
               |    Ein/Ausgabe    |
               |                   |
               +-------------------+
@endditaa
```

## 1945:  Von-Neumann Architektur

John von Neumann beschrieb 1945 in seinem Aufsatz ”First Draft of a Report on the EDVAC“ ein strukturelles Konzept, wonach Computerprogramme und die zu verarbeitenden Daten zusammen im gleichen Speicher abgelegt werden sollten. [Link](http://abelgo.cn/cs101/papers/Neumann.pdf)

Einige der Ideen des Konzepts wurden bereits von Konrad Zuse erkannt und teilweise in der Z1 und der Z3 realisiert.

```text @plantUML.png
@startditaa
  ^  |
  |  V
+-+----+    +----------------------------+
|c8F8  |    |c88F   Speicherwerk         |
|      |<-->|   +--------+  +--------+   |
|      |    |   |Programm|  | Daten  |   |
|      |    |   +--------+  +--------+   |
|      |    +------+---------------------+
|      |        ^  |             ^
| E/A- |        :  |             |
| Werk |        |  V             V
|      |    +---+------+    +------------+
|      |    |cF88      |    | Rechenwerk |
|      |    |          |    |cFF4        |
|      |    |          |<-=-+ +--------+ |
|      |<-=-+Steuerwerk|    | |Register| |
|      |    |          +-=->| +--------+ |
|      |    |          |    | |  ALU   | |
|      |    |          |    | +--------+ |
+------+    +----------+    +------------+
@endditaa
```