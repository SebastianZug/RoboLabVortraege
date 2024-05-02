<!--
author:   Sebastian Zug

email:    sebastian.zug@informatik.tu-freiberg.de

version:  0.0.2

language: en

comment:  Einführung in das Robot Operating System II 

narrator: DE

-->

[![LiaScript](https://raw.githubusercontent.com/LiaScript/LiaScript/master/badges/course.svg)](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/main/10_ROS_II/presentation.md)

# TUBAF Bits&Bytes

----------------------------------------------------------------

Donnerstag, 2.05.2024, 17 Uhr, RoboLab der TU Bergakademie Freiberg

---------------------

Prof. Dr. Sebastian Zug

> _Im nunmehr schon 9. und 10. Bits&Bytes-Vortrag stellt diese und nächste Woche Prof. Dr. Sebastian Zug die generellen Konzepte von Programmierframeworks für autonome Systeme dar und führt darauf aufbauend in die Nutzung des "Robot Operating Systems" (ROS) ein. Dabei werden grundlegende Kommunikationskonzepte und Abstraktionen besprochen, um diese dann anhand von kleinen Projekten praktisch greifbar zu machen._


In der vorangegangenen Veranstaltung wurde die grundlegende Struktur von ROS2 vorgestellt. Dabei wurde auf die Knoten, die Topics und die Services eingegangen. Nunmehr soll der Überblick konzeptionell abgerundet werden, um dann auf konkrete Fragen der Implementierung einzugehen. 

## Kurze Erinnerung ...

> + ROS2 ist ein Framework für die Entwicklung von Software für Roboter. 
> + Die Funktionalität ist in Knoten organisiert, die miteinander kommunizieren.
> + Verschiedene Kommunikationsmodelle implementieren den Datenaustausch dazwischen (Publish-Subscribe, Client Server Varianten).

Versuchen wir uns am Beispiel einer RGBD-Kamera zu erinnern.

```
ros2 launch openni2_camera camera_with_cloud.launch.py
```

+ Darstellung der Visualisierung (rqt, rviz)
+ Anpassung der Qualitätsattribute in rviz
+ Nutzung des Parameter Servers (Service)

## Kommunikationskonzepte in ROS


                     {{0-1}}
**************************************************

> Middleware im Kontext verteilter Anwendungen ist eine Software, die über die vom Betriebssystem bereitgestellten Dienste hinaus Dienste bereitstellt, um den verschiedenen Komponenten eines verteilten Systems die Kommunikation und Verwaltung von Daten zu ermöglichen.

Middleware unterstützt und vereinfacht komplexe verteilte Anwendungen, sonst müsste die Anwendung Fragen wie:

+ Welche Informationen sind verfügbar?
+ In welchem Format liegen die Daten vor, bzw. wie muss ich meine Informationen verpacken?
+ Wer bietet die Informationen an?
+ Welche Laufzeit haben die Daten maximal?
...

**************************************************

                     {{1-2}}
**************************************************

ROS2 integriert ein abstraktes Interface für ein Einbettung von Middleware-Lösungen, die den DDS Standard implementieren. DDS stellt einen "Globalen Daten Raum" zur Verfügung, der diese allen interessierten
verteilten Anwendungen zur Verfügung stellt.

vgl. https://index.ros.org/doc/ros2/Concepts/DDS-and-ROS-middleware-implementations/

<!--
style="width: 80%; min-width: 520px; max-width: 820px;"
-->
```ascii

              User Applications
+---------+---------+---------+
| rclcpp  | rclpy   | rcljava | ...
+---------+---------+---------+-----------------------------+
| rcl (C API) ROS client library interface                  |
| Services, Parameters, Time, Names ...                     |
+-----------------------------------------------------------+
| rmw (C API) ROS middleware interface                      |
| Pub/Sub, Services, Discovery, ...                         |
+-----------+-----------+-------------+-----+---------------+
| DDS       | DDS       | DDS         | ... | Intra-process |
| Adapter 0 | Adapter 1 | Adapter 2   |     |      API      |
+-----------+-----------+-------------+     +---------------+
| FastRTPS  | RTI       | PrismTech   | ...
|           | Context   | OpenSplice  |
+-----------+-----------+-------------+                                        .
```

**************************************************

                     {{2-3}}
**************************************************

| Ohne Kontrollflussübergabe | Mit Kontrollflussübergabe |
| -------------------------- | ------------------------- |
| __Publish/Subscribe__ ... DDS implmentiert das Publish/Subscribe Paradigma in Kombination mit Quality of Service Attributen. Diese dienen der Koordination des Datenaustausches unter Berücksichtigung von zwingenden Anforderungen des Subscribers bzw. dem Verhalten des Publishers. | __Services and Actions__ ... DDS verfügt derzeit nicht über einen Request-Response-Mechanismus, mit dem die entsprechenden Konzept der Dienste in ROS umgesetzt werden könnten. Derzeit wird in der OMG DDS-Arbeitsgruppe eine RPC-Spezifikation zur Ratifizierung geprüft, und mehrere der DDS-Anbieter haben einen Entwurf für die Implementierung der RPC-API. |


**************************************************

                     {{3-4}}
**************************************************

__Welche Rolle spielen die QoS Eigenschaften des Kommunkations-Layers?__

ROS 2 bietet eine Vielzahl von Quality of Service (QoS)-Richtlinien, mit denen Sie die Kommunikation zwischen Knoten und die Datenhaltung optimieren können. Im Gegensatz zu ROS1, das vorrangig auf TCP setzte, kann ROS2 von Transparenz der jeweiligen DDS-Implementierungen profitieren.

+ *Durability* ... legt fest, ob und wie lange Daten, die bereits ausgetauscht worden sind,  "am Leben bleiben". `volatile` bedeutet, dass dahingehend kein Aufwand investiert wird, `transient` oder `persistent` gehen darüber hinaus.
+ *Reliability* ...  Die Reliability-QoS definiert, ob alle geschriebenen Datensätze (irgendwann) bei allen Readern angekommen sein müssen. Bei zuverlässiger (`reliable`) Kommunikation werden geschriebene Datensätze eines Topics, die aus irgendwelchen Gründen auf dem Netzwerk verloren gehen, von der Middleware wiederhergestellt, um diese Daten verlässlich den Readern zustellen zu können. Im Unterschied dazu definiert `best effort` eine schnellstmögliche Zustellung.
+ *History* ... definiert, wie viele der letzten zu sendenden Daten und empfangenen Daten gespeichert werden. `Keep last` speichert n Samples, wobei die n durch den QoS Parameter _Depth_ definiert wird. `Keep all` speichert alle Samples
+ *Depth* ... erfasst die Größe der Queue für die History fest, wenn `Keep last` gewählt wurde.

**************************************************

### ROS Publish-Subscribe

...  wurde bereits in der vergangenen Woche besprochen.


### ROS Services

Bisher haben wir über asynchrone Kommunikationsmechanismen gesprochen. Ein Publisher triggert ggf. mehrere Subscriber. Die damit einhergehende Flexibilität kann aber nicht alle Anwendungsfälle abdecken:

+ Berechne einen neuen Pfad
+ Aktiviere die Kamera
+ ...

In diesem Fall liegt eine Interaktion in Form eines Remote-Procedure-Calls (RPC) vor. 

Die Anfrage / Antwort erfolgt über einen Dienst, der durch ein Nachrichtenpaar definiert ist, eine für die Anfrage und eine für die Antwort. Ein bereitstellender ROS-Knoten bietet einen Dienst unter einem String-Namen an, und ein Client ruft den Dienst auf, indem er die Anforderungsnachricht sendet und in seiner Ausführung innehält und auf die Antwort wartet.

#### Manuelle Interaktion mit ROS-Services

```
ros2 run turtlesim turtlesim_node
```


![RoboterSystem](./image/08_ROS_Kommunikation/SingleTurtle.png)<!-- style="width: 80%; min-width: 420px; max-width: 800px;"-->


Wie explorieren Sie die Services, die durch den `turtlesim_node` bereitgestellt werden?

`ros2` stellt zwei Schnittstellen für die Arbeit mit den Services bereit.

+ `service` erlaubt den Zugriff auf die tatsächlich angebotenen Services während
+ `interfaces`  die Definitionsformate offeriert

```bash
> ros2 interface list | grep turtlesim
turtlesim/srv/Kill
turtlesim/srv/SetPen
turtlesim/srv/Spawn
turtlesim/srv/TeleportAbsolute
turtlesim/srv/TeleportRelative
```

Offenbar stellt die Turtlesim-Umgebung 5 Services bereit, deren Bedeutung selbsterklärend ist. Das Format lässt sich entsprechend aus den srv Dateien ablesen:

```
> ros2 interface show turtlesim/srv/Spawn
float32 x
float32 y
float32 theta
string name # Optional. A unique name will be created and returned if this is empty
---
string name
```

Versuchen wir also eine Service mittels `ros2 service call` manuell zu starten. Der Aufruf setzt sich aus mehreren Elementen zusammen, deren Konfiguration zwischen den ROS2 Versionen schwanken. An erster Stelle steht der Dienstname gefolgt von der Service-Definition und dem eigentlichen Parametersatz.

```
> ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

Mit den anderen Services (`Kill`) kann dessen Verhalten nun adaptiert werden.

> Beachten Sie den Unterschied bei der Umsetzung ... unser Knoten im Terminal "wartet" auf die Antwort.

#### Anwendung in Parameter Konfiguration

Eine besondere Variante der Services stellen die Parameter dar. Dies sind knoteninterne Größen, die über Services angepasst werden können. Darunter fallen zum Beispiel die Konfigurationsdaten

+ einer Kamera,
+ die gewünschte maximale Reichweite eines Ultraschallsensors,
+ die Schwellwerte bei der Featureextraktion, Linienerkennung, etc.
+ ...

Der Vorteil der Parameter liegt darin, dass diese ohne Neukompilierung angepasst werden können.

Zur Illustration des Mechanismus soll wiederum auf die Turtelsim-Umgebung zurückgegriffen werden.

```
> ros2 param list
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
> ros2 param describe /turtlesim background_b
  Parameter name: background_b
    Type: integer
    Description: Blue channel of the background color
    Constraints:
      Min value: 0
      Max value: 255
> ros2 param get /turtlesim background_b
  Integer value is: 86
> ros2 param set /turtlesim background_b 10
  Set parameter successful
```

Der Hintergrund der Simulationsumgebung ändert sich entsprechend.

Der Vorteil des Parameterkonzepts liegt nun darin, dass wir:

+ das Set der Parameter während der Laufzeit anpassen können. Damit kann das Testen der Anwendung im Feld (zum Beispiel bei der Konfiguration von Sensoren) ohne Neustart/Neukompilierung realisiert werden.
+ die gewählten Sets einzeln oder im Block abspeichern können. Diese Konfigurationsfiles werden als yaml Dateien abgelegt und können für unterschiedliche Einsatzszenarien aufgerufen werden.

```
> ros2 param dump /turtlesim
  Saving to:  ./turtlesim.yaml
> cat turtlesim.yaml
  turtlesim:
    ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 69
      use_sim_time: false
```

Der Aufruf kann dann entweder in der Kommandozeile erfolgen

```
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

> Die Implementierung erfolgt anhand eines knoten-internen Parameterservers der wie folgt initialisiert wird.


### ROS Actions

_Actions_ sind für lang laufende Aufgaben vorgesehen. Der Client wartet dabei nicht auf das Eintreten des Resultats sondern setzt seine Arbeit fort. Entsprechend wird eine _Action_ als asynchroner Call bezeichnet, der sich an einen _Action_ _Server_ richtet.

Das _Action_ Konzept von ROS spezifiziert 3 Nachrichtentypen, die der Client an den Server richten kann:

| Nachricht | Richtung    | Bedeutung                                                                                                                                                                                                                                                                                                                |
| --------- | --- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Goal      | Client -> Server    | ... definiert die Parameter des Ziels einer Operation. Im Falle einer Bewegung der Basis wäre das Ziel eine _PoseStamped_-Nachricht, die Informationen darüber enthält, wohin sich der Roboter in der Welt bewegen soll. Darüber hinaus werden Randbedingungen definiert, wie zum Beispiel die maximale Geschwindigkeit. |
| Feedback  | Server -> Client    | ... ermöglicht es eine Information zum aktuellen Stand der Abarbeitung zu erhalten. Für das Bewegen der Basis kann dies die aktuelle Pose des Roboters entlang des Weges sein.                                                                                                                                           |
| Result    |Server -> Client       | ... wird vom ActionServer an den ActionClient gesendet, wenn das Ziel erreicht ist. Dies ist anders als Feedback, da es genau einmal gesendet wird.                                                                                                                                                                      |

```text @plantUML.png
@startuml

Action_client -> Action_server: goal request
activate Action_client
activate Action_server

Action_server -> User_method: activate algorithm
activate User_method
Action_server --> Action_client: goal response

Action_client -> Action_server: result request

User_method -> Action_client: publish feedback
User_method -> Action_client: publish feedback
User_method -> Action_client: publish feedback

User_method -> Action_server: set result
deactivate User_method

Action_server --> Action_client: result response
deactivate Action_server
deactivate Action_client
@enduml
```

Beschrieben wird das Interface wiederum mit einem eigenen Filetyp, den sogenannten `.action` Files. Im Beispiel sehe Sie eine _Action_, die sich auf die Bewegung eines Outdoorroboters zu einer angegebenen GPS-Koordinate bezieht.

```
# Define the goal
float64 latitude
float64 longitude
---
# Define the result
uint32 travel_duration # s
uint32 distance_traveled # m
---
# Define a feedback message
uint32 remaining_distance # m
```

Hinzu kommt noch die Möglichkeit eine _Action_ mit _chancel_ zu stoppen. Hierfür ist aber keine explizite Schnittstelle notwendig.

#### Beispiel

Das Beispiel wurde der ROS2 Dokumentation unter [Link](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/) entnommen.

```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

Danach können Sie für unseren Turtlesim Umgebung sämtliche Kommunikationsinterfaces auf einen Blick erfassen:

```
> ros2 node info /turtlesim
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
> ros2 action list -t
  /turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
> ros2 action info /turtle1/rotate_absolute
  Action: /turtle1/rotate_absolute
  Action clients: 1
      /teleop_turtle
  Action servers: 1
      /turtlesim
```

Welche Elemente des Turtlesim-Interfaces können Sie erklären? Wie gehen Sie vor, um sich bestimmter Schnittstellen zu vergewissern?

Welche Struktur hat das Action-Interface?

```
> ros2 interface show turtlesim/action/RotateAbsolute

  # The desired heading in radians
  float32 theta
  ---
  # The angular displacement in radians to the starting position
  float32 delta
  ---
  # The remaining rotation in radians
  float32 remaining
```

Die Definition des Ziels erfolgt mittels

```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute {'theta: -1.57'} --feedback
```

## ROS Packages 

ROS1 und 2 sind in Paketen organisiert, diese kann man als Container für zusammengehörigen Code betrachten.

Softwareengineering Ziele:

+ Alle Funktionalität sollte so strukturiert werden, dass Sie in einem anderen Kontext erweitert und wiederverwendet werden kann.
+ Jede Komponente (Memberfunktion, Klasse, Package) sollte eine einzige Verantwortlichkeit definieren.
+ Die Kopplung zwischen den Paketen sollte allein über den Datenaustausch realisiert werden.
+ Verzicht auf unnötige Funktionalität - *you don't pay for what you don't use*

### Build Prozess

Warum ist ein Build-System erforderlich? Für eine einzelne Datei ist der Vorgang überschaubar

+ Eingabe: Code und Konfigurationsdateien
+ Ausgabe: Artifacts (Code, Daten, Dokumentation)
+ Vorbereiten der Verwendung

Für ein ganzes Set von Paketen ist deutlich mehr Aufwand erforderlich:

+ Voraussetzungen:

  + Verfügbarkeit von System-Abhängigkeiten (abhängige Pakete)
  + Setzen der notwendigen Umgebungsvariablen

+ Eingabe: Code und Konfigurationsdateien der Pakete

+ Build Prozess:

  + Berechnen der Abhängigkeiten und Festlegen einer Build-Reihenfolge
  + Bauen einzelner Pakete UND Installation, Update der Umgebung

+ Ausgabe: Set von ausführbaren Knoten

> **Merke:** ROS / ROS2 umfasst eine Fülle von Tools für die Organisation dieses Prozesses.

### Anwendung

Wenn Sie Ihren Code installieren oder mit anderen teilen möchten, muss dieser in einem Paket organisiert sein.

```bash    InspectPackages
> ros2 pkg
usage: ros2 pkg [-h] Call `ros2 pkg <command> -h` for more detailed usage. ...

Various package related sub-commands

optional arguments:
  -h, --help            show this help message and exit

Commands:
  create       Create a new ROS2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package

  Call `ros2 pkg <command> -h` for more detailed usage.
> ros2 pkg list
action_msgs
action_tutorials
actionlib_msgs
ament_cmake
ament_cmake_auto
ament_cmake_copyright
```

Ein Paket umfasst aber nicht nur den eigentlichen Code sondern auch:

+ eine Spezifikation der Abhängigkeiten
+ die Konfiguration des Build-Systems
+ die Definition der nutzerspezifischen Messages
+ die `launch` Files für den Start der Anwendungen und deren Parametrisierung

Ein minimales Paket umfasst zwei Dateien:

+ package.xml - Informationen über das Paket selbst (Autor, Version, ...)
+ CMakeLists.txt file - beschreibt wie das Paket gebaut wird und welche Abhängigkeiten bestehen

Die Paketerstellung in ROS 2 verwendet [ament](https://design.ros2.org/articles/ament.html) als Build-System und [colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html) als Build-Tool.

Pakete können in gemeinsamen `Workspaces` angelegt werden.

```
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/my_package
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

Diese Struktur wird durch das jeweilige Build-System automatisch erweitert. `colcon` erstellt standardmäßig weitere Verzeichnisse in der Struktur des Projektes:

+ Das `build`-Verzeichnis befindet sich dort, wo Zwischendateien gespeichert werden. Für jedes Paket wird ein Unterordner erstellt, in dem z.B. CMake aufgerufen wird.

+ Das Installationsverzeichnis ist der Ort, an dem jedes Paket installiert wird. Standardmäßig wird jedes Paket in einem separaten Unterverzeichnis installiert.

+ Das `log` Verzeichnis enthält verschiedene Protokollinformationen zu jedem Colcon-Aufruf.

> Lassen Sie uns das Ganze mal praktisch ausprobieren ... [python example](https://liascript.github.io/course/?https://raw.githubusercontent.com/SebastianZug/RoboLabVortraege/main/09_ROS_I/presentation.md#9)


## ROS Demo

Vorstellung eines aktuellen Projektes von Lion Waurich und Caio Marcas Menz.