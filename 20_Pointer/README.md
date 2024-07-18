<!--

author:   Sebastian Zug 
email:    sebastian.zug@informatik.tu-freiberg.de 
version:  1.1.2
language: de
narrator: Deutsch Female

comment: Einführung in die Programmierung für Nicht-Informatiker
logo: ./img/LogoCodeExample.png

import: https://github.com/liascript/CodeRunner
        https://github.com/LiaTemplates/AVR8js/main/README.md

-->

# Zeigs mir! Anwendung von Pointern in C und C++

<h2>Datenanlyse mit Python und Pandas </h2>

Vortrag im Rahmen der Bits&Bytes Vorträge des RoboLab Freiberg 

Juli 2024

Prof. Dr. Sebastian Zug

-------------------------------------

## Grundkonzept Zeiger

Das Konzept des Zeigers (englisch Pointer) erweitert das Spektrum der Inhalte von Variablen. Eine Variable speichert einen Wert, ein Zeiger speichert eine Adresse. Der Wert, auf den der Zeiger zeigt, kann durch Dereferenzierung des Zeigers erreicht werden.


```ascii
  Variablen-     Speicher-      Inhalt
  name           addresse
                                +----------+
                 0000           |          |
                                +----------+
                 0001           |          |
                                +----------+
  a   ------>    0002       +---| 00001007 | Adresse
                          z |   +----------+
                 0003     e |   |          |
                          i |   +----------+
                 ....     g |   |          |
                          t |   +----------+
                 1005       |   |          |
                          a |   +----------+
                 1006     u |   |          |
                          f |   +----------+
  b   ------>    1007    <--+   | 00001101 |  Wert = 13
                                +----------+
                 1008           |          |
                                +----------+
                 ....           |          |                                                                    .
```


Welche Vorteile ergeben sich aus der Nutzung von Zeigern, bzw. welche
Programmiertechniken lassen sich realisieren:

* dynamische Verwaltung von Speicherbereichen,
* Übergabe von Datenobjekte an Funktionen via "call-by-reference",
* Übergabe von Funktionen als Argumente an andere Funktionen,
* Umsetzung rekursiver Datenstrukturen wie Listen und Bäume.

> Der Vollständigkeit halber sei erwähnt, dass C anders als C++ keine Referenzen im eigentlichen Sinne kennt. Hier ist die Übergabe der Adresse einer Variablen als Parameter gemeint und nicht das Konstrukt "Reference".

### Definition von Zeigern

Die Definition eines Zeigers besteht aus dem Datentyp des Zeigers und dem
gewünschten Zeigernamen. Der Datentyp eines Zeigers besteht wiederum aus dem
Datentyp des Werts auf den gezeigt wird sowie aus einem Asterisk. Ein Datentyp
eines Zeigers wäre also z. B. `double*`.

```cpp
/* kann eine Adresse aufnehmen, die auf einen Wert vom Typ Integer zeigt */
int* zeiger1;
/* das Leerzeichen kann sich vor oder nach dem Stern befinden */
float *zeiger2;
/* ebenfalls möglich */
char * zeiger3;
/* Definition von zwei Zeigern */
int *zeiger4, *zeiger5;
/* Definition eines Zeigers und einer Variablen vom Typ Integer */
int *zeiger6, ganzzahl;
```


### Verwendung

> **Merke:** Zeiger müssen vor der Verwendung initialisiert werden.

Der Zeiger kann initialisiert werden durch die Zuweisung:

* der Adresse einer Variable, wobei die Adresse mit Hilfe des Adressoperators
  `&` ermittelt wird,
* eines Arrays,
* eines weiteren Zeigers oder
* des Wertes von `NULL`.

```cpp                      PointerExamples.cpp
#include <iostream>
using namespace std;

int main(void)
{
  int a = 0;
  int * ptr_a = &a;       /* mit Adressoperator */

  int feld[10];
  int * ptr_feld = feld;  /* mit Array */

  int * ptr_b = ptr_a;    /* mit weiterem Zeiger */

  int * ptr_Null = NULL;  /* mit NULL */

  cout<<"Pointer ptr_a    "<<ptr_a<<"\n";
  cout<<"Pointer ptr_feld "<<ptr_feld<<"\n";
  cout<<"Pointer ptr_b    "<<ptr_b<<"\n";
  cout<<"Pointer ptr_Null "<<ptr_Null<<"\n";
  return 0;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

> [Visualisierung des Beispiels](https://pythontutor.com/render.html#code=%23include%20%3Ciostream%3E%0Ausing%20namespace%20std%3B%0A%0Aint%20main%28void%29%0A%7B%0A%20%20int%20a%20%3D%200%3B%0A%20%20int%20*%20ptr_a%20%3D%20%26a%3B%20%20%20%20%20%20%20/*%20mit%20Adressoperator%20*/%0A%0A%20%20int%20feld%5B10%5D%3B%0A%20%20int%20*%20ptr_feld%20%3D%20feld%3B%20%20/*%20mit%20Array%20*/%0A%0A%20%20int%20*%20ptr_b%20%3D%20ptr_a%3B%20%20%20%20/*%20mit%20weiterem%20Zeiger%20*/%0A%0A%20%20int%20*%20ptr_Null%20%3D%20NULL%3B%20%20/*%20mit%20NULL%20*/%0A%0A%20%20cout%3C%3C%22Pointer%20ptr_a%20%20%20%20%22%3C%3Cptr_a%3C%3C%22%5Cn%22%3B%0A%20%20cout%3C%3C%22Pointer%20ptr_feld%20%22%3C%3Cptr_feld%3C%3C%22%5Cn%22%3B%0A%20%20cout%3C%3C%22Pointer%20ptr_b%20%20%20%20%22%3C%3Cptr_b%3C%3C%22%5Cn%22%3B%0A%20%20cout%3C%3C%22Pointer%20ptr_Null%20%22%3C%3Cptr_Null%3C%3C%22%5Cn%22%3B%0A%20%20return%200%3B%0A%7D&cppShowMemAddrs=true&cumulative=false&curInstr=11&heapPrimitives=nevernest&mode=display&origin=opt-frontend.js&py=cpp_g%2B%2B9.3.0&rawInputLstJSON=%5B%5D&textReferences=false)


{{1}}
Die konkrete Zuordnung einer Variablen im Speicher wird durch den Compiler und
das Betriebssystem bestimmt. Entsprechend kann die Adresse einer Variablen nicht
durch den Programmierer festgelegt werden. Ohne Manipulationen ist die Adresse
einer Variablen über die gesamte Laufzeit des Programms unveränderlich, ist aber
bei mehrmaligen Programmstarts unterschiedlich.

{{1}}
In den Ausgaben von Pointer wird dann eine
hexadezimale Adresse ausgegeben.

{{1}}
Zeiger können mit dem "Wert" `NULL` als ungültig markiert werden. Eine
Dereferenzierung führt dann meistens zu einem Laufzeitfehler nebst
Programmabbruch. NULL ist ein Macro und wird in mehreren Header-Dateien
definiert (mindestens in `<cstddef>` (`stddef.h`)). Die Definition ist vom Standard
implementierungsabhängig vorgegeben und vom Compilerhersteller passend
implementiert, z. B.

{{1}}
```cpp
#define NULL 0
#define NULL 0L
#define NULL (void *) 0
```

{{2}}
Und umgekehrt, wie erhalten wir den Wert, auf den der Pointer zeigt? Hierfür
benötigen wir den *Inhaltsoperator* `*`.

{{2}}
```cpp                   DereferencingPointers.cpp
#include <iostream>
using namespace std;

int main(void)
{
  int a = 15;
  int * ptr_a = &a;
  cout<<"Wert von a                     "<<a<<"\n";
  cout<<"Pointer ptr_a                  "<<ptr_a<<"\n";
  cout<<"Wert hinter dem Pointer ptr_a  "<<*ptr_a<<"\n";
  *ptr_a = 10;
  cout<<"Wert von a                     "<<a<<"\n";
  cout<<"Wert hinter dem Pointer ptr_a  "<<*ptr_a<<"\n";
  return 0;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

{{2}}
> Schauen wir wiederum auf eine grafische Darstellung [PythonTutor](http://pythontutor.com/iframe-embed.html#code=%23include%20%3Cstdio.h%3E%0A%23include%20%3Cstdlib.h%3E%0A%0Aint%20main%28void%29%0A%7B%0A%20%20int%20a%20%3D%2015%3B%0A%20%20int%20*%20ptr_a%20%3D%20%26a%3B%0A%20%20printf%28%22Wert%20von%20a%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%25d%5Cn%22,%20a%29%3B%0A%20%20printf%28%22Pointer%20ptr_a%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%25p%5Cn%22,%20ptr_a%29%3B%0A%20%20printf%28%22Wert%20hinter%20dem%20Pointer%20ptr_a%20%20%25d%5Cn%22,%20*ptr_a%29%3B%0A%20%20*ptr_a%20%3D%2010%3B%0A%20%20printf%28%22Wert%20von%20a%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%25d%5Cn%22,%20a%29%3B%0A%20%20printf%28%22Wert%20hinter%20dem%20Pointer%20ptr_a%20%20%25d%5Cn%22,%20*ptr_a%29%3B%0A%20%20return%20EXIT_SUCCESS%3B%0A%7D&codeDivHeight=400&codeDivWidth=350&cumulative=false&curInstr=0&heapPrimitives=nevernest&origin=opt-frontend.js&py=c&rawInputLstJSON=%5B%5D&textReferences)

### Fehlerquellen

Fehlender Adressoperator bei der Zuweisung

```cpp               PointerFailuresI.cpp
#include <iostream>
using namespace std;

int main(void)
{
  int a = 5;
  int * ptr_a;
  ptr_a = a;
  cout<<"Pointer ptr_a                  "<<ptr_a<<"\n";
  cout<<"Wert hinter dem Pointer ptr_a  "<<*ptr_a<<"\n";
  cout<<"Aus Maus!\n";
  return 0;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

{{1}}
Fehlender Dereferenzierungsoperator beim Zugriff

{{1}}
```cpp          PointerFailuresII.cpp
#include <iostream>
using namespace std;

int main(void)
{
  int a = 5;
  int * ptr_a = &a;
  cout<<"Pointer ptr_a                  "<<(void*)ptr_a<<"\n";
  cout<<"Wert hinter dem Pointer ptr_a  "<<ptr_a<<"\n";
  cout<<"Aus Maus!\n";
  return 0;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

{{2}}
Uninitialierte Pointer zeigen "irgendwo ins nirgendwo"!

{{2}}
```cpp                  PointerFailuresIII.cpp
#include <iostream>
using namespace std;

int main(void)
{
  int * ptr_a;
  *ptr_a = 10;
  // korrekte Initalisierung
  // int * ptr_a = NULL;
  // Prüfung auf gültige Adresse
  // if (ptr_a != NULL) *ptr_a = 10;
  cout<<"Pointer ptr_a                  "<<ptr_a<<"\n";
  cout<<"Wert hinter dem Pointer ptr_a  "<<*ptr_a<<"\n";
  cout<<"Aus Maus!\n";
  return 0;
}
```
@LIA.evalWithDebug(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

### Arrays und Zeiger

Initialisierung und genereller Zugriff auf die einzelnen Elemente des Arrays
sind über einen Index möglich.

```cpp                     ArrayExample.cpp
#include <iostream>
using namespace std;

int main(void) {
  int a[3];       // Array aus 3 int Werten
  a[0] = -2;
  a[1] = 5;
  a[2] = 99;
  for (int i=0; i<3; i++)
    cout<<a[i]<<" ";

  cout<<"\nNur zur Info "<< sizeof(a);
  cout<<"\nZahl der Elemente "<< sizeof(a) / sizeof(int);

  int * ptr_a = a;
  for (ptr_a = a; ptr_a < a+sizeof(a) / sizeof(int); ptr_a++)
    cout<<*ptr_a<<" ";

  return 0;
  }
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

> Schauen wir uns das Ganze noch in einer Animation an: [PythonTutor](https://pythontutor.com/render.html#code=%23include%20%3Ciostream%3E%0Ausing%20namespace%20std%3B%0A%0Aint%20main%28void%29%20%7B%0A%20%20int%20a%5B3%5D%3B%20%20%20%20%20%20%20//%20Array%20aus%203%20int%20Werten%0A%20%20a%5B0%5D%20%3D%20-2%3B%0A%20%20a%5B1%5D%20%3D%205%3B%0A%20%20a%5B2%5D%20%3D%2099%3B%0A%20%20for%20%28int%20i%3D0%3B%20i%3C3%3B%20i%2B%2B%29%0A%20%20%20%20cout%3C%3Ca%5Bi%5D%3C%3C%22%20%22%3B%0A%0A%20%20cout%3C%3C%22%5CnNur%20zur%20Info%20%22%3C%3C%20sizeof%28a%29%3B%0A%20%20cout%3C%3C%22%5CnZahl%20der%20Elemente%20%22%3C%3C%20sizeof%28a%29%20/%20sizeof%28int%29%3B%0A%0A%20%20int%20*%20ptr_a%20%3D%20a%3B%0A%20%20for%20%28ptr_a%20%3D%20a%3B%20ptr_a%20%3C%20a%2Bsizeof%28a%29%20/%20sizeof%28int%29%3B%20ptr_a%2B%2B%29%0A%20%20%20%20cout%3C%3C*ptr_a%3C%3C%22%20%22%3B%0A%0A%20%20return%200%3B%0A%20%20%7D&cppShowMemAddrs=true&cumulative=false&curInstr=0&heapPrimitives=nevernest&mode=display&origin=opt-frontend.js&py=cpp_g%2B%2B9.3.0&rawInputLstJSON=%5B%5D&textReferences=false)

## Anwendung von Pointern

### Parameterübergabe und Rückgabewerte

In vielen Programmiersprachen, darunter in C und C++ werden zwei Arten der
Parameterübergabe realisiert.

**call-by-value**

In allen Beispielen bis jetzt wurden Parameter an die Funktionen *call-by-value*,
übergeben. Das bedeutet, dass innerhalb der aufgerufenen Funktion mit einer
Kopie der Variable gearbeitet wird und die Änderungen sich nicht auf den
ursprünglichen Wert auswirken.

```cpp          Student.cpp
#include <iostream>
using namespace std;

void doSomething(int a) {
   // eine KOPIE von a wird um 1 erhöht
   cout << ++a << " a in der Schleife\n";
}

int main(void) {
  int a = 5;
  cout << a << " a in main\n";
  doSomething(a);
  cout << a << " a in main\n";
  return 0;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)


**call-by-reference**

Bei einer Übergabe als Referenz wirken sich Änderungen an den Parametern auf die
ursprünglichen Werte aus. *Call-by-reference* wird unbedingt notwendig, wenn eine
Funktion mehrere Rückgabewerte hat.

Mit Hilfe des Zeigers wird in C die "call-by-reference"- Parameterübergabe
realisiert. In der Liste der formalen Parameter wird ein Zeiger eines
passenden Typs definiert. Beim Funktionsaufruf wird als Argument statt
Variable eine Adresse übergeben. Beachten Sie, dass für den Zugriff auf den Inhalt des Zeigers (einer Adresse) der Inhaltsoperator `*` benötigt wird.

```c                    ParameterI.c
#include <iostream>

//                       Die Adresse der Variable
//                       wird als Parameter übergeben.
//                        |
//                        v
void inkrementieren(int *variable){
  // Dereferenzierung des Zeigers und Inkrementierung
  // Die Klammer muss gesetzt werden, da der Operator
  // * eine höhere Priorität hat als der Inkrementoperator
  (*variable)++;
}

int main(void) {
  int a=0;
  inkrementieren(&a);
  cout<<"a = "<<a<<"\n";
  inkrementieren(&a);
  cout<<"a = "<<a<<"\n";
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

Der Vorteil der Verwendung der Zeiger als Parameter besteht darin, dass
in der Funktion mehrere Variablen auf eine elegante Weise verändert
werden können. Die Funktion hat somit quasi mehrere Ergebnisse.

```c     ParameterIII.c
#include <iostream>
#include <cmath>
using namespace std;

void tauschen(char *anna, char *hanna){
  char aux=*anna;
  *anna=*hanna;
  *hanna=aux;
}

int main(void) {
  char anna='A', hanna='H';
  cout<<anna<<" und "<<hanna<<"\n";
  tauschen(&anna, &hanna);
  cout<<anna<<" und "<<hanna<<"\n";
  return 0;
}
```
@LIA.eval(`["main.c"]`, `gcc -Wall main.c -o a.out`, `./a.out`)

> Achtung: C++ kennt zusätzlich des Konzept der Referenzen, die eine elegantere
> Schreibweise für die "call-by-reference"-Parameterübergabe ermöglichen.

### Dynamische Datenobjekte

                    {{0-1}}
********************************************************

**Exkurs: Aufbau des Speichers**

Wie wird der Speicher von einem C++ Programm eigentlich verwaltet? Wie wird diese Struktur ausgehend vom Start eines Programmes aufgebaut?

```ascii
        ------>  +----------------------------+
   höhere        | Kommandozeilen Parameter   |    (1)
   Adresse       | und Umgebungsvariablen     |
                 +----------------------------+
                 | Stack                      |    (2)
                 |............................|
                 |             |              |
                 |             v              |
                 |                            |
                 |             ^              |
                 |             |              |
                 |............................|
                 | Heap                       |    (3)
                 +----------------------------+
                 | uninitialisierte Daten     |    (4)
                 +----------------------------+
                 | initialisierte Daten       |    (5)
  kleinere       +----------------------------+
  Adresse        | Programmcode               |    (6)
       ------>   +----------------------------+                                                                     .
```

| Parameter                   | Stack                          | Heap                            |
|-----------------------------|--------------------------------|---------------------------------|
| Allokation und Deallokation | Automatisch durch den Compiler | Manuel durch den Programmierer  |
| Kosten                      | gering                         | ggf. höher durch Fragmentierung |
| Flexibilität                | feste Größe                    | Anpassungen möglich             |

C++ bietet die Möglichkeit den Speicherplatz für eine Variable zur Laufzeit zur Verfügung zu stellen.
Mit `new`-Operator wird der Speicherplatz bereit gestellt und mit `delete`-Operator (`delete[]`) wieder freigegeben.
                    
********************************************************

                    {{1-2}}
********************************************************

`new` erkennt die benötigte Speichermenge am angegebenen Datentyp und reserviert für die Variable auf dem Heap die entsperechde Byte-Menge.

```cpp                   new.cpp
#include <iostream>
using namespace std;

int main(void)
{
  int * ptr_a;
  ptr_a=new int;
  *ptr_a = 10;
  cout<<"Pointer ptr_a                  "<<ptr_a<<"\n";
  cout<<"Wert hinter dem Pointer ptr_a  "<<*ptr_a<<"\n";
  cout<<"Aus Maus!\n";
  delete ptr_a;
  delete ptr_a; // Fehler
  return 0;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)


> [Visualisierung mit Pythontutor](https://pythontutor.com/render.html#code=%23include%20%3Ciostream%3E%0Ausing%20namespace%20std%3B%0A%0Aint%20main%28void%29%0A%7B%0A%20%20int%20*%20ptr_a%3B%0A%20%20ptr_a%3Dnew%20int%3B%0A%20%20*ptr_a%20%3D%2010%3B%0A%20%20cout%3C%3C%22Pointer%20ptr_a%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%20%22%3C%3Cptr_a%3C%3C%22%5Cn%22%3B%0A%20%20cout%3C%3C%22Wert%20hinter%20dem%20Pointer%20ptr_a%20%20%22%3C%3C*ptr_a%3C%3C%22%5Cn%22%3B%0A%20%20cout%3C%3C%22Aus%20Maus!%5Cn%22%3B%0A%20%20delete%20ptr_a%3B%0A%20%20return%200%3B%0A%7D&cppShowMemAddrs=true&cumulative=false&curInstr=3&heapPrimitives=nevernest&mode=display&origin=opt-frontend.js&py=cpp_g%2B%2B9.3.0&rawInputLstJSON=%5B%5D&textReferences=false)

Achtung: 

+ `delete` daft nur einmal auf ein Objekt angewendet werden
+ `delete` daft ausschließlich auf mit new angelegte Objekte oder NULL-Pointer angewandt werden
+ Nach der Verwendung von `delete` ist das Objekt *undefiniert* (nicht gleich NULL)

> **Merke:** Die Verwendung von Zeigern kann zur unerwünschten Speicherfragmentierung und die Programmierfehler zu den Programmabstürzen und Speicherlecks führen. *Intelligente* Zeiger stellen sicher, dass Programme frei von Arbeitsspeicher- und Ressourcenverlusten sind.

********************************************************

                    {{2-3}}
********************************************************

```cpp                   newArray.cpp
#include <iostream>

class Person {
public:
    std::string name;
    int age;

    Person(std::string n, int a) : name(n), age(a) {}
    void display() {
        std::cout << "Name: " << name << ", Age: " << age << std::endl;
    }
};

int main() {
    // Anzahl der Personen
    const int numPersons = 3;

    // Array von Pointern auf Person-Objekte
    Person* persons[numPersons];

    // Erzeugen und Zuweisen der Person-Objekte
    persons[0] = new Person("Alice", 30);
    persons[1] = new Person("Bob", 25);
    persons[2] = new Person("Charlie", 20);

    // Anzeigen der Person-Objekte
    for (int i = 0; i < numPersons; ++i) {
        persons[i]->display();
    }

    // Freigeben des zugewiesenen Speichers
    for (int i = 0; i < numPersons; ++i) {
        delete persons[i];
    }

    return 0;
}
```
@LIA.eval(`["main.cpp"]`, `g++ -Wall main.cpp -o a.out`, `./a.out`)

> [Visualisierung mit Pythontutor](https://pythontutor.com/render.html#code=%23include%20%3Ciostream%3E%0A%0Aclass%20Person%20%7B%0Apublic%3A%0A%20%20%20%20std%3A%3Astring%20name%3B%0A%20%20%20%20int%20age%3B%0A%0A%20%20%20%20Person%28std%3A%3Astring%20n,%20int%20a%29%20%3A%20name%28n%29,%20age%28a%29%20%7B%7D%0A%20%20%20%20void%20display%28%29%20%7B%0A%20%20%20%20%20%20%20%20std%3A%3Acout%20%3C%3C%20%22Name%3A%20%22%20%3C%3C%20name%20%3C%3C%20%22,%20Age%3A%20%22%20%3C%3C%20age%20%3C%3C%20std%3A%3Aendl%3B%0A%20%20%20%20%7D%0A%7D%3B%0A%0Aint%20main%28%29%20%7B%0A%20%20%20%20//%20Anzahl%20der%20Personen%0A%20%20%20%20const%20int%20numPersons%20%3D%203%3B%0A%0A%20%20%20%20//%20Array%20von%20Pointern%20auf%20Person-Objekte%0A%20%20%20%20Person*%20persons%5BnumPersons%5D%3B%0A%0A%20%20%20%20//%20Erzeugen%20und%20Zuweisen%20der%20Person-Objekte%0A%20%20%20%20persons%5B0%5D%20%3D%20new%20Person%28%22Alice%22,%2030%29%3B%0A%20%20%20%20persons%5B1%5D%20%3D%20new%20Person%28%22Bob%22,%2025%29%3B%0A%20%20%20%20persons%5B2%5D%20%3D%20new%20Person%28%22Charlie%22,%2020%29%3B%0A%0A%20%20%20%20//%20Anzeigen%20der%20Person-Objekte%0A%20%20%20%20for%20%28int%20i%20%3D%200%3B%20i%20%3C%20numPersons%3B%20%2B%2Bi%29%20%7B%0A%20%20%20%20%20%20%20%20persons%5Bi%5D-%3Edisplay%28%29%3B%0A%20%20%20%20%7D%0A%0A%20%20%20%20//%20Freigeben%20des%20zugewiesenen%20Speichers%0A%20%20%20%20for%20%28int%20i%20%3D%200%3B%20i%20%3C%20numPersons%3B%20%2B%2Bi%29%20%7B%0A%20%20%20%20%20%20%20%20delete%20persons%5Bi%5D%3B%0A%20%20%20%20%7D%0A%0A%20%20%20%20return%200%3B%0A%7D&cppShowMemAddrs=true&cumulative=false&curInstr=0&heapPrimitives=nevernest&mode=display&origin=opt-frontend.js&py=cpp_g%2B%2B9.3.0&rawInputLstJSON=%5B%5D&textReferences=false)

********************************************************

## Smart Pointer 

                    {{0-1}}
********************************************************

Die Wirkungsweise eines intelligenten C++-Zeigers ähnelt dem Vorgehen bei der Objekterstellung in Sprachen wie C#: Sie erstellen das Objekt und überlassen es dann dem System, das Objekt zur richtigen Zeit zu löschen. Der Unterschied besteht darin, dass im Hintergrund keine separate Speicherbereinigung ausgeführt wird – der Arbeitsspeicher wird durch die C++-Standardregeln für den Gültigkeitsbereich verwaltet, sodass die Laufzeitumgebung schneller und effizienter ist.

C++11 implementiert verschiedene Pointer-Klassen für unterschiedliche Zwecke. Diese werden im folgenden vorgestellt:

+ `std::unique_ptr` — smart pointer der den Zugriff auf eine dynamisch allokierte Ressource verwaltet.
+ `std::shared_ptr` — smart pointer der den Zugriff auf eine dynamisch allokierte Ressource verwaltet, wobei mehrere Instanzen für ein und die selbe Ressource bestehen können.
+ `std::weak_ptr` — analog zum `std::shared_ptr` aber ohne Überwachung der entsprechenden Pointerinstanzen.


********************************************************

                    {{1-2}}
********************************************************

`std::shared_ptr` sind intelligente Zeiger, die ein Objekt über einen Zeiger "verwalten". Mehrere `shared_ptr` Instanzen können das selbe Objekt besitzen. Das Objekt wird zerstört, wenn:

+ der letzte shared_ptr, der das Objekt besitzt, zerstört wird oder
+ dem letzten shared_ptr, der das Objekt besitzt, ein neues Objekt mittles operator= oder reset() zugewiesen wird.

Das Objekt wird entweder mittels einer delete-expression oder einem benutzerdefiniertem deleter zerstört, der dem shared_ptr während der Erzeugung übergeben wird.

```cpp                     SharedPointer.cpp
#include <iostream>
#include <memory>   

class MyClass{
  public:
    MyClass(){
      std::cout << "Constructor executed" << std::endl;
    }
    ~MyClass(){
      std::cout << "Deconstructor executed" << std::endl;
    }
    void print(){
      std::cout << "That's all!" << std::endl;
    }
};

int main()
{
  {
    std::shared_ptr<MyClass> A = std::make_shared<MyClass>();
    A->print();
    std::shared_ptr<MyClass> B = std::make_shared<MyClass>();
    B.use_count();
  }
  std::cout << "Scope left" << std::endl;
  return EXIT_SUCCESS;
}
```
@LIA.eval(`["main.c"]`, `g++ -Wall main.c -o a.out`, `./a.out`)

Vergleiche [cppreference](https://en.cppreference.com/w/cpp/memory/shared_ptr) für die Nutzung der API.


********************************************************

## Weiterführende Tutorials

!?[](https://www.youtube.com/watch?v=7GFYpxmldHc)

!?[](https://www.youtube.com/watch?v=UOB7-B2MfwA)