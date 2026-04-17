# PixtralVision – Bilder per Mistral-API beschreiben

Zeigt, wie Mistrals multimodales Modell `pixtral-large-latest` ein Bild
entgegennimmt und dazu eine textuelle Beschreibung liefert.

## Dateien

| Datei            | Zweck                                                                  |
| :--------------- | :--------------------------------------------------------------------- |
| `pyproject.toml` | Abhängigkeiten (mistralai, python-dotenv, pillow).                     |
| `.env.example`   | Vorlage – nach `.env` kopieren und Key eintragen.                      |
| `.gitignore`     | `.env`, `.venv/` u.ä. aus dem Repo halten.                             |
| `describe.py`    | CLI-Skript: Bildpfad rein, Beschreibung raus.                          |
| `images/OCR.png` | Handschriftliche Formel als Beispielbild.                              |
| `images/eisen_kohlenstoff.png` | Eisen-Kohlenstoff-Phasendiagramm (Wikimedia Commons, CC BY-SA 4.0). |

## Ausführen

```bash
cd codebeispiele/PixtralVision
uv sync
cp .env.example .env   # Key eintragen
```

Handschriftliche Formel in LaTeX umsetzen:

```bash
uv run describe.py images/OCR.png \
  "Auf dem Bild ist eine handschriftliche mathematische Formel zu sehen. \
   Gib sie als LaTeX-Code aus, eingeschlossen in \$\$ ... \$\$. \
   Antworte ausschließlich mit dem LaTeX-Code, ohne erklärenden Text."
```

Liefert z. B.:

```latex
$$y = 2x^2 - 4x + 5$$
```

Komplexes Diagramm interpretieren (Eisen-Kohlenstoff-Phasendiagramm):

```bash
uv run describe.py images/eisen_kohlenstoff.png \
  "Welche Aussage trifft das Diagramm? Beschreibe sachlich, was abgebildet ist, \
   welche Achsen verwendet werden und welche Phasen oder Bereiche markiert sind."
```

Dieses Beispiel zeigt die **Grenzen** von Pixtral: obwohl das Bild eindeutig ein
Eisen-Kohlenstoff-Diagramm ist, generalisiert das Modell gern zu einem
klassischen T/p-Phasendiagramm reiner Substanzen, vertauscht Achsen und erfindet
Zahlenwerte. Die Beschreibung klingt souverän, ist aber sachlich falsch –
ein schönes Argument dafür, LLM-Ausgaben in fachlich spezialisierten Domänen
immer gegenzuprüfen.

## Wie es funktioniert

1. Das Bild wird geladen und per Pillow auf max. 1024 px Längskante skaliert –
   das spart Bild-Tokens und entspannt Rate-Limits.
2. Das skalierte Bild wird als JPEG Base64-kodiert.
3. Im `messages`-Array wird der `content` zur Liste – ein Element für den
   Text-Prompt, eines mit `type: image_url` und einem `data:`-URI für das Bild.
4. Pixtral antwortet wie eine normale Chat-Completion mit Text in `choices[0].message.content`.

Statt Base64 funktioniert auch eine echte URL (`https://...`) – nützlich bei
Bildern, die ohnehin online verfügbar sind. Base64 ist sinnvoll für lokale
Dateien und für Daten, die nicht öffentlich erreichbar sein sollen.

## Einsatzfelder

+ Handschriftliche Notizen und Formeln in Markup (LaTeX, Markdown) überführen.
+ Automatische Beschreibung von Diagrammen/Plots für Barrierefreiheit (Alt-Texte).
+ Screenshot-OCR für Altunterlagen.
+ Plausibilitätschecks bei Abbildungen in Lehrmaterial – mit der Einschränkung,
  dass Fachdomänen (Phasendiagramme, Schaltpläne, Normzeichnungen) besondere
  Sorgfalt bei der Validierung erfordern.
