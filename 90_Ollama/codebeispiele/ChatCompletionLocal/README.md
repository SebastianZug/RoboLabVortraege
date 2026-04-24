# ChatCompletionLocal – Minimalbeispiel gegen Ollama

Spiegelbild zum Mistral-Projekt unter `89_MistralAPI/codebeispiele/ChatCompletion/`,
aber gegen einen lokal laufenden Ollama-Server. Derselbe Code-Stil, kein API-Key,
keine Netzwerkabhängigkeit.

## Voraussetzung

Ollama muss auf dem Rechner laufen und das Modell `mistral-small` geladen sein:

```bash
# Ollama installieren (Linux)
curl -fsSL https://ollama.com/install.sh | sh

# Modell laden (~14 GB)
ollama pull mistral-small

# Server läuft danach automatisch auf http://localhost:11434
```

Test ohne Python:

```bash
curl http://localhost:11434/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model": "mistral-small", "messages": [{"role": "user", "content": "Hallo"}]}'
```

## Einrichtung

```bash
cd codebeispiele/ChatCompletionLocal
uv sync
```

Keine `.env` nötig – Ollama verlangt keinen API-Key.

## Ausführen

```bash
uv run chat.py
```

## Warum `openai` statt `mistralai`?

Das `openai`-SDK ist der kleinste gemeinsame Nenner: Es funktioniert gegen jede
OpenAI-kompatible HTTP-API – OpenAI selbst, Mistral, Ollama, Groq, Together,
vLLM auf der DGX. Ein `base_url`-Wechsel reicht, um zwischen Backends zu
wechseln, ohne Code-Änderungen.

## Weiteres Modell ausprobieren

```bash
ollama pull qwen2.5:14b
```

Dann in `chat.py` `model="mistral-small"` durch `model="qwen2.5:14b"` ersetzen.
