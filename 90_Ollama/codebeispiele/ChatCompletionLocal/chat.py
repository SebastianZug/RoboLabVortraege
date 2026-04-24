from openai import OpenAI

client = OpenAI(
    base_url="http://localhost:11434/v1",
    api_key="ollama",
)

resp = client.chat.completions.create(
    model="mistral-small",
    messages=[
        {"role": "user",
         "content": "Nenne drei Fakten zur TU Bergakademie Freiberg."}
    ],
)

print(resp.choices[0].message.content)
