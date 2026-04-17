import os
from dotenv import load_dotenv
from mistralai.client import Mistral

load_dotenv()

client = Mistral(api_key=os.environ["MISTRAL_API_KEY"])

resp = client.chat.complete(
    model="mistral-small-latest",
    messages=[
        {"role": "user",
         "content": "Nenne drei Fakten zur TU Bergakademie Freiberg."}
    ],
)

print(resp.choices[0].message.content)
