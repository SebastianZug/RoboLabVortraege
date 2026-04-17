import os
import pandas as pd
from dotenv import load_dotenv
from pydantic import BaseModel, field_validator
from mistralai.client import Mistral

load_dotenv()

# timeout_ms hebt den Default (~30 s) an, damit längere Antworten nicht abbrechen.
client = Mistral(api_key=os.environ["MISTRAL_API_KEY"], timeout_ms=120_000)


class AbstractInfo(BaseModel):
    field: str
    method: str
    languages: list[str]

    # Falls das Modell doch eine Liste schickt: zu Komma-String zusammenfügen.
    @field_validator("field", "method", mode="before")
    @classmethod
    def _list_to_string(cls, v):
        return ", ".join(v) if isinstance(v, list) else v


SYSTEM_PROMPT = (
    "Extrahiere aus dem Abstract drei Informationen und gib sie als JSON zurück:\n"
    "- 'field'    : EIN kurzer String (max. 5 Wörter), der das Forschungsgebiet nennt.\n"
    "- 'method'   : EIN kurzer String (max. 10 Wörter), der die zentrale Methode nennt.\n"
    "- 'languages': Liste von Strings mit allen explizit genannten Programmiersprachen.\n"
    "                Falls keine genannt ist, gib eine leere Liste [] zurück.\n"
    "Antworte ausschließlich mit JSON, ohne erklärenden Text."
)

def classify(abstract: str) -> AbstractInfo:
    resp = client.chat.complete(
        model="mistral-small-latest",
        temperature=0.0,
        response_format={"type": "json_object"},
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": abstract},
        ],
    )
    return AbstractInfo.model_validate_json(resp.choices[0].message.content)


def main() -> None:
    df = pd.read_csv("abstracts.csv")
    results = df["text"].apply(lambda t: pd.Series(classify(t).model_dump()))
    df[["field", "method", "languages"]] = results

    print(df[["id", "field", "method", "languages"]].to_string(index=False))

if __name__ == "__main__":
    main()
