import base64
import io
import os
import sys
from pathlib import Path
from dotenv import load_dotenv
from PIL import Image
from mistralai.client import Mistral

load_dotenv()

client = Mistral(api_key=os.environ["MISTRAL_API_KEY"], timeout_ms=120_000)

# Pixtral rechnet ~1 Token pro 30x30-px-Patch; Downscaling spart Tokens und Zeit.
MAX_EDGE_PX = 1024


def encode_image(path: Path) -> tuple[str, str]:
    """Lädt das Bild, skaliert es auf MAX_EDGE_PX herunter und gibt (mime, base64) zurück."""
    img = Image.open(path)
    img.thumbnail((MAX_EDGE_PX, MAX_EDGE_PX))  # in-place, behält Seitenverhältnis

    buf = io.BytesIO()
    if img.mode in ("RGBA", "P"):
        img = img.convert("RGB")
    img.save(buf, format="JPEG", quality=85)
    return "image/jpeg", base64.b64encode(buf.getvalue()).decode()


def describe_image(path: Path, prompt: str) -> str:
    mime, b64 = encode_image(path)

    resp = client.chat.complete(
        model="pixtral-large-latest",
        messages=[{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url",
                 "image_url": f"data:{mime};base64,{b64}"},
            ],
        }],
    )
    return resp.choices[0].message.content


def main() -> None:
    if len(sys.argv) < 2:
        print("Aufruf: uv run describe.py <bilddatei> [prompt]")
        sys.exit(1)

    path = Path(sys.argv[1])
    prompt = sys.argv[2] if len(sys.argv) > 2 else (
        "Beschreibe sachlich, was auf dem Bild zu sehen ist. "
        "Nenne erkennbare Objekte, Geräte oder Strukturen."
    )

    print(describe_image(path, prompt))


if __name__ == "__main__":
    main()
