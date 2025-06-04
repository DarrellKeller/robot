import os
import requests
from tqdm import tqdm

# URL of the voice model
VOICE_MODEL_URL = "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/ryan/low/en_US-ryan-low.onnx?download=true"
# Filename to save the model as
VOICE_MODEL_FILENAME = "en_US-ryan-low.onnx"

# URL of the voice model config
VOICE_CONFIG_URL = "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/ryan/low/en_US-ryan-low.onnx.json?download=true"
# Filename to save the config as
VOICE_CONFIG_FILENAME = "en_US-ryan-low.onnx.json"


def download_file(url, filename):
    """Downloads a file from a URL to a local filename with a progress bar."""
    try:
        response = requests.get(url, stream=True)
        response.raise_for_status()  # Raise an exception for HTTP errors

        total_size = int(response.headers.get('content-length', 0))
        
        with open(filename, 'wb') as f, tqdm(
            desc=filename,
            total=total_size,
            unit='iB',
            unit_scale=True,
            unit_divisor=1024,
        ) as bar:
            for data in response.iter_content(chunk_size=1024):
                size = f.write(data)
                bar.update(size)
        print(f"Successfully downloaded {filename}")
    except requests.exceptions.RequestException as e:
        print(f"Error downloading {filename}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred while downloading {filename}: {e}")

if __name__ == "__main__":
    print(f"Downloading voice model: {VOICE_MODEL_FILENAME}...")
    download_file(VOICE_MODEL_URL, VOICE_MODEL_FILENAME)
    
    print(f"\nDownloading voice config: {VOICE_CONFIG_FILENAME}...")
    download_file(VOICE_CONFIG_URL, VOICE_CONFIG_FILENAME)

    print("\nDownload process finished.") 