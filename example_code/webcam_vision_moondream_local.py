import cv2
from openai import OpenAI
import base64
import time
import os

# Configuration for Ollama (or other local OpenAI-compatible server)
OLLAMA_BASE_URL = "http://localhost:11434/v1"  # Default Ollama API endpoint
OLLAMA_API_KEY = "ollama"  # Ollama doesn't require a key, but the library might expect one.
OLLAMA_MODEL = "moondream" # Make sure this model is pulled in Ollama: `ollama pull moondream`

client = OpenAI(base_url=OLLAMA_BASE_URL, api_key=OLLAMA_API_KEY)

MAX_TOKENS = 150  # Adjust as needed for moondream verbosity
IMAGE_CAPTURE_INTERVAL_SECONDS = 10

def capture_image_from_webcam():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return None
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("Error: Could not read frame.")
        return None
    return frame

def encode_image_to_base64(frame):
    _, buffer = cv2.imencode('.jpg', frame)
    return base64.b64encode(buffer).decode('utf-8')

def analyze_image_with_ollama(base64_image, prompt):
    print(f"Sending request to Ollama model: {OLLAMA_MODEL}...")
    try:
        response = client.chat.completions.create(
            model=OLLAMA_MODEL,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
                        },
                    ],
                }
            ],
            max_tokens=MAX_TOKENS
        )
        description = response.choices[0].message.content
        return description
    except Exception as e:
        print(f"Error calling Ollama API: {e}")
        return None

if __name__ == "__main__":
    print("Starting webcam vision analysis with local Ollama (Moondream)...")
    print(f"Using model: {OLLAMA_MODEL} via {OLLAMA_BASE_URL}")
    
    default_prompt = "Describe this image briefly."
    
    try:
        while True:
            current_prompt = input(f"Enter prompt (default: '{default_prompt}'): ") or default_prompt
            frame = capture_image_from_webcam()
            if frame is not None:
                base64_image_data = encode_image_to_base64(frame)
                description = analyze_image_with_ollama(base64_image_data, current_prompt)
                if description:
                    print(f"\nOllama's Description: {description}\n")
                else:
                    print("Failed to get description from Ollama.")
            else:
                print("Failed to capture image.")
            time.sleep(IMAGE_CAPTURE_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        cv2.destroyAllWindows() 