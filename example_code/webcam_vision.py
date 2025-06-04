import cv2
from openai import OpenAI
import base64
import time
import os
import json

# --- Configuration ---
# API_KEY = os.environ.get("OPENAI_API_KEY") # Recommended: Set as environment variable
# if not API_KEY:
#     raise ValueError("OPENAI_API_KEY environment variable not set.")
# client = OpenAI(api_key=API_KEY)

# For local server like LM Studio or Ollama with OpenAI compatible endpoint:
client = OpenAI(base_url="http://localhost:1234/v1", api_key="not-needed")


MODEL_NAME = "gpt-4-vision-preview" # Standard OpenAI model
# For local/custom model, use the model name your server provides, e.g., "moondream" or a specific GGUF name
# MODEL_NAME = "moondream" # Example for a local model if your server uses this name

MAX_TOKENS = 300
IMAGE_CAPTURE_INTERVAL_SECONDS = 10 # Time between image captures

def capture_image_from_webcam():
    """Captures a single frame from the webcam."""
    cap = cv2.VideoCapture(0) # 0 is usually the default webcam
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return None
    
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("Error: Could not read frame from webcam.")
        return None
    
    return frame

def encode_image_to_base64(frame):
    """Encodes a CV2 frame (numpy array) to a base64 string."""
    _, buffer = cv2.imencode('.jpg', frame)
    return base64.b64encode(buffer).decode('utf-8')

def analyze_image_with_gpt4v(base64_image, prompt):
    """Sends the image and prompt to GPT-4 Vision API and returns the description."""
    print("Sending request to Vision API...")
    try:
        response = client.chat.completions.create(
            model=MODEL_NAME,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            }
                        },
                    ],
                }
            ],
            max_tokens=MAX_TOKENS,
        )
        # print("Full API Response:", response) # Debugging: print the full response
        description = response.choices[0].message.content
        return description
    except Exception as e:
        print(f"Error calling Vision API: {e}")
        return None

if __name__ == "__main__":
    print("Starting webcam vision analysis...")
    print(f"Using model: {MODEL_NAME}")
    print(f"Capturing image every {IMAGE_CAPTURE_INTERVAL_SECONDS} seconds. Press Ctrl+C to stop.")

    default_prompt = "Describe this image in detail. What objects do you see? What is happening?"
    
    try:
        while True:
            prompt_input = input(f"Enter your prompt (or press Enter for default: '{default_prompt}'): ")
            current_prompt = prompt_input if prompt_input else default_prompt

            print("\nCapturing image...")
            frame = capture_image_from_webcam()
            
            if frame is not None:
                print("Encoding image...")
                base64_image_data = encode_image_to_base64(frame)
                
                print("Analyzing image...")
                start_time = time.time()
                description = analyze_image_with_gpt4v(base64_image_data, current_prompt)
                end_time = time.time()
                
                if description:
                    print("\n--- Vision Analysis ---")
                    print(f"Prompt: {current_prompt}")
                    print(f"Description: {description}")
                    print(f"Time taken for analysis: {end_time - start_time:.2f} seconds")
                    print("-----------------------\n")
                else:
                    print("Failed to get description from API.")
            else:
                print("Failed to capture image.")
            
            print(f"Waiting for {IMAGE_CAPTURE_INTERVAL_SECONDS} seconds before next capture...")
            time.sleep(IMAGE_CAPTURE_INTERVAL_SECONDS)
            
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        # Clean up resources if any (e.g., close windows if cv2.imshow was used)
        cv2.destroyAllWindows() 