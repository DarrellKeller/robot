import cv2
from PIL import Image
import mlx.core as mx
from mlx_vlm import load
from mlx_vlm.generate import generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config
import time

MODEL_NAME = "mlx-community/SmolVLM2-500M-Video-Instruct-mlx"
DEFAULT_PROMPT = "Describe the scene in great detail."

# Global variables for the model and processor
model = None
processor = None
config = None

def initialize_vision_model(model_name=MODEL_NAME):
    """Loads the MLX-VLM model and processor."""
    global model, processor, config
    if model is not None:
        print("Vision model already initialized.")
        return True
    try:
        print(f"Loading vision model: {model_name}")
        model, processor = load(model_name)
        config = load_config(model_name)
        print("Vision model loaded successfully.")
        return True
    except Exception as e:
        print(f"Error loading vision model: {e}")
        return False

def capture_image():
    """Captures an image from the webcam."""
    cap = cv2.VideoCapture(0) # Assumes webcam is at index 0
    if not cap.isOpened():
        print("Could not open webcam")
        return None
    
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        print("Failed to capture image")
        return None
    
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(frame_rgb)
    return pil_image

def analyze_image(pil_image, prompt=DEFAULT_PROMPT):
    """Analyzes the given PIL image using the loaded MLX-VLM model."""
    global model, processor, config
    if model is None or processor is None or config is None:
        print("Vision model not initialized. Call initialize_vision_model() first.")
        return {"error": "Vision model not initialized."}
    
    try:
        formatted_prompt = apply_chat_template(
            processor, config, prompt, num_images=1
        )
        response = generate(
            model=model,
            processor=processor,
            image=[pil_image],
            prompt=formatted_prompt
        )
        if isinstance(response, list):
            description = response[0]
        else:
            description = response
        return {"description": description.strip()}
    except Exception as e:
        print(f"Error during image analysis: {e}")
        return {"error": str(e)}

def capture_and_analyze_image(prompt=DEFAULT_PROMPT):
    """Captures an image and analyzes it."""
    print("Capturing image...")
    img = capture_image()
    if img:
        print("Analyzing image...")
        return analyze_image(img, prompt)
    else:
        return {"error": "Failed to capture image for analysis."}

if __name__ == '__main__':
    # Example usage:
    if initialize_vision_model():
        for i in range(2):
            print(f"\n--- Analysis Attempt {i+1} ---")
            start_time = time.time()
            analysis_result = capture_and_analyze_image(
                prompt="Describe what you see."
            )
            end_time = time.time()
            
            if 'error' in analysis_result:
                print(f"Error: {analysis_result['error']}")
            else:
                print(f"Description: {analysis_result['description']}")
            print(f"Capture and analysis took {end_time - start_time:.2f} seconds.")
            time.sleep(1)
    else:
        print("Failed to initialize vision model.") 