import cv2
from PIL import Image
import mlx.core as mx
from mlx_vlm import load
from mlx_vlm.generate import generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config
import time
import os

try:
    import google.generativeai as genai
except Exception:
    genai = None

LFM_450M_MODEL = "mlx-community/LFM2.5-VL-450M-6bit"
MODEL_NAME = LFM_450M_MODEL
DEFAULT_PROMPT = "Describe the scene in great detail."
GEMINI_MODEL_NAME = "gemini-3-flash-preview"
VISION_BACKEND = "local" # "local" or "gemini"
_gemini_api_key = None

# Global variables for the model and processor
model = None
processor = None
config = None


def configure_local_vision_processor(model_name):
    """Apply the MLX compatibility setting required by LFM2.5-VL-450M."""
    if model_name != LFM_450M_MODEL:
        return

    # The 450M MLX conversion declares 256 raw image patches, but its 512px
    # tiles contain 1,024 patches before pixel unshuffle. Keeping the full
    # tile prevents an image-token shape mismatch during inference.
    image_processor = processor.image_processor
    tile_patch_count = (image_processor.tile_size // image_processor.patch_size) ** 2
    image_processor.max_num_patches = max(
        image_processor.max_num_patches, tile_patch_count
    )

def configure_vision_backend(backend, gemini_api_key=None, gemini_model_name=None):
    """Configure which vision backend to use (local MLX or Gemini)."""
    global VISION_BACKEND, GEMINI_MODEL_NAME, _gemini_api_key
    if gemini_model_name:
        GEMINI_MODEL_NAME = gemini_model_name

    backend = (backend or "").strip().lower()
    if backend not in ("local", "gemini"):
        print(f"Unknown vision backend '{backend}'. Defaulting to local.")
        VISION_BACKEND = "local"
        return True

    if backend == "gemini":
        if genai is None:
            print("Gemini vision backend requested but google-generativeai is not installed.")
            return False
        _gemini_api_key = gemini_api_key or os.getenv("GEMINI_API_KEY")
        if not _gemini_api_key:
            print("Gemini vision backend requested but GEMINI_API_KEY is not set.")
            return False
        genai.configure(api_key=_gemini_api_key)
        VISION_BACKEND = "gemini"
        print(f"Vision backend set to Gemini ({GEMINI_MODEL_NAME}).")
        return True

    VISION_BACKEND = "local"
    print(f"Vision backend set to local ({MODEL_NAME}).")
    return True

def initialize_vision_model(model_name=MODEL_NAME):
    """Loads the MLX-VLM model and processor."""
    global model, processor, config
    if VISION_BACKEND == "gemini":
        print(f"Using Gemini vision backend ({GEMINI_MODEL_NAME}); local model will not be loaded.")
        return True
    if model is not None:
        print("Vision model already initialized.")
        return True
    try:
        print(f"Loading vision model: {model_name}")
        model, processor = load(model_name)
        config = load_config(model_name)
        configure_local_vision_processor(model_name)
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
    if VISION_BACKEND == "gemini":
        if genai is None:
            return {"error": "Gemini backend unavailable (missing google-generativeai)."}
        try:
            model = genai.GenerativeModel(GEMINI_MODEL_NAME)
            response = model.generate_content(
                [prompt, pil_image],
                generation_config={"temperature": 0}
            )
            description = (response.text or "").strip()
            return {"description": description}
        except Exception as e:
            print(f"Error during Gemini image analysis: {e}")
            return {"error": str(e)}
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
        # mlx-vlm 0.7+ returns a GenerationResult; older releases returned
        # either a string or a list. Support both so the robot's existing
        # vision path remains usable after the runtime upgrade.
        if hasattr(response, "text"):
            description = response.text
        elif isinstance(response, list):
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
