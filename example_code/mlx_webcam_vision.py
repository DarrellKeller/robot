import cv2
import json
import time
import argparse
from PIL import Image
import mlx.core as mx
from mlx_vlm import load
from mlx_vlm.generate import generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.utils import load_config

def capture_image():
    # Initialize webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        raise Exception("Could not open webcam")
    
    # Capture frame
    ret, frame = cap.read()
    
    if not ret:
        raise Exception("Failed to capture image")
    
    # Release webcam
    cap.release()
    
    # Convert frame to RGB (from BGR)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Convert to PIL Image
    pil_image = Image.fromarray(frame_rgb)
    
    return pil_image

def analyze_image(image, model, processor, config, prompt="Can you describe this image?"):
    try:
        formatted_prompt = apply_chat_template(
            processor, config, prompt, num_images=1
        )
        response = generate(
            model=model,
            processor=processor,
            image=[image],
            prompt=formatted_prompt
        )
        if isinstance(response, list):
            description = response[0]
        else:
            description = response
        result = {"description": description}
        return result
    except Exception as e:
        return {"error": str(e)}

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Webcam vision using MLX-VLM")
    parser.add_argument("--model", type=str, default="mlx-community/SmolVLM2-500M-Video-Instruct-mlx",
                      help="Model to use for image analysis")
    parser.add_argument("--prompt", type=str, default="Describe this image in 2 sentences. note that you are short ",
                      help="Prompt for image analysis")
    args = parser.parse_args()

    # Load the model and processor
    print(f"Loading model: {args.model}")
    model, processor = load(args.model)
    config = load_config(args.model)
    
    print("Starting continuous image analysis...")
    print("Press Ctrl+C to stop")
    
    while True:
        try:
            print("\n" + "="*50)
            print("Capturing image from webcam...")
            image = capture_image()
            
            print("Analyzing image with MLX-VLM...")
            start_time = time.time()
            result = analyze_image(image, model, processor, config, args.prompt)
            end_time = time.time()
            
            print("\nAnalysis Result:")
            print(json.dumps(result, indent=2))
            print(f"\nAnalysis took {end_time - start_time:.2f} seconds")
            
            # Wait for 2 seconds before next capture
            print("\nWaiting 2 seconds before next capture...")
            time.sleep(2)
            
        except KeyboardInterrupt:
            print("\nStopping image analysis...")
            break
        except Exception as e:
            print(f"An error occurred: {str(e)}")
            print("Retrying in 2 seconds...")
            time.sleep(2)
            continue

if __name__ == "__main__":
    main() 