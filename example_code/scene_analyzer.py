import ollama
import json
import time

def analyze_scenes(scene_descriptions):
    try:
        # Format the scenes into a clear prompt
        scenes_text = "\n".join([f"Scene {i+1}: {desc}" for i, desc in enumerate(scene_descriptions)])
        
        prompt = f"""Given these three scene descriptions, analyze them and choose the most appropriate one to proceed with.
        Return your response in JSON format with the following structure:
        {{
            "reasoning": "Think through, if you were a robot why you would drive forward here",
            "chosen_scene": <number>
        }}

        Here are the scenes:
        {scenes_text}
        """

        response = ollama.chat(
            model='gemma3',
            format='json',
            messages=[{{
                'role': 'user',
                'content': prompt
            }}]
        )
        
        # Extract the response content
        response_text = response['message']['content']
        
        # Try to parse the response as JSON
        try:
            json_response = json.loads(response_text)
            return json_response
        except json.JSONDecodeError:
            return {"error": "Response was not valid JSON", "raw_response": response_text}
            
    except Exception as e:
        return {"error": str(e)}

def main():
    # Example scene descriptions (you can modify these)
    scenes = [
        "A bright, modern office space with multiple computer monitors and a person sitting at a desk",
        "A dark room with a single lamp and a person reading a book in a comfortable chair",
        "An outdoor garden setting with flowers in bloom and a small table with tea cups"
    ]
    
    print("Starting scene analysis...")
    print("Press Ctrl+C to stop")
    
    while True:
        try:
            start_time = time.time()
            result = analyze_scenes(scenes)
            end_time = time.time()
            
            print(f"\nResponse time: {end_time - start_time:.2f} seconds")
            print(json.dumps(result, indent=2))
            
        except KeyboardInterrupt:
            print("\nStopping scene analysis...")
            break
        except Exception as e:
            print(f"An error occurred: {str(e)}")
            continue

if __name__ == "__main__":
    main() 