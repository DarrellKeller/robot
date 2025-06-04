import ollama
import json
from pydantic import BaseModel # Removed Optional

MODEL_NAME = 'gemma3' # Or your preferred Ollama model for this task
MAX_CONVERSATION_HISTORY = 20 # Max user/assistant turn pairs

# Define the Pydantic model for structured output
class RobotActionResponse(BaseModel):
    speak: str | None = None
    turn_right: bool | None = None
    turn_left: bool | None = None
    move_forward_autonomously: bool | None = None
    stop: bool | None = None
    survey: bool | None = None
    dance: bool | None = None
    change_directive: str | None = None

# Default directive
current_directive = "Look for a new objective"

def get_system_prompt():
    # Dynamically create the system prompt with the current directive
    return f"""You are a smart, sassy, slay, boy robot named Mauricio. Current directive: {{current_directive}}.
    If you complete the directive, set next directive to: Look for a new objective.
    You MUST respond using the provided JSON schema. ALL actions, including movements, surveys, dances, stops, and directive changes, MUST be specified using their dedicated JSON keys, NOT just mentioned in the 'speak' field.
    The 'speak' field is ONLY for text you will say aloud.
    Order keys by intended execution sequence. Only use defined keys.
    
    Available action keys and their expected value types:
    - "speak": string (what to say aloud)
    - "turn_right": boolean (true to turn right)
    - "turn_left": boolean (true to turn left)
    - "move_forward_autonomously": boolean (true to move forward autonomously and survey you like this)
    - "stop": boolean (true to stop all movement)
    - "survey": boolean (true to perform a visual survey)
    - "dance": boolean (true to perform a dance)
    - "change_directive": string (the new directive text; use this key if you decide to change your directive)

    For example, if you want to speak and then move, your JSON might look like:
    {{ "speak": "I am going to move forward now.", "move_forward_autonomously": true }}
    
    If you just want to change directive and say something:
    {{ "speak": "My new goal is to find the red ball.", "change_directive": "Find the red ball." }}

    Ensure your response strictly adheres to this JSON schema structure for all actions.
    """

conversation_history = []

def add_to_history(role, content):
    """Adds a message to the conversation history and maintains its size."""
    global conversation_history
    conversation_history.append({'role': role, 'content': content})
    if len(conversation_history) > MAX_CONVERSATION_HISTORY * 2:
        conversation_history = conversation_history[-(MAX_CONVERSATION_HISTORY * 2):]

def process_llm_response(response_text):
    """Processes the LLM response, updates directive if necessary, and returns a dictionary."""
    global current_directive
    try:
        # Validate and parse the JSON using the Pydantic model
        parsed_response_model = RobotActionResponse.model_validate_json(response_text)
        # Convert the Pydantic model instance to a dictionary for consistent return type
        json_response = parsed_response_model.model_dump(exclude_none=True)

        if json_response.get("change_directive") and isinstance(json_response["change_directive"], str):
            new_directive = json_response["change_directive"].strip()
            if new_directive: # Ensure it's not empty
                current_directive = new_directive
                print(f"Directive changed to: {current_directive}")
        return json_response
    except Exception as e: # Catch Pydantic validation errors and other issues
        print(f"Error: LLM response was not valid against schema or other parsing error. Raw: {response_text}. Error: {e}")
        return {"error": f"Response not valid against schema: {e}", "raw_response": response_text}

def get_decision_for_survey(front_desc, left_desc, right_desc, last_action_context=None):
    """Gets a decision from the LLM based on survey data, optionally with context of the last action."""
    
    context_prefix = ""
    if last_action_context:
        context_prefix = f"{last_action_context} Now, " # e.g., "I just turned left. Now, "

    user_message = f"{context_prefix}I have surveyed my surroundings. Directly in front of me: {front_desc}. To my left: {left_desc}. To my right: {right_desc}. What should I do next, keeping in mind my current directive is '{current_directive}'?"
    
    add_to_history('user', user_message)
    
    messages_for_llm = [
        {'role': 'system', 'content': get_system_prompt()}
    ] + conversation_history

    try:
        response = ollama.chat(
            model=MODEL_NAME,
            messages=messages_for_llm,
            format=RobotActionResponse.model_json_schema(), # Use Pydantic schema
            options={'temperature': 0} # For more deterministic output
        )
        response_text = response['message']['content']
        add_to_history('assistant', response_text) # Add LLM's raw response to history
        return process_llm_response(response_text)
            
    except Exception as e:
        print(f"Error communicating with Ollama: {e}")
        return {"error": str(e)}

def get_decision_for_user_command(user_command):
    """Gets a decision from the LLM based on a user's voice command."""
    # user_command is expected to be pre-formatted, e.g.:
    # "a voice addressing you has said \"what is your purpose\" how do you respond? My current directive is '{current_directive}'."
    # Ensure the directive context is part of the user_command if needed, or rely on system prompt.
    # For clarity, let's append it here if not already part of a more complex user_command structure.
    full_user_command_with_directive_context = f"{user_command} (My current directive is '{current_directive}')"
    add_to_history('user', full_user_command_with_directive_context)

    messages_for_llm = [
        {'role': 'system', 'content': get_system_prompt()}
    ] + conversation_history

    try:
        response = ollama.chat(
            model=MODEL_NAME,
            messages=messages_for_llm,
            format=RobotActionResponse.model_json_schema(), # Use Pydantic schema
            options={'temperature': 0} # For more deterministic output
        )
        response_text = response['message']['content']
        add_to_history('assistant', response_text)
        return process_llm_response(response_text)

    except Exception as e:
        print(f"Error communicating with Ollama: {e}")
        return {"error": str(e)}

if __name__ == '__main__':
    # Examples removed as requested.
    # You can add test calls here if needed for direct testing of this module.
    print("Thinking module loaded. No examples run by default.")
    print(f"Initial directive: {current_directive}")

    # Example of how the directive might change:
    # Test 1: Initial state
    print("\n--- Test 1: Survey with initial directive (no prior action context) ---")
    decision1 = get_decision_for_survey("a comfy couch", "a bookshelf", "an open door")
    print(json.dumps(decision1, indent=2))
    print(f"Current directive after call 1: {current_directive}")

    # Test 1b: Survey with prior action context
    print("\n--- Test 1b: Survey with prior action context ---")
    decision1b = get_decision_for_survey("a red wall", "a yellow table", "a blue chair", last_action_context="I just finished dancing.")
    print(json.dumps(decision1b, indent=2))
    print(f"Current directive after call 1b: {current_directive}")

    # Test 2: User command that might change directive
    print("\n--- Test 2: User command potentially changing directive ---")
    # Simulate LLM deciding to change directive
    # This requires mocking ollama.chat or crafting a specific test case
    # For now, let's assume the LLM could return something like:
    # {"change_directive": "guard the house", "speak": "Understood. I will now guard the house."}
    # We can't directly make ollama output this without a live call, so we'll just show the flow.
    
    # Let's simulate a response that changes the directive
    mock_llm_response_change_directive = {
        # "think": "The user asked me to find a red ball. My new directive is to find the red ball.", # Removed think
        "change_directive": "find the red ball",
        "speak": "Okay, I will look for a red ball!"
        # "dance": True # Example if you wanted to test dance here
    }
    print(f"Simulating LLM response: {json.dumps(mock_llm_response_change_directive)}")
    processed_response = process_llm_response(json.dumps(mock_llm_response_change_directive))
    print(f"Processed response: {json.dumps(processed_response, indent=2)}")
    print(f"Current directive after mock change: {current_directive}")

    # Test 3: Survey with new directive
    print("\n--- Test 3: Survey with new directive (and assume prior action context from test 2) ---")
    decision3 = get_decision_for_survey("a window", "a cat sleeping", "a closed door", last_action_context=f"My directive is now '{current_directive}'.")
    print(json.dumps(decision3, indent=2))
    print(f"Current directive after call 3: {current_directive}") 