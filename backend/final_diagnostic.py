import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv("LLM_API_KEY")

def test_config(version, model_name):
    print(f"\n--- Testing Version: {version} | Model: {model_name} ---")
    try:
        # Re-configure for each test to be sure
        genai.configure(api_key=api_key)
        # Note: The current SDK might not have a simple 'version' param in configure 
        # but we can try to see if it works with defaults first.
        
        # Testing embed_content
        res = genai.embed_content(
            model=model_name,
            content="test",
            task_type="retrieval_query"
        )
        print(f"✅ SUCCESS: Generated embedding of length {len(res['embedding'])}")
        return True
    except Exception as e:
        print(f"❌ FAILED: {str(e)}")
        return False

models_to_try = [
    "models/embedding-001",
    "models/text-embedding-004",
]

# We can't easily force version in the high-level genai.embed_content 
# without changing internals, but let's see what the default does.
for m in models_to_try:
    test_config("default", m)

# Let's also try to list ALL models just to see the names
print("\n--- Listing All Models Available to this Key ---")
try:
    for m in genai.list_models():
        if 'embedContent' in m.supported_generation_methods:
            print(f"Supported Embed Model: {m.name}")
except Exception as e:
    print(f"Error listing models: {e}")
