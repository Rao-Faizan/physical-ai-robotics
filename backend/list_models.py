import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()
genai.configure(api_key=os.getenv("LLM_API_KEY"))

print("Listing models with 'embed' in name or supported methods:")
for m in genai.list_models():
    if 'embedContent' in m.supported_generation_methods:
        print(f"Model: {m.name}, Display: {m.display_name}")
