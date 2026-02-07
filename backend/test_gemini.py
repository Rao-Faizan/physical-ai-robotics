import os
import google.generativeai as genai

KEY = "AIzaSyCXS9YL4Z0cu7Utqse16Q4VjZoH6i8pvVs"
genai.configure(api_key=KEY)

try:
    model = genai.GenerativeModel('gemini-1.5-flash')
    response = model.generate_content("Translate 'Hello' to Urdu. Only the word.")
    print(f"Response: '{response.text.strip()}'")
except Exception as e:
    print(f"Error: {e}")
