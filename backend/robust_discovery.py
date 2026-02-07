import os
import requests
import json
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv("LLM_API_KEY")

print(f"--- Gemini Model Discovery ---")
url = f"https://generativelanguage.googleapis.com/v1beta/models?key={api_key}"

try:
    response = requests.get(url, timeout=10)
    print(f"Status: {response.status_code}")
    if response.status_code == 200:
        data = response.json()
        models = data.get("models", [])
        print(f"Found {len(models)} models.")
        for m in models:
            name = m.get("name")
            methods = m.get("supportedGenerationMethods", [])
            if "embedContent" in methods:
                print(f"✅ EMBEDDING MODEL: {name}")
                print(f"   Supported Methods: {methods}")
    else:
        print(f"Error: {response.text}")
except Exception as e:
    print(f"Exception: {e}")

print("\n--- Testing text-embedding-004 on v1 ---")
url_v1 = f"https://generativelanguage.googleapis.com/v1/models/text-embedding-004:embedContent?key={api_key}"
payload = {
    "model": "models/text-embedding-004",
    "content": {"parts": [{"text": "Testing v1 endpoint"}]},
    "taskType": "RETRIEVAL_QUERY"
}
try:
    res = requests.post(url_v1, json=payload, timeout=10)
    print(f"v1 Status: {res.status_code}")
    if res.status_code == 200:
        print("✅ v1/text-embedding-004 WORKS!")
    else:
        print(f"❌ v1 Failed: {res.text[:200]}")
except Exception as e:
    print(f"Exception v1: {e}")
