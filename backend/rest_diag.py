import os
import httpx
from dotenv import load_dotenv

load_dotenv()
api_key = os.getenv("LLM_API_KEY")

versions = ["v1", "v1beta"]
models = ["text-embedding-004", "embedding-001"]

print(f"Testing with API Key: {api_key[:5]}...{api_key[-5:]}")

def test_endpoint(version, model):
    url = f"https://generativelanguage.googleapis.com/{version}/models/{model}:embedContent?key={api_key}"
    payload = {
        "model": f"models/{model}",
        "content": {"parts": [{"text": "Hello"}]},
        "taskType": "RETRIEVAL_QUERY"
    }
    try:
        with httpx.Client(timeout=10.0) as client:
            resp = client.post(url, json=payload)
            print(f"[{version}] [{model}] Status: {resp.status_code}")
            if resp.status_code == 200:
                print(f"  ✅ SUCCESS!")
                return True
            else:
                print(f"  ❌ Error: {resp.text[:200]}")
    except Exception as e:
        print(f"  💥 Exception: {e}")
    return False

results = []
for v in versions:
    for m in models:
        if test_endpoint(v, m):
            results.append((v, m))

if results:
    print("\nWorking Configurations:")
    for v, m in results:
        print(f"- {v} with {m}")
else:
    print("\nNo working configuration found. Checking model list...")
    # Try to list models via v1beta
    list_url = f"https://generativelanguage.googleapis.com/v1beta/models?key={api_key}"
    try:
        resp = httpx.get(list_url)
        if resp.status_code == 200:
            data = resp.json()
            embed_models = [m["name"] for m in data.get("models", []) if "embedContent" in m.get("supportedGenerationMethods", [])]
            print(f"Available Embed Models: {embed_models}")
        else:
            print(f"Failed to list models: {resp.text}")
    except Exception as e:
        print(f"Exception listing models: {e}")
