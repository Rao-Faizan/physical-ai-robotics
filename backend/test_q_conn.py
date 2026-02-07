from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
import requests

load_dotenv()

url = os.getenv("QDRANT_URL")
key = os.getenv("QDRANT_API_KEY")

print(f"URL: {url}")

try:
    print("Testing with requests first...")
    # Add /collections to the url if it doesn't have it
    test_url = f"{url}/collections" if not url.endswith("/") else f"{url}collections"
    resp = requests.get(test_url, headers={"api-key": key}, timeout=10)
    print(f"Requests result: {resp.status_code}")
    print(f"Response: {resp.text}")
except Exception as e:
    print(f"Requests error: {e}")

try:
    print("\nTesting with QdrantClient...")
    client = QdrantClient(url=url, api_key=key, timeout=10)
    colls = client.get_collections()
    print(f"Client success: {[c.name for c in colls.collections]}")
except Exception as e:
    print(f"Client error: {e}")
