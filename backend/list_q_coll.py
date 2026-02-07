import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

print("Loading .env...")
load_dotenv()

q_url = os.getenv("QDRANT_URL")
q_key = os.getenv("QDRANT_API_KEY")

print(f"Connecting to: {q_url}")
client = QdrantClient(url=q_url, api_key=q_key)

try:
    collections = client.get_collections()
    print("Available Collections:")
    for c in collections.collections:
        print(f"- {c.name}")
except Exception as e:
    print(f"Error: {e}")
