from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

q_url = os.getenv("QDRANT_URL")
q_key = os.getenv("QDRANT_API_KEY")

client = QdrantClient(url=q_url, api_key=q_key)

try:
    collections = client.get_collections()
    print("--- Available Collections ---")
    for c in collections.collections:
        info = client.get_collection(c.name)
        print(f"Name: {c.name}")
        print(f"Vector size: {info.config.params.vectors.size}")
        print(f"Count: {info.points_count}")
        print("-" * 20)
except Exception as e:
    print(f"Error: {e}")
