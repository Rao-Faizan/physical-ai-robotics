import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

q_url = os.getenv("QDRANT_URL")
q_key = os.getenv("QDRANT_API_KEY")
q_coll = "book-hackathon"

client = QdrantClient(url=q_url, api_key=q_key)
try:
    collection_info = client.get_collection(collection_name=q_coll)
    print(f"Collection: {q_coll}")
    print(f"Vector size: {collection_info.config.params.vectors.size}")
    print(f"Distance: {collection_info.config.params.vectors.distance}")
except Exception as e:
    print(f"Error: {e}")
