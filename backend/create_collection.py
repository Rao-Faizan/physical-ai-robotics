from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
from app.config import settings

def main():
    client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)
    collection = settings.qdrant_collection_name
    dim = settings.embedding_dimensions
    
    print(f"🛠️ Creating collection: {collection} with {dim} dimensions...")
    try:
        client.create_collection(
            collection_name=collection,
            vectors_config=VectorParams(size=dim, distance=Distance.COSINE)
        )
        print("✅ Collection created successfully!")
    except Exception as e:
        print(f"⚠️ Collection might already exist: {e}")

if __name__ == "__main__":
    main()
