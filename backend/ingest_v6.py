import os
import glob
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
from app.services.embedding_service import embedding_service
from app.config import settings

def run():
    client = QdrantClient(url=settings.qdrant_url)
    collection = "physical-ai-robotics-v6"
    
    print(f"🚀 STARTING INGESTION INTO {collection}...")
    
    # 1. Create
    try:
        client.create_collection(
            collection_name=collection,
            vectors_config=VectorParams(size=768, distance=Distance.COSINE)
        )
        print("✅ Collection Created")
    except:
        print("💡 Collection already exists")

    # 2. Files
    files = glob.glob("/app/docs/**/*.md", recursive=True)
    idx = 1
    for f_path in files:
        with open(f_path, 'r', encoding='utf-8') as f:
            text = f.read()
        
        rel = os.path.relpath(f_path, "/app/docs")
        chunks = [text[i:i+800] for i in range(0, len(text), 600)]
        
        points = []
        for chunk in chunks:
            try:
                # This will use Gemini (768) after my provider cleanup
                emb = embedding_service.generate_embedding(chunk)
                if len(emb) != 768:
                    print(f"⚠️ Wrong dim: {len(emb)}. Skipping.")
                    continue

                points.append(PointStruct(id=idx, vector=emb, payload={"text": chunk, "file": rel}))
                idx += 1
            except Exception as e:
                print(f"❌ Error indexing chunk: {e}")
        
        if points:
            client.upsert(collection_name=collection, points=points)
            print(f"✅ Indexed {rel}")

    print(f"🎉 SUCCESS! Total {idx-1} chunks.")

if __name__ == "__main__":
    run()
