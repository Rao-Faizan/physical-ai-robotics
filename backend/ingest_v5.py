import os
import glob
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance

# CONFIG
API_KEY = "AIzaSyCcJOemEC8PLMMFzLbvSpGIq2ljHW3I8No" # Your verified key
QDRANT_URL = "http://qdrant:6333"
COLLECTION = "physical-ai-robotics-v5"
DOCS_DIR = "/app/docs"

def run():
    genai.configure(api_key=API_KEY)
    client = QdrantClient(url=QDRANT_URL)
    
    # 1. Create Collection (V5)
    print(f"🛠️ Creating {COLLECTION}...")
    try:
        client.create_collection(
            collection_name=COLLECTION,
            vectors_config=VectorParams(size=768, distance=Distance.COSINE)
        )
    except Exception:
        print("⚠️ Collection exists or error.")

    # 2. Ingest
    files = glob.glob(f"{DOCS_DIR}/**/*.md", recursive=True)
    print(f"📚 Found {len(files)} files.")
    
    idx = 1
    for f_path in files:
        with open(f_path, 'r', encoding='utf-8') as f:
            text = f.read()
        
        rel = os.path.relpath(f_path, DOCS_DIR)
        print(f"⏳ Processing {rel}...")
        
        # Chunking
        chunks = [text[i:i+800] for i in range(0, len(text), 600)]
        
        points = []
        for chunk in chunks:
            try:
                # This model was confirmed present in your list
                res = genai.embed_content(
                    model="models/gemini-embedding-001",
                    content=chunk,
                    task_type="retrieval_document"
                )
                emb = res['embedding']
                
                points.append(PointStruct(
                    id=idx,
                    vector=emb,
                    payload={"text": chunk, "file": rel}
                ))
                idx += 1
            except Exception as e:
                print(f"❌ Chunk error: {e}")
        
        if points:
            client.upsert(collection_name=COLLECTION, points=points)

    print(f"🎉 SUCCESS! Indexed {idx-1} chunks into V5.")

if __name__ == "__main__":
    run()
