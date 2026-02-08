import os
import glob
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from app.config import settings

def ingest():
    genai.configure(api_key=settings.llm_api_key)
    q_client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)
    collection = settings.qdrant_collection_name
    
    # AUTO-DISCOVER EMBEDDING MODEL
    print("🔍 Discovering available embedding model...")
    best_model = None
    for m in genai.list_models():
        if 'embedContent' in m.supported_generation_methods:
            best_model = m.name
            break
    
    if not best_model:
        print("❌ No embedding model found!")
        return

    print(f"✅ Using model: {best_model} for collection: {collection}")
    
    docs_path = "/app/docs"
    files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
    idx = 1
    
    for file_path in files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        rel_path = os.path.relpath(file_path, docs_path)
        chunks = [content[i:i+1000] for i in range(0, len(content), 800)]
        
        points = []
        for c_idx, chunk in enumerate(chunks):
            try:
                res = genai.embed_content(
                    model=best_model,
                    content=chunk,
                    task_type="retrieval_document"
                )
                emb = res['embedding']
                
                points.append(PointStruct(
                    id=idx,
                    vector=emb,
                    payload={"text": chunk, "file": rel_path}
                ))
                idx += 1
            except Exception as e:
                print(f"❌ {rel_path} error: {e}")
        
        if points:
            q_client.upsert(collection_name=collection, points=points)
            print(f"🗳️ Indexed {rel_path}")

    print(f"🎉 FINAL DONE! Total {idx-1} chunks.")

if __name__ == "__main__":
    ingest()
