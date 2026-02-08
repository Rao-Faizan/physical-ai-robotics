import os
import glob
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct

API_KEY = "AIzaSyCcJOemEC8PLMMFzLbvSpGIq2ljHW3I8No"
genai.configure(api_key=API_KEY)
q_client = QdrantClient(url="http://qdrant:6333")
COLLECTION = "physical-ai-robotics-final"

def run():
    files = glob.glob("/app/docs/**/*.md", recursive=True)
    print(f"🚀 Ingesting {len(files)} files into {COLLECTION}...")
    
    idx = 2000 # Different range for safety
    for f_path in files:
        with open(f_path, 'r', encoding='utf-8') as f:
            text = f.read()
            
        rel = os.path.relpath(f_path, "/app/docs")
        chunks = [text[i:i+800] for i in range(0, len(text), 600)]
        
        points = []
        for chunk in chunks:
            try:
                res = genai.embed_content(model="models/gemini-embedding-001", content=chunk)
                emb = res['embedding']
                points.append(PointStruct(id=idx, vector=emb, payload={"text": chunk, "file": rel}))
                idx += 1
            except Exception as e: print(f"❌ Error: {e}")
            
        if points:
            q_client.upsert(collection_name=COLLECTION, points=points)
            print(f"✅ {rel}")

    print(f"🎉 MISSION COMPLETE! {idx-2000} chunks indexed.")

if __name__ == "__main__":
    run()
