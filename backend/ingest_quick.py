import os
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance

API_KEY = "AIzaSyCcJOemEC8PLMMFzLbvSpGIq2ljHW3I8No"
genai.configure(api_key=API_KEY)
q_client = QdrantClient(url="http://qdrant:6333")
COLLECTION = "physical-ai-robotics-v6"

def ingest_quick():
    file_path = "/app/docs/module-01-ros2/week-3-basics.md"
    if not os.path.exists(file_path):
        print(f"❌ File not found: {file_path}")
        return

    with open(file_path, 'r', encoding='utf-8') as f:
        text = f.read()
    
    # Simple chunk
    chunks = [text[0:1000], text[1000:2000]]
    
    print(f"🚀 Indexing 2 chunks from ROS 2 Introduction...")
    for i, chunk in enumerate(chunks):
        res = genai.embed_content(
            model="models/gemini-embedding-001",
            content=chunk,
            task_type="retrieval_document"
        )
        emb = res['embedding']
        
        q_client.upsert(
            collection_name=COLLECTION,
            points=[PointStruct(id=1000+i, vector=emb, payload={"text": chunk, "file": "week-1.md"})]
        )
    print("✅ DONE! Quick index complete.")

if __name__ == "__main__":
    ingest_quick()
