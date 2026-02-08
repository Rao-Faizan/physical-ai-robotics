import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct

API_KEY = "AIzaSyCcJOemEC8PLMMFzLbvSpGIq2ljHW3I8No"
COLLECTION = "physical-ai-robotics-final" # BRAND NEW
genai.configure(api_key=API_KEY)
q_client = QdrantClient(url="http://qdrant:6333")

def run():
    # 1. Test embedding to see ACTUAL dimension
    test_res = genai.embed_content(model="models/gemini-embedding-001", content="test")
    emb = test_res['embedding']
    dim = len(emb)
    print(f"📏 ACTUAL DIMENSION DETECTED: {dim}")

    # 2. Create collection with detected dimension
    try:
        q_client.delete_collection(COLLECTION)
    except: pass
    
    q_client.create_collection(
        collection_name=COLLECTION,
        vectors_config=VectorParams(size=dim, distance=Distance.COSINE)
    )
    print(f"✅ Collection {COLLECTION} created with {dim} dims.")

    # 3. Simple Ingest
    text = "ROS 2 (Robot Operating System 2) is the standard middleware for physical AI. Nodes communicate via topics."
    q_client.upsert(
        collection_name=COLLECTION,
        points=[PointStruct(id=1, vector=emb, payload={"text": text, "file": "test.md"})]
    )
    print("🎉 SEED DATA INGESTED!")

if __name__ == "__main__":
    run()
