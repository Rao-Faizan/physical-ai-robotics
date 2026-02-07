import os
import sys
import logging
from pathlib import Path
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Load environment
load_dotenv()

# Add parent dir to path
sys.path.append(os.getcwd())

from app.services.embedding_service import embedding_service

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def run_seeding():
    q_url = os.getenv("QDRANT_URL")
    q_key = os.getenv("QDRANT_API_KEY")
    collection_name = "book-hackathon"
    dimensions = 1536 # OpenAI text-embedding-3-small

    print(f"--- Seeding Qdrant ---")
    print(f"URL: {q_url}")
    print(f"Collection: {collection_name}")
    print(f"Dimensions: {dimensions}")

    client = QdrantClient(url=q_url, api_key=q_key)

    # 1. Create Collection
    print(f"Creating collection {collection_name}...")
    try:
        if client.collection_exists(collection_name):
            client.delete_collection(collection_name)
        
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=dimensions, distance=Distance.COSINE)
        )
        print("✅ Collection created.")
    except Exception as e:
        print(f"❌ Failed to create collection: {e}")
        return

    # 2. Process Files
    docs_dir = Path("frontend/docs")
    if not docs_dir.exists():
        print(f"❌ Docs directory not found at {docs_dir}")
        return

    md_files = list(docs_dir.rglob("*.md"))
    print(f"Found {len(md_files)} files.")

    chunks = []
    for file_path in md_files:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            # Simple chunking for demo/fix
            text_chunks = [content[i:i+2000] for i in range(0, len(content), 2000)]
            for i, text in enumerate(text_chunks):
                chunks.append({
                    "text": text,
                    "metadata": {
                        "file": str(file_path),
                        "module": "General",
                        "chapter": file_path.stem,
                        "url": f"/docs/{file_path.stem}"
                    }
                })

    print(f"Generated {len(chunks)} chunks.")

    # 3. Upload in batches
    batch_size = 5
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i+batch_size]
        texts = [c["text"] for c in batch]
        
        try:
            print(f"Generating embeddings for batch {i//batch_size + 1}...")
            embeddings = embedding_service.generate_embeddings_batch(texts)
            
            points = []
            for j, (chunk, emb) in enumerate(zip(batch, embeddings)):
                points.append(PointStruct(
                    id=i + j,
                    vector=emb,
                    payload={
                        "text": chunk["text"],
                        **chunk["metadata"]
                    }
                ))
            
            client.upsert(collection_name=collection_name, points=points)
            print(f"Uploaded batch {i//batch_size + 1}")
        except Exception as e:
            print(f"❌ Error in batch {i//batch_size + 1}: {e}")

    print("✅ Seeding complete!")

if __name__ == "__main__":
    run_seeding()
