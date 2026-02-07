
import os
import time
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
LLM_API_KEY = os.getenv("LLM_API_KEY")
COLLECTION_NAME = "textbook_rag"
EMBEDDING_MODEL = "models/text-embedding-004"
EMBEDDING_DIM = 768

# Sample Data (Physical AI & Humanoid Robotics)
SAMPLE_CHUNKS = [
    {
        "text": "Physical AI refers to the synthesis of AI and robotics, where intelligence is embedded into the physical structure of the robot itself. It involves co-designing the body and brain of the robot.",
        "metadata": {"module": "Introduction", "chapter": "What is Physical AI?", "url": "/docs/intro"}
    },
    {
        "text": "Humanoid robotics focuses on creating robots that mimic human form and behavior. Key challenges include bipedal locomotion, dexterity in manipulation, and human-robot interaction.",
        "metadata": {"module": "Humanoids", "chapter": "Basics of Humanoids", "url": "/docs/humanoids"}
    },
    {
        "text": "ROS 2 (Robot Operating System 2) is the industry standard middleware for robotics. It provides a communication layer for nodes to exchange messages via topics, services, and actions.",
        "metadata": {"module": "ROS 2", "chapter": "Architecture", "url": "/docs/ros2"}
    },
    {
        "text": "Reinforcement Learning (RL) is crucial for training humanoid robots to walk. Policies are trained in simulation (e.g., Isaac Gym) and then transferred to the real robot (Sim2Real).",
        "metadata": {"module": "Control", "chapter": "RL for Walking", "url": "/docs/rl"}
    }
]

def init_qdrant():
    print(f"Connecting to Qdrant at {QDRANT_URL}...")
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    
    print(f"Configuring Gemini with key: {LLM_API_KEY[:5]}...")
    genai.configure(api_key=LLM_API_KEY)

    # Check if collection exists
    collections = client.get_collections()
    if any(c.name == COLLECTION_NAME for c in collections.collections):
        print(f"Collection '{COLLECTION_NAME}' already exists. Recreating...")
        client.delete_collection(COLLECTION_NAME)
    
    # Create collection
    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=EMBEDDING_DIM, distance=Distance.COSINE)
    )
    print(f"Created collection '{COLLECTION_NAME}'")

    # Ingest data
    points = []
    print("Generating embeddings...")
    for idx, chunk in enumerate(SAMPLE_CHUNKS):
        response = genai.embed_content(
            model=EMBEDDING_MODEL,
            content=chunk["text"],
            task_type="retrieval_document"
        )
        embedding = response['embedding']
        
        points.append(PointStruct(
            id=idx + 1,
            vector=embedding,
            payload={
                "text": chunk["text"],
                **chunk["metadata"]
            }
        ))
        time.sleep(1) # Rate limit protection

    client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )
    print(f"Successfully ingested {len(points)} sample chunks.")

if __name__ == "__main__":
    init_qdrant()
