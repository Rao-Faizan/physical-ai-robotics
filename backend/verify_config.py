import os
from dotenv import load_dotenv
import google.generativeai as genai
from qdrant_client import QdrantClient
import httpx

load_dotenv()

def test_gemini():
    print("--- Testing Gemini ---")
    gemini_key = os.getenv("LLM_API_KEY")
    if not gemini_key:
        print("❌ LLM_API_KEY not found in .env")
        return
    try:
        genai.configure(api_key=gemini_key)
        model = genai.GenerativeModel('gemini-1.5-flash')
        # Use a timeout if possible, but the SDK handle it internally. 
        # We just try a simple call.
        response = model.generate_content("Say 'Gemini OK'", generation_config={"max_output_tokens": 10})
        print(f"✅ Gemini Response: {response.text.strip()}")
    except Exception as e:
        print(f"❌ Gemini Error: {e}")

def test_qdrant():
    print("\n--- Testing Qdrant ---")
    q_url = os.getenv("QDRANT_URL")
    q_key = os.getenv("QDRANT_API_KEY")
    
    if not q_url:
        print("❌ QDRANT_URL not found")
        return

    print(f"Attempting to connect to: {q_url}")
    try:
        # Use a short timeout
        client = QdrantClient(url=q_url, api_key=q_key, timeout=10)
        collections = client.get_collections()
        coll_names = [c.name for c in collections.collections]
        print(f"✅ Connected! Collections: {coll_names}")
    except Exception as e:
        print(f"❌ Qdrant Connection failed: {e}")
        
        # Try with port 6333 explicitly if not present
        if ":6333" not in q_url:
            alt_url = f"{q_url}:6333"
            print(f"Retrying with port 6333: {alt_url}")
            try:
                client = QdrantClient(url=alt_url, api_key=q_key, timeout=5)
                collections = client.get_collections()
                print(f"✅ Success with port 6333!")
            except Exception as e2:
                print(f"❌ Still failed with port 6333: {e2}")

if __name__ == "__main__":
    test_gemini()
    test_qdrant()
