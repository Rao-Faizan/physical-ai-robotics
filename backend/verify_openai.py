import os
import sys

# Add current directory to path
sys.path.append(os.getcwd())

from app.config import settings
from app.services.embedding_service import embedding_service
from app.services.agent_service import agent_service

def verify():
    print("--- 🔍 Project Awesome Verification ---")
    print(f"LLM Provider: {settings.llm_provider}")
    
    # 1. Test Embedding
    print("\n[1/3] Testing Embedding...")
    try:
        emb = embedding_service.generate_embedding("Test query for OpenAI")
        print(f"✅ Embedding Success! Dimensions: {len(emb)}")
    except Exception as e:
        print(f"❌ Embedding Failed: {e}")

    # 2. Test Agent (Chat)
    print("\n[2/3] Testing Chat Agent...")
    try:
        # Using a very simple query that doesn't necessarily need RAG to check LLM connectivity
        import asyncio
        async def test_chat():
            res = await agent_service.process_query("What is Humanoid Robotics?")
            print(f"✅ Chat Success! Answer preview: {res['answer'][:100]}...")
        
        asyncio.run(test_chat())
    except Exception as e:
        print(f"❌ Chat Failed: {e}")

    # 3. Test Provider Logic
    print("\n[3/3] Testing Provider Logic...")
    if settings.openai_api_key:
        print("✅ OpenAI API Key detected.")
    else:
        print("⚠️ OpenAI API Key missing.")

if __name__ == "__main__":
    verify()
