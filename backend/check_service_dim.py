import sys
import os
from dotenv import load_dotenv

# Add current dir to path
sys.path.append(os.getcwd())

load_dotenv()

from app.services.embedding_service import embedding_service

try:
    print(f"Provider: {os.getenv('LLM_PROVIDER', 'openai')}")
    text = "hello world"
    emb = embedding_service.generate_embedding(text)
    print(f"Embedding length: {len(emb)}")
    
    # Also check if Gemini key is being used
    if hasattr(embedding_service, 'gemini_key'):
         print(f"Gemini Key: {embedding_service.gemini_key[:5]}...")
    
except Exception as e:
    print(f"Error: {e}")
