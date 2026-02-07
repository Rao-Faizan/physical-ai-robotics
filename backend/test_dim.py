import os
import sys
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

text = "Test query"
model = "text-embedding-3-small"

print(f"Testing model: {model}")
try:
    response = client.embeddings.create(input=[text], model=model)
    embedding = response.data[0].embedding
    print(f"Dimension: {len(embedding)}")
except Exception as e:
    print(f"Error: {e}")
