"""
Self-Discovering Embedding service using direct Gemini REST API calls.
Automatically finds working models and endpoints for the provided API key.
"""

import os
import logging
from typing import List, Optional
import requests
from openai import OpenAI
from app.config import settings

logger = logging.getLogger(__name__)

class EmbeddingService:
    """
    Highly resilient service that supports both Gemini and OpenAI embeddings.
    """

    def __init__(self):
        self.gemini_key = settings.llm_api_key
        self.openai_key = settings.openai_api_key
        self.base_url = "https://generativelanguage.googleapis.com"
        
        # OpenAI Client (only if key exists)
        self.client = OpenAI(api_key=self.openai_key) if self.openai_key else None
        
        # Gemini Working configuration (cached after discovery)
        self._working_model: Optional[str] = None
        self._working_version: Optional[str] = "v1beta"
        self._available_models: List[str] = []

    def _generate_openai_embedding(self, text: str) -> List[float]:
        """Generates embedding using OpenAI's text-embedding-3-small."""
        if not self.client:
            raise ValueError("OpenAI API key not configured")
        
        response = self.client.embeddings.create(
            input=[text],
            model="text-embedding-3-small",
            dimensions=1536
        )
        emb = response.data[0].embedding
        return emb

    def _discover_best_model(self) -> str:
        """ Queries Gemini to see which embedding models are available. """
        if self._working_model:
            return self._working_model

        if not self.gemini_key:
            raise ValueError("LLM_API_KEY is not configured")

        discovery_url = f"{self.base_url}/v1beta/models?key={self.gemini_key}"
        
        try:
            logger.info("🔍 Discovering available Gemini models...")
            response = requests.get(discovery_url, timeout=15)
            response.raise_for_status()
            data = response.json()
            models_data = data.get("models", [])
            
            # Find models that support 'embedContent'
            embed_models = [
                m["name"] for m in models_data 
                if "embedContent" in m.get("supportedGenerationMethods", [])
            ]
            
            self._available_models = embed_models
            
            # Priority list for selection
            priorities = [
                "models/text-embedding-004",
                "models/embedding-001",
                "models/gemini-embedding-exp-0812"
            ]
            
            for p in priorities:
                if p in embed_models:
                    self._working_model = p
                    logger.info(f"✅ Selected best available model: {p}")
                    return p
            
            if embed_models:
                self._working_model = embed_models[0]
                logger.info(f"⚠️ Preferred models not found. Using first available: {self._working_model}")
                return self._working_model
            
            all_names = [m["name"] for m in models_data]
            raise RuntimeError(f"No embedding models found for this key. Available models: {all_names}")

        except Exception as e:
            logger.error(f"❌ Model discovery failed: {str(e)}")
            # Fallback to a hardcoded guess if discovery fails (e.g., network issue)
            self._working_model = "models/text-embedding-004"
            return self._working_model

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generates embedding using the configured provider.
        """
        if settings.llm_provider == "openai":
            try:
                return self._generate_openai_embedding(text)
            except Exception as e:
                logger.error(f"❌ OpenAI Embedding failed: {str(e)}. Falling back to Gemini.")
        
        # Fallback/Default to Gemini
        return self._generate_gemini_embedding(text)

    def _generate_gemini_embedding(self, text: str) -> List[float]:
        """
        Generates embedding using discovered Gemini model.
        """
        model = self._discover_best_model()
        api_key = self.gemini_key
        
        if not api_key:
             raise ValueError("Gemini API key not configured")

        # Try discovered config
        for version in ["v1", "v1beta"]:
            endpoint = f"{self.base_url}/{version}/{model}:embedContent?key={api_key}"
            payload = {
                "model": model,
                "content": {"parts": [{"text": text}]},
                "taskType": "RETRIEVAL_QUERY"
            }
            
            try:
                response = requests.post(endpoint, json=payload, timeout=15)
                if response.status_code == 200:
                    data = response.json()
                    # Cache successful version
                    self._working_version = version
                    return data["embedding"]["values"]
                
                logger.debug(f"Attempt failed for {version}/{model}: {response.status_code}")
                
            except Exception as e:
                logger.debug(f"Exception during request for {version}/{model}: {e}")

        # If we reach here, both versions failed for the discovered model
        error_msg = f"Could not generate embedding. Discovery found: {self._available_models}. Tried {model} on v1 and v1beta."
        logger.error(error_msg)
        raise RuntimeError(error_msg)

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Simplified batch processing.
        """
        return [self.generate_embedding(t) for t in texts]

# Global service instance
embedding_service = EmbeddingService()
