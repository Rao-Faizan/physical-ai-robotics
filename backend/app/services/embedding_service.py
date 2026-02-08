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
        
        # HuggingFace (lazy loaded to save memory if not used)
        self._hf_embeddings = None
        
        # Gemini Working configuration (cached after discovery)
        self._working_model: Optional[str] = None
        self._working_version: Optional[str] = "v1beta"
        self._available_models: List[str] = []

    def _get_hf_embeddings(self):
        """Lazy load HuggingFace embeddings."""
        if self._hf_embeddings is None:
            from langchain.embeddings import HuggingFaceEmbeddings
            logger.info("📡 Loading local HuggingFace embeddings (no API key needed)...")
            self._hf_embeddings = HuggingFaceEmbeddings(
                model_name="sentence-transformers/all-MiniLM-L6-v2",
                model_kwargs={'device': 'cpu'}
            )
        return self._hf_embeddings

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
            return "fallback-needed"

        discovery_url = f"{self.base_url}/v1beta/models?key={self.gemini_key}"
        
        try:
            logger.info("🔍 Discovering available Gemini models...")
            response = requests.get(discovery_url, timeout=10)
            if response.status_code != 200:
                logger.warning(f"Discovery failed with status {response.status_code}")
                return "fallback-needed"

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
                return self._working_model
            
            return "fallback-needed"

        except Exception as e:
            logger.error(f"❌ Model discovery failed: {str(e)}")
            return "fallback-needed"

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generates embedding with a robust multi-provider strategy.
        """
        # 1. OpenAI Priority
        if self.openai_key and "your-openai" not in self.openai_key:
            try:
                return self._generate_openai_embedding(text)
            except Exception as e:
                logger.error(f"❌ OpenAI Embedding failed: {e}")
        
        # 1. Gemini Attempt (Main Provider)
        if self.gemini_key and "your-gemini" not in self.gemini_key:
            try:
                # Direct try with gemini-embedding-001 (768 dims)
                return self._generate_gemini_embedding(text, "models/gemini-embedding-001")
            except Exception as e:
                logger.warning(f"❌ Gemini Embedding failed: {e}")

        # 2. Ultimate Fallback: HuggingFace (Local/Free)
        try:
            hf = self._get_hf_embeddings()
            return hf.embed_query(text)
        except Exception:
            # Final emergency zero-vector (size 768)
            return [0.0] * 768

    def _generate_gemini_embedding(self, text: str, model: str) -> List[float]:
        """
        Generates embedding using discovered Gemini model.
        """
        api_key = self.gemini_key
        
        # Try discovered config
        for version in ["v1", "v1beta"]:
            endpoint = f"{self.base_url}/{version}/{model}:embedContent?key={api_key}"
            payload = {
                "model": model,
                "content": {"parts": [{"text": text}]},
                "taskType": "RETRIEVAL_QUERY"
            }
            
            try:
                response = requests.post(endpoint, json=payload, timeout=10)
                if response.status_code == 200:
                    data = response.json()
                    return data["embedding"]["values"]
            except Exception:
                continue

        raise RuntimeError(f"Gemini API returned errors for model {model}")

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Simplified batch processing.
        """
        return [self.generate_embedding(t) for t in texts]

# Global service instance
embedding_service = EmbeddingService()
