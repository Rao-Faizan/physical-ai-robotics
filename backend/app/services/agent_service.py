import logging
import time
from typing import List, Optional, Dict, Any
from enum import Enum
import google.generativeai as genai
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue

from app.config import settings
from app.services.embedding_service import embedding_service

logger = logging.getLogger(__name__)

class Intent(Enum):
    KNOWLEDGE_SEARCH = "knowledge_search"
    GENERAL_CHAT = "general_chat"
    TRANSLATE = "translate"
    PERSONALIZE = "personalize"

class AgentService:
    def __init__(self):
        self.provider = settings.llm_provider
        self.openai_key = settings.openai_api_key
        self.gemini_key = settings.llm_api_key
        
        # Clients
        self.openai_client = OpenAI(api_key=self.openai_key) if self.openai_key else None
        if self.gemini_key:
            genai.configure(api_key=self.gemini_key)
        
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection_name

    def detect_intent(self, query: str) -> Intent:
        """
        Simple keyword-based intent detection (can be upgraded to LLM-based).
        """
        query_lower = query.lower()
        
        if any(w in query_lower for w in ["translate", "urdu", "mein batao", "tarjuma"]):
            return Intent.TRANSLATE
        
        if any(w in query_lower for w in ["personalize", "beginner", "advanced", "expert"]):
            return Intent.PERSONALIZE
            
        # Default to knowledge search for this project
        return Intent.KNOWLEDGE_SEARCH

    async def process_query(self, query: str, context_overrides: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Main entry point for processing queries.
        """
        intent = self.detect_intent(query)
        logger.info(f"detected intent: {intent}")
        
        if intent == Intent.KNOWLEDGE_SEARCH:
            return await self._handle_knowledge_search(query, context_overrides or {})
        else:
            # For now, handle everything else as general RAG or simple chat
            return await self._handle_knowledge_search(query, context_overrides or {})

    async def _handle_knowledge_search(self, query: str, context_overrides: Dict[str, Any]) -> Dict[str, Any]:
        start_time = time.time()
        
        # 1. Embed
        embedding = embedding_service.generate_embedding(query)
        logger.info(f"DEBUG: Query embedding generated with dimension: {len(embedding)}")
        
        # 2. Search (with self-healing for missing collection)
        module_filter = context_overrides.get("module_filter")
        search_filter = None
        if module_filter:
            search_filter = Filter(must=[FieldCondition(key="module", match=MatchValue(value=module_filter))])
            
        try:
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=embedding,
                query_filter=search_filter,
                limit=5
            )
        except Exception as e:
            if "404" in str(e) or "not found" in str(e).lower():
                logger.warning(f"Collection '{self.collection_name}' missing. Attempting self-healing...")
                self._ensure_collection_exists()
                # Try search again
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=embedding,
                    query_filter=search_filter,
                    limit=5
                )
            else:
                logger.error(f"Qdrant search error: {e}")
                search_results = []

        # 3. Build Context
        context_parts = []
        sources = []
        for idx, result in enumerate(search_results):
            payload = result.payload
            context_parts.append(f"[Source {idx+1}] ({payload.get('file', 'unknown')})\n{payload.get('text', '')}")
            sources.append({
                "module": payload.get('module', 'Unknown'),
                "chapter": payload.get('chapter', 'Unknown'),
                "url": payload.get('url', '/'),
                "relevance_score": round(result.score, 3)
            })
        
        context_text = "\n\n".join(context_parts)
        
        # 4. Generate Answer
        system_prompt = "You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook. Answer using ONLY the provided context. Cite sources [Source N]."
        user_prompt = f"Context:\n{context_text}\n\nQuestion: {query}"
        
        answer = self.generate_llm_response(system_prompt, user_prompt)
        
        return {
            "answer": answer,
            "sources": sources,
            "processing_time": int((time.time() - start_time) * 1000)
        }

    def _ensure_collection_exists(self):
        """Creates the collection and adds a welcome chunk if it doesn't exist."""
        from qdrant_client.models import VectorParams, Distance, PointStruct
        
        # Dimensions based on current provider
        dims = 1536 if self.provider == "openai" else 768
        
        try:
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=dims, distance=Distance.COSINE)
            )
            logger.info(f"✅ Created collection '{self.collection_name}' with {dims} dimensions.")
            
            # Add a seed point so it's not empty
            welcome_text = "Welcome to the Physical AI & Humanoid Robotics Textbook Assistant! How can I help you today?"
            welcome_emb = embedding_service.generate_embedding(welcome_text)
            
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[
                    PointStruct(
                        id=0,
                        vector=welcome_emb,
                        payload={
                            "text": welcome_text,
                            "module": "System",
                            "chapter": "Welcome",
                            "url": "/"
                        }
                    )
                ]
            )
        except Exception as e:
            logger.error(f"Failed to auto-create collection: {e}")

    def generate_llm_response(self, system_prompt: str, user_prompt: str, temperature: float = 0.3) -> str:
        if self.provider == "openai" and self.openai_client:
            try:
                response = self.openai_client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=temperature
                )
                return response.choices[0].message.content
            except Exception as e:
                logger.error(f"OpenAI error: {e}. Falling back to Gemini.")
        
        # Gemini Fallback
        model = genai.GenerativeModel('gemini-1.5-flash')
        full_prompt = f"{system_prompt}\n\n{user_prompt}"
        response = model.generate_content(
            full_prompt,
            generation_config=genai.types.GenerationConfig(temperature=temperature)
        )
        return response.text

# Global instance
agent_service = AgentService()
