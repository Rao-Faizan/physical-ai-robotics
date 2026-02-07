"""
RAG Chatbot API Routes (+50 points)
Google Gemini + Qdrant Vector Search
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import os
import time
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from app.config import settings

router = APIRouter()

from app.services.agent_service import agent_service

router = APIRouter()

class ChatQuery(BaseModel):
    query: str
    user_id: Optional[int] = None
    selected_text: Optional[str] = None
    module_filter: Optional[str] = None


class Source(BaseModel):
    module: str
    chapter: str
    section: Optional[str] = None
    url: str
    relevance_score: float


class ChatResponse(BaseModel):
    answer: str
    sources: List[Source]
    conversation_id: str
    processing_time: int
    timestamp: str


@router.post("/query", response_model=ChatResponse)
async def chat_query(query: ChatQuery):
    """
    RAG Chatbot Endpoint - Delegate to AgentService
    """
    try:
        query_text = query.selected_text or query.query
        context_overrides = {
            "module_filter": query.module_filter
        }
        
        result = await agent_service.process_query(query_text, context_overrides)

        return ChatResponse(
            answer=result["answer"],
            sources=[Source(**s) for s in result["sources"]],
            conversation_id=f"conv_{int(time.time())}",
            processing_time=result["processing_time"],
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        )

    except Exception as e:
        print(f"RAG Error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"RAG processing error: {str(e)}"
        )

    except Exception as e:
        # Log error
        print(f"RAG Error: {str(e)}")

        # Return fallback response
        raise HTTPException(
            status_code=500,
            detail=f"RAG processing error: {str(e)}"
        )


@router.get("/health")
async def health_check():
    """
    Check if RAG system is operational
    """
    try:
        # Check Gemini API
        gemini_status = "configured" if os.getenv("LLM_API_KEY") else "missing_api_key"

        # Check Qdrant
        try:
            collections = qdrant_client.get_collections()
            qdrant_status = "connected"
            collection_exists = any(c.name == COLLECTION_NAME for c in collections.collections)
        except:
            qdrant_status = "disconnected"
            collection_exists = False

        return {
            "status": "healthy" if (gemini_status == "configured" and collection_exists) else "degraded",
            "services": {
                "gemini": gemini_status,
                "qdrant": qdrant_status,
                "collection_exists": collection_exists,
                "collection_name": COLLECTION_NAME
            }
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }


@router.get("/history/{user_id}")
async def get_chat_history(user_id: int, limit: int = 10):
    """Get chat history for a user (Future: implement with database)"""
    # TODO: Implement database query for chat history
    return {
        "user_id": user_id,
        "messages": [],
        "note": "Chat history feature will be implemented in Phase 2"
    }
