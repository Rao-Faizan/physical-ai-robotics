"""
Application configuration settings.
Loads environment variables and provides centralized config access.
"""

import os
from pydantic_settings import BaseSettings, SettingsConfigDict
from dotenv import load_dotenv

load_dotenv()


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # LLM/Embedding Keys
    llm_api_key: str = os.getenv("LLM_API_KEY", "")
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    
    @property
    def llm_provider(self) -> str:
        """Determines which provider to use. OpenAI takes precedence if key exists."""
        if self.openai_api_key:
            return "openai"
        return "gemini"
    
    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "physical-ai-robotics-final")

    # Embedding Configuration
    embedding_model: str = os.getenv("EMBEDDING_MODEL", "models/gemini-embedding-001")
    embedding_dimensions: int = int(os.getenv("EMBEDDING_DIMENSIONS", "3072"))

    # Chunking Configuration
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "50"))

    # Database
    database_url: str = os.getenv("DATABASE_URL", "sqlite:///./hackathon.db")

    # Security
    secret_key: str = os.getenv("SECRET_KEY", "your-secret-key-here-change-in-production")

    # Pydantic v2 configuration
    model_config = SettingsConfigDict(
        env_file=".env",
        case_sensitive=False,
        extra="ignore"  # Ignore extra fields in .env
    )


# Global settings instance
settings = Settings()
