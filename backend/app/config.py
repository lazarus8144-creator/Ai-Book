"""
Application Configuration
=========================

Manages environment variables and settings using pydantic-settings.
"""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import List
import json


class Settings(BaseSettings):
    """Application settings from environment variables"""

    # Groq Configuration (FREE LLM)
    GROQ_API_KEY: str
    GROQ_MODEL: str = "llama-3.1-70b-versatile"
    GROQ_TEMPERATURE: float = 0.1
    GROQ_MAX_TOKENS: int = 500

    # Embeddings Configuration (FREE local)
    EMBEDDING_MODEL: str = "sentence-transformers/all-MiniLM-L6-v2"
    EMBEDDING_DEVICE: str = "cpu"

    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str = "textbook_chunks"

    # Application Configuration
    ENVIRONMENT: str = "development"
    LOG_LEVEL: str = "INFO"
    ADMIN_TOKEN: str

    # CORS - Parse JSON string to list
    ALLOWED_ORIGINS: str = '["http://localhost:3000"]'

    # Rate Limiting
    RATE_LIMIT_QUERIES_PER_MINUTE: int = 20
    RATE_LIMIT_QUERIES_PER_HOUR: int = 100

    # Performance
    EMBEDDING_CACHE_SIZE: int = 1000
    VECTOR_SEARCH_LIMIT: int = 5
    MAX_CHUNK_SIZE: int = 800
    CHUNK_OVERLAP: int = 100

    # Optional: Claude Subagents
    ANTHROPIC_API_KEY: str = ""
    ENABLE_SUBAGENT_ENHANCEMENT: bool = False

    class Config:
        env_file = ".env"
        case_sensitive = True

    def get_allowed_origins(self) -> List[str]:
        """Parse ALLOWED_ORIGINS from JSON string to list"""
        try:
            return json.loads(self.ALLOWED_ORIGINS)
        except:
            return ["http://localhost:3000"]


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance"""
    return Settings()


settings = get_settings()
