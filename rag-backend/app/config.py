"""
Configuration module for RAG Chatbot Backend

Loads environment variables and provides typed configuration objects.
"""

from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    # OpenAI Configuration
    openai_api_key: str
    openai_model: str = "gpt-4o-mini"
    openai_embedding_model: str = "text-embedding-3-small"
    openai_max_tokens: int = 500
    openai_temperature: float = 0.3

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "robotics_textbook"
    qdrant_vector_size: int = 1536  # text-embedding-3-small dimension

    # API Configuration
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    api_reload: bool = True

    # CORS Configuration
    cors_origins: str = "http://localhost:3000"  # Comma-separated list

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string"""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    # Logging
    log_level: str = "INFO"

    # RAG Configuration
    chunk_size: int = 500
    chunk_overlap: int = 50
    top_k_results: int = 5

    # Environment
    environment: str = "development"

    # Demo Mode (for offline/no-API-calls testing)
    # NOTE: Keep True to avoid OpenAI API costs. Only set to False with valid API quota.
    demo_mode: bool = True

    # Database Configuration
    database_url: str = "postgresql://user:password@localhost:5432/textbook"

    # Authentication Configuration
    jwt_secret_key: str = "your-secret-key-change-in-production"
    jwt_algorithm: str = "HS256"
    access_token_expire_minutes: int = 1440  # 24 hours

    # Email Configuration (Resend)
    resend_api_key: str = ""
    from_email: str = "noreply@example.com"

    class Config:
        env_file = ".env"
        case_sensitive = False


@lru_cache()
def get_settings() -> Settings:
    """
    Get cached settings instance

    Using lru_cache ensures settings are loaded once and reused
    """
    return Settings()


# Export settings instance for easy import
settings = get_settings()
