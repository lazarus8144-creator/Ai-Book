"""
OpenAI embeddings wrapper
"""

from typing import List
from openai import OpenAI
import logging

from app.config import settings
from app.rag.mock_embeddings import get_mock_embedding_service

logger = logging.getLogger(__name__)


class EmbeddingService:
    """
    Service for generating embeddings using OpenAI
    """

    def __init__(self):
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.model = settings.openai_embedding_model

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Text to embed

        Returns:
            Embedding vector (1536 dimensions for text-embedding-3-small)
        """
        try:
            response = self.client.embeddings.create(
                input=text,
                model=self.model,
            )
            embedding = response.data[0].embedding
            logger.debug(f"Generated embedding for text ({len(text)} chars)")
            return embedding

        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            raise

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts (batch)

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        try:
            response = self.client.embeddings.create(
                input=texts,
                model=self.model,
            )
            embeddings = [item.embedding for item in response.data]
            logger.info(f"Generated {len(embeddings)} embeddings")
            return embeddings

        except Exception as e:
            logger.error(f"Failed to generate batch embeddings: {e}")
            raise


# Global embedding service instance
_embedding_service = None


def get_embedding_service():
    """Get or create global EmbeddingService instance"""
    global _embedding_service

    # Use mock service in demo mode
    if settings.demo_mode:
        logger.info("🎭 DEMO MODE: Using mock embeddings (no OpenAI API calls)")
        return get_mock_embedding_service()

    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
