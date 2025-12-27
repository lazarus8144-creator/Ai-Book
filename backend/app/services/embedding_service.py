"""
Embedding Service
==================

Handles text embedding generation using OpenAI's embedding models.
"""

from typing import List
from openai import AsyncOpenAI
from app.config import settings


class EmbeddingService:
    """Service for generating text embeddings"""

    def __init__(self):
        self.client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = settings.OPENAI_EMBEDDING_MODEL

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Input text to embed

        Returns:
            List of embedding values (1536 dimensions for text-embedding-3-small)
        """
        response = await self.client.embeddings.create(
            model=self.model,
            input=text
        )
        return response.data[0].embedding

    async def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        response = await self.client.embeddings.create(
            model=self.model,
            input=texts
        )
        return [item.embedding for item in response.data]


# Singleton instance
embedding_service = EmbeddingService()
