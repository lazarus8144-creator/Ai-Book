"""
Free Embedding Service
=======================

Uses sentence-transformers for FREE local embeddings (no API costs!).
"""

from typing import List
from sentence_transformers import SentenceTransformer
from app.config import settings


class FreeEmbeddingService:
    """Service for generating embeddings using sentence-transformers (100% FREE)"""

    def __init__(self):
        # Load model once (downloads ~80MB first time, then cached)
        print(f"📥 Loading embedding model: {settings.EMBEDDING_MODEL}")
        self.model = SentenceTransformer(
            settings.EMBEDDING_MODEL,
            device=settings.EMBEDDING_DEVICE
        )
        print(f"✅ Embedding model loaded! Dimension: {self.model.get_sentence_embedding_dimension()}")

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Input text to embed

        Returns:
            List of embedding values (384 dimensions for all-MiniLM-L6-v2)
        """
        embedding = self.model.encode(text, convert_to_numpy=True)
        return embedding.tolist()

    async def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        embeddings = self.model.encode(texts, convert_to_numpy=True, show_progress_bar=True)
        return [emb.tolist() for emb in embeddings]


# Singleton instance
embedding_service = FreeEmbeddingService()
