"""
Mock embeddings service for demo/offline mode
Generates deterministic fake embeddings without calling OpenAI API
"""

import hashlib
from typing import List
import logging

logger = logging.getLogger(__name__)


class MockEmbeddingService:
    """
    Mock embedding service that generates fake but consistent embeddings

    Uses text hash to create deterministic 1536-dimensional vectors
    This allows vector search to work without OpenAI API calls
    """

    def __init__(self):
        self.model = "mock-embedding-model"
        self.dimensions = 1536

    def embed_text(self, text: str) -> List[float]:
        """
        Generate a fake embedding from text hash

        Args:
            text: Text to embed

        Returns:
            Fake 1536-dimensional embedding vector
        """
        logger.debug(f"[DEMO MODE] Generating mock embedding for text ({len(text)} chars)")
        return self._generate_fake_embedding(text)

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate fake embeddings for multiple texts

        Args:
            texts: List of texts to embed

        Returns:
            List of fake embedding vectors
        """
        logger.info(f"[DEMO MODE] Generating {len(texts)} mock embeddings")
        return [self._generate_fake_embedding(text) for text in texts]

    def _generate_fake_embedding(self, text: str) -> List[float]:
        """
        Generate a deterministic fake embedding from text

        Uses MD5 hash of text to generate consistent vectors
        Similar texts will have similar embeddings (simple cosine similarity)
        """
        # Create hash of text
        text_hash = hashlib.md5(text.lower().encode()).hexdigest()

        # Convert hash to seed
        seed_value = int(text_hash[:8], 16)

        # Generate deterministic pseudo-random vector
        vector = []
        current = seed_value

        for i in range(self.dimensions):
            # Simple LCG (Linear Congruential Generator)
            current = (1103515245 * current + 12345) % (2**31)
            # Normalize to [-1, 1]
            value = (current / (2**30)) - 1.0
            vector.append(value)

        # Normalize vector to unit length (for cosine similarity)
        magnitude = sum(x * x for x in vector) ** 0.5
        normalized = [x / magnitude for x in vector]

        return normalized


def get_mock_embedding_service() -> MockEmbeddingService:
    """Get mock embedding service instance"""
    return MockEmbeddingService()
