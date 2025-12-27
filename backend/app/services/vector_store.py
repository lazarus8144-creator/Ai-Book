"""
Vector Store Service
====================

Handles interactions with Qdrant vector database.
"""

from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, ScoredPoint
from app.config import settings


class VectorStore:
    """Service for vector storage and retrieval using Qdrant"""

    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME

    def ensure_collection(self, vector_size: int = 384):
        """
        Create collection if it doesn't exist.

        Args:
            vector_size: Dimension of embedding vectors (default: 384 for sentence-transformers/all-MiniLM-L6-v2)
        """
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.collection_name not in collection_names:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE
                )
            )

    async def search(
        self,
        query_vector: List[float],
        limit: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors.

        Args:
            query_vector: Query embedding vector
            limit: Maximum number of results

        Returns:
            List of search results with metadata
        """
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            limit=limit
        ).points

        return [
            {
                "id": result.id,
                "score": result.score,
                "payload": result.payload
            }
            for result in results
        ]

    def upsert_points(self, points: List[PointStruct]):
        """
        Insert or update points in the collection.

        Args:
            points: List of PointStruct objects to upsert
        """
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the collection"""
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "exists": True,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count
            }
        except Exception:
            return {"exists": False}


# Singleton instance
vector_store = VectorStore()
