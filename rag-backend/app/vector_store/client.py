"""
Qdrant client wrapper for vector operations
"""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse
import logging

from app.config import settings

logger = logging.getLogger(__name__)


class VectorStore:
    """
    Qdrant vector database client wrapper

    Provides methods for:
    - Creating collections
    - Upserting vectors with payloads
    - Searching vectors by similarity
    - Deleting collections
    """

    def __init__(self):
        """Initialize Qdrant client with settings"""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name
        self.vector_size = settings.qdrant_vector_size

    async def ensure_collection_exists(self) -> bool:
        """
        Create collection if it doesn't exist

        Returns:
            True if collection was created or already exists
        """
        try:
            collections = self.client.get_collections().collections
            collection_names = [col.name for col in collections]

            if self.collection_name not in collection_names:
                logger.info(f"Creating collection: {self.collection_name}")
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size,
                        distance=models.Distance.COSINE,
                    ),
                )
                logger.info(f"Collection {self.collection_name} created successfully")
            else:
                logger.info(f"Collection {self.collection_name} already exists")

            return True

        except Exception as e:
            logger.error(f"Failed to ensure collection exists: {e}")
            raise

    def upsert_vectors(
        self,
        vectors: List[List[float]],
        payloads: List[Dict[str, Any]],
        ids: Optional[List[str]] = None,
    ) -> bool:
        """
        Upsert vectors with payloads to Qdrant

        Args:
            vectors: List of embedding vectors
            payloads: List of metadata dictionaries (must match vectors length)
            ids: Optional list of unique IDs (auto-generated if None)

        Returns:
            True if upsert succeeded
        """
        try:
            if len(vectors) != len(payloads):
                raise ValueError("vectors and payloads must have same length")

            if ids is None:
                ids = [f"vec_{i}" for i in range(len(vectors))]

            points = [
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload=payload,
                )
                for point_id, vector, payload in zip(ids, vectors, payloads)
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )

            logger.info(f"Upserted {len(vectors)} vectors to {self.collection_name}")
            return True

        except Exception as e:
            logger.error(f"Failed to upsert vectors: {e}")
            raise

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None,
        score_threshold: float = 0.5,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant

        Args:
            query_vector: Embedding vector to search for
            top_k: Number of results to return
            filters: Optional metadata filters (e.g., {"module": "module-1-ros2"})
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of search results with score, payload, id
        """
        try:
            # Build filter if provided
            query_filter = None
            if filters:
                must_conditions = []
                for key, value in filters.items():
                    must_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value),
                        )
                    )
                query_filter = models.Filter(must=must_conditions)

            # Perform search
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                query_filter=query_filter,
                score_threshold=score_threshold,
            )

            # Format results
            formatted_results = [
                {
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                }
                for result in results
            ]

            logger.info(
                f"Search returned {len(formatted_results)} results (threshold: {score_threshold})"
            )
            return formatted_results

        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection statistics

        Returns:
            Dictionary with collection info (vector count, status, etc.)
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "vectors_count": collection_info.vectors_count,
                "points_count": collection_info.points_count,
                "status": collection_info.status,
            }
        except UnexpectedResponse:
            logger.warning(f"Collection {self.collection_name} does not exist")
            return {
                "name": self.collection_name,
                "vectors_count": 0,
                "points_count": 0,
                "status": "not_found",
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise

    def delete_collection(self) -> bool:
        """
        Delete the collection (use with caution!)

        Returns:
            True if deletion succeeded
        """
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to delete collection: {e}")
            raise


# Global vector store instance
_vector_store: Optional[VectorStore] = None


def get_vector_store() -> VectorStore:
    """
    Get or create global VectorStore instance

    Returns:
        Singleton VectorStore instance
    """
    global _vector_store
    if _vector_store is None:
        _vector_store = VectorStore()
    return _vector_store
