"""
Health check and monitoring routes
"""

from fastapi import APIRouter, HTTPException
import logging

from app.models.chat import HealthResponse, CollectionStatsResponse
from app.vector_store.client import get_vector_store

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint

    Returns:
        HealthResponse with service statuses
    """
    try:
        # Check Qdrant connection
        vector_store = get_vector_store()
        collection_info = vector_store.get_collection_info()

        qdrant_status = (
            "healthy" if collection_info.get("status") != "not_found" else "not_configured"
        )

        return HealthResponse(
            status="healthy",
            services={
                "api": "running",
                "qdrant": qdrant_status,
                "openai": "configured",  # Assumes API key is set
            },
        )

    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return HealthResponse(
            status="degraded",
            services={
                "api": "running",
                "qdrant": "error",
                "openai": "unknown",
            },
        )


@router.get("/collections/stats", response_model=CollectionStatsResponse)
async def get_collection_stats():
    """
    Get Qdrant collection statistics

    Returns:
        CollectionStatsResponse with vector counts and status
    """
    try:
        vector_store = get_vector_store()
        info = vector_store.get_collection_info()

        return CollectionStatsResponse(
            name=info["name"],
            vectors_count=info["vectors_count"],
            points_count=info["points_count"],
            status=info["status"],
        )

    except Exception as e:
        logger.error(f"Failed to get collection stats: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to retrieve collection statistics",
        )
