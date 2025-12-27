"""
Health Check Endpoint
=====================

Health check to verify FREE services are running.
"""

from fastapi import APIRouter
from app.models import HealthResponse
from app.config import settings

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint for FREE setup.

    Returns service status and version.
    """
    # Check Qdrant
    qdrant_ok = bool(settings.QDRANT_URL and settings.QDRANT_API_KEY)

    # Check Groq
    groq_ok = bool(settings.GROQ_API_KEY and settings.GROQ_API_KEY != "YOUR_GROQ_KEY_HERE")

    # Check Embeddings
    embeddings_ok = bool(settings.EMBEDDING_MODEL)

    status = "healthy" if (qdrant_ok and groq_ok and embeddings_ok) else "degraded"

    return HealthResponse(
        status=status,
        qdrant_connected=qdrant_ok,
        groq_available=groq_ok,
        embeddings_loaded=embeddings_ok,
        version="1.0.0-free"
    )
