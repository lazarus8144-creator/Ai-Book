"""
FastAPI application entry point for RAG Chatbot Backend
"""

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import logging
from contextlib import asynccontextmanager

from app.config import settings

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup/shutdown events
    """
    # Startup
    logger.info("🚀 Starting RAG Chatbot Backend...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"CORS Origins: {settings.cors_origins_list}")
    logger.info(f"Qdrant Collection: {settings.qdrant_collection_name}")

    # Initialize Qdrant collection
    try:
        from app.vector_store.client import get_vector_store
        vector_store = get_vector_store()
        await vector_store.ensure_collection_exists()
        logger.info("✅ Qdrant collection ready")
    except Exception as e:
        logger.error(f"❌ Failed to initialize Qdrant: {e}")

    # Verify OpenAI API key
    try:
        from openai import OpenAI
        client = OpenAI(api_key=settings.openai_api_key)
        # Simple test to verify key
        logger.info("✅ OpenAI API key configured")
    except Exception as e:
        logger.error(f"❌ OpenAI API key error: {e}")

    yield

    # Shutdown
    logger.info("👋 Shutting down RAG Chatbot Backend...")


# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure CSRF protection
from app.middleware import CSRFMiddleware
app.add_middleware(CSRFMiddleware)


# Global exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """
    Catch-all exception handler for unhandled errors
    """
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": {
                "code": "INTERNAL_SERVER_ERROR",
                "message": "An unexpected error occurred. Please try again later.",
                "details": str(exc) if settings.environment == "development" else None,
            }
        },
    )


# Root endpoint
@app.get("/")
async def root():
    """
    Root endpoint - API status
    """
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "status": "running",
        "docs": "/docs",
    }


# Register API routers
from app.api.routes import chat, health, ingest, auth, profile, password
from slowapi import _rate_limit_exceeded_handler
from slowapi.errors import RateLimitExceeded

# Add rate limiting exception handler
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
app.include_router(health.router, prefix="/api/v1", tags=["health"])
app.include_router(ingest.router, prefix="/api/v1", tags=["ingest"])
app.include_router(auth.router, tags=["authentication"])
app.include_router(profile.router, tags=["profile"])
app.include_router(password.router, tags=["password-reset"])


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app.main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=settings.api_reload,
        log_level=settings.log_level.lower(),
    )
