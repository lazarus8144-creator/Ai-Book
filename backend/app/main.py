"""
FastAPI Main Application
========================

Entry point for the RAG chatbot backend.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.routers import health, query

# Initialize FastAPI
app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="Retrieval-Augmented Generation API for textbook Q&A",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.get_allowed_origins(),
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health.router, prefix="/api/v1", tags=["Health"])
app.include_router(query.router, prefix="/api/v1", tags=["Query"])


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Physical AI Textbook RAG API",
        "docs": "/docs",
        "health": "/api/v1/health"
    }


@app.on_event("startup")
async def startup_event():
    """Run on application startup"""
    print(f"🚀 Starting RAG API in {settings.ENVIRONMENT} mode")
    print(f"📚 Qdrant: {settings.QDRANT_URL}")
    print(f"🔧 CORS origins: {settings.get_allowed_origins()}")
