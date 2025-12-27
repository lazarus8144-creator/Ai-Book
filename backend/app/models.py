"""
Pydantic Models
===============

Request/Response schemas for API endpoints.
"""

from pydantic import BaseModel, Field, field_validator
from typing import List, Optional, Dict, Any


class QueryRequest(BaseModel):
    """Request schema for POST /api/v1/query"""
    question: str = Field(..., min_length=5, max_length=500)
    max_results: Optional[int] = Field(5, ge=1, le=10)

    @field_validator('question')
    @classmethod
    def validate_question(cls, v: str) -> str:
        # Remove excessive whitespace
        v = " ".join(v.split())

        # Check for injection attempts
        forbidden = ["ignore previous", "system:", "assistant:"]
        if any(pattern in v.lower() for pattern in forbidden):
            raise ValueError("Invalid question format")

        return v


class Source(BaseModel):
    """Source citation for RAG response"""
    module: str
    title: str
    url: str
    score: float = Field(..., ge=0.0, le=1.0)


class QueryResponse(BaseModel):
    """Response schema for POST /api/v1/query"""
    answer: str
    sources: List[Source]
    response_time_ms: int
    tokens_used: int
    metadata: Optional[Dict[str, Any]] = {}


class HealthResponse(BaseModel):
    """Response schema for GET /api/v1/health"""
    status: str
    qdrant_connected: bool
    groq_available: bool
    embeddings_loaded: bool
    version: str = "1.0.0-free"
