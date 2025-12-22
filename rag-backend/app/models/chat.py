"""
Chat-related Pydantic models
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Literal
from datetime import datetime


class Citation(BaseModel):
    """Source citation for RAG answers"""

    text: str = Field(..., description="Relevant text snippet from source")
    url: str = Field(..., description="URL to textbook section")
    module: str = Field(..., description="Module name (e.g., 'module-1-ros2')")
    chapter: str = Field(..., description="Chapter name (e.g., '01-introduction')")
    heading: Optional[str] = Field(None, description="Section heading if available")
    score: float = Field(..., description="Relevance score (0-1)")


class ChatQuery(BaseModel):
    """Chat query request"""

    query: str = Field(..., min_length=1, max_length=1000, description="User question")
    mode: Literal["full_book", "selected_text"] = Field(
        default="full_book",
        description="Query mode: full_book or selected_text",
    )
    selected_text: Optional[str] = Field(
        None, max_length=10000, description="User-selected text for context-restricted mode"
    )
    filters: Optional[dict] = Field(
        None, description="Optional filters (e.g., {'module': 'module-1-ros2'})"
    )
    user_id: Optional[int] = Field(
        None, description="Optional authenticated user ID for personalization"
    )
    skill_level: Optional[Literal["beginner", "intermediate", "advanced"]] = Field(
        None, description="Optional user skill level for response personalization"
    )


class ChatResponse(BaseModel):
    """Chat response with answer and citations"""

    answer: str = Field(..., description="Generated answer text")
    sources: List[Citation] = Field(..., description="Source citations")
    agent_used: str = Field(..., description="Agent that processed query")
    response_time: float = Field(..., description="Response time in seconds")
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class HealthResponse(BaseModel):
    """Health check response"""

    status: str
    services: dict
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class CollectionStatsResponse(BaseModel):
    """Qdrant collection statistics"""

    name: str
    vectors_count: int
    points_count: int
    status: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
