"""
Document and ingestion-related models
"""

from pydantic import BaseModel, Field
from typing import List, Optional


class DocumentChunk(BaseModel):
    """A chunk of textbook content with metadata"""

    text: str = Field(..., description="Chunk text content")
    module: str = Field(..., description="Module identifier")
    chapter: str = Field(..., description="Chapter identifier")
    heading: Optional[str] = Field(None, description="Section heading")
    url: str = Field(..., description="URL path to section")
    chunk_index: int = Field(..., description="Index of chunk within document")
    token_count: int = Field(..., description="Number of tokens in chunk")


class IngestionRequest(BaseModel):
    """Request to trigger document ingestion"""

    docs_path: str = Field(
        default="textbook/docs",
        description="Path to markdown documents directory",
    )
    force_reindex: bool = Field(
        default=False,
        description="Force re-indexing even if vectors exist",
    )


class IngestionResponse(BaseModel):
    """Response from ingestion process"""

    files_processed: int
    chunks_created: int
    embeddings_generated: int
    qdrant_upserted: int
    errors: List[str] = Field(default_factory=list)
    duration_seconds: float
