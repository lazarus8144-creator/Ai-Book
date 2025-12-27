"""
Query Router
============

Handles RAG query endpoints using 100% FREE services.
"""

from fastapi import APIRouter, HTTPException
from app.models import QueryRequest, QueryResponse
from app.services.rag_pipeline_free import rag_pipeline

router = APIRouter()


@router.post("/query", response_model=QueryResponse)
async def query_textbook(request: QueryRequest):
    """
    Query the textbook using RAG.

    Args:
        request: Query request with question and optional parameters

    Returns:
        QueryResponse with answer, sources, and metadata
    """
    try:
        result = await rag_pipeline.query(
            question=request.question,
            max_results=request.max_results or 5
        )
        return QueryResponse(**result)
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Query failed: {str(e)}"
        )
