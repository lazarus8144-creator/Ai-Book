"""
Query Router
============

Handles RAG query endpoints using 100% FREE services.
"""

from fastapi import APIRouter, HTTPException
from app.models import QueryRequest, QueryResponse
from app.services.rag_pipeline_free import rag_pipeline
import traceback
from loguru import logger

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
        logger.info(f"Processing query: {request.question[:100]}...")
        result = await rag_pipeline.query(
            question=request.question,
            max_results=request.max_results or 10  # Increased from 5
        )
        logger.info(f"Query successful. Response time: {result.get('response_time_ms', 0)}ms")
        return QueryResponse(**result)
    except Exception as e:
        logger.error(f"Query failed: {str(e)}")
        logger.error(f"Traceback: {traceback.format_exc()}")
        raise HTTPException(
            status_code=500,
            detail=f"Query failed: {str(e)}"
        )
