"""
Document ingestion API routes
"""

from fastapi import APIRouter, HTTPException, BackgroundTasks
import logging
import time

from app.models.documents import IngestionRequest, IngestionResponse
from app.rag.ingestion import get_ingestion_pipeline

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/ingest/documents", response_model=IngestionResponse)
async def ingest_documents(request: IngestionRequest, background_tasks: BackgroundTasks):
    """
    Trigger document ingestion pipeline

    Args:
        request: IngestionRequest with docs_path and force_reindex
        background_tasks: FastAPI background tasks

    Returns:
        IngestionResponse with ingestion statistics
    """
    start_time = time.time()

    try:
        logger.info(f"Starting ingestion from: {request.docs_path}")
        logger.info(f"Force reindex: {request.force_reindex}")

        # Get ingestion pipeline
        pipeline = get_ingestion_pipeline()

        # Run ingestion
        result = await pipeline.ingest_directory(
            docs_path=request.docs_path,
            force_reindex=request.force_reindex,
        )

        # Calculate duration
        duration = time.time() - start_time

        # Build response
        response = IngestionResponse(
            files_processed=result["files_processed"],
            chunks_created=result["chunks_created"],
            embeddings_generated=result["embeddings_generated"],
            qdrant_upserted=result["qdrant_upserted"],
            errors=result["errors"],
            duration_seconds=duration,
        )

        logger.info(
            f"Ingestion completed in {duration:.2f}s: "
            f"{result['files_processed']} files, {result['chunks_created']} chunks"
        )

        return response

    except FileNotFoundError as e:
        logger.error(f"Directory not found: {e}")
        raise HTTPException(
            status_code=404,
            detail=f"Directory not found: {request.docs_path}",
        )
    except Exception as e:
        logger.error(f"Ingestion failed: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Document ingestion failed. Check server logs for details.",
        )
