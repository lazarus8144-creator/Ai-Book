#!/usr/bin/env python3
"""
Document ingestion script

Usage:
    python scripts/ingest_docs.py --docs-path ../textbook/docs [--force-reindex]
"""

import asyncio
import argparse
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.rag.ingestion import get_ingestion_pipeline
from app.config import settings
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def main():
    parser = argparse.ArgumentParser(description="Ingest textbook documents into Qdrant")
    parser.add_argument(
        "--docs-path",
        type=str,
        default="../textbook/docs",
        help="Path to textbook docs directory (default: ../textbook/docs)",
    )
    parser.add_argument(
        "--force-reindex",
        action="store_true",
        help="Force re-indexing (delete existing collection first)",
    )

    args = parser.parse_args()

    # Resolve absolute path
    docs_path = Path(args.docs_path).resolve()

    if not docs_path.exists():
        logger.error(f"❌ Directory not found: {docs_path}")
        sys.exit(1)

    logger.info("=" * 60)
    logger.info("📚 Textbook Document Ingestion")
    logger.info("=" * 60)
    logger.info(f"Docs Path: {docs_path}")
    logger.info(f"Qdrant URL: {settings.qdrant_url}")
    logger.info(f"Collection: {settings.qdrant_collection_name}")
    logger.info(f"Force Reindex: {args.force_reindex}")
    logger.info("=" * 60)

    try:
        pipeline = get_ingestion_pipeline()

        logger.info("🚀 Starting ingestion pipeline...")
        result = await pipeline.ingest_directory(
            docs_path=str(docs_path),
            force_reindex=args.force_reindex,
        )

        logger.info("=" * 60)
        logger.info("✅ Ingestion Complete!")
        logger.info("=" * 60)
        logger.info(f"Files Processed: {result['files_processed']}")
        logger.info(f"Chunks Created: {result['chunks_created']}")
        logger.info(f"Embeddings Generated: {result['embeddings_generated']}")
        logger.info(f"Vectors Upserted: {result['qdrant_upserted']}")

        if result["errors"]:
            logger.warning(f"Errors Encountered: {len(result['errors'])}")
            for error in result["errors"]:
                logger.warning(f"  - {error}")
        else:
            logger.info("Errors: None")

        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"❌ Ingestion failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
