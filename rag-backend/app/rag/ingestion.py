"""
Document ingestion pipeline for markdown files
"""

import os
import logging
from typing import List, Dict, Any
from pathlib import Path
import frontmatter
import re

from app.rag.chunking import get_chunking_service
from app.rag.embeddings import get_embedding_service
from app.vector_store.client import get_vector_store
from app.models.documents import DocumentChunk

logger = logging.getLogger(__name__)


class IngestionPipeline:
    """
    Pipeline for ingesting markdown documents into Qdrant

    Steps:
    1. Load markdown files from directory
    2. Parse frontmatter and extract metadata
    3. Chunk text semantically
    4. Generate embeddings
    5. Upsert to Qdrant with metadata
    """

    def __init__(self):
        self.chunking_service = get_chunking_service()
        self.embedding_service = get_embedding_service()
        self.vector_store = get_vector_store()

    async def ingest_directory(
        self, docs_path: str, force_reindex: bool = False
    ) -> Dict[str, Any]:
        """
        Ingest all markdown files from directory

        Args:
            docs_path: Path to docs directory
            force_reindex: If True, delete existing collection first

        Returns:
            Dictionary with ingestion statistics
        """
        try:
            # Ensure collection exists
            await self.vector_store.ensure_collection_exists()

            if force_reindex:
                logger.warning("Force reindex: deleting existing collection")
                self.vector_store.delete_collection()
                await self.vector_store.ensure_collection_exists()

            # Find all markdown files
            markdown_files = self._find_markdown_files(docs_path)
            logger.info(f"Found {len(markdown_files)} markdown files")

            # Process each file
            all_chunks = []
            files_processed = 0
            errors = []

            for file_path in markdown_files:
                try:
                    chunks = self._process_file(file_path, docs_path)
                    all_chunks.extend(chunks)
                    files_processed += 1
                    logger.info(f"Processed {file_path} → {len(chunks)} chunks")
                except Exception as e:
                    error_msg = f"Failed to process {file_path}: {e}"
                    logger.error(error_msg)
                    errors.append(error_msg)

            # Generate embeddings for all chunks
            logger.info(f"Generating embeddings for {len(all_chunks)} chunks...")
            chunk_texts = [chunk["text"] for chunk in all_chunks]
            embeddings = self.embedding_service.embed_texts(chunk_texts)

            # Prepare payloads for Qdrant
            payloads = []
            ids = []
            for idx, chunk in enumerate(all_chunks):
                payload = {
                    "text": chunk["text"],
                    "module": chunk["metadata"]["module"],
                    "chapter": chunk["metadata"]["chapter"],
                    "heading": chunk["metadata"].get("heading", ""),
                    "url": chunk["metadata"]["url"],
                    "chunk_index": chunk["chunk_index"],
                    "token_count": chunk["token_count"],
                }
                payloads.append(payload)
                ids.append(f"{chunk['metadata']['chapter']}_chunk_{idx}")

            # Upsert to Qdrant
            logger.info(f"Upserting {len(embeddings)} vectors to Qdrant...")
            self.vector_store.upsert_vectors(
                vectors=embeddings,
                payloads=payloads,
                ids=ids,
            )

            return {
                "files_processed": files_processed,
                "chunks_created": len(all_chunks),
                "embeddings_generated": len(embeddings),
                "qdrant_upserted": len(embeddings),
                "errors": errors,
            }

        except Exception as e:
            logger.error(f"Ingestion pipeline failed: {e}")
            raise

    def _find_markdown_files(self, docs_path: str) -> List[str]:
        """
        Recursively find all .md files in directory

        Args:
            docs_path: Path to docs directory

        Returns:
            List of absolute file paths
        """
        markdown_files = []
        docs_dir = Path(docs_path)

        if not docs_dir.exists():
            raise FileNotFoundError(f"Directory not found: {docs_path}")

        for file_path in docs_dir.rglob("*.md"):
            # Skip certain files
            if file_path.name in ["README.md", "_category_.json"]:
                continue
            markdown_files.append(str(file_path))

        return sorted(markdown_files)

    def _process_file(self, file_path: str, docs_root: str) -> List[Dict[str, Any]]:
        """
        Process a single markdown file

        Args:
            file_path: Absolute path to markdown file
            docs_root: Root docs directory for URL generation

        Returns:
            List of chunk dictionaries
        """
        # Parse markdown with frontmatter
        with open(file_path, "r", encoding="utf-8") as f:
            post = frontmatter.load(f)

        content = post.content
        metadata = post.metadata

        # Extract module and chapter from file path
        relative_path = Path(file_path).relative_to(docs_root)
        path_parts = relative_path.parts

        # Determine module (e.g., "module-1-ros2")
        module = "unknown"
        if len(path_parts) > 0 and path_parts[0].startswith("module-"):
            module = path_parts[0]

        # Determine chapter (filename without extension)
        chapter = relative_path.stem

        # Generate URL path
        url_path = "/" + str(relative_path.with_suffix("")).replace("\\", "/")

        # Extract headings from content for better chunking
        headings = self._extract_headings(content)

        # Chunk the content
        chunk_metadata = {
            "module": module,
            "chapter": chapter,
            "url": url_path,
            "title": metadata.get("title", chapter),
        }

        chunks = self.chunking_service.chunk_with_metadata(content, chunk_metadata)

        return chunks

    def _extract_headings(self, content: str) -> List[str]:
        """
        Extract markdown headings from content

        Args:
            content: Markdown content

        Returns:
            List of heading texts
        """
        heading_pattern = r"^#{1,6}\s+(.+)$"
        headings = re.findall(heading_pattern, content, re.MULTILINE)
        return headings


# Global ingestion pipeline
_ingestion_pipeline = None


def get_ingestion_pipeline() -> IngestionPipeline:
    """Get or create global IngestionPipeline instance"""
    global _ingestion_pipeline
    if _ingestion_pipeline is None:
        _ingestion_pipeline = IngestionPipeline()
    return _ingestion_pipeline
