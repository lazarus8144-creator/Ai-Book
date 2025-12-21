"""
Text chunking utilities for document ingestion
"""

from typing import List
from langchain_text_splitters import RecursiveCharacterTextSplitter
import tiktoken
import logging

from app.config import settings

logger = logging.getLogger(__name__)


class ChunkingService:
    """
    Service for splitting text into semantic chunks
    """

    def __init__(self):
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap
        self.encoding = tiktoken.encoding_for_model("gpt-4")

        # RecursiveCharacterTextSplitter with semantic separators
        self.splitter = RecursiveCharacterTextSplitter(
            chunk_size=self.chunk_size,
            chunk_overlap=self.chunk_overlap,
            length_function=self._count_tokens,
            separators=[
                "\n\n## ",  # H2 heading
                "\n\n### ",  # H3 heading
                "\n\n",  # Paragraph
                "\n",  # Line
                " ",  # Word
                "",  # Character
            ],
        )

    def _count_tokens(self, text: str) -> int:
        """
        Count tokens in text using tiktoken

        Args:
            text: Text to count tokens for

        Returns:
            Token count
        """
        return len(self.encoding.encode(text))

    def chunk_text(self, text: str) -> List[str]:
        """
        Split text into semantic chunks

        Args:
            text: Text to chunk

        Returns:
            List of text chunks
        """
        try:
            chunks = self.splitter.split_text(text)
            logger.info(f"Split text into {len(chunks)} chunks")
            return chunks

        except Exception as e:
            logger.error(f"Failed to chunk text: {e}")
            raise

    def chunk_with_metadata(self, text: str, metadata: dict) -> List[dict]:
        """
        Chunk text and attach metadata to each chunk

        Args:
            text: Text to chunk
            metadata: Metadata dict to attach (module, chapter, url, etc.)

        Returns:
            List of dicts with 'text', 'metadata', 'chunk_index', 'token_count'
        """
        try:
            chunks = self.chunk_text(text)
            result = []

            for idx, chunk in enumerate(chunks):
                result.append(
                    {
                        "text": chunk,
                        "metadata": {**metadata, "chunk_index": idx},
                        "chunk_index": idx,
                        "token_count": self._count_tokens(chunk),
                    }
                )

            logger.info(
                f"Created {len(result)} chunks with metadata for {metadata.get('chapter', 'unknown')}"
            )
            return result

        except Exception as e:
            logger.error(f"Failed to chunk with metadata: {e}")
            raise


# Global chunking service
_chunking_service = None


def get_chunking_service() -> ChunkingService:
    """Get or create global ChunkingService instance"""
    global _chunking_service
    if _chunking_service is None:
        _chunking_service = ChunkingService()
    return _chunking_service
