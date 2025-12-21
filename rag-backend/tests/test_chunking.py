"""
Chunking service tests
"""

import pytest
from app.rag.chunking import ChunkingService


def test_chunking_service_initialization():
    """Test chunking service initializes correctly"""
    service = ChunkingService()
    assert service.chunk_size == 500
    assert service.chunk_overlap == 50
    assert service.splitter is not None


def test_chunk_text_basic():
    """Test basic text chunking"""
    service = ChunkingService()
    text = "This is a test paragraph. " * 100  # Long text
    chunks = service.chunk_text(text)
    assert len(chunks) > 0
    assert all(isinstance(chunk, str) for chunk in chunks)


def test_chunk_with_metadata():
    """Test chunking with metadata attachment"""
    service = ChunkingService()
    text = "This is test content for chunking. " * 50
    metadata = {
        "module": "module-1-ros2",
        "chapter": "01-introduction",
        "url": "/module-1-ros2/01-introduction",
        "title": "Introduction to ROS 2",
    }

    chunks = service.chunk_with_metadata(text, metadata)

    assert len(chunks) > 0
    for chunk in chunks:
        assert "text" in chunk
        assert "metadata" in chunk
        assert "chunk_index" in chunk
        assert "token_count" in chunk
        assert chunk["metadata"]["module"] == "module-1-ros2"
        assert chunk["metadata"]["chapter"] == "01-introduction"


def test_empty_text_chunking():
    """Test chunking handles empty text"""
    service = ChunkingService()
    chunks = service.chunk_text("")
    assert len(chunks) == 0 or (len(chunks) == 1 and chunks[0] == "")
