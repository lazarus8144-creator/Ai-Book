"""
API endpoint tests
"""

import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)


def test_root_endpoint():
    """Test root endpoint returns API info"""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["message"] == "RAG Chatbot API"
    assert data["version"] == "1.0.0"
    assert data["status"] == "running"


def test_health_endpoint():
    """Test health check endpoint"""
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert "services" in data
    assert "timestamp" in data


def test_chat_query_validation():
    """Test chat query validates empty input"""
    response = client.post(
        "/api/v1/chat/query",
        json={"query": "", "mode": "full_book"},
    )
    # Should return 422 for validation error (empty query)
    assert response.status_code == 422


def test_chat_query_mode_validation():
    """Test chat query validates mode"""
    response = client.post(
        "/api/v1/chat/query",
        json={"query": "What is ROS 2?", "mode": "invalid_mode"},
    )
    # Should return 422 for invalid mode
    assert response.status_code == 422


# Note: Full integration tests require Qdrant and OpenAI to be configured
# These are smoke tests that verify API structure only
