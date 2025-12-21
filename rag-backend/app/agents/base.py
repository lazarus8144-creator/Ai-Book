"""
Base agent interface for all RAG agents
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Any
from pydantic import BaseModel


class AgentContext(BaseModel):
    """Context passed to agents"""

    query: str
    selected_text: str | None = None
    filters: Dict[str, Any] | None = None
    conversation_history: List[Dict[str, str]] = []


class AgentResponse(BaseModel):
    """Response from agents"""

    answer: str
    sources: List[Dict[str, Any]]
    confidence: float
    agent_name: str


class BaseAgent(ABC):
    """
    Base class for all RAG agents

    Provides common interface for modular intelligence components.
    Each agent implements a specific skill (e.g., BookSearch, SelectedText, Citation).
    """

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    async def process(self, context: AgentContext) -> AgentResponse:
        """
        Process query and return response

        Args:
            context: AgentContext with query, filters, history

        Returns:
            AgentResponse with answer and sources
        """
        pass

    def _validate_context(self, context: AgentContext) -> bool:
        """
        Validate that context meets agent requirements

        Returns:
            True if valid, raises ValueError otherwise
        """
        if not context.query or not context.query.strip():
            raise ValueError("Query cannot be empty")
        return True
