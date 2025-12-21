"""
AgentOrchestrator - Routes queries to appropriate agents
"""

import logging
from typing import Dict

from app.agents.base import BaseAgent, AgentContext, AgentResponse
from app.agents.book_search import BookSearchAgent
from app.agents.selected_text import SelectedTextAgent

logger = logging.getLogger(__name__)


class AgentOrchestrator:
    """
    Orchestrator for routing queries to appropriate agents

    Routing logic:
    - If mode == "selected_text" and selected_text exists → SelectedTextAgent
    - Otherwise → BookSearchAgent
    """

    def __init__(self):
        self.agents: Dict[str, BaseAgent] = {
            "book_search": BookSearchAgent(),
            "selected_text": SelectedTextAgent(),
        }

    async def process_query(self, context: AgentContext) -> AgentResponse:
        """
        Route query to appropriate agent and return response

        Args:
            context: AgentContext with query, mode, selected_text, filters

        Returns:
            AgentResponse from the selected agent
        """
        try:
            # Determine which agent to use
            if context.selected_text and context.selected_text.strip():
                agent_key = "selected_text"
            else:
                agent_key = "book_search"

            logger.info(f"[Orchestrator] Routing to {agent_key}")
            agent = self.agents[agent_key]

            # Process query with selected agent
            response = await agent.process(context)
            return response

        except Exception as e:
            logger.error(f"[Orchestrator] Error processing query: {e}")
            raise


# Global orchestrator instance
_orchestrator = None


def get_orchestrator() -> AgentOrchestrator:
    """Get or create global AgentOrchestrator instance"""
    global _orchestrator
    if _orchestrator is None:
        _orchestrator = AgentOrchestrator()
    return _orchestrator
