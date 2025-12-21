"""
Chat API routes
"""

from fastapi import APIRouter, HTTPException, status
import logging
import time

from app.models.chat import ChatQuery, ChatResponse, Citation
from app.agents.base import AgentContext
from app.agents.orchestrator import get_orchestrator

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/chat/query", response_model=ChatResponse)
async def query_chatbot(query: ChatQuery):
    """
    Process chat query and return answer with citations

    Args:
        query: ChatQuery with question, mode, selected_text, filters

    Returns:
        ChatResponse with answer, sources, agent_used, response_time
    """
    start_time = time.time()

    try:
        logger.info(f"Received query: {query.query[:50]}... (mode: {query.mode})")

        # Create agent context
        context = AgentContext(
            query=query.query,
            selected_text=query.selected_text,
            filters=query.filters,
        )

        # Route to appropriate agent via orchestrator
        orchestrator = get_orchestrator()
        agent_response = await orchestrator.process_query(context)

        # Convert agent sources to Citation models
        citations = [
            Citation(
                text=source["text"],
                url=source["url"],
                module=source["module"],
                chapter=source["chapter"],
                heading=source.get("heading"),
                score=source["score"],
            )
            for source in agent_response.sources
        ]

        # Build response
        response_time = time.time() - start_time
        response = ChatResponse(
            answer=agent_response.answer,
            sources=citations,
            agent_used=agent_response.agent_name,
            response_time=response_time,
        )

        logger.info(
            f"Query processed in {response_time:.2f}s by {agent_response.agent_name}"
        )
        return response

    except ValueError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to process query. Please try again later.",
        )
