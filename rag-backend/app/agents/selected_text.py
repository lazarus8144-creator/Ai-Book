"""
SelectedTextAgent - Context-restricted RAG agent
"""

from openai import OpenAI
import logging
import time

from app.agents.base import BaseAgent, AgentContext, AgentResponse
from app.config import settings
from app.agents.mock_llm import get_mock_llm_service

logger = logging.getLogger(__name__)


class SelectedTextAgent(BaseAgent):
    """
    Agent for answering questions based ONLY on user-selected text

    Skills:
    1. Validate selected text exists
    2. Use selected text as context (no vector search)
    3. Generate answer constrained to selected text only
    """

    def __init__(self):
        super().__init__("SelectedTextAgent")
        self.openai_client = OpenAI(api_key=settings.openai_api_key)

    async def process(self, context: AgentContext) -> AgentResponse:
        """
        Process selected-text query

        Args:
            context: AgentContext with query and selected_text

        Returns:
            AgentResponse with answer based only on selected text
        """
        self._validate_context(context)
        start_time = time.time()

        if not context.selected_text:
            raise ValueError("selected_text is required for SelectedTextAgent")

        try:
            logger.info(
                f"[{self.name}] Processing query with {len(context.selected_text)} chars of selected text"
            )

            # Generate answer using only selected text (or mock in demo mode)
            if settings.demo_mode:
                logger.info(f"[{self.name}] 🎭 DEMO MODE: Generating mock answer for selected text")
                mock_llm = get_mock_llm_service()
                answer = mock_llm.generate_answer(context.query, [context.selected_text], is_selected_text=True)
            else:
                answer = self._generate_answer(context.query, context.selected_text)

            # No vector search, so source is just the selected text itself
            sources = [
                {
                    "text": context.selected_text[:200] + "...",
                    "url": "",  # URL not available for selected text
                    "module": "selected_text",
                    "chapter": "user_selection",
                    "heading": None,
                    "score": 1.0,
                }
            ]

            elapsed = time.time() - start_time
            logger.info(f"[{self.name}] Completed in {elapsed:.2f}s")

            return AgentResponse(
                answer=answer,
                sources=sources,
                confidence=1.0,  # High confidence since we're using exact selected text
                agent_name=self.name,
            )

        except Exception as e:
            logger.error(f"[{self.name}] Error processing query: {e}")
            raise

    def _generate_answer(self, query: str, selected_text: str) -> str:
        """
        Generate answer using OpenAI with selected text as context

        Args:
            query: User question
            selected_text: User-selected text from page

        Returns:
            Generated answer string
        """
        try:
            system_prompt = """You are a helpful educational assistant for a Physical AI & Humanoid Robotics textbook.

Your task:
1. Answer the user's question using ONLY the selected text they highlighted
2. Do NOT use any external knowledge or information from other parts of the textbook
3. If the selected text doesn't contain enough information to answer, say so clearly
4. Be concise and focused on the selected content
5. Offer to search the full book if needed

Format:
- Provide a clear, direct answer based ONLY on the selected text
- Keep answers under 150 words
- If insufficient information, suggest "Would you like me to search the full book for more details?"
"""

            user_prompt = f"""Selected text from page:
{selected_text}

User Question: {query}

Please answer based ONLY on the selected text above."""

            response = self.openai_client.chat.completions.create(
                model=settings.openai_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                max_tokens=settings.openai_max_tokens,
                temperature=settings.openai_temperature,
            )

            answer = response.choices[0].message.content
            logger.debug(f"[{self.name}] Generated answer: {answer[:100]}...")
            return answer

        except Exception as e:
            logger.error(f"[{self.name}] Failed to generate answer: {e}")
            raise
