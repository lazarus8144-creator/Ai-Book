"""
BookSearchAgent - Full-book vector search agent
"""

from typing import List, Dict, Any
from openai import OpenAI
import logging
import time

from app.agents.base import BaseAgent, AgentContext, AgentResponse
from app.vector_store.client import get_vector_store
from app.rag.embeddings import get_embedding_service
from app.config import settings
from app.agents.mock_llm import get_mock_llm_service

logger = logging.getLogger(__name__)


class BookSearchAgent(BaseAgent):
    """
    Agent for full-book vector search and answer generation

    Skills:
    1. Embed user query
    2. Search Qdrant for relevant chunks
    3. Build context from top-k results
    4. Call OpenAI to generate answer with citations
    """

    def __init__(self):
        super().__init__("BookSearchAgent")
        self.vector_store = get_vector_store()
        self.embedding_service = get_embedding_service()
        self.openai_client = OpenAI(api_key=settings.openai_api_key)

    async def process(self, context: AgentContext) -> AgentResponse:
        """
        Process full-book search query

        Args:
            context: AgentContext with query and filters

        Returns:
            AgentResponse with answer and sources
        """
        self._validate_context(context)
        start_time = time.time()

        try:
            # Step 1: Embed query
            logger.info(f"[{self.name}] Embedding query: {context.query[:50]}...")
            query_embedding = self.embedding_service.embed_text(context.query)

            # Step 2: Search vector store
            logger.info(f"[{self.name}] Searching vector store (top_k={settings.top_k_results})")
            search_results = self.vector_store.search(
                query_vector=query_embedding,
                top_k=settings.top_k_results,
                filters=context.filters,
                score_threshold=0.5,
            )

            if not search_results:
                logger.warning(f"[{self.name}] No results found for query")
                return AgentResponse(
                    answer="I couldn't find any relevant information about that in the textbook. Please try rephrasing your question or ask about a different topic.",
                    sources=[],
                    confidence=0.0,
                    agent_name=self.name,
                )

            # Step 3: Build context from results
            context_texts = []
            sources = []

            for idx, result in enumerate(search_results):
                payload = result["payload"]
                context_texts.append(
                    f"[Source {idx+1}] {payload.get('text', '')}"
                )
                sources.append(
                    {
                        "text": payload.get("text", "")[:200] + "...",
                        "url": payload.get("url", ""),
                        "module": payload.get("module", ""),
                        "chapter": payload.get("chapter", ""),
                        "heading": payload.get("heading", ""),
                        "score": result["score"],
                    }
                )

            # Step 4: Generate answer with OpenAI (or mock in demo mode)
            if settings.demo_mode:
                logger.info(f"[{self.name}] 🎭 DEMO MODE: Generating mock answer")
                mock_llm = get_mock_llm_service()
                answer = mock_llm.generate_answer(context.query, context_texts, is_selected_text=False)
            else:
                logger.info(f"[{self.name}] Generating answer with OpenAI")
                answer = self._generate_answer(context.query, context_texts)

            # Calculate confidence based on top result score
            confidence = search_results[0]["score"] if search_results else 0.0

            elapsed = time.time() - start_time
            logger.info(f"[{self.name}] Completed in {elapsed:.2f}s")

            return AgentResponse(
                answer=answer,
                sources=sources,
                confidence=confidence,
                agent_name=self.name,
            )

        except Exception as e:
            logger.error(f"[{self.name}] Error processing query: {e}")
            raise

    def _generate_answer(self, query: str, context_texts: List[str]) -> str:
        """
        Generate answer using OpenAI with context

        Args:
            query: User question
            context_texts: List of relevant text chunks

        Returns:
            Generated answer string
        """
        try:
            context_str = "\n\n".join(context_texts)

            system_prompt = """You are a helpful educational assistant for a Physical AI & Humanoid Robotics textbook.

Your task:
1. Answer the user's question using ONLY the provided context from the textbook
2. Be accurate, clear, and educational in tone
3. Reference specific sources when possible using [Source N] notation
4. If the context doesn't contain enough information, acknowledge limitations
5. Do not make up information or use external knowledge

Format:
- Provide a clear, concise answer
- Include [Source N] citations inline where relevant
- Keep answers focused and under 200 words"""

            user_prompt = f"""Context from textbook:
{context_str}

User Question: {query}

Please provide an accurate answer based on the context above."""

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
