"""
LLM Service
===========

Handles answer generation using OpenAI's chat models.
"""

from typing import List, Dict, Any
from openai import AsyncOpenAI
from app.config import settings


class LLMService:
    """Service for generating answers using LLM"""

    def __init__(self):
        self.client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = settings.OPENAI_CHAT_MODEL
        self.temperature = settings.OPENAI_TEMPERATURE
        self.max_tokens = settings.OPENAI_MAX_TOKENS

    async def generate_answer(
        self,
        question: str,
        context_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Generate an answer based on question and context.

        Args:
            question: User's question
            context_chunks: List of relevant document chunks

        Returns:
            Dictionary with answer and token usage
        """
        # Build context from chunks
        context = "\n\n".join([
            f"[Source: {chunk['payload'].get('title', 'Unknown')}]\n{chunk['payload'].get('text', '')}"
            for chunk in context_chunks
        ])

        # Create system prompt
        system_prompt = """You are a helpful AI assistant for a Physical AI textbook.
Answer questions based ONLY on the provided context. If the context doesn't contain
enough information, say so. Be concise and accurate."""

        # Create user prompt
        user_prompt = f"""Context:
{context}

Question: {question}

Please provide a clear, concise answer based on the context above."""

        # Generate response
        response = await self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=self.temperature,
            max_tokens=self.max_tokens
        )

        return {
            "answer": response.choices[0].message.content,
            "tokens_used": response.usage.total_tokens,
            "model": self.model
        }


# Singleton instance
llm_service = LLMService()
