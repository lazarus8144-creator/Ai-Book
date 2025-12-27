"""
Free LLM Service
=================

Uses Groq API for FREE LLM inference (generous free tier, no credit card!).
"""

from typing import List, Dict, Any
from groq import Groq
from app.config import settings


class FreeLLMService:
    """Service for generating answers using Groq (100% FREE)"""

    def __init__(self):
        self.client = Groq(api_key=settings.GROQ_API_KEY)
        self.model = settings.GROQ_MODEL
        self.temperature = settings.GROQ_TEMPERATURE
        self.max_tokens = settings.GROQ_MAX_TOKENS
        print(f"✅ Groq LLM initialized: {self.model}")

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
        system_prompt = """You are an expert AI teaching assistant for a Physical AI & Humanoid Robotics textbook.

Your role:
- Provide comprehensive, accurate answers based on the textbook content provided
- For summaries: synthesize information from multiple sources into clear, structured overviews
- For specific questions: give detailed explanations with examples
- Use technical terminology correctly
- If information is incomplete, provide what you know and indicate what's missing

Be helpful, thorough, and educational."""

        # Create user prompt
        user_prompt = f"""Context:
{context}

Question: {question}

Please provide a clear, concise answer based on the context above."""

        # Generate response using Groq (FREE!)
        response = self.client.chat.completions.create(
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
llm_service = FreeLLMService()
