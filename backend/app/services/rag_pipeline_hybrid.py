"""
Hybrid RAG Pipeline
===================

Best of both worlds:
- Groq for LLM (100% FREE, fast!)
- OpenAI for embeddings (almost free - $0.0001/1K tokens)

This works NOW while local embeddings install!
"""

import time
from typing import Dict, Any, List
from app.services.embedding_service_hybrid import embedding_service
from app.services.vector_store import vector_store
from app.services.llm_service_free import llm_service


class HybridRAGPipeline:
    """RAG pipeline using Groq (FREE) + OpenAI embeddings (cheap)"""

    def __init__(self):
        self.embedding_service = embedding_service
        self.vector_store = vector_store
        self.llm_service = llm_service

    async def query(
        self,
        question: str,
        max_results: int = 5
    ) -> Dict[str, Any]:
        """
        Execute full RAG pipeline.

        Args:
            question: User's question
            max_results: Maximum number of context chunks to retrieve

        Returns:
            Dictionary with answer, sources, and metadata
        """
        start_time = time.time()

        # Step 1: Generate query embedding (OpenAI - cheap)
        query_embedding = await self.embedding_service.embed_text(question)

        # Step 2: Search for similar chunks (Qdrant)
        search_results = await self.vector_store.search(
            query_vector=query_embedding,
            limit=max_results
        )

        # Step 3: Generate answer using Groq (FREE!)
        if not search_results:
            return {
                "answer": "I couldn't find relevant information to answer your question.",
                "sources": [],
                "response_time_ms": int((time.time() - start_time) * 1000),
                "tokens_used": 0
            }

        llm_response = await self.llm_service.generate_answer(
            question=question,
            context_chunks=search_results
        )

        # Step 4: Format response
        sources = [
            {
                "module": result["payload"].get("module", "unknown"),
                "title": result["payload"].get("title", "Unknown"),
                "url": result["payload"].get("url", ""),
                "score": round(result["score"], 3)
            }
            for result in search_results
        ]

        response_time_ms = int((time.time() - start_time) * 1000)

        return {
            "answer": llm_response["answer"],
            "sources": sources,
            "response_time_ms": response_time_ms,
            "tokens_used": llm_response["tokens_used"]
        }


# Singleton instance
rag_pipeline = HybridRAGPipeline()
