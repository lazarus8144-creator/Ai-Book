"""
Enhanced RAG Pipeline with Claude Subagent Integration
========================================================

This is the upgraded RAG pipeline that includes optional subagent enhancement.

Usage:
    pipeline = EnhancedRAGPipeline(enable_subagent=True)
    result = await pipeline.query("What is ROS 2?")
"""

import time
from typing import List, Dict, Optional
from loguru import logger

from app.services.embeddings import EmbeddingService
from app.services.vector_store import VectorStore
from app.services.llm_service import LLMService
from app.services.subagent_orchestrator import SubagentOrchestrator, subagent_metrics
from app.models import QueryResponse, Source


class EnhancedRAGPipeline:
    """
    RAG Pipeline with optional Claude Subagent enhancement

    Flow:
    1. Embed user question
    2. Search Qdrant for relevant chunks
    3. Generate answer with GPT-4o-mini
    4. [OPTIONAL] Enhance answer with Claude Subagent
    5. Return response with sources
    """

    def __init__(self, enable_subagent: bool = False):
        """
        Initialize enhanced RAG pipeline

        Args:
            enable_subagent: Enable Claude subagent enhancement (Phase 3 bonus)
        """
        self.embeddings = EmbeddingService()
        self.vector_store = VectorStore()
        self.llm = LLMService()
        self.subagent = SubagentOrchestrator(enabled=enable_subagent)
        self.enable_subagent = enable_subagent

        logger.info(
            f"🚀 Enhanced RAG Pipeline initialized | "
            f"Subagent: {'✅ Enabled' if enable_subagent else '❌ Disabled'}"
        )

    async def query(
        self,
        question: str,
        max_results: int = 5,
        enable_enhancement: Optional[bool] = None
    ) -> QueryResponse:
        """
        Process a RAG query with optional subagent enhancement

        Args:
            question: User's question
            max_results: Number of chunks to retrieve
            enable_enhancement: Override instance setting for this query

        Returns:
            QueryResponse with answer, sources, and metadata
        """
        start_time = time.time()

        # Step 1: Embed question
        logger.debug(f"🔍 Embedding question: {question[:50]}...")
        query_embedding = self.embeddings.get_embedding(question)

        # Step 2: Vector search in Qdrant
        logger.debug(f"🔎 Searching Qdrant for top {max_results} chunks...")
        search_results = self.vector_store.search(
            vector=query_embedding,
            limit=max_results,
            score_threshold=0.7
        )

        # Step 3: Handle no results
        if not search_results:
            logger.warning(f"⚠️  No relevant results found for: {question[:50]}")
            return self._no_results_response(start_time)

        # Step 4: Build context from retrieved chunks
        context = self._build_context(search_results)
        module = self._extract_primary_module(search_results)

        # Step 5: Generate answer with LLM
        logger.debug("🤖 Generating answer with GPT-4o-mini...")
        answer, tokens = await self.llm.generate(question, context)

        # Step 6: Optional subagent enhancement
        use_subagent = enable_enhancement if enable_enhancement is not None else self.enable_subagent

        if use_subagent:
            logger.debug("🎯 Enhancing answer with Claude Subagent...")
            enhanced = await self.subagent.enhance_answer(
                question=question,
                rag_answer=answer,
                textbook_context=context,
                module=module
            )

            # Use enhanced answer
            answer = enhanced.answer
            tokens += 500  # Approximate subagent token usage

            # Record metrics
            subagent_metrics.record_enhancement(enhanced)

            # Add enhancement metadata
            enhancement_metadata = {
                "enhanced_by_subagent": True,
                "accuracy_score": enhanced.accuracy_score,
                "has_code_example": enhanced.has_code_example,
                "follow_up_questions": enhanced.follow_up_questions,
                "enhancement_time_ms": enhanced.enhancement_time_ms
            }
        else:
            enhancement_metadata = {"enhanced_by_subagent": False}

        # Step 7: Format response
        response_time_ms = int((time.time() - start_time) * 1000)

        logger.info(
            f"✅ Query completed | "
            f"Time: {response_time_ms}ms | "
            f"Tokens: {tokens} | "
            f"Sources: {len(search_results)} | "
            f"Enhanced: {use_subagent}"
        )

        return QueryResponse(
            answer=answer,
            sources=self._format_sources(search_results),
            response_time_ms=response_time_ms,
            tokens_used=tokens,
            metadata=enhancement_metadata  # Include enhancement info
        )

    def _build_context(self, results: List[Dict]) -> str:
        """Build context string from search results"""
        context_parts = []
        for i, result in enumerate(results, 1):
            payload = result.payload
            context_parts.append(
                f"--- Source {i}: {payload['module']} - {payload['title']} ---\n"
                f"{payload['text']}\n"
            )
        return "\n".join(context_parts)

    def _format_sources(self, results: List[Dict]) -> List[Source]:
        """Convert search results to Source objects"""
        sources = []
        for result in results:
            p = result.payload
            sources.append(Source(
                module=p['module'],
                chapter=p['chapter'],
                title=p['title'],
                excerpt=p['text'][:200] + "..." if len(p['text']) > 200 else p['text'],
                score=result.score,
                url=p['url']
            ))
        return sources

    def _extract_primary_module(self, results: List[Dict]) -> Optional[str]:
        """Extract the most common module from results"""
        if not results:
            return None

        modules = [r.payload.get('module') for r in results]
        return max(set(modules), key=modules.count)

    def _no_results_response(self, start_time: float) -> QueryResponse:
        """Return response when no relevant chunks found"""
        return QueryResponse(
            answer=(
                "I couldn't find relevant information in the textbook to answer this question. "
                "Please try rephrasing your question or ask about topics covered in the "
                "Physical AI & Humanoid Robotics modules:\n\n"
                "• Module 1: ROS 2 (Robot Operating System)\n"
                "• Module 2: Digital Twin (Gazebo & Unity)\n"
                "• Module 3: NVIDIA Isaac (AI-Robot Brain)\n"
                "• Module 4: Vision-Language-Action Models"
            ),
            sources=[],
            response_time_ms=int((time.time() - start_time) * 1000),
            tokens_used=0,
            metadata={"enhanced_by_subagent": False}
        )

    def get_subagent_stats(self) -> Dict:
        """Get subagent performance metrics"""
        return subagent_metrics.get_stats()
