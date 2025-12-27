"""
Claude Code Subagent Orchestrator
===================================

Phase 3 Bonus Feature: +50 points

Uses Claude Sonnet 4.5 subagents to enhance RAG answers with:
1. Technical accuracy verification
2. Code examples when appropriate
3. Follow-up question suggestions
4. Answer quality scoring

Usage:
    orchestrator = SubagentOrchestrator()
    enhanced = await orchestrator.enhance_answer(question, rag_answer, context)
"""

import json
import time
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from loguru import logger

try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False
    logger.warning("anthropic package not installed. Subagent features disabled.")

from app.config import settings


@dataclass
class EnhancedAnswer:
    """Enhanced RAG answer with subagent improvements"""
    answer: str
    original_answer: str
    accuracy_score: int
    has_code_example: bool
    follow_up_questions: List[str]
    enhancement_time_ms: int
    enhanced_by_subagent: bool


class SubagentOrchestrator:
    """
    Orchestrates Claude Code subagents for answer enhancement

    Workflow:
    1. RAG pipeline generates initial answer
    2. Subagent reviews answer for accuracy
    3. Subagent adds code examples if topic requires it
    4. Subagent suggests relevant follow-up questions
    5. Return enhanced answer to user
    """

    def __init__(self, enabled: bool = None):
        """
        Initialize subagent orchestrator

        Args:
            enabled: Override settings.ENABLE_SUBAGENT_ENHANCEMENT
        """
        self.enabled = enabled if enabled is not None else getattr(settings, 'ENABLE_SUBAGENT_ENHANCEMENT', False)

        if self.enabled and not ANTHROPIC_AVAILABLE:
            logger.error("Subagent enhancement enabled but anthropic package not installed")
            self.enabled = False

        if self.enabled:
            api_key = getattr(settings, 'ANTHROPIC_API_KEY', None)
            if not api_key:
                logger.error("ANTHROPIC_API_KEY not set. Disabling subagent enhancement.")
                self.enabled = False
            else:
                self.client = anthropic.Anthropic(api_key=api_key)
                logger.info("✅ Claude Subagent Orchestrator initialized")

    async def enhance_answer(
        self,
        question: str,
        rag_answer: str,
        textbook_context: str,
        module: Optional[str] = None
    ) -> EnhancedAnswer:
        """
        Enhance RAG answer using Claude subagent

        Args:
            question: Original user question
            rag_answer: Answer from RAG pipeline
            textbook_context: Retrieved textbook chunks
            module: Module name (e.g., "module-1-ros2") for context

        Returns:
            EnhancedAnswer with improvements
        """
        if not self.enabled:
            # Return original answer without enhancement
            return EnhancedAnswer(
                answer=rag_answer,
                original_answer=rag_answer,
                accuracy_score=75,  # Assume decent quality
                has_code_example=False,
                follow_up_questions=[],
                enhancement_time_ms=0,
                enhanced_by_subagent=False
            )

        start_time = time.time()

        try:
            # Create enhancement prompt
            prompt = self._build_enhancement_prompt(
                question=question,
                rag_answer=rag_answer,
                context=textbook_context,
                module=module
            )

            # Call Claude subagent
            message = self.client.messages.create(
                model="claude-sonnet-4-5-20250929",
                max_tokens=2000,
                temperature=0.2,
                messages=[
                    {
                        "role": "user",
                        "content": prompt
                    }
                ]
            )

            # Parse response
            response_text = message.content[0].text
            enhanced_data = self._parse_enhancement_response(response_text)

            # Calculate enhancement time
            enhancement_time_ms = int((time.time() - start_time) * 1000)

            # Log enhancement
            logger.info(
                f"🤖 Subagent enhanced answer | "
                f"Accuracy: {enhanced_data['accuracy_score']}/100 | "
                f"Time: {enhancement_time_ms}ms"
            )

            return EnhancedAnswer(
                answer=enhanced_data['enhanced_answer'],
                original_answer=rag_answer,
                accuracy_score=enhanced_data['accuracy_score'],
                has_code_example=enhanced_data['has_code_example'],
                follow_up_questions=enhanced_data['follow_up_questions'],
                enhancement_time_ms=enhancement_time_ms,
                enhanced_by_subagent=True
            )

        except Exception as e:
            logger.error(f"Subagent enhancement failed: {e}")

            # Fallback to original answer
            return EnhancedAnswer(
                answer=rag_answer,
                original_answer=rag_answer,
                accuracy_score=70,
                has_code_example=False,
                follow_up_questions=[],
                enhancement_time_ms=0,
                enhanced_by_subagent=False
            )

    def _build_enhancement_prompt(
        self,
        question: str,
        rag_answer: str,
        context: str,
        module: Optional[str]
    ) -> str:
        """Build prompt for Claude subagent"""

        module_context = ""
        if module:
            module_map = {
                "module-1-ros2": "ROS 2 (Robot Operating System)",
                "module-2-digital-twin": "Digital Twin (Gazebo & Unity simulation)",
                "module-3-nvidia-isaac": "NVIDIA Isaac (AI-powered robotics)",
                "module-4-vla": "Vision-Language-Action models"
            }
            module_context = f"\n\nModule Context: This question is about {module_map.get(module, module)}."

        return f"""You are an expert teaching assistant reviewing a textbook answer for technical accuracy and educational value.

STUDENT QUESTION:
{question}

RAG-GENERATED ANSWER:
{rag_answer}

TEXTBOOK CONTEXT (Retrieved Chunks):
{context[:2000]}
{module_context}

YOUR TASKS:
1. **Verify Technical Accuracy** (0-100): Rate the answer's correctness based on the textbook context
2. **Enhance the Answer**:
   - Fix any technical inaccuracies
   - Add clarity and structure if needed
   - If the topic involves coding/commands, add a minimal working code example
   - Keep it concise (2-3 paragraphs max)
   - Maintain educational tone

3. **Suggest Follow-Up Questions**: Provide 2 relevant questions the student might ask next

CRITICAL RULES:
- ONLY use information from the textbook context provided
- If the RAG answer is already excellent, keep it mostly unchanged
- Code examples should be minimal, tested, and directly relevant
- Follow-up questions should deepen understanding of the current topic

RESPONSE FORMAT (JSON):
{{
    "accuracy_score": 85,
    "enhanced_answer": "Enhanced answer here with optional code example",
    "has_code_example": true,
    "follow_up_questions": [
        "How does X relate to Y?",
        "What are the performance implications of Z?"
    ],
    "enhancement_notes": "Brief note on what was improved"
}}

Return ONLY valid JSON, no markdown fences."""

    def _parse_enhancement_response(self, response_text: str) -> Dict[str, Any]:
        """Parse Claude's JSON response"""
        try:
            # Remove markdown code fences if present
            clean_text = response_text.strip()
            if clean_text.startswith('```'):
                clean_text = clean_text.split('```')[1]
                if clean_text.startswith('json'):
                    clean_text = clean_text[4:]

            data = json.loads(clean_text)

            # Validate required fields
            required = ['accuracy_score', 'enhanced_answer', 'follow_up_questions']
            for field in required:
                if field not in data:
                    raise ValueError(f"Missing field: {field}")

            # Ensure types
            return {
                'accuracy_score': int(data['accuracy_score']),
                'enhanced_answer': str(data['enhanced_answer']),
                'has_code_example': bool(data.get('has_code_example', '```' in data['enhanced_answer'])),
                'follow_up_questions': list(data['follow_up_questions'][:3]),  # Max 3
                'enhancement_notes': data.get('enhancement_notes', '')
            }

        except Exception as e:
            logger.error(f"Failed to parse subagent response: {e}")
            logger.debug(f"Raw response: {response_text[:500]}")
            raise


class SubagentMetrics:
    """Track subagent performance metrics"""

    def __init__(self):
        self.total_enhancements = 0
        self.avg_accuracy_score = 0.0
        self.avg_enhancement_time_ms = 0.0
        self.code_examples_added = 0

    def record_enhancement(self, enhanced: EnhancedAnswer):
        """Record metrics from an enhancement"""
        if not enhanced.enhanced_by_subagent:
            return

        self.total_enhancements += 1

        # Update rolling averages
        alpha = 0.1  # Smoothing factor
        self.avg_accuracy_score = (
            alpha * enhanced.accuracy_score +
            (1 - alpha) * self.avg_accuracy_score
        )
        self.avg_enhancement_time_ms = (
            alpha * enhanced.enhancement_time_ms +
            (1 - alpha) * self.avg_enhancement_time_ms
        )

        if enhanced.has_code_example:
            self.code_examples_added += 1

    def get_stats(self) -> Dict[str, Any]:
        """Get current metrics"""
        return {
            "total_enhancements": self.total_enhancements,
            "avg_accuracy_score": round(self.avg_accuracy_score, 1),
            "avg_enhancement_time_ms": round(self.avg_enhancement_time_ms, 0),
            "code_examples_added": self.code_examples_added,
            "code_example_rate": (
                round(self.code_examples_added / self.total_enhancements * 100, 1)
                if self.total_enhancements > 0 else 0.0
            )
        }


# Global metrics instance
subagent_metrics = SubagentMetrics()
