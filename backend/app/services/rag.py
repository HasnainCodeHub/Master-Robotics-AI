"""
RAG (Retrieval-Augmented Generation) service.

This is the core service implementing the RAG pipeline with strict grounding
and refusal logic per specs/rag/retrieval-policy.md and specs/rag/refusal-rules.md.

NON-NEGOTIABLE RULES:
1. All answers MUST be grounded in retrieved textbook content
2. No external knowledge is permitted
3. Refusal is ALWAYS preferred over speculation
"""

import re
from typing import List, Optional
from uuid import UUID

from app.core.config import get_settings
from app.core.logging import get_logger
from app.db.qdrant import search_chunks
from app.schemas.chat import ChatResponse, SourceReference, RefusalReason
from app.services.embedding import embed_text
from app.services.generation import generate_grounded_answer

logger = get_logger(__name__)

# Out-of-scope patterns from refusal-rules.md
OUT_OF_SCOPE_PATTERNS = [
    # Control theory
    r"\b(PID tuning|optimal control|MPC|model predictive)\b",
    r"\b(Kalman filter|LQR|sliding mode)\b",
    # Low-level motor control
    r"\b(PWM|pulse width|current control|motor driver)\b",
    r"\b(H-bridge|MOSFET|brushless DC)\b",
    # Hardware design
    r"\b(PCB|printed circuit|mechanical CAD|SolidWorks)\b",
    r"\b(fabrication|3D print|machining)\b",
    # Training models
    r"\b(train|training|fine-tune|finetuning)\s+(a\s+)?(model|LLM|neural)\b",
    r"\b(backpropagation|gradient descent|loss function)\b",
    # ROS 1
    r"\bROS\s*1\b",
    r"\b(catkin|rosbuild|roslaunch.*\.launch)\b",
    # Other simulators
    r"\b(Webots|CoppeliaSim|V-REP|PyBullet)\b",
]

# Refusal messages
REFUSAL_MESSAGES = {
    RefusalReason.EMPTY_RETRIEVAL: (
        "I can only answer questions from the textbook content. "
        "This topic doesn't appear to be covered in the course materials."
    ),
    RefusalReason.INSUFFICIENT_CONTEXT: (
        "I found some related content, but not enough to provide a confident answer. "
        "Please try rephrasing your question or selecting specific text from the chapter."
    ),
    RefusalReason.OUT_OF_SCOPE: (
        "This topic is outside the scope of this course. "
        "The Physical AI & Humanoid Robotics textbook covers: "
        "Physical AI Foundations, ROS 2 Fundamentals, Simulation & Digital Twin, "
        "NVIDIA Isaac Ecosystem, Vision-Language-Action Systems, and Integrated Humanoid Capstone. "
        "For {topic}, please refer to specialized resources."
    ),
    RefusalReason.SELECTED_TEXT_INSUFFICIENT: (
        "The selected text doesn't contain enough information to answer your question. "
        "Please select a different section or ask without selected text to search the full textbook."
    ),
}


def is_out_of_scope(question: str) -> tuple[bool, Optional[str]]:
    """
    Check if the question is about a topic outside the curriculum scope.

    Returns:
        Tuple of (is_out_of_scope, detected_topic).
    """
    question_lower = question.lower()

    for pattern in OUT_OF_SCOPE_PATTERNS:
        match = re.search(pattern, question, re.IGNORECASE)
        if match:
            detected_topic = match.group(0)
            logger.info(
                "out_of_scope_detected",
                question=question[:100],
                topic=detected_topic,
            )
            return True, detected_topic

    return False, None


def is_selected_text_sufficient(selected_text: str, question: str) -> bool:
    """
    Check if selected text is sufficient to answer the question.

    This is a simple heuristic - in production you might use
    semantic similarity or other methods.
    """
    # Very short text is likely insufficient
    if len(selected_text.strip()) < 50:
        return False

    # Check for some overlap between question terms and selected text
    question_words = set(question.lower().split())
    text_words = set(selected_text.lower().split())

    # Remove common words
    common_words = {"what", "is", "the", "a", "an", "how", "do", "does", "can", "this", "that", "explain", "me", "to"}
    question_words -= common_words
    text_words -= common_words

    # At least some overlap
    overlap = question_words & text_words
    return len(overlap) >= 1


def create_refusal_response(
    reason: RefusalReason,
    session_id: Optional[UUID] = None,
    topic: Optional[str] = None,
) -> ChatResponse:
    """
    Create a refusal response with the appropriate message.
    """
    message = REFUSAL_MESSAGES[reason]
    if topic and "{topic}" in message:
        message = message.format(topic=topic)

    logger.info("refusal_response_created", reason=reason.value)

    return ChatResponse(
        answer=message,
        sources=[],
        was_refusal=True,
        refusal_reason=reason,
        session_id=session_id,
    )


async def process_question(
    question: str,
    selected_text: Optional[str] = None,
    session_id: Optional[UUID] = None,
) -> ChatResponse:
    """
    Process a question through the RAG pipeline.

    This implements the refusal flow from specs/rag/refusal-rules.md:
    1. If selected_text provided, use only that context
    2. If selected_text insufficient, REFUSE
    3. Otherwise, retrieve from Qdrant
    4. If no chunks retrieved, REFUSE
    5. If chunks below confidence threshold, REFUSE
    6. If question is out-of-scope, REFUSE
    7. Only then, generate grounded answer

    Args:
        question: User's question.
        selected_text: Optional user-selected text.
        session_id: Optional session ID to include in response.

    Returns:
        ChatResponse with answer or refusal.
    """
    settings = get_settings()

    logger.info(
        "processing_question",
        question_length=len(question),
        has_selected_text=selected_text is not None,
    )

    # 1. Handle selected text mode
    if selected_text:
        # 2. Check if selected text is sufficient
        if not is_selected_text_sufficient(selected_text, question):
            return create_refusal_response(RefusalReason.SELECTED_TEXT_INSUFFICIENT, session_id=session_id)

        # Use selected text as context
        context = selected_text
        sources = []  # No chunk references for selected text

        logger.debug("using_selected_text", text_length=len(selected_text))

    else:
        # 3. Check for out-of-scope BEFORE retrieval
        out_of_scope, topic = is_out_of_scope(question)
        if out_of_scope:
            return create_refusal_response(RefusalReason.OUT_OF_SCOPE, session_id=session_id, topic=topic)

        # 4. Retrieve from Qdrant
        question_embedding = embed_text(question)
        chunks = search_chunks(
            query_vector=question_embedding,
            limit=settings.retrieval_limit,
        )

        # 5. Check for empty retrieval
        if not chunks:
            logger.info("empty_retrieval", question=question[:100])
            return create_refusal_response(RefusalReason.EMPTY_RETRIEVAL, session_id=session_id)

        # Log all retrieved chunks for debugging
        logger.debug(
            "chunks_retrieved",
            count=len(chunks),
            scores=[c["score"] for c in chunks],
        )

        # 6. Filter by confidence threshold
        relevant_chunks = [c for c in chunks if c["score"] >= settings.retrieval_threshold]

        if not relevant_chunks:
            # All chunks below threshold
            max_score = max(c["score"] for c in chunks)
            logger.info(
                "insufficient_context",
                question=question[:100],
                max_score=max_score,
                threshold=settings.retrieval_threshold,
            )
            return create_refusal_response(RefusalReason.INSUFFICIENT_CONTEXT, session_id=session_id)

        # Build context from relevant chunks
        context = "\n\n---\n\n".join([c["text"] for c in relevant_chunks])
        sources = [
            SourceReference(
                chunk_id=c["id"],
                module_id=c["module_id"],
                chapter_id=c["chapter_id"],
                section_heading=c["section_heading"],
                score=c["score"],
            )
            for c in relevant_chunks
        ]

        logger.info(
            "context_built",
            chunks_used=len(relevant_chunks),
            context_length=len(context),
        )

    # 7. Generate grounded answer
    answer = await generate_grounded_answer(question, context)

    logger.info(
        "answer_generated",
        answer_length=len(answer),
        sources_count=len(sources),
    )

    return ChatResponse(
        answer=answer,
        sources=sources,
        was_refusal=False,
        refusal_reason=None,
        session_id=session_id,
    )
