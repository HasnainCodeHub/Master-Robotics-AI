"""Chat endpoint for RAG queries."""

from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy.ext.asyncio import AsyncSession

from app.core.auth import get_current_user_required
from app.db.postgres import get_db
from app.models.user import User
from app.schemas.chat import (
    ChatRequest,
    ChatResponse,
    SessionListResponse,
    SessionSummary,
    SessionDetail,
    MessageResponse,
    CreateSessionResponse,
    SourceReference,
    SearchRequest,
    SearchResponse,
)
from app.services.rag import process_question
from app.services.search import search_textbook
from app.services.session import (
    create_or_get_session,
    save_message,
    list_user_sessions,
    get_session_with_messages,
    create_new_session,
    delete_session,
    get_last_active_session,
)
from app.services.generation import run_agent
from app.models.chat import MessageRole
from app.core.logging import get_logger

logger = get_logger(__name__)

router = APIRouter()


def is_greeting(text: str) -> bool:
    """
    Detect if the input is a simple greeting.

    Args:
        text: User input text.

    Returns:
        True if the text is a greeting, False otherwise.
    """
    greetings = {
        "hi", "hello", "hey", "assalam", "assalam o alaikum",
        "salam", "good morning", "good evening", "good afternoon"
    }
    normalized = text.lower().strip("!?., ")
    return normalized in greetings


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> ChatResponse:
    """
    Send a question to the RAG chatbot and receive a grounded answer or refusal.

    - If `selected_text` is provided, the answer is limited to that context only.
    - If retrieval is insufficient, the chatbot will refuse to answer.
    - All answers are grounded ONLY in retrieved textbook content.
    """
    logger.info(
        "chat_request",
        user_id=str(user.id),
        question_length=len(request.question),
        has_selected_text=request.selected_text is not None,
        session_id=str(request.session_id) if request.session_id else None,
    )

    # Get or create session for authenticated user
    session = await create_or_get_session(db, request.session_id, user_id=user.id)

    # Save user message
    await save_message(
        db=db,
        session_id=session.id,
        role=MessageRole.USER,
        content=request.question,
    )

    # Check if this is a greeting - bypass RAG if so
    if is_greeting(request.question):
        logger.info(
            "greeting_detected",
            question=request.question,
            session_id=str(session.id),
        )

        # Run agent directly with minimal context for greeting handling
        # This allows the agent to introduce the textbook scope without deep technical content
        greeting_context = (
            "This is the Physical AI & Humanoid Robotics textbook. "
            "Topics covered include: Physical AI Foundations, ROS 2 Fundamentals, "
            "Simulation & Digital Twin, NVIDIA Isaac Ecosystem, "
            "Vision-Language-Action Systems, and Integrated Humanoid Capstone."
        )
        greeting_response = await run_agent(
            question=request.question,
            context=greeting_context,
        )

        # Save greeting response
        await save_message(
            db=db,
            session_id=session.id,
            role=MessageRole.ASSISTANT,
            content=greeting_response,
            sources=None,
            was_refusal=False,
            refusal_reason=None,
        )

        logger.info(
            "greeting_response",
            session_id=str(session.id),
            answer_length=len(greeting_response),
        )

        return ChatResponse(
            answer=greeting_response,
            sources=[],
            was_refusal=False,
            refusal_reason=None,
            session_id=session.id,
        )

    # Process question through RAG pipeline
    response = await process_question(
        question=request.question,
        selected_text=request.selected_text,
        session_id=session.id,
    )

    # Save assistant response
    await save_message(
        db=db,
        session_id=session.id,
        role=MessageRole.ASSISTANT,
        content=response.answer,
        sources={"chunks": [s.model_dump() for s in response.sources], "used_selected_text": request.selected_text is not None},
        was_refusal=response.was_refusal,
        refusal_reason=response.refusal_reason,
    )

    logger.info(
        "chat_response",
        was_refusal=response.was_refusal,
        refusal_reason=response.refusal_reason,
        sources_count=len(response.sources),
        session_id=str(session.id),
    )

    return response


# ============================================
# Session Management Endpoints
# ============================================


def _convert_sources(sources_data: dict | None) -> list[SourceReference] | None:
    """Convert stored sources JSON to SourceReference list."""
    if not sources_data or "chunks" not in sources_data:
        return None
    try:
        return [SourceReference(**chunk) for chunk in sources_data["chunks"]]
    except Exception:
        return None


@router.get("/sessions", response_model=SessionListResponse)
async def list_sessions(
    limit: int = Query(50, ge=1, le=100),
    offset: int = Query(0, ge=0),
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> SessionListResponse:
    """
    List all chat sessions for the authenticated user.
    Sessions are sorted by date (newest first).
    """
    sessions, total = await list_user_sessions(db, user.id, limit, offset)

    summaries = []
    for session in sessions:
        # Get first user message as preview
        preview = None
        for msg in session.messages:
            if msg.role == MessageRole.USER:
                preview = msg.content[:100] + "..." if len(msg.content) > 100 else msg.content
                break

        summaries.append(SessionSummary(
            id=session.id,
            started_at=session.started_at,
            message_count=len(session.messages),
            preview=preview,
        ))

    return SessionListResponse(sessions=summaries, total=total)


@router.get("/sessions/last", response_model=SessionDetail | None)
async def get_last_session(
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> SessionDetail | None:
    """
    Get the user's most recently active session with full message history.
    Returns null if no sessions exist.
    """
    session = await get_last_active_session(db, user.id)

    if not session:
        return None

    messages = [
        MessageResponse(
            id=msg.id,
            role=msg.role.value,
            content=msg.content,
            sources=_convert_sources(msg.sources),
            was_refusal=msg.was_refusal,
            refusal_reason=msg.refusal_reason,
            created_at=msg.created_at,
        )
        for msg in session.messages
    ]

    return SessionDetail(
        id=session.id,
        started_at=session.started_at,
        messages=messages,
    )


@router.get("/sessions/{session_id}", response_model=SessionDetail)
async def get_session(
    session_id: UUID,
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> SessionDetail:
    """
    Get a specific chat session with full message history.
    """
    session = await get_session_with_messages(db, session_id, user.id)

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    messages = [
        MessageResponse(
            id=msg.id,
            role=msg.role.value,
            content=msg.content,
            sources=_convert_sources(msg.sources),
            was_refusal=msg.was_refusal,
            refusal_reason=msg.refusal_reason,
            created_at=msg.created_at,
        )
        for msg in session.messages
    ]

    return SessionDetail(
        id=session.id,
        started_at=session.started_at,
        messages=messages,
    )


@router.post("/sessions", response_model=CreateSessionResponse)
async def create_session(
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> CreateSessionResponse:
    """
    Create a new chat session explicitly.
    """
    session = await create_new_session(db, user.id)
    await db.commit()

    return CreateSessionResponse(
        session_id=session.id,
        created_at=session.created_at,
    )


@router.delete("/sessions/{session_id}", status_code=204)
async def delete_session_endpoint(
    session_id: UUID,
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> None:
    """
    Delete a chat session and all its messages.
    """
    deleted = await delete_session(db, session_id, user.id)

    if not deleted:
        raise HTTPException(status_code=404, detail="Session not found")

    await db.commit()


# ============================================
# Search Endpoint
# ============================================


@router.post("/search", response_model=SearchResponse)
async def search(
    request: SearchRequest,
    user: User = Depends(get_current_user_required),
) -> SearchResponse:
    """
    Search the textbook content using semantic search.

    This endpoint:
    - Performs hybrid semantic + keyword search
    - Returns ranked results with relevance scores
    - Does NOT generate answers (search only)
    - Requires authentication

    Use the results to navigate to book sections or send to the Tutor.
    """
    logger.info(
        "search_request",
        user_id=str(user.id),
        query=request.query[:50],
        limit=request.limit,
    )

    results = await search_textbook(
        query=request.query,
        limit=request.limit,
    )

    logger.info(
        "search_response",
        user_id=str(user.id),
        results_count=len(results),
    )

    return SearchResponse(
        query=request.query,
        results=results,
        total=len(results),
    )
