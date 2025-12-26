"""Chat session management service."""

from typing import Optional, Any, Dict, List
from uuid import UUID

from sqlalchemy import select, func, delete
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from app.models.chat import ChatSession, ChatMessage, MessageRole
from app.core.logging import get_logger

logger = get_logger(__name__)


async def create_or_get_session(
    db: AsyncSession,
    session_id: Optional[UUID] = None,
    user_id: Optional[UUID] = None,
) -> ChatSession:
    """
    Get an existing session or create a new one.

    Args:
        db: Database session.
        session_id: Optional existing session ID.
        user_id: Optional user ID to associate with the session.

    Returns:
        ChatSession instance.
    """
    if session_id:
        # Try to get existing session
        result = await db.execute(
            select(ChatSession).where(ChatSession.id == session_id)
        )
        session = result.scalar_one_or_none()
        if session:
            # Verify session belongs to user if user_id provided
            if user_id and session.user_id and session.user_id != user_id:
                logger.warning(
                    "session_user_mismatch",
                    session_id=str(session_id),
                    session_user_id=str(session.user_id),
                    request_user_id=str(user_id),
                )
                # Create new session for this user instead
            else:
                logger.debug("session_found", session_id=str(session_id))
                return session
        logger.warning("session_not_found", session_id=str(session_id))

    # Create new session for user
    session = ChatSession(user_id=user_id)
    db.add(session)
    await db.flush()
    logger.info("session_created", session_id=str(session.id), user_id=str(user_id) if user_id else None)
    return session


async def save_message(
    db: AsyncSession,
    session_id: UUID,
    role: MessageRole,
    content: str,
    sources: Optional[Dict[str, Any]] = None,
    was_refusal: bool = False,
    refusal_reason: Optional[str] = None,
) -> ChatMessage:
    """
    Save a chat message to the database.

    Args:
        db: Database session.
        session_id: Session ID.
        role: Message role (user or assistant).
        content: Message content.
        sources: Optional source references (for assistant messages).
        was_refusal: Whether this was a refusal response.
        refusal_reason: Reason for refusal if applicable.

    Returns:
        ChatMessage instance.
    """
    message = ChatMessage(
        session_id=session_id,
        role=role,
        content=content,
        sources=sources,
        was_refusal=was_refusal,
        refusal_reason=refusal_reason,
    )
    db.add(message)
    await db.flush()

    logger.debug(
        "message_saved",
        message_id=str(message.id),
        session_id=str(session_id),
        role=role.value,
        was_refusal=was_refusal,
    )

    return message


async def list_user_sessions(
    db: AsyncSession,
    user_id: UUID,
    limit: int = 50,
    offset: int = 0,
) -> tuple[List[ChatSession], int]:
    """
    List all chat sessions for a user with message count.

    Args:
        db: Database session.
        user_id: User ID.
        limit: Maximum sessions to return.
        offset: Pagination offset.

    Returns:
        Tuple of (sessions list, total count).
    """
    # Get total count
    count_query = select(func.count()).select_from(ChatSession).where(
        ChatSession.user_id == user_id
    )
    total_result = await db.execute(count_query)
    total = total_result.scalar() or 0

    # Get sessions with first message for preview
    query = (
        select(ChatSession)
        .where(ChatSession.user_id == user_id)
        .options(selectinload(ChatSession.messages))
        .order_by(ChatSession.started_at.desc())
        .limit(limit)
        .offset(offset)
    )
    result = await db.execute(query)
    sessions = list(result.scalars().all())

    logger.debug(
        "sessions_listed",
        user_id=str(user_id),
        count=len(sessions),
        total=total,
    )

    return sessions, total


async def get_session_with_messages(
    db: AsyncSession,
    session_id: UUID,
    user_id: UUID,
) -> Optional[ChatSession]:
    """
    Get a session with all its messages.

    Args:
        db: Database session.
        session_id: Session ID.
        user_id: User ID (for authorization check).

    Returns:
        ChatSession with messages or None if not found/unauthorized.
    """
    query = (
        select(ChatSession)
        .where(ChatSession.id == session_id)
        .options(selectinload(ChatSession.messages))
    )
    result = await db.execute(query)
    session = result.scalar_one_or_none()

    if not session:
        logger.warning("session_not_found", session_id=str(session_id))
        return None

    # Authorization check
    if session.user_id != user_id:
        logger.warning(
            "session_access_denied",
            session_id=str(session_id),
            session_user_id=str(session.user_id),
            request_user_id=str(user_id),
        )
        return None

    # Sort messages by created_at
    session.messages = sorted(session.messages, key=lambda m: m.created_at)

    logger.debug(
        "session_fetched",
        session_id=str(session_id),
        message_count=len(session.messages),
    )

    return session


async def create_new_session(
    db: AsyncSession,
    user_id: UUID,
) -> ChatSession:
    """
    Create a new chat session for a user.

    Args:
        db: Database session.
        user_id: User ID.

    Returns:
        New ChatSession instance.
    """
    session = ChatSession(user_id=user_id)
    db.add(session)
    await db.flush()

    logger.info(
        "session_created_explicitly",
        session_id=str(session.id),
        user_id=str(user_id),
    )

    return session


async def delete_session(
    db: AsyncSession,
    session_id: UUID,
    user_id: UUID,
) -> bool:
    """
    Delete a chat session and all its messages.

    Args:
        db: Database session.
        session_id: Session ID.
        user_id: User ID (for authorization check).

    Returns:
        True if deleted, False if not found or unauthorized.
    """
    # First verify ownership
    query = select(ChatSession).where(ChatSession.id == session_id)
    result = await db.execute(query)
    session = result.scalar_one_or_none()

    if not session:
        logger.warning("delete_session_not_found", session_id=str(session_id))
        return False

    if session.user_id != user_id:
        logger.warning(
            "delete_session_access_denied",
            session_id=str(session_id),
            session_user_id=str(session.user_id),
            request_user_id=str(user_id),
        )
        return False

    # Delete session (cascades to messages)
    await db.delete(session)
    await db.flush()

    logger.info(
        "session_deleted",
        session_id=str(session_id),
        user_id=str(user_id),
    )

    return True


async def get_last_active_session(
    db: AsyncSession,
    user_id: UUID,
) -> Optional[ChatSession]:
    """
    Get the user's most recently active session.

    Args:
        db: Database session.
        user_id: User ID.

    Returns:
        Most recent ChatSession or None.
    """
    query = (
        select(ChatSession)
        .where(ChatSession.user_id == user_id)
        .options(selectinload(ChatSession.messages))
        .order_by(ChatSession.started_at.desc())
        .limit(1)
    )
    result = await db.execute(query)
    session = result.scalar_one_or_none()

    if session:
        session.messages = sorted(session.messages, key=lambda m: m.created_at)
        logger.debug(
            "last_active_session_found",
            session_id=str(session.id),
            user_id=str(user_id),
        )

    return session
