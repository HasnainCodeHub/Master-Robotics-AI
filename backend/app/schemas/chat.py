"""Chat request and response schemas."""

from datetime import datetime
from enum import Enum
from typing import List, Optional
from uuid import UUID

from pydantic import BaseModel, Field, field_validator


class RefusalReason(str, Enum):
    """Reason for refusing to answer."""

    EMPTY_RETRIEVAL = "empty_retrieval"
    INSUFFICIENT_CONTEXT = "insufficient_context"
    OUT_OF_SCOPE = "out_of_scope"
    SELECTED_TEXT_INSUFFICIENT = "selected_text_insufficient"


class ChatMode(str, Enum):
    """Mode of chat interaction."""

    GENERAL = "general"
    EXPLAIN_SELECTED_TEXT = "explain_selected_text"
    TRANSLATE_URDU = "translate_urdu"


class ChatRequest(BaseModel):
    """Request body for POST /api/chat."""

    question: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The user's question",
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=5000,
        description="Optional selected text to limit answer context",
    )
    session_id: Optional[UUID] = Field(
        None,
        description="Optional session ID for continuity",
    )
    mode: ChatMode = Field(
        ChatMode.GENERAL,
        description="Chat mode for contextual actions",
    )

    @field_validator("question")
    @classmethod
    def strip_question(cls, v: str) -> str:
        """Strip whitespace from question."""
        return v.strip()

    @field_validator("selected_text")
    @classmethod
    def strip_selected_text(cls, v: Optional[str]) -> Optional[str]:
        """Strip whitespace and convert empty string to None."""
        if v is None:
            return None
        v = v.strip()
        return v if v else None


class SourceReference(BaseModel):
    """Reference to a source chunk used in the answer."""

    chunk_id: str
    module_id: str
    chapter_id: str
    section_heading: str
    score: float = Field(..., ge=0, le=1)


class ChatResponse(BaseModel):
    """Response body for POST /api/chat."""

    answer: str
    sources: List[SourceReference]
    was_refusal: bool
    refusal_reason: Optional[RefusalReason] = None
    session_id: UUID

    class Config:
        use_enum_values = True


# ============================================
# Session Management Schemas
# ============================================


class MessageResponse(BaseModel):
    """A single message in a chat session."""

    id: UUID
    role: str
    content: str
    sources: Optional[List[SourceReference]] = None
    was_refusal: bool = False
    refusal_reason: Optional[str] = None
    created_at: datetime

    class Config:
        from_attributes = True


class SessionSummary(BaseModel):
    """Summary of a chat session for listing."""

    id: UUID
    started_at: datetime
    message_count: int
    preview: Optional[str] = None  # First user message preview

    class Config:
        from_attributes = True


class SessionDetail(BaseModel):
    """Full session detail with messages."""

    id: UUID
    started_at: datetime
    messages: List[MessageResponse]

    class Config:
        from_attributes = True


class SessionListResponse(BaseModel):
    """Response for listing sessions."""

    sessions: List[SessionSummary]
    total: int


class CreateSessionResponse(BaseModel):
    """Response for creating a new session."""

    session_id: UUID
    created_at: datetime


# ============================================
# Search Schemas
# ============================================


class SearchRequest(BaseModel):
    """Request body for POST /api/search."""

    query: str = Field(
        ...,
        min_length=2,
        max_length=200,
        description="Search query text",
    )
    limit: int = Field(
        10,
        ge=1,
        le=20,
        description="Maximum number of results to return",
    )

    @field_validator("query")
    @classmethod
    def strip_query(cls, v: str) -> str:
        """Strip whitespace from query."""
        return v.strip()


class SearchResult(BaseModel):
    """A single search result from the textbook."""

    chunk_id: str
    text: str = Field(..., description="Content snippet")
    module_id: str
    chapter_id: str
    section_heading: str
    score: float = Field(..., ge=0, le=1)
    highlight: Optional[str] = Field(None, description="Highlighted snippet matching query")


class SearchResponse(BaseModel):
    """Response body for POST /api/search."""

    query: str
    results: List[SearchResult]
    total: int
