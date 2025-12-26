"""Content chunk schemas for debug endpoint."""

from typing import List

from pydantic import BaseModel


class ContentChunkResponse(BaseModel):
    """Single content chunk response."""

    id: str
    text: str  # Preview (first 200 chars)
    module_id: str
    chapter_id: str
    section_heading: str
    chunk_index: int


class ContentChunksResponse(BaseModel):
    """Response for GET /api/content/chunks."""

    chunks: List[ContentChunkResponse]
    total: int
