"""Content debug endpoint."""

from typing import Optional

from fastapi import APIRouter, Query

from app.db.qdrant import get_chunks_by_chapter
from app.schemas.content import ContentChunksResponse, ContentChunkResponse
from app.core.logging import get_logger

logger = get_logger(__name__)

router = APIRouter()


@router.get("/content/chunks", response_model=ContentChunksResponse)
async def get_content_chunks(
    module_id: str = Query(..., description="Module identifier"),
    chapter_id: Optional[str] = Query(None, description="Chapter identifier (optional filter)"),
) -> ContentChunksResponse:
    """
    Debug/admin endpoint to inspect content chunks for a module/chapter.

    Returns chunk previews (first 200 characters) for debugging purposes.
    """
    logger.info(
        "get_content_chunks",
        module_id=module_id,
        chapter_id=chapter_id,
    )

    chunks = get_chunks_by_chapter(module_id, chapter_id)

    return ContentChunksResponse(
        chunks=[
            ContentChunkResponse(
                id=chunk["id"],
                text=chunk["text"],
                module_id=chunk["module_id"],
                chapter_id=chunk["chapter_id"],
                section_heading=chunk["section_heading"],
                chunk_index=chunk["chunk_index"],
            )
            for chunk in chunks
        ],
        total=len(chunks),
    )
