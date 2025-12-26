"""Qdrant Cloud connection and operations."""

from typing import List, Optional

from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    Distance,
    PointStruct,
    VectorParams,
    Filter,
    FieldCondition,
    MatchValue,
)

from app.core.config import get_settings
from app.core.logging import get_logger

logger = get_logger(__name__)

# Global client instance
_client: Optional[QdrantClient] = None


def get_qdrant_client() -> QdrantClient:
    """Get or create Qdrant client instance."""
    global _client
    if _client is None:
        settings = get_settings()
        _client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        logger.info("qdrant_client_created", url=settings.qdrant_url)
    return _client


async def check_qdrant_health() -> bool:
    """Check if Qdrant is reachable and healthy."""
    try:
        client = get_qdrant_client()
        collections = client.get_collections()
        logger.debug("qdrant_health_check", collections_count=len(collections.collections))
        return True
    except Exception as e:
        logger.error("qdrant_health_check_failed", error=str(e))
        return False


def ensure_collection_exists() -> None:
    """Create the textbook_chunks collection if it doesn't exist or recreate if dimension changed."""
    settings = get_settings()
    client = get_qdrant_client()

    collections = client.get_collections()
    collection_names = [c.name for c in collections.collections]

    need_create = False

    if settings.qdrant_collection in collection_names:
        # Check if dimension matches
        collection_info = client.get_collection(settings.qdrant_collection)
        current_dim = collection_info.config.params.vectors.size

        if current_dim != settings.embedding_dimension:
            logger.warning(
                "qdrant_dimension_mismatch",
                current=current_dim,
                expected=settings.embedding_dimension,
            )
            # Delete and recreate collection
            client.delete_collection(settings.qdrant_collection)
            logger.info("qdrant_collection_deleted", collection=settings.qdrant_collection)
            need_create = True
        else:
            logger.info("qdrant_collection_exists", collection=settings.qdrant_collection)
    else:
        need_create = True

    if need_create:
        client.create_collection(
            collection_name=settings.qdrant_collection,
            vectors_config=VectorParams(
                size=settings.embedding_dimension,
                distance=Distance.COSINE,
            ),
        )
        logger.info(
            "qdrant_collection_created",
            collection=settings.qdrant_collection,
            dimension=settings.embedding_dimension,
        )

        # Create payload indexes for filtering
        client.create_payload_index(
            collection_name=settings.qdrant_collection,
            field_name="module_id",
            field_schema="keyword",
        )
        client.create_payload_index(
            collection_name=settings.qdrant_collection,
            field_name="chapter_id",
            field_schema="keyword",
        )
        logger.info("qdrant_payload_indexes_created")


def upsert_chunks(chunks: List[dict]) -> int:
    """
    Upsert content chunks into Qdrant.

    Args:
        chunks: List of dicts with keys: id, vector, text, module_id, chapter_id, section_heading, chunk_index

    Returns:
        Number of chunks upserted.
    """
    settings = get_settings()
    client = get_qdrant_client()

    points = [
        PointStruct(
            id=chunk["id"],
            vector=chunk["vector"],
            payload={
                "text": chunk["text"],
                "module_id": chunk["module_id"],
                "chapter_id": chunk["chapter_id"],
                "section_heading": chunk["section_heading"],
                "chunk_index": chunk["chunk_index"],
            },
        )
        for chunk in chunks
    ]

    client.upsert(collection_name=settings.qdrant_collection, points=points)
    logger.info("qdrant_chunks_upserted", count=len(points))
    return len(points)


def search_chunks(
    query_vector: List[float],
    limit: int = 5,
    module_id: Optional[str] = None,
    chapter_id: Optional[str] = None,
) -> List[dict]:
    """
    Search for similar chunks in Qdrant.

    Args:
        query_vector: Embedding vector for the query.
        limit: Maximum number of results.
        module_id: Optional filter by module.
        chapter_id: Optional filter by chapter.

    Returns:
        List of matching chunks with scores.
    """
    settings = get_settings()
    client = get_qdrant_client()

    # Build filter if provided
    query_filter = None
    if module_id or chapter_id:
        conditions = []
        if module_id:
            conditions.append(
                FieldCondition(field="module_id", match=MatchValue(value=module_id))
            )
        if chapter_id:
            conditions.append(
                FieldCondition(field="chapter_id", match=MatchValue(value=chapter_id))
            )
        query_filter = Filter(must=conditions)

    results = client.query_points(
        collection_name=settings.qdrant_collection,
        query=query_vector,
        query_filter=query_filter,
        limit=limit,
        with_payload=True,
    )

    return [
        {
            "id": str(point.id),
            "score": point.score,
            "text": point.payload.get("text", ""),
            "module_id": point.payload.get("module_id", ""),
            "chapter_id": point.payload.get("chapter_id", ""),
            "section_heading": point.payload.get("section_heading", ""),
            "chunk_index": point.payload.get("chunk_index", 0),
        }
        for point in results.points
    ]


def get_chunks_by_chapter(
    module_id: str, chapter_id: Optional[str] = None
) -> List[dict]:
    """Get all chunks for a module/chapter (for debug endpoint)."""
    settings = get_settings()
    client = get_qdrant_client()

    conditions = [FieldCondition(field="module_id", match=MatchValue(value=module_id))]
    if chapter_id:
        conditions.append(
            FieldCondition(field="chapter_id", match=MatchValue(value=chapter_id))
        )

    results = client.scroll(
        collection_name=settings.qdrant_collection,
        scroll_filter=Filter(must=conditions),
        limit=100,
        with_payload=True,
        with_vectors=False,
    )[0]

    return [
        {
            "id": str(point.id),
            "text": point.payload.get("text", "")[:200] + "...",  # Preview only
            "module_id": point.payload.get("module_id", ""),
            "chapter_id": point.payload.get("chapter_id", ""),
            "section_heading": point.payload.get("section_heading", ""),
            "chunk_index": point.payload.get("chunk_index", 0),
        }
        for point in results
    ]
