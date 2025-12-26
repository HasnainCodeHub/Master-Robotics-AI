"""
Search service for hybrid semantic + keyword search.

This service provides search-only functionality (no answer generation).
Results are ranked by semantic similarity from Qdrant.
"""

import re
from typing import List, Optional

from app.core.logging import get_logger
from app.db.qdrant import search_chunks
from app.services.embedding import embed_text
from app.schemas.chat import SearchResult

logger = get_logger(__name__)


def highlight_query_terms(text: str, query: str, max_length: int = 200) -> str:
    """
    Create a highlighted snippet showing query terms in context.

    Args:
        text: Full chunk text.
        query: Search query.
        max_length: Maximum length of snippet.

    Returns:
        Highlighted snippet with query terms emphasized.
    """
    # Get query terms (words with 3+ chars)
    query_terms = [term.lower() for term in query.split() if len(term) >= 3]

    if not query_terms:
        return text[:max_length] + ("..." if len(text) > max_length else "")

    # Find first occurrence of any query term
    text_lower = text.lower()
    first_match_pos = len(text)

    for term in query_terms:
        pos = text_lower.find(term)
        if pos != -1 and pos < first_match_pos:
            first_match_pos = pos

    # Extract snippet around match
    if first_match_pos < len(text):
        start = max(0, first_match_pos - 50)
        end = min(len(text), start + max_length)
        snippet = text[start:end]

        # Add ellipsis if truncated
        if start > 0:
            snippet = "..." + snippet
        if end < len(text):
            snippet = snippet + "..."
    else:
        # No match found, return beginning
        snippet = text[:max_length]
        if len(text) > max_length:
            snippet += "..."

    return snippet


async def search_textbook(
    query: str,
    limit: int = 10,
    min_score: float = 0.3,
) -> List[SearchResult]:
    """
    Search the textbook using semantic search.

    This function:
    1. Embeds the query using the same model as ingestion
    2. Searches Qdrant for similar chunks
    3. Filters by minimum score threshold
    4. Returns ranked results with highlights

    Args:
        query: Search query text.
        limit: Maximum number of results.
        min_score: Minimum similarity score (0-1).

    Returns:
        List of SearchResult objects.
    """
    logger.info("search_textbook", query=query[:100], limit=limit)

    # Embed the query
    query_vector = embed_text(query)

    # Search Qdrant
    chunks = search_chunks(
        query_vector=query_vector,
        limit=limit * 2,  # Fetch extra to allow filtering
    )

    # Filter by minimum score and build results
    results = []
    for chunk in chunks:
        if chunk["score"] < min_score:
            continue

        if len(results) >= limit:
            break

        # Create highlighted snippet
        highlight = highlight_query_terms(chunk["text"], query)

        results.append(SearchResult(
            chunk_id=chunk["id"],
            text=chunk["text"][:500],  # Truncate for response
            module_id=chunk["module_id"],
            chapter_id=chunk["chapter_id"],
            section_heading=chunk["section_heading"],
            score=chunk["score"],
            highlight=highlight,
        ))

    logger.info(
        "search_results",
        query=query[:50],
        total_chunks=len(chunks),
        filtered_results=len(results),
    )

    return results
