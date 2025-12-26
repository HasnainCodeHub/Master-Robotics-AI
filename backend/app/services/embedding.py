"""
Gemini embedding service via OpenAI-compatible endpoint.

Uses Google's text-embedding-004 model (768 dimensions) for vector embeddings.
This is a FREE tier model with generous quotas.
"""

from typing import List

from openai import OpenAI

from app.core.config import get_settings
from app.core.logging import get_logger

logger = get_logger(__name__)

# Global client instance
_client = None


def get_gemini_embedding_client() -> OpenAI:
    """Get or create Gemini embedding client via OpenAI-compatible endpoint."""
    global _client
    if _client is None:
        settings = get_settings()

        if not settings.google_api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is not set")

        _client = OpenAI(
            api_key=settings.google_api_key.strip(),
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )
        logger.info("gemini_embedding_client_created")
    return _client


def embed_text(text: str) -> List[float]:
    """
    Generate embedding for a single text using Gemini.

    Args:
        text: Text to embed.

    Returns:
        Embedding vector (768 dimensions for text-embedding-004).
    """
    settings = get_settings()
    client = get_gemini_embedding_client()

    response = client.embeddings.create(
        model=settings.embedding_model,  # text-embedding-004
        input=text,
    )

    embedding = response.data[0].embedding
    logger.debug("text_embedded", text_length=len(text), dimension=len(embedding))
    return embedding


def embed_texts(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batch using Gemini.

    Args:
        texts: List of texts to embed.

    Returns:
        List of embedding vectors.
    """
    if not texts:
        return []

    settings = get_settings()
    client = get_gemini_embedding_client()

    # Gemini supports batching, but let's be safe with smaller batches
    batch_size = 100
    all_embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i : i + batch_size]
        response = client.embeddings.create(
            model=settings.embedding_model,
            input=batch,
        )
        batch_embeddings = [item.embedding for item in response.data]
        all_embeddings.extend(batch_embeddings)
        logger.debug(
            "batch_embedded",
            batch_start=i,
            batch_size=len(batch),
            total=len(texts),
        )

    logger.info("texts_embedded", count=len(texts))
    return all_embeddings
