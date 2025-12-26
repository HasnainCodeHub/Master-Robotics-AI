"""
Translation API endpoint for on-demand Urdu translation.

CRITICAL ISOLATION:
- This endpoint MUST NOT import from app.services.rag
- This endpoint MUST NOT import from app.services.generation
- This endpoint MUST NOT import from app.db.qdrant
- Translation is PRESENTATION ONLY

This endpoint translates visible chapter text on user request.
It does NOT affect RAG retrieval, embeddings, or answer generation.
"""

from fastapi import APIRouter, HTTPException, status

from app.core.logging import get_logger
from app.schemas.translate import (
    TranslateRequest,
    TranslateResponse,
    TranslateError,
)
from app.services.translation import translate_text

logger = get_logger(__name__)

router = APIRouter()


@router.post(
    "/translate",
    response_model=TranslateResponse,
    responses={
        400: {"model": TranslateError, "description": "Invalid request"},
        500: {"model": TranslateError, "description": "Translation failed"},
    },
)
async def translate(request: TranslateRequest) -> TranslateResponse:
    """
    Translate text to Urdu (on-demand, presentation only).

    This endpoint:
    - Accepts visible chapter text
    - Returns translated text with technical terms preserved
    - Does NOT interact with RAG, vector store, or answer generation
    - Is stateless (no caching on server)

    Technical terms from the curriculum (ROS 2, Gazebo, Isaac, VLA, etc.)
    are automatically preserved in English.
    """
    logger.info(
        "translate_request",
        text_length=len(request.text),
        target_language=request.target_language.value,
        custom_preserve_terms=len(request.preserve_terms) if request.preserve_terms else 0,
    )

    try:
        result = await translate_text(
            text=request.text,
            target_language=request.target_language.value,
            preserve_terms=request.preserve_terms,
        )

        logger.info(
            "translate_success",
            original_length=len(result["original_text"]),
            translated_length=len(result["translated_text"]),
            preserved_terms_count=len(result["preserved_terms"]),
        )

        return TranslateResponse(**result)

    except ValueError as e:
        logger.warning("translate_validation_error", error=str(e))
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": str(e),
                "code": "VALIDATION_ERROR",
                "fallback_text": request.text,
            },
        )

    except Exception as e:
        logger.error("translate_error", error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Translation service temporarily unavailable",
                "code": "TRANSLATION_FAILED",
                "fallback_text": request.text,
            },
        )
