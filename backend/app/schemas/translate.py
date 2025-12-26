"""Translation request/response schemas."""

from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, Field


class TargetLanguage(str, Enum):
    """Supported translation target languages."""

    URDU = "ur"


class TranslateRequest(BaseModel):
    """Translation request schema."""

    text: str = Field(
        ...,
        min_length=1,
        max_length=50000,
        description="Text to translate (max 50,000 characters)",
    )
    target_language: TargetLanguage = Field(
        default=TargetLanguage.URDU,
        description="Target language code",
    )
    preserve_terms: Optional[List[str]] = Field(
        default=None,
        description="Additional terms to preserve in English (besides default list)",
    )


class TranslateResponse(BaseModel):
    """Translation response schema."""

    original_text: str = Field(
        description="Original English text",
    )
    translated_text: str = Field(
        description="Translated text in target language",
    )
    target_language: TargetLanguage = Field(
        description="Target language code",
    )
    preserved_terms: List[str] = Field(
        description="Terms that were preserved in English",
    )
    translation_provider: str = Field(
        default="openai",
        description="Translation service provider used",
    )


class TranslateError(BaseModel):
    """Translation error response."""

    error: str = Field(description="Error message")
    code: str = Field(description="Error code")
    fallback_text: Optional[str] = Field(
        default=None,
        description="Original text to display as fallback",
    )
