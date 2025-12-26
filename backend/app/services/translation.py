"""
Translation service for on-demand Urdu translation using Gemini.

CRITICAL ISOLATION RULES:
- This service MUST NOT import from app.services.rag
- This service MUST NOT import from app.services.generation
- This service MUST NOT import from app.db.qdrant
- Translation is PRESENTATION ONLY and does not affect RAG behavior

Technical terms are preserved in English per specs/localization/translation-policy.md.
"""

import re
from typing import List, Optional, Set

from openai import AsyncOpenAI

from app.core.config import get_settings
from app.core.logging import get_logger

logger = get_logger(__name__)

# Default terms to preserve in English (never translate)
# From specs/localization/translation-policy.md Section 2.3
DEFAULT_PRESERVE_TERMS: Set[str] = {
    # Robotics & ROS 2 Terms
    "ROS 2", "ROS 2 Humble", "ROS 2 Iron", "ROS",
    "node", "topic", "service", "action", "launch",
    "publisher", "subscriber", "client", "server",
    "message", "srv", "rclpy", "rclcpp",
    "colcon", "ament", "URDF", "SDF", "Xacro",
    "tf2", "nav2", "MoveIt 2", "MoveIt",
    # NVIDIA & Simulation Terms
    "NVIDIA", "CUDA", "cuDNN", "TensorRT",
    "Isaac Sim", "Isaac ROS", "Omniverse",
    "Gazebo", "Gazebo Fortress", "Digital Twin",
    "Jetson", "Jetson Orin", "Jetson Nano",
    # AI/ML Terms
    "VLA", "Vision-Language-Action",
    "LLM", "transformer", "embedding",
    "inference", "model", "weights",
    "OpenAI", "Claude", "GPT", "Gemini",
    "PyTorch", "TensorFlow",
    # Programming Terms
    "Python", "C++", "CMake",
    "pip", "conda", "Docker",
    "git", "GitHub",
    "API", "REST", "JSON",
    "function", "class", "method",
    # Hardware Terms
    "GPU", "CPU", "RAM",
    "sensor", "actuator", "motor",
    "lidar", "camera", "IMU",
    "RGB", "depth", "point cloud",
}


def get_gemini_client() -> AsyncOpenAI:
    """Get Gemini client via OpenAI-compatible endpoint for translation."""
    settings = get_settings()

    if not settings.google_api_key:
        raise ValueError("GOOGLE_API_KEY environment variable is not set")

    return AsyncOpenAI(
        api_key=settings.google_api_key.strip(),
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    )


def build_preserve_pattern(terms: Set[str]) -> re.Pattern:
    """Build regex pattern to match terms to preserve."""
    # Sort by length (longest first) to match longer terms before shorter ones
    sorted_terms = sorted(terms, key=len, reverse=True)
    # Escape special regex characters and join with OR
    escaped_terms = [re.escape(term) for term in sorted_terms]
    pattern = r'\b(' + '|'.join(escaped_terms) + r')\b'
    return re.compile(pattern, re.IGNORECASE)


def protect_terms(text: str, terms: Set[str]) -> tuple[str, dict[str, str]]:
    """
    Replace terms to preserve with placeholders before translation.

    Returns:
        Tuple of (protected_text, placeholder_map)
    """
    pattern = build_preserve_pattern(terms)
    placeholder_map: dict[str, str] = {}
    counter = [0]  # Use list to allow modification in nested function

    def replace_with_placeholder(match: re.Match) -> str:
        term = match.group(0)
        # Use consistent placeholder for same term
        if term not in placeholder_map:
            placeholder = f"__PRESERVE_{counter[0]}__"
            placeholder_map[term] = placeholder
            counter[0] += 1
        return placeholder_map[term]

    protected_text = pattern.sub(replace_with_placeholder, text)
    return protected_text, placeholder_map


def restore_terms(translated_text: str, placeholder_map: dict[str, str]) -> str:
    """Restore original terms from placeholders after translation."""
    result = translated_text
    # Reverse the map: placeholder -> original term
    for original, placeholder in placeholder_map.items():
        result = result.replace(placeholder, original)
    return result


async def translate_to_urdu(
    text: str,
    additional_preserve_terms: Optional[List[str]] = None,
) -> tuple[str, List[str]]:
    """
    Translate English text to Urdu while preserving technical terms.

    Args:
        text: English text to translate.
        additional_preserve_terms: Extra terms to preserve beyond defaults.

    Returns:
        Tuple of (translated_text, list_of_preserved_terms).

    Raises:
        Exception: If translation fails.
    """
    # Combine default and additional terms
    all_terms = DEFAULT_PRESERVE_TERMS.copy()
    if additional_preserve_terms:
        all_terms.update(additional_preserve_terms)

    # Protect terms before translation
    protected_text, placeholder_map = protect_terms(text, all_terms)

    # Get preserved terms that were actually found in text
    preserved_terms = list(placeholder_map.keys())

    logger.info(
        "translation_started",
        text_length=len(text),
        preserved_terms_count=len(preserved_terms),
    )

    # Build translation prompt
    system_prompt = """You are a professional translator specializing in technical and educational content translation from English to Urdu.

CRITICAL RULES:
1. Translate the text accurately to Urdu while preserving the exact meaning
2. DO NOT translate any text that looks like __PRESERVE_N__ (these are placeholders)
3. Keep placeholders exactly as they appear in the input
4. Maintain paragraph structure and formatting
5. Use formal Urdu appropriate for educational/technical content
6. If a sentence is unclear, translate it as literally as possible

Output ONLY the translated text, nothing else."""

    user_prompt = f"""Translate the following English text to Urdu. Keep all __PRESERVE_N__ placeholders unchanged.

Text to translate:
{protected_text}"""

    # Call Gemini for translation via OpenAI-compatible endpoint
    client = get_gemini_client()

    try:
        response = await client.chat.completions.create(
            model="gemini-2.0-flash",  # Free tier Gemini model
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            temperature=0.3,  # Lower temperature for more consistent translations
            max_tokens=len(text) * 3,  # Urdu may be longer than English
        )

        translated_protected = response.choices[0].message.content.strip()

        # Restore original terms
        translated_text = restore_terms(translated_protected, placeholder_map)

        logger.info(
            "translation_completed",
            original_length=len(text),
            translated_length=len(translated_text),
            preserved_terms=preserved_terms,
        )

        return translated_text, preserved_terms

    except Exception as e:
        logger.error(
            "translation_failed",
            error=str(e),
            text_length=len(text),
        )
        raise


async def translate_text(
    text: str,
    target_language: str = "ur",
    preserve_terms: Optional[List[str]] = None,
) -> dict:
    """
    Main translation function.

    Args:
        text: Text to translate.
        target_language: Target language code (only 'ur' supported).
        preserve_terms: Additional terms to preserve in English.

    Returns:
        Dictionary with translation result.
    """
    if target_language != "ur":
        raise ValueError(f"Unsupported target language: {target_language}. Only 'ur' (Urdu) is supported.")

    if not text or not text.strip():
        raise ValueError("Text cannot be empty")

    translated_text, preserved_terms = await translate_to_urdu(
        text=text,
        additional_preserve_terms=preserve_terms,
    )

    return {
        "original_text": text,
        "translated_text": translated_text,
        "target_language": target_language,
        "preserved_terms": preserved_terms,
        "translation_provider": "gemini",
    }
