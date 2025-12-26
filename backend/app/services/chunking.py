"""Content chunking service following specs/rag/chunking-rules.md."""

import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional
from uuid import uuid4

import tiktoken

from app.core.logging import get_logger

logger = get_logger(__name__)

# Constants from chunking-rules.md
MIN_TOKENS = 100
MAX_TOKENS = 1000
OVERLAP_TOKENS = 50

# Token encoder for text-embedding-3-small
_encoder = None


def get_encoder():
    """Get or create tiktoken encoder."""
    global _encoder
    if _encoder is None:
        _encoder = tiktoken.get_encoding("cl100k_base")
    return _encoder


def count_tokens(text: str) -> int:
    """Count tokens in text using cl100k_base encoding."""
    encoder = get_encoder()
    return len(encoder.encode(text))


@dataclass
class ContentChunk:
    """A semantically complete chunk of content."""

    id: str
    text: str
    module_id: str
    chapter_id: str
    section_heading: str
    chunk_index: int

    @property
    def token_count(self) -> int:
        return count_tokens(self.text)


def extract_frontmatter(content: str) -> tuple[dict, str]:
    """
    Extract YAML frontmatter from markdown content.

    Returns:
        Tuple of (frontmatter_dict, content_without_frontmatter)
    """
    frontmatter = {}
    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            # Parse YAML frontmatter
            fm_lines = parts[1].strip().split("\n")
            for line in fm_lines:
                if ":" in line:
                    key, value = line.split(":", 1)
                    frontmatter[key.strip()] = value.strip().strip('"').strip("'")
            content = parts[2].strip()
    return frontmatter, content


def remove_excluded_content(content: str) -> str:
    """
    Remove content that shouldn't be chunked.

    Excluded:
    - Self-assessment questions sections
    - What's Next sections
    - Prerequisites sections (at start)
    """
    # Remove self-assessment questions
    content = re.sub(
        r"##\s*Self-Assessment.*?(?=##|\Z)",
        "",
        content,
        flags=re.DOTALL | re.IGNORECASE,
    )

    # Remove What's Next sections
    content = re.sub(
        r"##\s*What['\"]?s Next.*?(?=##|\Z)",
        "",
        content,
        flags=re.DOTALL | re.IGNORECASE,
    )

    # Remove navigation elements at end
    content = re.sub(r"---\s*$", "", content)

    return content.strip()


def split_by_heading(content: str, level: str = "##") -> List[tuple[str, str]]:
    """
    Split content by heading level.

    Returns:
        List of (heading, content) tuples.
    """
    pattern = rf"^({level}\s+[^\n]+)"
    parts = re.split(pattern, content, flags=re.MULTILINE)

    sections = []
    current_heading = ""
    current_content = ""

    for part in parts:
        if part.startswith(level):
            if current_heading or current_content.strip():
                sections.append((current_heading, current_content.strip()))
            current_heading = part.strip()
            current_content = ""
        else:
            current_content += part

    if current_heading or current_content.strip():
        sections.append((current_heading, current_content.strip()))

    return sections


def create_heading_prefix(module_name: str, chapter_name: str, section_heading: str) -> str:
    """Create heading prefix for semantic clarity."""
    return f"[Module: {module_name}]\n[Chapter: {chapter_name}]\n[Section: {section_heading}]\n\n"


def add_overlap(chunks: List[ContentChunk], overlap_tokens: int = OVERLAP_TOKENS) -> List[ContentChunk]:
    """
    Add trailing overlap between adjacent chunks.

    The end of chunk N overlaps into the beginning of chunk N+1.
    """
    if len(chunks) <= 1:
        return chunks

    encoder = get_encoder()
    result = []

    for i, chunk in enumerate(chunks):
        if i == 0:
            result.append(chunk)
            continue

        # Get overlap from previous chunk
        prev_tokens = encoder.encode(chunks[i - 1].text)
        overlap_content = ""
        if len(prev_tokens) > overlap_tokens:
            overlap_content = encoder.decode(prev_tokens[-overlap_tokens:])

        # Prepend overlap to current chunk
        new_text = overlap_content + " " + chunk.text if overlap_content else chunk.text
        result.append(
            ContentChunk(
                id=chunk.id,
                text=new_text.strip(),
                module_id=chunk.module_id,
                chapter_id=chunk.chapter_id,
                section_heading=chunk.section_heading,
                chunk_index=chunk.chunk_index,
            )
        )

    return result


def chunk_chapter(
    content: str,
    module_id: str,
    chapter_id: str,
    module_name: Optional[str] = None,
    chapter_name: Optional[str] = None,
) -> List[ContentChunk]:
    """
    Chunk a chapter's content following chunking-rules.md.

    Args:
        content: Raw markdown content.
        module_id: Module slug (e.g., "module-2-ros2").
        chapter_id: Chapter filename without extension.
        module_name: Human-readable module name.
        chapter_name: Human-readable chapter name.

    Returns:
        List of ContentChunk objects.
    """
    if module_name is None:
        module_name = module_id.replace("-", " ").title()
    if chapter_name is None:
        chapter_name = chapter_id.replace("-", " ").title()

    # Extract frontmatter and clean content
    frontmatter, content = extract_frontmatter(content)
    content = remove_excluded_content(content)

    # Override chapter name from frontmatter if available
    if "title" in frontmatter:
        chapter_name = frontmatter["title"]

    chunks = []
    chunk_index = 0

    # Split by H2 headings
    h2_sections = split_by_heading(content, "##")

    for h2_heading, h2_content in h2_sections:
        if not h2_content.strip():
            continue

        section_heading = h2_heading if h2_heading else "Introduction"
        section_text = f"{h2_heading}\n{h2_content}" if h2_heading else h2_content

        # Check if section is too large
        if count_tokens(section_text) > MAX_TOKENS:
            # Split by H3
            h3_sections = split_by_heading(section_text, "###")
            for h3_heading, h3_content in h3_sections:
                if not h3_content.strip():
                    continue

                sub_heading = h3_heading if h3_heading else section_heading
                sub_text = f"{h3_heading}\n{h3_content}" if h3_heading else h3_content

                # Still too large? Split by paragraphs
                if count_tokens(sub_text) > MAX_TOKENS:
                    paragraphs = sub_text.split("\n\n")
                    current_chunk = ""
                    for para in paragraphs:
                        if count_tokens(current_chunk + para) > MAX_TOKENS and current_chunk:
                            # Create chunk
                            prefix = create_heading_prefix(module_name, chapter_name, sub_heading)
                            chunks.append(
                                ContentChunk(
                                    id=str(uuid4()),
                                    text=prefix + current_chunk.strip(),
                                    module_id=module_id,
                                    chapter_id=chapter_id,
                                    section_heading=sub_heading,
                                    chunk_index=chunk_index,
                                )
                            )
                            chunk_index += 1
                            current_chunk = para
                        else:
                            current_chunk += "\n\n" + para

                    if current_chunk.strip():
                        prefix = create_heading_prefix(module_name, chapter_name, sub_heading)
                        chunks.append(
                            ContentChunk(
                                id=str(uuid4()),
                                text=prefix + current_chunk.strip(),
                                module_id=module_id,
                                chapter_id=chapter_id,
                                section_heading=sub_heading,
                                chunk_index=chunk_index,
                            )
                        )
                        chunk_index += 1
                else:
                    prefix = create_heading_prefix(module_name, chapter_name, sub_heading)
                    chunks.append(
                        ContentChunk(
                            id=str(uuid4()),
                            text=prefix + sub_text.strip(),
                            module_id=module_id,
                            chapter_id=chapter_id,
                            section_heading=sub_heading,
                            chunk_index=chunk_index,
                        )
                    )
                    chunk_index += 1
        else:
            prefix = create_heading_prefix(module_name, chapter_name, section_heading)
            chunks.append(
                ContentChunk(
                    id=str(uuid4()),
                    text=prefix + section_text.strip(),
                    module_id=module_id,
                    chapter_id=chapter_id,
                    section_heading=section_heading,
                    chunk_index=chunk_index,
                )
            )
            chunk_index += 1

    # Filter out chunks that are too small
    chunks = [c for c in chunks if c.token_count >= MIN_TOKENS]

    # Add overlap between chunks
    chunks = add_overlap(chunks)

    logger.info(
        "chapter_chunked",
        module_id=module_id,
        chapter_id=chapter_id,
        chunk_count=len(chunks),
    )

    return chunks
