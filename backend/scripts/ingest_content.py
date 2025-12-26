#!/usr/bin/env python3
"""
Content ingestion script for the Physical AI textbook.

This script processes all markdown files in docs/docs/ and uploads
them to Qdrant as vector embeddings.

Usage:
    python scripts/ingest_content.py [--dry-run] [--module MODULE_ID]
"""

import argparse
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.core.config import get_settings
from app.core.logging import setup_logging, get_logger
from app.db.qdrant import ensure_collection_exists, upsert_chunks
from app.services.chunking import chunk_chapter
from app.services.embedding import embed_texts

setup_logging()
logger = get_logger(__name__)


# Module name mapping for human-readable prefixes
MODULE_NAMES = {
    "module-1": "Physical AI Foundations",
    "module-2": "ROS 2 Fundamentals",
    "module-3": "Simulation & Digital Twin",
    "module-4": "NVIDIA Isaac Ecosystem",
    "module-5": "Vision-Language-Action",
    "capstone": "Integrated Humanoid Capstone",
    "appendix": "Appendix",
}


def discover_content_files(docs_path: Path, module_filter: str = None) -> list[Path]:
    """
    Discover all markdown files to process.

    Args:
        docs_path: Path to docs/docs/ directory.
        module_filter: Optional module ID to filter by.

    Returns:
        List of markdown file paths.
    """
    files = []

    # Find all chapter files (skip index.md)
    for md_file in docs_path.rglob("*.md"):
        # Skip index files
        if md_file.name == "index.md":
            continue

        # Skip glossary and intro (they don't follow chapter structure)
        if md_file.name in ["glossary.md", "intro.md"]:
            continue

        # Get relative path to determine module
        rel_path = md_file.relative_to(docs_path)
        parts = rel_path.parts

        # Filter by module if specified
        if module_filter and len(parts) > 0:
            module_id = parts[0]
            if module_id != module_filter:
                continue

        files.append(md_file)

    return sorted(files)


def extract_module_chapter_ids(file_path: Path, docs_path: Path) -> tuple[str, str]:
    """
    Extract module and chapter IDs from file path.

    Args:
        file_path: Path to markdown file.
        docs_path: Base docs directory.

    Returns:
        Tuple of (module_id, chapter_id).
    """
    rel_path = file_path.relative_to(docs_path)
    parts = rel_path.parts

    if len(parts) >= 2:
        module_id = parts[0]
        chapter_id = parts[1].replace(".md", "")
    else:
        module_id = "root"
        chapter_id = parts[0].replace(".md", "")

    return module_id, chapter_id


def process_file(
    file_path: Path,
    docs_path: Path,
    dry_run: bool = False,
) -> int:
    """
    Process a single markdown file.

    Args:
        file_path: Path to markdown file.
        docs_path: Base docs directory.
        dry_run: If True, don't upload to Qdrant.

    Returns:
        Number of chunks created.
    """
    module_id, chapter_id = extract_module_chapter_ids(file_path, docs_path)

    # Get human-readable module name
    module_name = MODULE_NAMES.get(module_id, module_id.replace("-", " ").title())

    logger.info(
        "processing_file",
        file=str(file_path.name),
        module_id=module_id,
        chapter_id=chapter_id,
    )

    # Read file content
    content = file_path.read_text(encoding="utf-8")

    # Chunk the content
    chunks = chunk_chapter(
        content=content,
        module_id=module_id,
        chapter_id=chapter_id,
        module_name=module_name,
    )

    if not chunks:
        logger.warning("no_chunks_created", file=str(file_path.name))
        return 0

    if dry_run:
        logger.info(
            "dry_run_chunks",
            file=str(file_path.name),
            chunk_count=len(chunks),
            sample_text=chunks[0].text[:100] + "...",
        )
        return len(chunks)

    # Generate embeddings
    texts = [chunk.text for chunk in chunks]
    embeddings = embed_texts(texts)

    # Prepare chunks for upsert
    chunk_data = [
        {
            "id": chunk.id,
            "vector": embedding,
            "text": chunk.text,
            "module_id": chunk.module_id,
            "chapter_id": chunk.chapter_id,
            "section_heading": chunk.section_heading,
            "chunk_index": chunk.chunk_index,
        }
        for chunk, embedding in zip(chunks, embeddings)
    ]

    # Upsert to Qdrant
    upsert_chunks(chunk_data)

    return len(chunks)


def main():
    parser = argparse.ArgumentParser(
        description="Ingest textbook content into Qdrant for RAG retrieval."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Process files but don't upload to Qdrant",
    )
    parser.add_argument(
        "--module",
        type=str,
        help="Only process a specific module (e.g., module-2)",
    )
    parser.add_argument(
        "--docs-path",
        type=str,
        default="../docs/docs",
        help="Path to docs/docs directory (relative to script or absolute)",
    )
    args = parser.parse_args()

    # Resolve docs path
    script_dir = Path(__file__).parent
    docs_path = Path(args.docs_path)
    if not docs_path.is_absolute():
        docs_path = (script_dir / docs_path).resolve()

    if not docs_path.exists():
        logger.error("docs_path_not_found", path=str(docs_path))
        sys.exit(1)

    logger.info(
        "ingestion_started",
        docs_path=str(docs_path),
        module_filter=args.module,
        dry_run=args.dry_run,
    )

    # Ensure Qdrant collection exists
    if not args.dry_run:
        ensure_collection_exists()

    # Discover files
    files = discover_content_files(docs_path, args.module)
    logger.info("files_discovered", count=len(files))

    if not files:
        logger.warning("no_files_found")
        return

    # Process each file
    total_chunks = 0
    for file_path in files:
        try:
            chunks = process_file(file_path, docs_path, args.dry_run)
            total_chunks += chunks
        except Exception as e:
            logger.error(
                "file_processing_failed",
                file=str(file_path.name),
                error=str(e),
            )

    logger.info(
        "ingestion_complete",
        files_processed=len(files),
        total_chunks=total_chunks,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    main()
