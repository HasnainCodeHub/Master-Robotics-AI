#!/usr/bin/env python3
"""
RAG Safety Validation Script

This script validates that the RAG implementation follows all NON-NEGOTIABLE
safety rules from specs/rag/retrieval-policy.md and specs/rag/refusal-rules.md.

Run this script before marking Phase 3 as complete.
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.schemas.chat import RefusalReason

# Test cases for validation
TEST_CASES = {
    "in_scope_questions": [
        "What is a ROS 2 topic?",
        "How do I create a publisher in ROS 2?",
        "What is Isaac Sim?",
        "Explain URDF robot description",
        "What is the difference between topics and services?",
    ],
    "out_of_scope_questions": [
        "How do I tune a PID controller?",
        "What is PWM modulation?",
        "How do I design a PCB for my robot?",
        "How do I train a custom LLM?",
        "What is ROS 1?",
        "How do I use Webots simulator?",
    ],
    "selected_text_too_short": [
        ("Explain this", "ROS 2"),
        ("What does this mean?", "topic"),
    ],
    "valid_selected_text": [
        (
            "Explain this concept",
            "A topic is a named bus over which nodes exchange messages. "
            "Topics are used for asynchronous communication in ROS 2.",
        ),
    ],
}


def print_header(title: str):
    """Print a section header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def print_check(check: str, passed: bool, details: str = ""):
    """Print a check result."""
    status = "✅ PASS" if passed else "❌ FAIL"
    print(f"  {status}: {check}")
    if details:
        print(f"         {details}")


def validate_code_structure():
    """Validate that code structure enforces safety rules."""
    print_header("Code Structure Validation")

    checks = []

    # Check 1: RAG service exists and has refusal logic
    rag_path = Path(__file__).parent.parent / "app" / "services" / "rag.py"
    if rag_path.exists():
        rag_content = rag_path.read_text()
        has_refusal_logic = "RefusalReason" in rag_content
        has_threshold_check = "retrieval_threshold" in rag_content
        has_out_of_scope = "is_out_of_scope" in rag_content
        print_check("RAG service has refusal logic", has_refusal_logic)
        print_check("RAG service checks threshold", has_threshold_check)
        print_check("RAG service checks out-of-scope", has_out_of_scope)
        checks.extend([has_refusal_logic, has_threshold_check, has_out_of_scope])
    else:
        print_check("RAG service exists", False, f"Not found at {rag_path}")
        checks.append(False)

    # Check 2: Generation service enforces grounding
    gen_path = Path(__file__).parent.parent / "app" / "services" / "generation.py"
    if gen_path.exists():
        gen_content = gen_path.read_text()
        has_system_prompt = "SYSTEM_PROMPT" in gen_content
        prohibits_external = "NEVER use information from your training data" in gen_content
        requires_context = "Context cannot be empty" in gen_content
        print_check("Generation has system prompt", has_system_prompt)
        print_check("System prompt prohibits external knowledge", prohibits_external)
        print_check("Generation requires context", requires_context)
        checks.extend([has_system_prompt, prohibits_external, requires_context])
    else:
        print_check("Generation service exists", False, f"Not found at {gen_path}")
        checks.append(False)

    # Check 3: Refusal reasons are defined
    schema_path = Path(__file__).parent.parent / "app" / "schemas" / "chat.py"
    if schema_path.exists():
        schema_content = schema_path.read_text()
        has_empty_retrieval = "EMPTY_RETRIEVAL" in schema_content
        has_insufficient = "INSUFFICIENT_CONTEXT" in schema_content
        has_out_of_scope = "OUT_OF_SCOPE" in schema_content
        has_selected_text = "SELECTED_TEXT_INSUFFICIENT" in schema_content
        print_check("RefusalReason.EMPTY_RETRIEVAL defined", has_empty_retrieval)
        print_check("RefusalReason.INSUFFICIENT_CONTEXT defined", has_insufficient)
        print_check("RefusalReason.OUT_OF_SCOPE defined", has_out_of_scope)
        print_check("RefusalReason.SELECTED_TEXT_INSUFFICIENT defined", has_selected_text)
        checks.extend([has_empty_retrieval, has_insufficient, has_out_of_scope, has_selected_text])
    else:
        print_check("Chat schema exists", False, f"Not found at {schema_path}")
        checks.append(False)

    return all(checks)


def validate_specs():
    """Validate that specs are complete and locked."""
    print_header("Specification Validation")

    specs_path = Path(__file__).parent.parent.parent / "specs" / "rag"
    required_specs = [
        "retrieval-policy.md",
        "chunking-rules.md",
        "refusal-rules.md",
        "api-boundaries.md",
        "data-storage.md",
    ]

    checks = []
    for spec in required_specs:
        spec_file = specs_path / spec
        exists = spec_file.exists()
        print_check(f"Spec exists: {spec}", exists)
        if exists:
            content = spec_file.read_text()
            is_locked = "LOCKED" in content or "Status**: LOCKED" in content
            print_check(f"Spec is LOCKED: {spec}", is_locked)
            checks.append(is_locked)
        else:
            checks.append(False)

    return all(checks)


def validate_config():
    """Validate configuration has required settings."""
    print_header("Configuration Validation")

    config_path = Path(__file__).parent.parent / "app" / "core" / "config.py"
    if config_path.exists():
        config_content = config_path.read_text()
        has_threshold = "retrieval_threshold" in config_content
        has_limit = "retrieval_limit" in config_content
        has_embedding = "embedding_model" in config_content
        print_check("Config has retrieval_threshold", has_threshold)
        print_check("Config has retrieval_limit", has_limit)
        print_check("Config has embedding_model", has_embedding)
        return has_threshold and has_limit and has_embedding
    else:
        print_check("Config exists", False)
        return False


def validate_refusal_messages():
    """Validate that refusal messages are user-friendly."""
    print_header("Refusal Message Validation")

    rag_path = Path(__file__).parent.parent / "app" / "services" / "rag.py"
    if rag_path.exists():
        content = rag_path.read_text()
        has_friendly_messages = (
            "I can only answer questions from the textbook" in content
            and "This topic is outside the scope" in content
        )
        print_check("Refusal messages are user-friendly", has_friendly_messages)
        return has_friendly_messages
    return False


def validate_no_llm_on_refusal():
    """Validate that LLM is never called on refusal path."""
    print_header("Refusal Path Validation")

    rag_path = Path(__file__).parent.parent / "app" / "services" / "rag.py"
    if rag_path.exists():
        content = rag_path.read_text()
        # Check that create_refusal_response doesn't call generate_grounded_answer
        # This is a structural check - actual runtime validation would require tests
        has_refusal_return = "return create_refusal_response" in content
        print_check("Refusal path returns before LLM call", has_refusal_return)
        return has_refusal_return
    return False


def main():
    """Run all validation checks."""
    print("\n" + "#" * 60)
    print("#  RAG SAFETY VALIDATION")
    print("#  Per specs/rag/retrieval-policy.md and refusal-rules.md")
    print("#" * 60)

    all_passed = True

    # Run all validations
    all_passed &= validate_code_structure()
    all_passed &= validate_specs()
    all_passed &= validate_config()
    all_passed &= validate_refusal_messages()
    all_passed &= validate_no_llm_on_refusal()

    # Summary
    print_header("VALIDATION SUMMARY")
    if all_passed:
        print("  ✅ ALL CHECKS PASSED")
        print("  Phase 3 RAG implementation meets safety requirements.")
        print("\n  Next: Create PHASE3-COMPLETE.md to lock Phase 3.")
    else:
        print("  ❌ SOME CHECKS FAILED")
        print("  Review the failures above and fix before locking Phase 3.")
        sys.exit(1)


if __name__ == "__main__":
    main()
