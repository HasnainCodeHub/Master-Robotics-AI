"""
Grounded answer generation service using OpenAI Agents SDK with Gemini.

This service generates answers strictly grounded in the provided context.
The system prompt explicitly prohibits using external knowledge.

NON-NEGOTIABLE RULES (from specs/rag/retrieval-policy.md):
1. Answer ONLY using the provided context
2. NEVER use information from training data
3. NEVER speculate or infer beyond the provided text
4. NEVER say "I think" or "In general" - only cite the textbook
"""

from agents import (
    Agent,
    Runner,
    RunConfig,
    OpenAIChatCompletionsModel,
    AsyncOpenAI,
)

from app.core.config import get_settings
from app.core.logging import get_logger

logger = get_logger(__name__)

# ============================================
# GLOBAL CLIENT (reuse single instance)
# ============================================

_gemini_client = None


def get_gemini_client() -> AsyncOpenAI:
    """Get or create Gemini client via OpenAI-compatible endpoint."""
    global _gemini_client
    if _gemini_client is None:
        settings = get_settings()
        api_key = settings.google_api_key

        if not api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is not set")

        _gemini_client = AsyncOpenAI(
            api_key=api_key.strip(),
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )
        logger.info("gemini_generation_client_created")
    return _gemini_client


# ============================================
# GEMINI MODEL VIA OPENAI AGENTS SDK
# ============================================

_llm_model = None


def get_llm_model() -> OpenAIChatCompletionsModel:
    """Get Gemini model wrapped in OpenAI Agents SDK model."""
    global _llm_model
    if _llm_model is None:
        client = get_gemini_client()
        _llm_model = OpenAIChatCompletionsModel(
            model="gemini-2.5-flash",
            openai_client=client,
        )
        logger.info("gemini_llm_model_created", model="gemini-2.0-flash")
    return _llm_model


# ============================================
# TUTOR AGENT (RAG-STRICT)
# ============================================

# System instructions enforcing grounded answering
AGENT_INSTRUCTIONS = """
You are a teaching assistant for the Physical AI & Humanoid Robotics textbook.

RULES (NON-NEGOTIABLE):
1. Answer ONLY using the provided CONTEXT below
2. If the CONTEXT does not contain the answer, say:
   "Based on the textbook content provided, I cannot find specific information about that."
3. NEVER use information from your training data
4. NEVER speculate or infer beyond the provided text
5. NEVER say "I think", "In general", "Typically", or "Usually"
6. When answering, reference the textbook content directly
7. If you're unsure, err on the side of not answering rather than guessing

SPECIAL GREETING BEHAVIOR:
- If the user's input is a greeting (e.g. "hi", "hello", "hey", "assalam o alaikum"):
  - Respond with a polite greeting
  - Briefly explain that you can help with the Physical AI & Humanoid Robotics textbook
  - Suggest 3–5 topics that ARE covered in the textbook
  - Do NOT go into details unless the user asks

The greeting response MUST:
- Stay general (no deep explanations)
- Mention only high-level topics
- Avoid technical depth
- Avoid adding knowledge not present in the textbook

FORMAT:
- Be concise but complete
- Use technical terms from the textbook
- Quote or paraphrase the textbook directly when possible
- If code is relevant, include it from the context
"""


def get_tutor_agent() -> Agent:
    """Create the RAG-grounded tutor agent."""
    return Agent(
        name="Physical AI Tutor",
        instructions=AGENT_INSTRUCTIONS,
        model=get_llm_model(),
    )


# ============================================
# AGENT RUNNER
# ============================================

async def run_agent(question: str, context: str) -> str:
    """
    Run the tutor agent with the given question and context.

    Args:
        question: User's question.
        context: Retrieved textbook content (MUST be non-empty).

    Returns:
        Grounded answer string from the agent.
    """
    prompt = (
        f"CONTEXT:\n{context}\n\n"
        f"---\n\n"
        f"QUESTION: {question}\n\n"
        f"Remember: Answer ONLY based on the CONTEXT above. Do not use any external knowledge."
    )

    tutor_agent = get_tutor_agent()

    logger.info(
        "running_agent",
        question_length=len(question),
        context_length=len(context),
    )

    try:
        result = await Runner.run(
            starting_agent=tutor_agent,
            input=prompt,
        )

        answer = result.final_output

        logger.info(
            "agent_completed",
            answer_length=len(answer) if answer else 0,
        )

        return answer
    except Exception as e:
        logger.error("agent_run_failed", error=str(e), error_type=type(e).__name__)
        raise


# ============================================
# PUBLIC API (Preserves existing interface)
# ============================================

async def generate_grounded_answer(question: str, context: str) -> str:
    """
    Generate an answer strictly grounded in the provided context.

    Args:
        question: User's question.
        context: Retrieved textbook content (MUST be non-empty).

    Returns:
        Grounded answer string.

    Raises:
        ValueError: If context is empty (should never happen - caller must check).
    """
    if not context or not context.strip():
        raise ValueError("Context cannot be empty - this is a safety violation")

    logger.debug(
        "generating_answer",
        question_length=len(question),
        context_length=len(context),
    )

    answer = await run_agent(question, context)

    logger.debug(
        "answer_generated",
        answer_length=len(answer) if answer else 0,
    )

    return answer
