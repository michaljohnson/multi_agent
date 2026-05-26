"""Provider-agnostic LLM client using LiteLLM.

Wraps litellm.completion() with retry logic and response helpers
so agent files don't need to know about provider-specific formats.

Supports any provider LiteLLM supports (Anthropic, OpenAI, Ollama, etc.)
by setting the LLM_MODEL env var and the appropriate API key.
"""

import json
import logging
import os
import re
import time
import uuid

# Silence LiteLLM's import-time warnings for optional AWS adapters we do not
# use (Bedrock and SageMaker need botocore which is not installed). Must be
# registered BEFORE `import litellm` so the warnings emitted on import are
# filtered out.
class _LitellmAwsWarningFilter(logging.Filter):
    def filter(self, record):
        msg = record.getMessage()
        return not (
            "bedrock-runtime" in msg
            or "sagemaker-runtime" in msg
            or "No module named 'botocore'" in msg
        )


logging.getLogger("LiteLLM").addFilter(_LitellmAwsWarningFilter())

import litellm  # noqa: E402

logger = logging.getLogger(__name__)


# Hermes-style tool-call XML emitted by Qwen 2.5/3.x when vLLM is NOT started
# with `--enable-auto-tool-choice --tool-call-parser hermes`. The model embeds
# the call inside `message.content`; `tool_calls` stays empty; `finish_reason`
# is `stop`. The detector re-parses client-side so the agent loop can dispatch
# normally. The parser is a no-op for providers (Anthropic, OpenAI) that emit
# native `tool_calls`, so this keeps the client model-agnostic without breaking
# the frontier-model path. Remove only when every served model emits native
# tool calls.
_HERMES_TOOL_CALL_RE = re.compile(
    r"<tool_call>\s*<function=([^>]+)>(.*?)</function>\s*</tool_call>",
    re.DOTALL,
)
_HERMES_PARAM_RE = re.compile(
    r"<parameter=([^>]+)>(.*?)</parameter>", re.DOTALL
)


def _extract_hermes_tool_calls(response) -> bool:
    """Detect Hermes-XML tool calls in `message.content` and synthesize
    OpenAI-format `tool_calls`. Mutates the response. Returns True if any
    tool calls were extracted; returns False (no-op) for Anthropic / OpenAI
    responses that already carry native tool calls."""
    try:
        choice = response.choices[0]
        msg = choice.message
        existing = getattr(msg, "tool_calls", None) or []
        if existing:
            return False
        content = getattr(msg, "content", None) or ""
        if "<tool_call>" not in content:
            return False
        matches = _HERMES_TOOL_CALL_RE.findall(content)
        if not matches:
            return False
        synthetic = []
        for func_name, body in matches:
            params = {
                k.strip(): v.strip()
                for k, v in _HERMES_PARAM_RE.findall(body)
            }
            synthetic.append(
                litellm.types.utils.ChatCompletionMessageToolCall(
                    id=f"call_{uuid.uuid4().hex[:24]}",
                    type="function",
                    function=litellm.types.utils.Function(
                        name=func_name.strip(),
                        arguments=json.dumps(params),
                    ),
                )
            )
        msg.tool_calls = synthetic
        choice.finish_reason = "tool_calls"
        # Strip parsed XML from content so it does not leak into transcripts.
        msg.content = _HERMES_TOOL_CALL_RE.sub("", content).strip() or None
        return True
    except Exception as e:
        logger.warning(f"  [llm_client] Hermes tool-call parse error: {e}")
        return False


DEFAULT_MODEL = os.environ.get("LLM_MODEL", "anthropic/claude-sonnet-4-20250514")

# Suppress litellm's verbose logging
litellm.suppress_debug_info = True

# Allow tool-call blocks in message history even when `tools=None` on a call.
# Without this, LiteLLM refuses to transform a conversation that already has
# assistant tool_calls + tool results if the current turn has no tool schemas.
# Relevant for replay/inspection turns (e.g. the look plumbing test).
litellm.modify_params = True


def call_llm(messages, tools=None, model=None, max_tokens=4096, max_retries=3):
    """Call an LLM via LiteLLM with retry on rate-limit errors.

    Args:
        messages: OpenAI-format message list (including system message).
        tools: OpenAI-format tool definitions, or None.
        model: LiteLLM model string (e.g. "anthropic/claude-sonnet-4-20250514",
               "gpt-4o", "ollama/llama3"). Defaults to LLM_MODEL env var.
        max_tokens: Maximum tokens in the response.
        max_retries: Number of retries on rate-limit errors.

    Returns:
        LiteLLM/OpenAI-format response object.
    """
    model = model or DEFAULT_MODEL
    for attempt in range(max_retries):
        try:
            kwargs = dict(model=model, messages=messages, max_tokens=max_tokens)
            if tools:
                kwargs["tools"] = tools
            response = litellm.completion(**kwargs)
            # Provider-agnostic post-processing: re-parse Hermes XML tool calls
            # for vLLM-served Qwen endpoints that lack the server-side parser.
            # No-op for Anthropic / OpenAI native tool calls.
            _extract_hermes_tool_calls(response)
            return response
        except litellm.RateLimitError:
            if attempt < max_retries - 1:
                wait = 2 ** (attempt + 1)
                logger.warning(f"Rate limited, retrying in {wait}s...")
                time.sleep(wait)
            else:
                raise


def wants_tool_use(response):
    """Check if the response wants to call tools."""
    return response.choices[0].finish_reason == "tool_calls"


def is_done(response):
    """Check if the response is a final turn (no more tool calls)."""
    return response.choices[0].finish_reason == "stop"


def get_tool_calls(response):
    """Extract tool calls from a response.

    Returns:
        List of (tool_call_id, function_name, arguments_dict) tuples.
    """
    tool_calls = response.choices[0].message.tool_calls or []
    result = []
    for tc in tool_calls:
        args = tc.function.arguments
        if isinstance(args, str):
            args = json.loads(args)
        result.append((tc.id, tc.function.name, args))
    return result


def get_text_content(response):
    """Extract text content from a response."""
    return response.choices[0].message.content or ""


def assistant_message(response):
    """Get the assistant message for appending to conversation history."""
    return response.choices[0].message


def tool_result_message(tool_call_id, content):
    """Format a tool result message (OpenAI format).

    `content` may be either a string (normal case) or a list of OpenAI-format
    content blocks (e.g. `[{"type": "text", ...}, {"type": "image_url", ...}]`)
    for tools that return images. LiteLLM forwards list content to the
    provider's native multi-modal tool-result format.
    """
    return {
        "role": "tool",
        "tool_call_id": tool_call_id,
        "content": content,
    }
