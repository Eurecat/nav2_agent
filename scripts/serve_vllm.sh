#!/usr/bin/env bash
# Serve a model with vLLM on the OpenAI-compatible endpoint used by nav2_agent's default configuration.
set -euo pipefail

MODEL="${MODEL:-google/gemma-4-E4B-it}"
SERVED_MODEL_NAME="${SERVED_MODEL_NAME:-gemma-4-e4b}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"
API_KEY="${API_KEY:-EMPTY}"
TOOL_CALL_PARSER="${TOOL_CALL_PARSER:-gemma4}"

exec vllm serve "$MODEL" \
  --served-model-name "$SERVED_MODEL_NAME" \
  --host "$HOST" \
  --port "$PORT" \
  --api-key "$API_KEY" \
  --generation-config vllm \
  --enable-auto-tool-choice \
  --tool-call-parser "$TOOL_CALL_PARSER" \
  "$@"
