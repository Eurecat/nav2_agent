#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Gemma 4 on Jetson Thor through NVIDIA's vLLM container.
# ==============================================================================
VLLM_IMAGE="${VLLM_IMAGE:-ghcr.io/nvidia-ai-iot/vllm:gemma4-jetson-thor}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"
DOCKER_PULL_ARGS="${DOCKER_PULL_ARGS:---pull always}"
DOCKER_GPU_ARGS="${DOCKER_GPU_ARGS:---runtime nvidia}"
HF_HOME="${HF_HOME:-$HOME/.cache/huggingface}"

MODEL_REPO="${MODEL_REPO:-google/gemma-4-E4B-it}"
SERVED_MODEL_NAME="${SERVED_MODEL_NAME:-gemma-4-e4b}"

MAX_MODEL_LEN="${MAX_MODEL_LEN:-8192}"
MAX_NUM_SEQS="${MAX_NUM_SEQS:-1}"
GPU_MEMORY_UTILIZATION="${GPU_MEMORY_UTILIZATION:-0.8}"
TENSOR_PARALLEL_SIZE="${TENSOR_PARALLEL_SIZE:-1}"
GENERATION_CONFIG="${GENERATION_CONFIG:-vllm}"

DOWNLOAD_DIR="${DOWNLOAD_DIR:-/root/.cache/huggingface}"
API_KEY="${API_KEY:-EMPTY}"

# Keep tool calling enabled, but do not enable the reasoning parser by default.
# The Gemma raw response already exposes OpenAI-style tool_calls; the extra
# reasoning parser is not needed for this text/tool workflow and can complicate
# downstream client parsing.
REASONING_PARSER="${REASONING_PARSER:-}"
ENABLE_AUTO_TOOL_CHOICE="${ENABLE_AUTO_TOOL_CHOICE:-1}"
TOOL_CALL_PARSER="${TOOL_CALL_PARSER:-gemma4}"

EXTRA_VLLM_ARGS="${EXTRA_VLLM_ARGS:-}"
DRY_RUN="${DRY_RUN:-0}"

mkdir -p "$HF_HOME"

parse_shell_args() {
  local raw_args="$1"
  local output_name="$2"

  if [[ -z "$raw_args" ]]; then
    eval "$output_name=()"
    return
  fi

  eval "$output_name=($raw_args)"
}

vllm_args=(
  vllm serve "$MODEL_REPO"
  --served-model-name "$SERVED_MODEL_NAME"
  --host "$HOST"
  --port "$PORT"
  --api-key "$API_KEY"
  --download-dir "$DOWNLOAD_DIR"
  --gpu-memory-utilization "$GPU_MEMORY_UTILIZATION"
  --max-model-len "$MAX_MODEL_LEN"
  --max-num-seqs "$MAX_NUM_SEQS"
  --tensor-parallel-size "$TENSOR_PARALLEL_SIZE"
)

if [[ -n "$GENERATION_CONFIG" ]]; then
  vllm_args+=(--generation-config "$GENERATION_CONFIG")
fi

if [[ -n "$REASONING_PARSER" ]]; then
  vllm_args+=(--reasoning-parser "$REASONING_PARSER")
fi

if [[ -n "$ENABLE_AUTO_TOOL_CHOICE" && "${ENABLE_AUTO_TOOL_CHOICE,,}" != "0" && "${ENABLE_AUTO_TOOL_CHOICE,,}" != "false" ]]; then
  vllm_args+=(--enable-auto-tool-choice)
fi

if [[ -n "$TOOL_CALL_PARSER" ]]; then
  vllm_args+=(--tool-call-parser "$TOOL_CALL_PARSER")
fi

if [[ -n "$EXTRA_VLLM_ARGS" ]]; then
  parse_shell_args "$EXTRA_VLLM_ARGS" extra_vllm_args
  vllm_args+=("${extra_vllm_args[@]}")
fi

printf -v vllm_serve_cmd '%q ' "${vllm_args[@]}"

container_cmd="$vllm_serve_cmd"

parse_shell_args "$DOCKER_PULL_ARGS" docker_pull_args
parse_shell_args "$DOCKER_GPU_ARGS" docker_gpu_args

docker_cmd=(
  sudo docker run --rm
  "${docker_pull_args[@]}"
  "${docker_gpu_args[@]}"
  --network host
  -v "$HF_HOME:/root/.cache/huggingface"
  -e HF_HOME=/root/.cache/huggingface
)

if [[ -n "${HUGGING_FACE_HUB_TOKEN:-}" ]]; then
  docker_cmd+=(-e "HUGGING_FACE_HUB_TOKEN=$HUGGING_FACE_HUB_TOKEN")
fi

docker_cmd+=(
  --entrypoint bash
  "$VLLM_IMAGE"
  -c "$container_cmd"
)

echo "=== Gemma 4 via NVIDIA Jetson vLLM host=${HOST} port=${PORT} model=${MODEL_REPO} served=${SERVED_MODEL_NAME} ==="
printf 'Executing:'
printf ' %q' "${docker_cmd[@]}"
printf '\n'

if [[ -n "$DRY_RUN" && "${DRY_RUN,,}" != "0" && "${DRY_RUN,,}" != "false" ]]; then
  exit 0
fi

exec "${docker_cmd[@]}"