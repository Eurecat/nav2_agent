#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Gemma 4 26B GGUF on Jetson Thor through NVIDIA's llama.cpp container.
# Exposes an OpenAI-compatible API on /v1 for nav2_agent.
# ==============================================================================
GEMMA_DIR="${GEMMA_DIR:-$HOME/.cache/huggingface/hub/ggml-org_gemma-4-26B-A4B-it-GGUF}"
GEMMA_MODEL="${GEMMA_MODEL:-gemma-4-26B-A4B-it-Q4_K_M.gguf}"
GEMMA_MMPROJ="${GEMMA_MMPROJ:-mmproj-gemma-4-26B-A4B-it-Q8_0.gguf}"
GEMMA_HF="${GEMMA_HF:-https://huggingface.co/ggml-org/gemma-4-26B-A4B-it-GGUF/resolve/main}"
LLAMA_IMAGE="${LLAMA_IMAGE:-ghcr.io/nvidia-ai-iot/llama_cpp:latest-jetson-thor}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"
DOCKER_GPU_ARGS="${DOCKER_GPU_ARGS:---runtime nvidia}"
DOCKER_PULL_ARGS="${DOCKER_PULL_ARGS:-}"
CONTEXT_SIZE="${CONTEXT_SIZE:-8192}"
PARALLEL="${PARALLEL:-1}"
REASONING="${REASONING:-off}"
EXTRA_LLAMA_ARGS="${EXTRA_LLAMA_ARGS:-}"
DRY_RUN="${DRY_RUN:-0}"

mkdir -p "$GEMMA_DIR"

download_if_missing() {
  local name="$1"
  local dst="$2"
  local url="$3"

  if [[ -f "$dst" ]]; then
    return
  fi

  echo "Downloading $name..."
  curl -L --fail --progress-bar -o "${dst}.tmp" "$url" || {
    rm -f "${dst}.tmp"
    echo "ERROR: failed downloading $name ($url)"
    exit 1
  }
  mv "${dst}.tmp" "$dst"
}

parse_shell_args() {
  local raw_args="$1"
  local output_name="$2"

  if [[ -z "$raw_args" ]]; then
    eval "$output_name=()"
    return
  fi

  eval "$output_name=($raw_args)"
}

llama_args=(
  llama-server
  -m /model.gguf
  --mmproj /mmproj.gguf
  --ctx-size "$CONTEXT_SIZE"
  --parallel "$PARALLEL"
  --mlock
  --reasoning "$REASONING"
  --host "$HOST"
  --port "$PORT"
)

if [[ -n "$EXTRA_LLAMA_ARGS" ]]; then
  parse_shell_args "$EXTRA_LLAMA_ARGS" extra_llama_args
  llama_args+=("${extra_llama_args[@]}")
fi

parse_shell_args "$DOCKER_PULL_ARGS" docker_pull_args
parse_shell_args "$DOCKER_GPU_ARGS" docker_gpu_args

docker_cmd=(
  docker run --rm
  "${docker_pull_args[@]}"
  "${docker_gpu_args[@]}"
  --network host
  -v "$GEMMA_DIR/$GEMMA_MODEL:/model.gguf:ro"
  -v "$GEMMA_DIR/$GEMMA_MMPROJ:/mmproj.gguf:ro"
  "$LLAMA_IMAGE"
  "${llama_args[@]}"
)

echo "=== Gemma 4 26B GGUF via llama.cpp host=${HOST} port=${PORT} model=${GEMMA_MODEL} ==="
printf 'Executing:'
printf ' %q' "${docker_cmd[@]}"
printf '\n'

if [[ -n "$DRY_RUN" && "${DRY_RUN,,}" != "0" && "${DRY_RUN,,}" != "false" ]]; then
  exit 0
fi

download_if_missing "$GEMMA_MODEL" "$GEMMA_DIR/$GEMMA_MODEL" "$GEMMA_HF/$GEMMA_MODEL"
download_if_missing "$GEMMA_MMPROJ" "$GEMMA_DIR/$GEMMA_MMPROJ" "$GEMMA_HF/$GEMMA_MMPROJ"

exec "${docker_cmd[@]}"
