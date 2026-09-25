# source this before running anything in this project
export PROJ="/home/machine/so101-yash"
export UV_CACHE_DIR="$PROJ/.caches/uv"
export HF_HOME="$PROJ/.caches/hf"
export MUJOCO_GL=egl        # headless rendering; falls back to osmesa
export TOKENIZERS_PARALLELISM=false
export UV_PYTHON_INSTALL_DIR="$PROJ/.caches/pythons"
export CODEX_HOME="$PROJ/.codex"  # project-local Codex config + API key (gitignored)
# Use the complete standalone package, including its Code Mode helper.
codex() { /home/machine/.local/bin/codex "$@"; }
