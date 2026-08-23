#!/usr/bin/env bash
# ============================================================
# Создание изолированного Python 3.12 venv для компьютерного зрения.
# Зависимости CV ставятся сюда, чтобы не конфликтовать с системным rclpy.
# ============================================================
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$PROJECT_ROOT/venv"

if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    echo "[setup_venv] venv создан: $VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

pip install --upgrade pip
pip install opencv-python
pip install ultralytics        # YOLO

# PyTorch с CUDA-поддержкой под GTX 1050 (CUDA 13.0, драйвер 580.x).
# Индекс cu130 = сборки под CUDA 13 (torch 2.13.x) — совпадает с тем,
# что реально установлено и работает в рабочем контейнере (torch.cuda.is_available()=True).
pip install torch==2.13.0 --index-url https://download.pytorch.org/whl/cu130

echo "[setup_venv] Готово. Активация: source venv/bin/activate"
