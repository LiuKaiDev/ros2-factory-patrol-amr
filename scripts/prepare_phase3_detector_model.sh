#!/usr/bin/env bash
set -euo pipefail

MODEL_NAME="object_detection_yolox_2022nov.onnx"
MODEL_SHA256="c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063"
CACHE_ROOT="${XDG_CACHE_HOME:-${HOME}/.cache}"
MODEL_DIR="${ROBOT_PERCEPTION_MODEL_DIR:-${CACHE_ROOT}/robot_perception/models}"
MODEL_PATH="${MODEL_DIR}/${MODEL_NAME}"
DOWNLOAD_PATH="${MODEL_PATH}.part"
DEFAULT_MODEL_URL="https://huggingface.co/opencv/opencv_zoo/resolve/main/models/object_detection_yolox/${MODEL_NAME}?download=true"
MODEL_URL="${ROBOT_PERCEPTION_MODEL_URL:-${DEFAULT_MODEL_URL}}"

command -v curl >/dev/null || {
  echo "FAIL: curl is required to download the detector model" >&2
  exit 1
}
mkdir -p "${MODEL_DIR}"

if [[ -f "${MODEL_PATH}" ]] &&
    echo "${MODEL_SHA256}  ${MODEL_PATH}" | sha256sum --check --status; then
  echo "PASS: verified existing model: ${MODEL_PATH}"
  exit 0
fi

echo "Downloading the official OpenCV Zoo YOLOX-S model..."
echo "Source: ${MODEL_URL}"
curl --fail --location --retry 3 --continue-at - \
  --output "${DOWNLOAD_PATH}" "${MODEL_URL}"

if ! echo "${MODEL_SHA256}  ${DOWNLOAD_PATH}" | sha256sum --check --status; then
  echo "FAIL: model checksum mismatch; leaving ${DOWNLOAD_PATH} for inspection" >&2
  exit 1
fi
mv "${DOWNLOAD_PATH}" "${MODEL_PATH}"
echo "PASS: installed and verified ${MODEL_PATH}"
