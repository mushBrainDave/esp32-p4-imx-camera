#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_commit="7101770dc6db2667b3c477cc31365dd1acd6db4e"
if [[ -z "${IDF_PATH:-}" ]] || [[ "$(git -C "$IDF_PATH" rev-parse HEAD)" != "$sdk_commit" ]]; then
  echo "Activate ESP-IDF v6.0.2 at commit $sdk_commit before building." >&2
  exit 1
fi
python3 "$repo_dir/tools/setup-dependencies.py"
app_dir="$repo_dir/components/esp_cam_sensor_imx/examples/imx219_rtsp_eth"
build_dir="$repo_dir/build-camera"
qualified_dir="$repo_dir/config/guition"
if [[ -f "$app_dir/dependencies.lock" ]] && ! cmp -s "$app_dir/dependencies.lock" "$qualified_dir/dependencies.lock.qualified"; then
  echo "Existing application dependency lock differs. Use a clean checkout; it was preserved." >&2
  exit 1
fi
mkdir -p "$build_dir"
cp "$qualified_dir/dependencies.lock.qualified" "$app_dir/dependencies.lock"
if [[ ! -f "$build_dir/sdkconfig" ]]; then
  cp "$qualified_dir/sdkconfig.qualified" "$build_dir/sdkconfig"
fi
idf.py -C "$app_dir" -B "$build_dir" -D "SDKCONFIG=$build_dir/sdkconfig" reconfigure build
