#!/usr/bin/env bash

set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repository_root"

GOOS=js GOARCH=wasm go build -o ./example/2d/public/main.wasm ./example/2d/src
GOOS=js GOARCH=wasm go build -o ./example/3d/public/main.wasm ./example/3d/src
