#!/usr/bin/env bash

set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repository_root"

GOOS=js GOARCH=wasm go build -o ./example/public/main.wasm ./example/src