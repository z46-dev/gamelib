$ErrorActionPreference = "Stop"
$env:GOOS = "js"
$env:GOARCH = "wasm"
$repositoryRoot = Split-Path -Parent $PSScriptRoot

Push-Location $repositoryRoot
try {
    go build -o ./example/2d/public/main.wasm ./example/2d/src
    go build -o ./example/3d/public/main.wasm ./example/3d/src
} finally {
    Pop-Location
}
