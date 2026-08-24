$ErrorActionPreference = "Stop"
$env:GOOS = "js"
$env:GOARCH = "wasm"
$repositoryRoot = Split-Path -Parent $PSScriptRoot

Push-Location $repositoryRoot
try {
    go build -o ./example/public/main.wasm ./example/src
} finally {
    Pop-Location
}
