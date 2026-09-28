# ============================================================================
# litearm-cpp 运行环境 (Windows / PowerShell, dot-source 用)
#
#   . .\env.ps1
#   .\run_example.ps1 01_hello
#
# 作用: 找到/构建 build\ 并把可执行文件目录加进 PATH; LITEARM_PORT 可覆盖 CDC 端口。
#
# ⚠ SDK 本身不读任何环境变量 —— LITEARM_PORT 只被样例/脚本这一层用。
# ⚠ Windows 上的**端口自动发现不可用** (要 SetupAPI 才能可靠读到 VID:PID),
#   请显式传端口或用 LITEARM_PORT。
# ============================================================================
$ErrorActionPreference = "Stop"

$LiteArmRepo = $PSScriptRoot
$LiteArmBuild = if ($env:LITEARM_BUILD) { $env:LITEARM_BUILD } else { Join-Path $LiteArmRepo "build" }

$probe = Join-Path $LiteArmBuild "Release\01_hello.exe"
if (-not (Test-Path $probe)) { $probe = Join-Path $LiteArmBuild "01_hello.exe" }
if (-not (Test-Path $probe)) {
    Write-Host "[litearm-cpp env] 首次运行, 正在构建 -> $LiteArmBuild"
    cmake -S $LiteArmRepo -B $LiteArmBuild -DCMAKE_BUILD_TYPE=Release | Out-Null
    cmake --build $LiteArmBuild --config Release | Out-Null
}

$env:LITEARM_REPO  = $LiteArmRepo
$env:LITEARM_BUILD = $LiteArmBuild
$env:LITEARM_BIN   = $LiteArmBuild
$env:PATH          = "$LiteArmBuild;$LiteArmBuild\Release;$env:PATH"
if (-not $env:LITEARM_PORT) { $env:LITEARM_PORT = "" }

Write-Host "[litearm-cpp env] repo=$LiteArmRepo"
Write-Host "  build        = $LiteArmBuild"
Write-Host "  LITEARM_PORT = '$($env:LITEARM_PORT)'"
Write-Host "  ⚠ Windows 上端口自动发现不可用, 请显式给 --port 或设 LITEARM_PORT"
