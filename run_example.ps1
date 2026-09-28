# ============================================================================
# 运行单个 example (Windows / PowerShell), 用法:
#
#   .\run_example.ps1 01_hello
#   .\run_example.ps1 02_movej --go --speed 0.2
#   $env:LITEARM_PORT="COM7"; .\run_example.ps1 01_hello
#
# 等价于: . .\env.ps1 然后跑 build\<name>.exe [args...]
# ============================================================================
param(
    [Parameter(Mandatory = $true, Position = 0)][string]$Name,
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$Rest
)
$ErrorActionPreference = "Stop"

$root = $PSScriptRoot
. (Join-Path $root "env.ps1")

$exe = Join-Path $LiteArmBuild "$Name.exe"
if (-not (Test-Path $exe)) { $exe = Join-Path $LiteArmBuild "Release\$Name.exe" }
if (-not (Test-Path $exe)) {
    Write-Host "找不到样例可执行文件: $Name (先跑 .\env.ps1 或 cmake --build build)"
    Get-ChildItem (Join-Path $root "examples\*.cpp") |
        Where-Object { $_.BaseName -notlike "_*" } |
        ForEach-Object { Write-Host "  $($_.BaseName)" }
    exit 1
}
& $exe @Rest
