param(
    [string]$Compiler = "C:/msys64/ucrt64/bin/g++.exe"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repo = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
if (-not (Get-Command $Compiler -ErrorAction SilentlyContinue))
{
    throw "Host g++ was not found; pass -Compiler with its path."
}

$build = Join-Path $repo "build/vision-tests"
New-Item -ItemType Directory -Path $build -Force | Out-Null
$source = Join-Path $PSScriptRoot "vision_protocol_test.cpp"
$includes = @("-I", (Join-Path $repo "app/robot"), "-I", (Join-Path $repo "libs/crc/include"))

# Also exercise wire validation under fast-math assumptions, independently of firmware flags.
foreach ($variant in @("debug", "fast-math"))
{
    $executable = Join-Path $build "vision_protocol_$variant.exe"
    $optimization = @("-O0", "-g")
    if ($variant -eq "fast-math")
    {
        $optimization = @("-O2", "-ffast-math")
    }
    $arguments = @("-std=c++17", "-Wall", "-Wextra", "-Werror", "-static") +
        $optimization + $includes + @($source, "-o", $executable)
    & $Compiler @arguments
    if ($LASTEXITCODE -ne 0)
    {
        throw "Vision protocol test compilation ($variant) failed with exit code $LASTEXITCODE."
    }
    Write-Host "Running vision protocol tests ($variant)."
    & $executable
    if ($LASTEXITCODE -ne 0)
    {
        throw "Vision protocol tests ($variant) failed with exit code $LASTEXITCODE."
    }
}
