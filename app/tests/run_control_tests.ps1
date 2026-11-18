param(
    [string]$Compiler = "C:/msys64/ucrt64/bin/g++.exe",
    [string]$CompileDatabase = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repo = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
if ([string]::IsNullOrEmpty($CompileDatabase))
{
    $CompileDatabase = Join-Path $repo "build/h723_mc02/compile_commands.json"
}
if (-not (Test-Path -LiteralPath $CompileDatabase))
{
    throw "A configured firmware compile_commands.json is required; pass -CompileDatabase."
}
if (-not (Get-Command $Compiler -ErrorAction SilentlyContinue))
{
    throw "Host g++ was not found; pass -Compiler with its path."
}

$database = Get-Content -LiteralPath $CompileDatabase -Raw | ConvertFrom-Json
$entry = $database | Where-Object {
    $_.file -match '[\\/]app[\\/]robot[\\/]control[\\/]control\.cpp$'
} | Select-Object -First 1
if ($null -eq $entry)
{
    throw "The compilation database has no app/robot/control/control.cpp entry."
}

# Reuse the firmware's real type headers and definitions, without ARM compiler options.
$flags = @()
$tokens = [regex]::Matches($entry.command, '(?:[^\s"]+|"[^"]*")+')
for ($i = 0; $i -lt $tokens.Count; ++$i)
{
    $token = $tokens[$i].Value.Replace('"', '')
    if ($token -eq "-I" -or $token -eq "-D")
    {
        ++$i
        $flags += $token + $tokens[$i].Value.Replace('"', '')
    }
    elseif ($token.StartsWith("-I") -or $token.StartsWith("-D"))
    {
        $flags += $token
    }
}

$build = Join-Path $repo "build/control-tests"
New-Item -ItemType Directory -Path $build -Force | Out-Null
$executable = Join-Path $build "control_test.exe"
$sources = @("control", "remote", "axis", "gimbal", "shooter", "chassis") | ForEach-Object {
    Join-Path $repo "app/robot/control/$_.cpp"
}
$sources += Join-Path $PSScriptRoot "control_test.cpp"
$arguments = @("-std=c++17", "-O0", "-g", "-Wall", "-Wextra", "-Werror", "-static") +
    $flags + $sources + @("-o", $executable)

Push-Location $entry.directory
try
{
    & $Compiler @arguments
    if ($LASTEXITCODE -ne 0)
    {
        throw "Control test compilation failed with exit code $LASTEXITCODE."
    }
    & $executable
    if ($LASTEXITCODE -ne 0)
    {
        throw "Control tests failed with exit code $LASTEXITCODE."
    }
}
finally
{
    Pop-Location
}
